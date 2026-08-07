#include "sx1262_radio.h"

#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "sx1262_radio_backend.h"

#define SX1262_RADIO_TX_WATCHDOG_MARGIN_US UINT64_C(5000)
#define SX1262_RADIO_RTC_STEPS_PER_SECOND UINT64_C(64000)
#define SX1262_RADIO_US_PER_SECOND UINT64_C(1000000)

typedef struct {
  curag_radio_state_t state;
  bool hardware_touched;
  bool initialization_failure_cached;
  bool rx_armed;
  err_curag_t initialization_error;
  diagn_context_t initialization_diag;
} sx1262_radio_singleton_t;

static sx1262_radio_singleton_t s_radio;

static const sx1262_radio_profile_t k_pilot_profile = {
    .frequency_hz = SX1262_RADIO_FREQUENCY_HZ,
    .bandwidth_hz = SX1262_RADIO_BANDWIDTH_HZ,
    .preamble_symbols = SX1262_RADIO_PREAMBLE_SYMBOLS,
    .ramp_us = SX1262_RADIO_TX_RAMP_US,
    .tcxo_startup_ms = 5U,
    .radiated_tx_power_dbm = SX1262_RADIO_TX_POWER_DBM,
    .tx_params_power_dbm = INT8_C(22),
    .spreading_factor = SX1262_RADIO_SPREADING_FACTOR,
    .coding_rate_denominator = SX1262_RADIO_CODING_RATE_DENOMINATOR,
    .sync_word = SX1262_RADIO_SYNC_WORD,
    .pa_duty_cycle = UINT8_C(0x02),
    .pa_hp_max = UINT8_C(0x02),
    .pa_device_sel = UINT8_C(0x00),
    .pa_lut = UINT8_C(0x01),
    .tcxo_voltage = UINT8_C(0x01), /* SX126x TCXO_CTRL_1_7V. */
    .low_data_rate_optimize = false,
    .explicit_header = true,
    .payload_crc = true,
    .boosted_rx = true,
    .dcdc_regulator = true,
    .dio2_rf_switch = true,
    .dio3_tcxo = true,
};

static err_curag_t radio_error(uint16_t code) {
  return curag_error_make(CURAG_EDOM_RADIO, code);
}

static void write_u16_le(uint8_t output[2], uint16_t value) {
  output[0] = (uint8_t)value;
  output[1] = (uint8_t)(value >> 8U);
}

static void write_u32_le(uint8_t output[4], uint32_t value) {
  output[0] = (uint8_t)value;
  output[1] = (uint8_t)(value >> 8U);
  output[2] = (uint8_t)(value >> 16U);
  output[3] = (uint8_t)(value >> 24U);
}

static void populate_diagnostic(diagn_context_t *out_diag,
                                curag_operation_t operation,
                                curag_radio_state_t observed_state,
                                const sx1262_radio_backend_error_t *detail) {
  if (out_diag == NULL) {
    return;
  }
  curag_diagnostic_context_clear(out_diag);
  out_diag->operation = operation;
  out_diag->context_schema = CURAG_RADIO_CONTEXT_V1;
  out_diag->context_length = CURAG_RADIO_CONTEXT_V1_LENGTH;
  out_diag->context[0] = (uint8_t)observed_state;
  out_diag->context[1] = detail->command_opcode;
  out_diag->context[2] = detail->stage;
  out_diag->context[3] = detail->flags;
  out_diag->context[4] = detail->backend_status_kind;
  write_u32_le(&out_diag->context[5], (uint32_t)detail->backend_status);
  out_diag->context[9] = detail->chip_status;
  write_u16_le(&out_diag->context[10], detail->irq_status);
  write_u16_le(&out_diag->context[12], detail->device_errors);
}

static err_curag_t semantic_failure(uint16_t error_code,
                                    curag_operation_t operation,
                                    curag_radio_stage_t stage,
                                    diagn_context_t *out_diag) {
  sx1262_radio_backend_error_t detail;
  sx1262_radio_backend_error_clear(&detail);
  detail.error_code = error_code;
  detail.stage = (uint8_t)stage;
  if (s_radio.hardware_touched) {
    detail.flags = CURAG_RADIO_CONTEXT_HARDWARE_TOUCHED;
  }
  populate_diagnostic(out_diag, operation, s_radio.state, &detail);
  return radio_error(error_code);
}

static err_curag_t backend_failure(const sx1262_radio_backend_error_t *detail,
                                   curag_operation_t operation,
                                   curag_radio_state_t observed_state,
                                   diagn_context_t *out_diag) {
  sx1262_radio_backend_error_t normalized = *detail;
  if (normalized.error_code == CURAG_ECODE_NONE) {
    normalized.error_code = CURAG_ERADIO_EIO;
  }
  if (s_radio.hardware_touched) {
    normalized.flags =
        (uint8_t)(normalized.flags | CURAG_RADIO_CONTEXT_HARDWARE_TOUCHED);
  }
  populate_diagnostic(out_diag, operation, observed_state, &normalized);
  return radio_error(normalized.error_code);
}

static void mark_active_failure(const sx1262_radio_backend_error_t *detail) {
  if ((detail->flags & CURAG_RADIO_CONTEXT_HARDWARE_TOUCHED) != 0U) {
    s_radio.hardware_touched = true;
  }
  s_radio.state = CURAG_RADIO_STATE_FAILED;
  s_radio.rx_armed = false;
}

static err_curag_t ensure_initialized(diagn_context_t *out_diag) {
  if (s_radio.state == CURAG_RADIO_STATE_READY) {
    return CURAG_OK;
  }
  if (s_radio.initialization_failure_cached) {
    if (out_diag != NULL) {
      *out_diag = s_radio.initialization_diag;
    }
    return s_radio.initialization_error;
  }
  if (s_radio.state != CURAG_RADIO_STATE_UNTOUCHED) {
    return semantic_failure(CURAG_ERADIO_EINVALID_STATE, CURAG_OP_VALIDATE,
                            CURAG_RADIO_STAGE_STATE_CHECK, out_diag);
  }

  sx1262_radio_backend_error_t detail;
  sx1262_radio_backend_error_clear(&detail);
  const curag_radio_state_t observed_state = s_radio.state;
  if (!sx1262_radio_backend_initialize(&k_pilot_profile, &detail)) {
    if ((detail.flags & CURAG_RADIO_CONTEXT_HARDWARE_TOUCHED) != 0U) {
      s_radio.hardware_touched = true;
    }
    s_radio.state = CURAG_RADIO_STATE_FAILED;
    s_radio.initialization_failure_cached = true;
    s_radio.initialization_error =
        backend_failure(&detail, CURAG_OP_INITIALIZE, observed_state,
                        &s_radio.initialization_diag);
    if (out_diag != NULL) {
      *out_diag = s_radio.initialization_diag;
    }
    return s_radio.initialization_error;
  }

  s_radio.hardware_touched = true;
  s_radio.state = CURAG_RADIO_STATE_READY;
  s_radio.rx_armed = false;
  return CURAG_OK;
}

static bool deadline_reached(uint64_t deadline_monotonic_us) {
  return sx1262_radio_backend_monotonic_us() >= deadline_monotonic_us;
}

static uint32_t tx_watchdog_steps(uint64_t now_us,
                                  uint64_t deadline_monotonic_us) {
  uint64_t usable_us = deadline_monotonic_us - now_us;
  if (usable_us > SX1262_RADIO_TX_WATCHDOG_MARGIN_US) {
    usable_us -= SX1262_RADIO_TX_WATCHDOG_MARGIN_US;
  }
  const uint64_t maximum_steps = UINT64_C(0x00ffffff);
  const uint64_t maximum_duration_us =
      (maximum_steps * SX1262_RADIO_US_PER_SECOND) /
      SX1262_RADIO_RTC_STEPS_PER_SECOND;
  if (usable_us >= maximum_duration_us) {
    return (uint32_t)maximum_steps;
  }
  uint64_t steps = (usable_us * SX1262_RADIO_RTC_STEPS_PER_SECOND) /
                   SX1262_RADIO_US_PER_SECOND;
  if (steps == 0U) {
    steps = 1U;
  }
  return (uint32_t)steps;
}

static err_curag_t
fail_active_operation(const sx1262_radio_backend_error_t *detail,
                      curag_operation_t operation, diagn_context_t *out_diag) {
  const curag_radio_state_t observed_state = s_radio.state;
  const err_curag_t error =
      backend_failure(detail, operation, observed_state, out_diag);
  mark_active_failure(detail);
  return error;
}

static err_curag_t
transmit_backend_failure(const sx1262_radio_backend_error_t *detail,
                         diagn_context_t *out_diag) {
  return fail_active_operation(detail, CURAG_OP_TRANSMIT, out_diag);
}

err_curag_t sx1262_radio_transmit_uplink(const uint8_t *payload,
                                         size_t payload_length,
                                         uint64_t deadline_monotonic_us,
                                         sx1262_radio_tx_result_t *out_result,
                                         diagn_context_t *out_diag) {
  curag_diagnostic_context_clear(out_diag);
  if (out_result != NULL) {
    memset(out_result, 0, sizeof(*out_result));
  }
  if (payload == NULL || payload_length == 0U ||
      payload_length > SX1262_RADIO_MAX_PAYLOAD_SIZE || out_result == NULL) {
    return semantic_failure(CURAG_ERADIO_EINVALID_ARGUMENT, CURAG_OP_VALIDATE,
                            CURAG_RADIO_STAGE_VALIDATE_INPUT, out_diag);
  }
  if (s_radio.state == CURAG_RADIO_STATE_SLEEPING ||
      (s_radio.state == CURAG_RADIO_STATE_FAILED &&
       !s_radio.initialization_failure_cached)) {
    return semantic_failure(CURAG_ERADIO_EINVALID_STATE, CURAG_OP_VALIDATE,
                            CURAG_RADIO_STAGE_STATE_CHECK, out_diag);
  }
  if (deadline_reached(deadline_monotonic_us)) {
    return semantic_failure(CURAG_ERADIO_EDEADLINE, CURAG_OP_TRANSMIT,
                            CURAG_RADIO_STAGE_VALIDATE_INPUT, out_diag);
  }

  err_curag_t error = ensure_initialized(out_diag);
  if (error != CURAG_OK) {
    return error;
  }

  sx1262_radio_backend_error_t detail;
  sx1262_radio_backend_error_clear(&detail);
  if (s_radio.rx_armed &&
      sx1262_radio_backend_set_standby(&detail) != SX1262_COMMAND_CONFIRMED) {
    return transmit_backend_failure(&detail, out_diag);
  }
  s_radio.rx_armed = false;

  if (sx1262_radio_backend_set_packet_params((uint8_t)payload_length, false,
                                             &detail) !=
          SX1262_COMMAND_CONFIRMED ||
      sx1262_radio_backend_configure_irq(SX1262_RADIO_IRQ_TX_DONE |
                                             SX1262_RADIO_IRQ_TIMEOUT,
                                         &detail) != SX1262_COMMAND_CONFIRMED ||
      sx1262_radio_backend_clear_irq(SX1262_RADIO_IRQ_ALL, &detail) !=
          SX1262_COMMAND_CONFIRMED ||
      sx1262_radio_backend_write_payload(payload, (uint8_t)payload_length,
                                         &detail) != SX1262_COMMAND_CONFIRMED) {
    return transmit_backend_failure(&detail, out_diag);
  }

  const uint64_t set_tx_at_us = sx1262_radio_backend_monotonic_us();
  if (set_tx_at_us >= deadline_monotonic_us) {
    return semantic_failure(CURAG_ERADIO_EDEADLINE, CURAG_OP_TRANSMIT,
                            CURAG_RADIO_STAGE_VALIDATE_INPUT, out_diag);
  }
  const sx1262_radio_backend_call_result_t start_result =
      sx1262_radio_backend_start_tx(
          tx_watchdog_steps(set_tx_at_us, deadline_monotonic_us), &detail);
  if (start_result != SX1262_COMMAND_FAILED) {
    out_result->tx_started = true;
    out_result->set_tx_at_us = set_tx_at_us;
  }
  if (start_result != SX1262_COMMAND_CONFIRMED) {
    return transmit_backend_failure(&detail, out_diag);
  }

  bool irq_observed = false;
  uint64_t irq_at_us = 0U;
  if (!sx1262_radio_backend_wait_dio1(deadline_monotonic_us, &irq_observed,
                                      &irq_at_us, &detail)) {
    return transmit_backend_failure(&detail, out_diag);
  }
  if (!irq_observed) {
    sx1262_radio_backend_error_clear(&detail);
    detail.error_code = CURAG_ERADIO_EDEADLINE;
    detail.stage = CURAG_RADIO_STAGE_WAIT_IRQ;
    return transmit_backend_failure(&detail, out_diag);
  }

  uint16_t irq_status = 0U;
  if (sx1262_radio_backend_get_irq(&irq_status, &detail) !=
      SX1262_COMMAND_CONFIRMED) {
    return transmit_backend_failure(&detail, out_diag);
  }
  if ((irq_status & SX1262_RADIO_IRQ_TX_DONE) != 0U &&
      irq_at_us >= set_tx_at_us) {
    out_result->tx_done = true;
    out_result->tx_done_at_us = irq_at_us;
  }
  if (sx1262_radio_backend_clear_irq(irq_status, &detail) !=
      SX1262_COMMAND_CONFIRMED) {
    return transmit_backend_failure(&detail, out_diag);
  }

  if (irq_status == SX1262_RADIO_IRQ_TX_DONE && irq_at_us >= set_tx_at_us &&
      irq_at_us <= deadline_monotonic_us) {
    return CURAG_OK;
  }

  sx1262_radio_backend_error_clear(&detail);
  detail.flags = (uint8_t)(CURAG_RADIO_CONTEXT_IRQ_STATUS_VALID |
                           CURAG_RADIO_CONTEXT_HARDWARE_TOUCHED);
  detail.irq_status = irq_status;
  detail.stage = CURAG_RADIO_STAGE_READ_IRQ;
  if ((irq_status & SX1262_RADIO_IRQ_TIMEOUT) != 0U ||
      irq_at_us > deadline_monotonic_us) {
    detail.error_code = CURAG_ERADIO_EDEADLINE;
  } else {
    uint16_t device_errors = 0U;
    sx1262_radio_backend_error_t device_detail;
    sx1262_radio_backend_error_clear(&device_detail);
    if (sx1262_radio_backend_get_device_errors(
            &device_errors, &device_detail) != SX1262_COMMAND_CONFIRMED) {
      return transmit_backend_failure(&device_detail, out_diag);
    }
    if (device_errors != 0U) {
      detail.error_code = CURAG_ERADIO_EDEVICE_ERROR;
      detail.flags =
          (uint8_t)(detail.flags | CURAG_RADIO_CONTEXT_DEVICE_ERRORS_VALID);
      detail.device_errors = device_errors;
    } else {
      detail.error_code = CURAG_ERADIO_EUNEXPECTED_IRQ;
    }
  }
  return transmit_backend_failure(&detail, out_diag);
}

static err_curag_t
receive_backend_failure(const sx1262_radio_backend_error_t *detail,
                        diagn_context_t *out_diag) {
  return fail_active_operation(detail, CURAG_OP_RECEIVE, out_diag);
}

static err_curag_t start_downlink_rx(diagn_context_t *out_diag) {
  sx1262_radio_backend_error_t detail;
  sx1262_radio_backend_error_clear(&detail);
  if (sx1262_radio_backend_set_standby(&detail) != SX1262_COMMAND_CONFIRMED ||
      sx1262_radio_backend_set_packet_params(
          (uint8_t)SX1262_RADIO_MAX_PAYLOAD_SIZE, true, &detail) !=
          SX1262_COMMAND_CONFIRMED ||
      sx1262_radio_backend_configure_irq(SX1262_RADIO_IRQ_RX_DONE |
                                             SX1262_RADIO_IRQ_HEADER_ERROR |
                                             SX1262_RADIO_IRQ_CRC_ERROR,
                                         &detail) != SX1262_COMMAND_CONFIRMED ||
      sx1262_radio_backend_clear_irq(SX1262_RADIO_IRQ_ALL, &detail) !=
          SX1262_COMMAND_CONFIRMED ||
      sx1262_radio_backend_start_single_rx(&detail) !=
          SX1262_COMMAND_CONFIRMED) {
    return receive_backend_failure(&detail, out_diag);
  }
  s_radio.rx_armed = true;
  return CURAG_OK;
}

err_curag_t
sx1262_radio_receive_downlink_until(uint64_t deadline_monotonic_us,
                                    sx1262_radio_rx_result_t *out_result,
                                    diagn_context_t *out_diag) {
  curag_diagnostic_context_clear(out_diag);
  if (out_result != NULL) {
    memset(out_result, 0, sizeof(*out_result));
  }
  if (out_result == NULL) {
    return semantic_failure(CURAG_ERADIO_EINVALID_ARGUMENT, CURAG_OP_VALIDATE,
                            CURAG_RADIO_STAGE_VALIDATE_INPUT, out_diag);
  }
  if (s_radio.state != CURAG_RADIO_STATE_READY) {
    return semantic_failure(CURAG_ERADIO_EINVALID_STATE, CURAG_OP_VALIDATE,
                            CURAG_RADIO_STAGE_STATE_CHECK, out_diag);
  }
  if (!s_radio.rx_armed && deadline_reached(deadline_monotonic_us)) {
    out_result->outcome = SX1262_RADIO_RX_DEADLINE;
    return CURAG_OK;
  }

  if (!s_radio.rx_armed) {
    const err_curag_t error = start_downlink_rx(out_diag);
    if (error != CURAG_OK) {
      return error;
    }
  }

  while (true) {
    sx1262_radio_backend_error_t detail;
    sx1262_radio_backend_error_clear(&detail);
    bool irq_observed = false;
    uint64_t irq_at_us = 0U;
    if (!sx1262_radio_backend_wait_dio1(deadline_monotonic_us, &irq_observed,
                                        &irq_at_us, &detail)) {
      return receive_backend_failure(&detail, out_diag);
    }
    if (!irq_observed) {
      out_result->outcome = SX1262_RADIO_RX_DEADLINE;
      return CURAG_OK;
    }
    s_radio.rx_armed = false;

    uint16_t irq_status = 0U;
    if (sx1262_radio_backend_get_irq(&irq_status, &detail) !=
        SX1262_COMMAND_CONFIRMED) {
      return receive_backend_failure(&detail, out_diag);
    }
    if (irq_at_us > deadline_monotonic_us) {
      if (sx1262_radio_backend_clear_irq(irq_status, &detail) !=
          SX1262_COMMAND_CONFIRMED) {
        return receive_backend_failure(&detail, out_diag);
      }
      out_result->outcome = SX1262_RADIO_RX_DEADLINE;
      return CURAG_OK;
    }
    if ((irq_status &
         (SX1262_RADIO_IRQ_HEADER_ERROR | SX1262_RADIO_IRQ_CRC_ERROR)) != 0U) {
      if (sx1262_radio_backend_clear_irq(irq_status, &detail) !=
          SX1262_COMMAND_CONFIRMED) {
        return receive_backend_failure(&detail, out_diag);
      }
      if (deadline_reached(deadline_monotonic_us)) {
        out_result->outcome = SX1262_RADIO_RX_DEADLINE;
        return CURAG_OK;
      }
      if (sx1262_radio_backend_start_single_rx(&detail) !=
          SX1262_COMMAND_CONFIRMED) {
        return receive_backend_failure(&detail, out_diag);
      }
      s_radio.rx_armed = true;
      continue;
    }
    if (irq_status != SX1262_RADIO_IRQ_RX_DONE) {
      if (sx1262_radio_backend_clear_irq(irq_status, &detail) !=
          SX1262_COMMAND_CONFIRMED) {
        return receive_backend_failure(&detail, out_diag);
      }
      sx1262_radio_backend_error_clear(&detail);
      detail.error_code = CURAG_ERADIO_EUNEXPECTED_IRQ;
      detail.stage = CURAG_RADIO_STAGE_READ_IRQ;
      detail.flags = (uint8_t)(CURAG_RADIO_CONTEXT_IRQ_STATUS_VALID |
                               CURAG_RADIO_CONTEXT_HARDWARE_TOUCHED);
      detail.irq_status = irq_status;
      return receive_backend_failure(&detail, out_diag);
    }
    if (sx1262_radio_backend_handle_rx_done(&detail) !=
        SX1262_COMMAND_CONFIRMED) {
      return receive_backend_failure(&detail, out_diag);
    }

    sx1262_radio_backend_rx_buffer_status_t buffer_status = {0};
    if (sx1262_radio_backend_get_rx_buffer_status(&buffer_status, &detail) !=
        SX1262_COMMAND_CONFIRMED) {
      return receive_backend_failure(&detail, out_diag);
    }
    if (buffer_status.payload_length > SX1262_RADIO_MAX_PAYLOAD_SIZE) {
      sx1262_radio_backend_error_clear(&detail);
      detail.error_code = CURAG_ERADIO_EIO;
      detail.stage = CURAG_RADIO_STAGE_READ_BUFFER;
      detail.flags = CURAG_RADIO_CONTEXT_HARDWARE_TOUCHED;
      return receive_backend_failure(&detail, out_diag);
    }
    if (buffer_status.payload_length > 0U &&
        sx1262_radio_backend_read_buffer(buffer_status.start_offset,
                                         out_result->payload,
                                         (uint8_t)buffer_status.payload_length,
                                         &detail) != SX1262_COMMAND_CONFIRMED) {
      return receive_backend_failure(&detail, out_diag);
    }

    sx1262_radio_backend_packet_status_t packet_status = {0};
    if (sx1262_radio_backend_get_packet_status(&packet_status, &detail) !=
        SX1262_COMMAND_CONFIRMED) {
      memset(out_result, 0, sizeof(*out_result));
      return receive_backend_failure(&detail, out_diag);
    }
    if (sx1262_radio_backend_clear_irq(irq_status, &detail) !=
        SX1262_COMMAND_CONFIRMED) {
      memset(out_result, 0, sizeof(*out_result));
      return receive_backend_failure(&detail, out_diag);
    }
    out_result->outcome = SX1262_RADIO_RX_PACKET;
    out_result->rx_done_at_us = irq_at_us;
    out_result->rssi_dbm_x2 = packet_status.rssi_dbm_x2;
    out_result->snr_db_x4 = packet_status.snr_db_x4;
    out_result->payload_length = (uint8_t)buffer_status.payload_length;
    /* Rearm after the snapshot so a later packet cannot replace its data. */
    if (!deadline_reached(deadline_monotonic_us)) {
      if (sx1262_radio_backend_start_single_rx(&detail) !=
          SX1262_COMMAND_CONFIRMED) {
        memset(out_result, 0, sizeof(*out_result));
        return receive_backend_failure(&detail, out_diag);
      }
      s_radio.rx_armed = true;
    }
    return CURAG_OK;
  }
}

err_curag_t sx1262_radio_sleep(diagn_context_t *out_diag) {
  curag_diagnostic_context_clear(out_diag);
  if (s_radio.state == CURAG_RADIO_STATE_UNTOUCHED ||
      (s_radio.initialization_failure_cached && !s_radio.hardware_touched) ||
      s_radio.state == CURAG_RADIO_STATE_SLEEPING) {
    return CURAG_OK;
  }

  const curag_radio_state_t observed_state = s_radio.state;
  sx1262_radio_backend_error_t standby_detail;
  sx1262_radio_backend_error_t sleep_detail;
  sx1262_radio_backend_error_clear(&standby_detail);
  sx1262_radio_backend_error_clear(&sleep_detail);
  const bool standby_ok = sx1262_radio_backend_set_standby(&standby_detail) ==
                          SX1262_COMMAND_CONFIRMED;
  const bool sleep_ok = sx1262_radio_backend_set_sleep_cold(&sleep_detail) ==
                        SX1262_COMMAND_CONFIRMED;

  s_radio.rx_armed = false;
  if (standby_ok && sleep_ok) {
    s_radio.state = CURAG_RADIO_STATE_SLEEPING;
    return CURAG_OK;
  }
  s_radio.state = CURAG_RADIO_STATE_FAILED;
  const sx1262_radio_backend_error_t *const first_failure =
      standby_ok ? &sleep_detail : &standby_detail;
  return backend_failure(first_failure, CURAG_OP_SLEEP, observed_state,
                         out_diag);
}
