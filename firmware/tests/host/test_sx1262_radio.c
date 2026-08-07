#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "fake_sx1262_radio_backend.h"
#include "node_common.h"
#include "sx1262_radio.h"

#define TEST_ASSERT(expression)                                                \
  do {                                                                         \
    if (!(expression)) {                                                       \
      fprintf(stderr, "%s:%d: assertion failed: %s\n", __FILE__, __LINE__,     \
              #expression);                                                    \
      return false;                                                            \
    }                                                                          \
  } while (0)

#define TEST_ASSERT_EQ_U64(expected, actual)                                   \
  do {                                                                         \
    const uint64_t test_expected_ = (uint64_t)(expected);                      \
    const uint64_t test_actual_ = (uint64_t)(actual);                          \
    if (test_expected_ != test_actual_) {                                      \
      fprintf(stderr, "%s:%d: expected %llu, got %llu: %s\n", __FILE__,        \
              __LINE__, (unsigned long long)test_expected_,                    \
              (unsigned long long)test_actual_, #actual);                      \
      return false;                                                            \
    }                                                                          \
  } while (0)

typedef bool (*test_function_t)(void);

typedef struct {
  const char *name;
  test_function_t function;
} test_case_t;

static uint16_t read_u16_le(const uint8_t input[2]) {
  return (uint16_t)input[0] | (uint16_t)((uint16_t)input[1] << 8U);
}

static uint32_t read_u32_le(const uint8_t input[4]) {
  return (uint32_t)input[0] | ((uint32_t)input[1] << 8U) |
         ((uint32_t)input[2] << 16U) | ((uint32_t)input[3] << 24U);
}

static bool assert_radio_error(err_curag_t result, uint16_t code) {
  TEST_ASSERT_EQ_U64(CURAG_EDOM_RADIO, curag_error_domain(result));
  TEST_ASSERT_EQ_U64(code, curag_error_code(result));
  return true;
}

static bool diagnostic_is_clear(const diagn_context_t *diagnostic) {
  const diagn_context_t empty = {0};
  return memcmp(diagnostic, &empty, sizeof(empty)) == 0;
}

static bool assert_diagnostic(const diagn_context_t *diagnostic,
                              curag_operation_t operation,
                              curag_radio_state_t state, uint8_t command_opcode,
                              curag_radio_stage_t stage, uint8_t flags,
                              uint8_t backend_kind, int32_t backend_status,
                              uint8_t chip_status, uint16_t irq_status,
                              uint16_t device_errors) {
  TEST_ASSERT_EQ_U64(operation, diagnostic->operation);
  TEST_ASSERT_EQ_U64(CURAG_RADIO_CONTEXT_V1, diagnostic->context_schema);
  TEST_ASSERT_EQ_U64(CURAG_RADIO_CONTEXT_V1_LENGTH, diagnostic->context_length);
  TEST_ASSERT_EQ_U64(state, diagnostic->context[0]);
  TEST_ASSERT_EQ_U64(command_opcode, diagnostic->context[1]);
  TEST_ASSERT_EQ_U64(stage, diagnostic->context[2]);
  TEST_ASSERT_EQ_U64(flags, diagnostic->context[3]);
  TEST_ASSERT_EQ_U64(backend_kind, diagnostic->context[4]);
  TEST_ASSERT((int32_t)read_u32_le(&diagnostic->context[5]) == backend_status);
  TEST_ASSERT_EQ_U64(chip_status, diagnostic->context[9]);
  TEST_ASSERT_EQ_U64(irq_status, read_u16_le(&diagnostic->context[10]));
  TEST_ASSERT_EQ_U64(device_errors, read_u16_le(&diagnostic->context[12]));
  return true;
}

static fake_radio_irq_event_t tx_done_event(uint64_t at_us) {
  return (fake_radio_irq_event_t){
      .at_us = at_us,
      .irq_status = SX1262_RADIO_IRQ_TX_DONE,
  };
}

static fake_radio_irq_event_t rx_event(uint64_t at_us, const uint8_t *payload,
                                       uint16_t payload_length) {
  fake_radio_irq_event_t event = {
      .at_us = at_us,
      .irq_status = SX1262_RADIO_IRQ_RX_DONE,
      .payload_length = payload_length,
      .start_offset = UINT8_C(37),
      .rssi_dbm_x2 = -173,
      .snr_db_x4 = 23,
  };
  if (payload != NULL && payload_length <= SX1262_RADIO_MAX_PAYLOAD_SIZE) {
    memcpy(event.payload, payload, payload_length);
  }
  return event;
}

static bool transmit_successfully(const uint8_t *payload, size_t length,
                                  uint64_t irq_at_us,
                                  sx1262_radio_tx_result_t *out_result) {
  const fake_radio_irq_event_t event = tx_done_event(irq_at_us);
  fake_sx1262_radio_backend_add_irq(&event);
  diagn_context_t diagnostic;
  const err_curag_t result = sx1262_radio_transmit_uplink(
      payload, length, irq_at_us + UINT64_C(10000), out_result, &diagnostic);
  TEST_ASSERT_EQ_U64(CURAG_OK, result);
  TEST_ASSERT(diagnostic_is_clear(&diagnostic));
  TEST_ASSERT(out_result->tx_started);
  TEST_ASSERT(out_result->tx_done);
  TEST_ASSERT_EQ_U64(irq_at_us, out_result->tx_done_at_us);
  TEST_ASSERT(out_result->tx_done_at_us >= out_result->set_tx_at_us);
  return true;
}

static bool initialize_with_tx(void) {
  const uint8_t payload[] = {UINT8_C(0xa5)};
  sx1262_radio_tx_result_t tx;
  return transmit_successfully(payload, sizeof(payload), UINT64_C(2000), &tx);
}

static size_t trace_find_after(fake_radio_operation_t operation,
                               size_t start_index) {
  for (size_t index = start_index; index < g_fake_sx1262_radio.trace_length;
       ++index) {
    if (g_fake_sx1262_radio.trace[index] == operation) {
      return index;
    }
  }
  return SIZE_MAX;
}

static bool test_profile_and_lazy_initialization(void) {
  fake_sx1262_radio_backend_reset();
  const uint8_t first[] = {1U, 2U, 3U};
  sx1262_radio_tx_result_t tx;
  TEST_ASSERT(transmit_successfully(first, sizeof(first), UINT64_C(2000), &tx));

  TEST_ASSERT_EQ_U64(1U, g_fake_sx1262_radio.calls[FAKE_RADIO_OP_INITIALIZE]);
  TEST_ASSERT(g_fake_sx1262_radio.initialized_profile_valid);
  const sx1262_radio_profile_t *const profile =
      &g_fake_sx1262_radio.initialized_profile;
  TEST_ASSERT_EQ_U64(868100000U, profile->frequency_hz);
  TEST_ASSERT_EQ_U64(125000U, profile->bandwidth_hz);
  TEST_ASSERT_EQ_U64(8U, profile->preamble_symbols);
  TEST_ASSERT_EQ_U64(40U, profile->ramp_us);
  TEST_ASSERT_EQ_U64(5U, profile->tcxo_startup_ms);
  TEST_ASSERT(profile->radiated_tx_power_dbm == 14);
  TEST_ASSERT(profile->tx_params_power_dbm == 22);
  TEST_ASSERT_EQ_U64(7U, profile->spreading_factor);
  TEST_ASSERT_EQ_U64(5U, profile->coding_rate_denominator);
  TEST_ASSERT_EQ_U64(0x12U, profile->sync_word);
  TEST_ASSERT_EQ_U64(0x02U, profile->pa_duty_cycle);
  TEST_ASSERT_EQ_U64(0x02U, profile->pa_hp_max);
  TEST_ASSERT_EQ_U64(0x00U, profile->pa_device_sel);
  TEST_ASSERT_EQ_U64(0x01U, profile->pa_lut);
  TEST_ASSERT_EQ_U64(0x01U, profile->tcxo_voltage);
  TEST_ASSERT(!profile->low_data_rate_optimize);
  TEST_ASSERT(profile->explicit_header);
  TEST_ASSERT(profile->payload_crc);
  TEST_ASSERT(profile->boosted_rx);
  TEST_ASSERT(profile->dcdc_regulator);
  TEST_ASSERT(profile->dio2_rf_switch);
  TEST_ASSERT(profile->dio3_tcxo);
  TEST_ASSERT(!g_fake_sx1262_radio.packet_param_inverted[0]);

  const uint8_t second[] = {4U, 5U};
  TEST_ASSERT(
      transmit_successfully(second, sizeof(second), UINT64_C(3000), &tx));
  TEST_ASSERT_EQ_U64(1U, g_fake_sx1262_radio.calls[FAKE_RADIO_OP_INITIALIZE]);
  TEST_ASSERT_EQ_U64(2U, g_fake_sx1262_radio.calls[FAKE_RADIO_OP_START_TX]);
  TEST_ASSERT_EQ_U64(sizeof(second),
                     g_fake_sx1262_radio.transmitted_payload_length);
  TEST_ASSERT(memcmp(second, g_fake_sx1262_radio.transmitted_payload,
                     sizeof(second)) == 0);
  return true;
}

static bool test_invalid_arguments_and_initial_deadline(void) {
  fake_sx1262_radio_backend_reset();
  const uint8_t payload = 1U;
  sx1262_radio_tx_result_t tx;
  diagn_context_t diagnostic;

  TEST_ASSERT(assert_radio_error(
      sx1262_radio_transmit_uplink(NULL, 1U, 2000U, &tx, &diagnostic),
      CURAG_ERADIO_EINVALID_ARGUMENT));
  TEST_ASSERT(assert_diagnostic(
      &diagnostic, CURAG_OP_VALIDATE, CURAG_RADIO_STATE_UNTOUCHED, 0U,
      CURAG_RADIO_STAGE_VALIDATE_INPUT, 0U, CURAG_RADIO_BACKEND_STATUS_NONE, 0,
      0U, 0U, 0U));
  TEST_ASSERT(assert_radio_error(
      sx1262_radio_transmit_uplink(&payload, 0U, 2000U, &tx, NULL),
      CURAG_ERADIO_EINVALID_ARGUMENT));
  TEST_ASSERT(assert_radio_error(
      sx1262_radio_transmit_uplink(&payload, 256U, 2000U, &tx, NULL),
      CURAG_ERADIO_EINVALID_ARGUMENT));
  TEST_ASSERT(assert_radio_error(
      sx1262_radio_transmit_uplink(&payload, 1U, 2000U, NULL, NULL),
      CURAG_ERADIO_EINVALID_ARGUMENT));
  TEST_ASSERT(assert_radio_error(
      sx1262_radio_transmit_uplink(&payload, 1U, 1000U, &tx, &diagnostic),
      CURAG_ERADIO_EDEADLINE));
  TEST_ASSERT_EQ_U64(0U, g_fake_sx1262_radio.calls[FAKE_RADIO_OP_INITIALIZE]);

  sx1262_radio_rx_result_t rx;
  TEST_ASSERT(assert_radio_error(
      sx1262_radio_receive_downlink_until(2000U, NULL, &diagnostic),
      CURAG_ERADIO_EINVALID_ARGUMENT));
  TEST_ASSERT(assert_radio_error(
      sx1262_radio_receive_downlink_until(2000U, &rx, &diagnostic),
      CURAG_ERADIO_EINVALID_STATE));
  return true;
}

static bool test_initialization_failure_is_cached(void) {
  fake_sx1262_radio_backend_reset();
  const sx1262_radio_backend_error_t failure = {
      .error_code = CURAG_ERADIO_EBUSY_TIMEOUT,
      .command_opcode = UINT8_C(0x89),
      .stage = CURAG_RADIO_STAGE_WAIT_BUSY,
      .flags = 0U,
      .backend_status_kind = CURAG_RADIO_BACKEND_STATUS_NONE,
  };
  fake_sx1262_radio_backend_fail(FAKE_RADIO_OP_INITIALIZE, 1U, &failure);
  const uint8_t payload = 7U;
  sx1262_radio_tx_result_t tx;
  diagn_context_t first;
  diagn_context_t second;
  const err_curag_t first_result =
      sx1262_radio_transmit_uplink(&payload, 1U, 5000U, &tx, &first);
  const err_curag_t second_result =
      sx1262_radio_transmit_uplink(&payload, 1U, 5000U, &tx, &second);
  TEST_ASSERT(assert_radio_error(first_result, CURAG_ERADIO_EBUSY_TIMEOUT));
  TEST_ASSERT_EQ_U64(first_result, second_result);
  TEST_ASSERT(memcmp(&first, &second, sizeof(first)) == 0);
  TEST_ASSERT(assert_diagnostic(
      &first, CURAG_OP_INITIALIZE, CURAG_RADIO_STATE_UNTOUCHED, 0x89U,
      CURAG_RADIO_STAGE_WAIT_BUSY, 0U, CURAG_RADIO_BACKEND_STATUS_NONE, 0, 0U,
      0U, 0U));
  TEST_ASSERT_EQ_U64(1U, g_fake_sx1262_radio.calls[FAKE_RADIO_OP_INITIALIZE]);
  TEST_ASSERT_EQ_U64(CURAG_OK, sx1262_radio_sleep(&first));
  TEST_ASSERT_EQ_U64(0U, g_fake_sx1262_radio.calls[FAKE_RADIO_OP_STANDBY]);
  return true;
}

static bool test_payload_boundaries_and_watchdog(void) {
  fake_sx1262_radio_backend_reset();
  uint8_t payload[SX1262_RADIO_MAX_PAYLOAD_SIZE];
  for (size_t index = 0U; index < sizeof(payload); ++index) {
    payload[index] = (uint8_t)index;
  }
  const fake_radio_irq_event_t event = tx_done_event(UINT64_C(99000));
  fake_sx1262_radio_backend_add_irq(&event);
  sx1262_radio_tx_result_t tx;
  diagn_context_t diagnostic;
  TEST_ASSERT_EQ_U64(CURAG_OK, sx1262_radio_transmit_uplink(
                                   payload, sizeof(payload), UINT64_C(101000),
                                   &tx, &diagnostic));
  TEST_ASSERT_EQ_U64(255U, g_fake_sx1262_radio.transmitted_payload_length);
  TEST_ASSERT(memcmp(payload, g_fake_sx1262_radio.transmitted_payload,
                     sizeof(payload)) == 0);
  TEST_ASSERT_EQ_U64(6080U, g_fake_sx1262_radio.last_tx_watchdog_steps);
  TEST_ASSERT_EQ_U64(1000U, tx.set_tx_at_us);

  const fake_radio_irq_event_t second = tx_done_event(UINT64_C(100000));
  fake_sx1262_radio_backend_add_irq(&second);
  TEST_ASSERT_EQ_U64(CURAG_OK, sx1262_radio_transmit_uplink(
                                   payload, 1U, UINT64_MAX, &tx, NULL));
  TEST_ASSERT_EQ_U64(0x00ffffffU, g_fake_sx1262_radio.last_tx_watchdog_steps);
  return true;
}

static bool test_deadline_reached_before_set_tx(void) {
  fake_sx1262_radio_backend_reset();
  g_fake_sx1262_radio.advance_us[FAKE_RADIO_OP_WRITE_PAYLOAD] = 5000U;
  const uint8_t payload = 3U;
  sx1262_radio_tx_result_t tx;
  diagn_context_t diagnostic;
  TEST_ASSERT(assert_radio_error(
      sx1262_radio_transmit_uplink(&payload, 1U, 5000U, &tx, &diagnostic),
      CURAG_ERADIO_EDEADLINE));
  TEST_ASSERT(!tx.tx_started);
  TEST_ASSERT_EQ_U64(0U, tx.set_tx_at_us);
  TEST_ASSERT_EQ_U64(0U, g_fake_sx1262_radio.calls[FAKE_RADIO_OP_START_TX]);
  return true;
}

static bool test_failure_before_set_tx_reports_no_attempt(void) {
  fake_sx1262_radio_backend_reset();
  const sx1262_radio_backend_error_t failure = {
      .error_code = CURAG_ERADIO_EIO,
      .command_opcode = UINT8_C(0x83),
      .stage = CURAG_RADIO_STAGE_WRITE_COMMAND,
      .flags = CURAG_RADIO_CONTEXT_HARDWARE_TOUCHED,
      .backend_status_kind = CURAG_RADIO_BACKEND_STATUS_ESP_ERR,
      .backend_status = -42,
  };
  fake_sx1262_radio_backend_fail(FAKE_RADIO_OP_START_TX, 1U, &failure);
  const uint8_t payload = 9U;
  sx1262_radio_tx_result_t tx;
  diagn_context_t diagnostic;
  TEST_ASSERT(assert_radio_error(
      sx1262_radio_transmit_uplink(&payload, 1U, 5000U, &tx, &diagnostic),
      CURAG_ERADIO_EIO));
  TEST_ASSERT(!tx.tx_started);
  TEST_ASSERT(!tx.tx_done);
  TEST_ASSERT_EQ_U64(0U, tx.set_tx_at_us);
  TEST_ASSERT(assert_diagnostic(
      &diagnostic, CURAG_OP_TRANSMIT, CURAG_RADIO_STATE_READY, 0x83U,
      CURAG_RADIO_STAGE_WRITE_COMMAND, CURAG_RADIO_CONTEXT_HARDWARE_TOUCHED,
      CURAG_RADIO_BACKEND_STATUS_ESP_ERR, -42, 0U, 0U, 0U));
  return true;
}

static bool test_uncertain_set_tx_preserves_attempt(void) {
  fake_sx1262_radio_backend_reset();
  const sx1262_radio_backend_error_t failure = {
      .error_code = CURAG_ERADIO_EIO,
      .command_opcode = UINT8_C(0xc0),
      .stage = CURAG_RADIO_STAGE_READ_COMMAND,
      .flags = CURAG_RADIO_CONTEXT_HARDWARE_TOUCHED,
      .backend_status_kind = CURAG_RADIO_BACKEND_STATUS_ESP_ERR,
      .backend_status = -23,
  };
  fake_sx1262_radio_backend_set_command_result(
      FAKE_RADIO_OP_START_TX, 1U, SX1262_COMMAND_UNCERTAIN, &failure);
  const uint8_t payload = 9U;
  sx1262_radio_tx_result_t tx;
  diagn_context_t diagnostic;
  TEST_ASSERT(assert_radio_error(
      sx1262_radio_transmit_uplink(&payload, 1U, 5000U, &tx, &diagnostic),
      CURAG_ERADIO_EIO));
  TEST_ASSERT(tx.tx_started);
  TEST_ASSERT(!tx.tx_done);
  TEST_ASSERT_EQ_U64(1000U, tx.set_tx_at_us);
  TEST_ASSERT_EQ_U64(0U, tx.tx_done_at_us);
  TEST_ASSERT_EQ_U64(0U, g_fake_sx1262_radio.calls[FAKE_RADIO_OP_WAIT_DIO1]);
  TEST_ASSERT(assert_diagnostic(
      &diagnostic, CURAG_OP_TRANSMIT, CURAG_RADIO_STATE_READY, 0xc0U,
      CURAG_RADIO_STAGE_READ_COMMAND, CURAG_RADIO_CONTEXT_HARDWARE_TOUCHED,
      CURAG_RADIO_BACKEND_STATUS_ESP_ERR, -23, 0U, 0U, 0U));
  return true;
}

static bool test_uncertain_command_before_set_tx_reports_no_attempt(void) {
  fake_sx1262_radio_backend_reset();
  const sx1262_radio_backend_error_t failure = {
      .error_code = CURAG_ERADIO_EBUSY_TIMEOUT,
      .command_opcode = UINT8_C(0x8c),
      .stage = CURAG_RADIO_STAGE_WAIT_BUSY,
      .flags = CURAG_RADIO_CONTEXT_HARDWARE_TOUCHED,
  };
  fake_sx1262_radio_backend_set_command_result(
      FAKE_RADIO_OP_PACKET_PARAMS, 1U, SX1262_COMMAND_UNCERTAIN, &failure);
  const uint8_t payload = 7U;
  sx1262_radio_tx_result_t tx;
  diagn_context_t diagnostic;
  TEST_ASSERT(assert_radio_error(
      sx1262_radio_transmit_uplink(&payload, 1U, 5000U, &tx, &diagnostic),
      CURAG_ERADIO_EBUSY_TIMEOUT));
  TEST_ASSERT(!tx.tx_started);
  TEST_ASSERT(!tx.tx_done);
  TEST_ASSERT_EQ_U64(0U, tx.set_tx_at_us);
  TEST_ASSERT_EQ_U64(0U, g_fake_sx1262_radio.calls[FAKE_RADIO_OP_START_TX]);
  return true;
}

static bool test_failure_after_set_tx_preserves_attempt(void) {
  fake_sx1262_radio_backend_reset();
  const sx1262_radio_backend_error_t failure = {
      .error_code = CURAG_ERADIO_EIO,
      .stage = CURAG_RADIO_STAGE_READ_IRQ,
      .flags = CURAG_RADIO_CONTEXT_HARDWARE_TOUCHED,
      .backend_status_kind = CURAG_RADIO_BACKEND_STATUS_ESP_ERR,
      .backend_status = -17,
  };
  fake_sx1262_radio_backend_fail(FAKE_RADIO_OP_GET_IRQ, 1U, &failure);
  const fake_radio_irq_event_t event = tx_done_event(UINT64_C(2000));
  fake_sx1262_radio_backend_add_irq(&event);
  const uint8_t payload = 9U;
  sx1262_radio_tx_result_t tx;
  diagn_context_t diagnostic;
  TEST_ASSERT(assert_radio_error(
      sx1262_radio_transmit_uplink(&payload, 1U, 5000U, &tx, &diagnostic),
      CURAG_ERADIO_EIO));
  TEST_ASSERT(tx.tx_started);
  TEST_ASSERT(!tx.tx_done);
  TEST_ASSERT_EQ_U64(1000U, tx.set_tx_at_us);
  TEST_ASSERT_EQ_U64(0U, tx.tx_done_at_us);
  return true;
}

static bool test_clear_failure_preserves_tx_done(void) {
  fake_sx1262_radio_backend_reset();
  const sx1262_radio_backend_error_t failure = {
      .error_code = CURAG_ERADIO_EIO,
      .command_opcode = UINT8_C(0x02),
      .stage = CURAG_RADIO_STAGE_CLEAR_IRQ,
      .flags = CURAG_RADIO_CONTEXT_HARDWARE_TOUCHED,
      .backend_status_kind = CURAG_RADIO_BACKEND_STATUS_ESP_ERR,
      .backend_status = -5,
  };
  fake_sx1262_radio_backend_fail(FAKE_RADIO_OP_CLEAR_IRQ, 2U, &failure);
  const fake_radio_irq_event_t event = tx_done_event(UINT64_C(2200));
  fake_sx1262_radio_backend_add_irq(&event);
  const uint8_t payload = 1U;
  sx1262_radio_tx_result_t tx;
  diagn_context_t diagnostic;
  TEST_ASSERT(assert_radio_error(
      sx1262_radio_transmit_uplink(&payload, 1U, 5000U, &tx, &diagnostic),
      CURAG_ERADIO_EIO));
  TEST_ASSERT(tx.tx_started);
  TEST_ASSERT(tx.tx_done);
  TEST_ASSERT_EQ_U64(2200U, tx.tx_done_at_us);
  return true;
}

static bool test_tx_timeout_and_unexpected_irq(void) {
  fake_sx1262_radio_backend_reset();
  const fake_radio_irq_event_t timeout = {
      .at_us = UINT64_C(4000),
      .irq_status = SX1262_RADIO_IRQ_TIMEOUT,
  };
  fake_sx1262_radio_backend_add_irq(&timeout);
  const uint8_t payload = 1U;
  sx1262_radio_tx_result_t tx;
  diagn_context_t diagnostic;
  TEST_ASSERT(assert_radio_error(
      sx1262_radio_transmit_uplink(&payload, 1U, 5000U, &tx, &diagnostic),
      CURAG_ERADIO_EDEADLINE));
  TEST_ASSERT(tx.tx_started);
  TEST_ASSERT(!tx.tx_done);
  TEST_ASSERT(assert_diagnostic(
      &diagnostic, CURAG_OP_TRANSMIT, CURAG_RADIO_STATE_READY, 0U,
      CURAG_RADIO_STAGE_READ_IRQ,
      CURAG_RADIO_CONTEXT_IRQ_STATUS_VALID |
          CURAG_RADIO_CONTEXT_HARDWARE_TOUCHED,
      CURAG_RADIO_BACKEND_STATUS_NONE, 0, 0U, SX1262_RADIO_IRQ_TIMEOUT, 0U));
  return true;
}

static bool test_combined_tx_done_and_timeout_preserves_progress(void) {
  fake_sx1262_radio_backend_reset();
  const fake_radio_irq_event_t done_and_timeout = {
      .at_us = UINT64_C(4000),
      .irq_status = SX1262_RADIO_IRQ_TX_DONE | SX1262_RADIO_IRQ_TIMEOUT,
  };
  fake_sx1262_radio_backend_add_irq(&done_and_timeout);
  const uint8_t payload = 1U;
  sx1262_radio_tx_result_t tx;
  diagn_context_t diagnostic;
  TEST_ASSERT(assert_radio_error(
      sx1262_radio_transmit_uplink(&payload, 1U, 5000U, &tx, &diagnostic),
      CURAG_ERADIO_EDEADLINE));
  TEST_ASSERT(tx.tx_started);
  TEST_ASSERT(tx.tx_done);
  TEST_ASSERT_EQ_U64(4000U, tx.tx_done_at_us);
  return true;
}

static bool test_unexpected_irq_reports_device_error(void) {
  fake_sx1262_radio_backend_reset();
  g_fake_sx1262_radio.device_errors = UINT16_C(0x1234);
  const fake_radio_irq_event_t event = {
      .at_us = UINT64_C(3000),
      .irq_status = SX1262_RADIO_IRQ_RX_DONE,
  };
  fake_sx1262_radio_backend_add_irq(&event);
  const uint8_t payload = 1U;
  sx1262_radio_tx_result_t tx;
  diagn_context_t diagnostic;
  TEST_ASSERT(assert_radio_error(
      sx1262_radio_transmit_uplink(&payload, 1U, 5000U, &tx, &diagnostic),
      CURAG_ERADIO_EDEVICE_ERROR));
  TEST_ASSERT(assert_diagnostic(&diagnostic, CURAG_OP_TRANSMIT,
                                CURAG_RADIO_STATE_READY, 0U,
                                CURAG_RADIO_STAGE_READ_IRQ,
                                CURAG_RADIO_CONTEXT_IRQ_STATUS_VALID |
                                    CURAG_RADIO_CONTEXT_DEVICE_ERRORS_VALID |
                                    CURAG_RADIO_CONTEXT_HARDWARE_TOUCHED,
                                CURAG_RADIO_BACKEND_STATUS_NONE, 0, 0U,
                                SX1262_RADIO_IRQ_RX_DONE, 0x1234U));
  return true;
}

static bool test_receive_exact_packet_and_iq_transition(void) {
  fake_sx1262_radio_backend_reset();
  TEST_ASSERT(initialize_with_tx());
  const uint8_t downlink[] = {0x90U, 0x00U, 0xffU, 0x42U};
  const fake_radio_irq_event_t event =
      rx_event(UINT64_C(3000), downlink, sizeof(downlink));
  fake_sx1262_radio_backend_add_irq(&event);
  sx1262_radio_rx_result_t rx;
  diagn_context_t diagnostic;
  TEST_ASSERT_EQ_U64(
      CURAG_OK, sx1262_radio_receive_downlink_until(5000U, &rx, &diagnostic));
  TEST_ASSERT_EQ_U64(SX1262_RADIO_RX_PACKET, rx.outcome);
  TEST_ASSERT_EQ_U64(3000U, rx.rx_done_at_us);
  TEST_ASSERT(rx.rssi_dbm_x2 == -173);
  TEST_ASSERT(rx.snr_db_x4 == 23);
  TEST_ASSERT_EQ_U64(sizeof(downlink), rx.payload_length);
  TEST_ASSERT(memcmp(downlink, rx.payload, sizeof(downlink)) == 0);
  TEST_ASSERT_EQ_U64(2U, g_fake_sx1262_radio.packet_param_count);
  TEST_ASSERT(!g_fake_sx1262_radio.packet_param_inverted[0]);
  TEST_ASSERT(g_fake_sx1262_radio.packet_param_inverted[1]);
  TEST_ASSERT_EQ_U64(255U, g_fake_sx1262_radio.packet_param_lengths[1]);
  TEST_ASSERT_EQ_U64(1U,
                     g_fake_sx1262_radio.calls[FAKE_RADIO_OP_HANDLE_RX_DONE]);

  const uint8_t uplink = 0x77U;
  sx1262_radio_tx_result_t tx;
  TEST_ASSERT(transmit_successfully(&uplink, 1U, UINT64_C(4000), &tx));
  TEST_ASSERT_EQ_U64(3U, g_fake_sx1262_radio.packet_param_count);
  TEST_ASSERT(!g_fake_sx1262_radio.packet_param_inverted[2]);
  TEST_ASSERT_EQ_U64(2U, g_fake_sx1262_radio.calls[FAKE_RADIO_OP_STANDBY]);
  return true;
}

static bool test_receive_pending_irq_at_deadline_wins(void) {
  fake_sx1262_radio_backend_reset();
  TEST_ASSERT(initialize_with_tx());

  sx1262_radio_rx_result_t rx;
  TEST_ASSERT_EQ_U64(CURAG_OK,
                     sx1262_radio_receive_downlink_until(3000U, &rx, NULL));
  TEST_ASSERT_EQ_U64(SX1262_RADIO_RX_DEADLINE, rx.outcome);

  /* Model DIO1 arriving at the boundary as the timed wait returns. */
  const uint8_t payload[] = {0x31U, 0x32U};
  const fake_radio_irq_event_t pending =
      rx_event(UINT64_C(3000), payload, sizeof(payload));
  fake_sx1262_radio_backend_add_irq(&pending);

  TEST_ASSERT_EQ_U64(CURAG_OK,
                     sx1262_radio_receive_downlink_until(3000U, &rx, NULL));
  TEST_ASSERT_EQ_U64(SX1262_RADIO_RX_PACKET, rx.outcome);
  TEST_ASSERT_EQ_U64(3000U, rx.rx_done_at_us);
  TEST_ASSERT_EQ_U64(sizeof(payload), rx.payload_length);
  TEST_ASSERT(memcmp(payload, rx.payload, sizeof(payload)) == 0);
  return true;
}

static bool test_receive_snapshots_before_clear_and_rearms(void) {
  fake_sx1262_radio_backend_reset();
  TEST_ASSERT(initialize_with_tx());
  const uint8_t payload[] = {0x41U, 0x42U, 0x43U};
  const fake_radio_irq_event_t packet =
      rx_event(UINT64_C(3000), payload, sizeof(payload));
  fake_sx1262_radio_backend_add_irq(&packet);

  const size_t trace_start = g_fake_sx1262_radio.trace_length;
  sx1262_radio_rx_result_t rx;
  TEST_ASSERT_EQ_U64(CURAG_OK,
                     sx1262_radio_receive_downlink_until(5000U, &rx, NULL));
  TEST_ASSERT_EQ_U64(SX1262_RADIO_RX_PACKET, rx.outcome);

  const size_t get_irq = trace_find_after(FAKE_RADIO_OP_GET_IRQ, trace_start);
  TEST_ASSERT(get_irq != SIZE_MAX);
  const size_t read_buffer =
      trace_find_after(FAKE_RADIO_OP_READ_BUFFER, get_irq + 1U);
  TEST_ASSERT(read_buffer != SIZE_MAX);
  const size_t packet_status =
      trace_find_after(FAKE_RADIO_OP_GET_PACKET_STATUS, read_buffer + 1U);
  TEST_ASSERT(packet_status != SIZE_MAX);
  const size_t clear_irq =
      trace_find_after(FAKE_RADIO_OP_CLEAR_IRQ, get_irq + 1U);
  TEST_ASSERT(clear_irq != SIZE_MAX);
  const size_t rearm = trace_find_after(FAKE_RADIO_OP_START_RX, clear_irq + 1U);
  TEST_ASSERT(rearm != SIZE_MAX);
  TEST_ASSERT(get_irq < read_buffer);
  TEST_ASSERT(read_buffer < packet_status);
  TEST_ASSERT(packet_status < clear_irq);
  TEST_ASSERT(clear_irq < rearm);
  TEST_ASSERT_EQ_U64(2U, g_fake_sx1262_radio.calls[FAKE_RADIO_OP_START_RX]);
  return true;
}

static bool test_receive_deadline_keeps_rx_armed(void) {
  fake_sx1262_radio_backend_reset();
  TEST_ASSERT(initialize_with_tx());
  sx1262_radio_rx_result_t rx;
  diagn_context_t diagnostic;
  memset(&rx, 0xa5, sizeof(rx));
  TEST_ASSERT_EQ_U64(
      CURAG_OK, sx1262_radio_receive_downlink_until(3000U, &rx, &diagnostic));
  TEST_ASSERT_EQ_U64(SX1262_RADIO_RX_DEADLINE, rx.outcome);
  TEST_ASSERT_EQ_U64(0U, rx.rx_done_at_us);
  TEST_ASSERT_EQ_U64(0U, rx.payload_length);
  TEST_ASSERT_EQ_U64(1U, g_fake_sx1262_radio.calls[FAKE_RADIO_OP_START_RX]);

  TEST_ASSERT_EQ_U64(
      CURAG_OK, sx1262_radio_receive_downlink_until(4000U, &rx, &diagnostic));
  TEST_ASSERT_EQ_U64(SX1262_RADIO_RX_DEADLINE, rx.outcome);
  TEST_ASSERT_EQ_U64(1U, g_fake_sx1262_radio.calls[FAKE_RADIO_OP_START_RX]);
  TEST_ASSERT_EQ_U64(3U, g_fake_sx1262_radio.calls[FAKE_RADIO_OP_WAIT_DIO1]);
  return true;
}

static bool test_receive_discards_phy_errors(void) {
  fake_sx1262_radio_backend_reset();
  TEST_ASSERT(initialize_with_tx());
  const fake_radio_irq_event_t header_error = {
      .at_us = UINT64_C(2500),
      .irq_status = SX1262_RADIO_IRQ_HEADER_ERROR,
  };
  const fake_radio_irq_event_t crc_error = {
      .at_us = UINT64_C(3000),
      .irq_status = SX1262_RADIO_IRQ_CRC_ERROR,
  };
  const uint8_t payload[] = {8U, 9U};
  const fake_radio_irq_event_t packet =
      rx_event(UINT64_C(3500), payload, sizeof(payload));
  fake_sx1262_radio_backend_add_irq(&header_error);
  fake_sx1262_radio_backend_add_irq(&crc_error);
  fake_sx1262_radio_backend_add_irq(&packet);
  sx1262_radio_rx_result_t rx;
  TEST_ASSERT_EQ_U64(CURAG_OK,
                     sx1262_radio_receive_downlink_until(5000U, &rx, NULL));
  TEST_ASSERT_EQ_U64(SX1262_RADIO_RX_PACKET, rx.outcome);
  TEST_ASSERT(memcmp(payload, rx.payload, sizeof(payload)) == 0);
  TEST_ASSERT_EQ_U64(4U, g_fake_sx1262_radio.calls[FAKE_RADIO_OP_WAIT_DIO1]);
  TEST_ASSERT_EQ_U64(4U, g_fake_sx1262_radio.calls[FAKE_RADIO_OP_GET_IRQ]);
  TEST_ASSERT_EQ_U64(1U, g_fake_sx1262_radio.calls[FAKE_RADIO_OP_READ_BUFFER]);
  return true;
}

static bool test_oversized_receive_is_bounded(void) {
  fake_sx1262_radio_backend_reset();
  TEST_ASSERT(initialize_with_tx());
  const fake_radio_irq_event_t event =
      rx_event(UINT64_C(3000), NULL, UINT16_C(256));
  fake_sx1262_radio_backend_add_irq(&event);
  sx1262_radio_rx_result_t rx;
  memset(&rx, 0xa5, sizeof(rx));
  diagn_context_t diagnostic;
  TEST_ASSERT(assert_radio_error(
      sx1262_radio_receive_downlink_until(5000U, &rx, &diagnostic),
      CURAG_ERADIO_EIO));
  TEST_ASSERT_EQ_U64(0U, g_fake_sx1262_radio.calls[FAKE_RADIO_OP_READ_BUFFER]);
  TEST_ASSERT_EQ_U64(SX1262_RADIO_RX_INVALID, rx.outcome);
  TEST_ASSERT_EQ_U64(0U, rx.payload_length);
  TEST_ASSERT(assert_diagnostic(
      &diagnostic, CURAG_OP_RECEIVE, CURAG_RADIO_STATE_READY, 0U,
      CURAG_RADIO_STAGE_READ_BUFFER, CURAG_RADIO_CONTEXT_HARDWARE_TOUCHED,
      CURAG_RADIO_BACKEND_STATUS_NONE, 0, 0U, 0U, 0U));
  return true;
}

static bool test_late_packet_is_discarded(void) {
  fake_sx1262_radio_backend_reset();
  TEST_ASSERT(initialize_with_tx());
  const uint8_t payload[] = {1U, 2U};
  fake_radio_irq_event_t late =
      rx_event(UINT64_C(6000), payload, sizeof(payload));
  late.surface_even_after_deadline = true;
  fake_sx1262_radio_backend_add_irq(&late);
  sx1262_radio_rx_result_t rx;
  TEST_ASSERT_EQ_U64(CURAG_OK,
                     sx1262_radio_receive_downlink_until(5000U, &rx, NULL));
  TEST_ASSERT_EQ_U64(SX1262_RADIO_RX_DEADLINE, rx.outcome);
  TEST_ASSERT_EQ_U64(0U, rx.payload_length);

  const uint8_t uplink = 4U;
  sx1262_radio_tx_result_t tx;
  TEST_ASSERT(transmit_successfully(&uplink, 1U, UINT64_C(7000), &tx));
  return true;
}

static bool test_same_absolute_deadline_is_not_extended(void) {
  fake_sx1262_radio_backend_reset();
  TEST_ASSERT(initialize_with_tx());
  const uint8_t invalid_application_packet[] = {0xdeU, 0xadU};
  const fake_radio_irq_event_t packet =
      rx_event(UINT64_C(3000), invalid_application_packet,
               sizeof(invalid_application_packet));
  fake_sx1262_radio_backend_add_irq(&packet);
  sx1262_radio_rx_result_t rx;
  TEST_ASSERT_EQ_U64(CURAG_OK,
                     sx1262_radio_receive_downlink_until(5000U, &rx, NULL));
  TEST_ASSERT_EQ_U64(SX1262_RADIO_RX_PACKET, rx.outcome);
  TEST_ASSERT_EQ_U64(CURAG_OK,
                     sx1262_radio_receive_downlink_until(5000U, &rx, NULL));
  TEST_ASSERT_EQ_U64(SX1262_RADIO_RX_DEADLINE, rx.outcome);
  TEST_ASSERT_EQ_U64(5000U, g_fake_sx1262_radio.now_us);
  TEST_ASSERT_EQ_U64(2U, g_fake_sx1262_radio.calls[FAKE_RADIO_OP_START_RX]);
  return true;
}

static bool test_receive_local_error_is_distinct(void) {
  fake_sx1262_radio_backend_reset();
  TEST_ASSERT(initialize_with_tx());
  const sx1262_radio_backend_error_t failure = {
      .error_code = CURAG_ERADIO_EIO,
      .command_opcode = UINT8_C(0x14),
      .stage = CURAG_RADIO_STAGE_READ_PACKET_STATUS,
      .flags = CURAG_RADIO_CONTEXT_HARDWARE_TOUCHED,
      .backend_status_kind = CURAG_RADIO_BACKEND_STATUS_ESP_ERR,
      .backend_status = -12,
  };
  fake_sx1262_radio_backend_fail(FAKE_RADIO_OP_GET_PACKET_STATUS, 1U, &failure);
  const uint8_t payload[] = {1U, 2U, 3U};
  const fake_radio_irq_event_t packet =
      rx_event(UINT64_C(3000), payload, sizeof(payload));
  fake_sx1262_radio_backend_add_irq(&packet);
  sx1262_radio_rx_result_t rx;
  memset(&rx, 0xa5, sizeof(rx));
  diagn_context_t diagnostic;
  TEST_ASSERT(assert_radio_error(
      sx1262_radio_receive_downlink_until(5000U, &rx, &diagnostic),
      CURAG_ERADIO_EIO));
  TEST_ASSERT_EQ_U64(SX1262_RADIO_RX_INVALID, rx.outcome);
  TEST_ASSERT_EQ_U64(0U, rx.payload_length);
  TEST_ASSERT(
      assert_diagnostic(&diagnostic, CURAG_OP_RECEIVE, CURAG_RADIO_STATE_READY,
                        0x14U, CURAG_RADIO_STAGE_READ_PACKET_STATUS,
                        CURAG_RADIO_CONTEXT_HARDWARE_TOUCHED,
                        CURAG_RADIO_BACKEND_STATUS_ESP_ERR, -12, 0U, 0U, 0U));
  return true;
}

static bool test_unexpected_receive_irq_fails(void) {
  fake_sx1262_radio_backend_reset();
  TEST_ASSERT(initialize_with_tx());
  const fake_radio_irq_event_t event = {
      .at_us = UINT64_C(3000),
      .irq_status = SX1262_RADIO_IRQ_TX_DONE,
  };
  fake_sx1262_radio_backend_add_irq(&event);
  sx1262_radio_rx_result_t rx;
  diagn_context_t diagnostic;
  TEST_ASSERT(assert_radio_error(
      sx1262_radio_receive_downlink_until(5000U, &rx, &diagnostic),
      CURAG_ERADIO_EUNEXPECTED_IRQ));
  TEST_ASSERT(assert_diagnostic(
      &diagnostic, CURAG_OP_RECEIVE, CURAG_RADIO_STATE_READY, 0U,
      CURAG_RADIO_STAGE_READ_IRQ,
      CURAG_RADIO_CONTEXT_IRQ_STATUS_VALID |
          CURAG_RADIO_CONTEXT_HARDWARE_TOUCHED,
      CURAG_RADIO_BACKEND_STATUS_NONE, 0, 0U, SX1262_RADIO_IRQ_TX_DONE, 0U));
  return true;
}

static bool test_sleep_lifecycle(void) {
  fake_sx1262_radio_backend_reset();
  diagn_context_t diagnostic;
  TEST_ASSERT_EQ_U64(CURAG_OK, sx1262_radio_sleep(&diagnostic));
  TEST_ASSERT_EQ_U64(0U, g_fake_sx1262_radio.calls[FAKE_RADIO_OP_INITIALIZE]);
  TEST_ASSERT_EQ_U64(0U, g_fake_sx1262_radio.calls[FAKE_RADIO_OP_STANDBY]);

  TEST_ASSERT(initialize_with_tx());
  TEST_ASSERT_EQ_U64(CURAG_OK, sx1262_radio_sleep(&diagnostic));
  TEST_ASSERT_EQ_U64(CURAG_OK, sx1262_radio_sleep(&diagnostic));
  TEST_ASSERT_EQ_U64(1U, g_fake_sx1262_radio.calls[FAKE_RADIO_OP_STANDBY]);
  TEST_ASSERT_EQ_U64(1U, g_fake_sx1262_radio.calls[FAKE_RADIO_OP_SLEEP_COLD]);

  const uint8_t payload = 1U;
  sx1262_radio_tx_result_t tx;
  TEST_ASSERT(assert_radio_error(
      sx1262_radio_transmit_uplink(&payload, 1U, 5000U, &tx, &diagnostic),
      CURAG_ERADIO_EINVALID_STATE));
  TEST_ASSERT_EQ_U64(1U, g_fake_sx1262_radio.calls[FAKE_RADIO_OP_START_TX]);
  return true;
}

static bool test_sleep_failure_retries_and_preserves_first_error(void) {
  fake_sx1262_radio_backend_reset();
  TEST_ASSERT(initialize_with_tx());
  const sx1262_radio_backend_error_t standby_failure = {
      .error_code = CURAG_ERADIO_EBUSY_TIMEOUT,
      .command_opcode = UINT8_C(0x80),
      .stage = CURAG_RADIO_STAGE_WAIT_BUSY,
      .flags = CURAG_RADIO_CONTEXT_HARDWARE_TOUCHED,
  };
  fake_sx1262_radio_backend_fail(FAKE_RADIO_OP_STANDBY, 1U, &standby_failure);
  diagn_context_t diagnostic;
  TEST_ASSERT(assert_radio_error(sx1262_radio_sleep(&diagnostic),
                                 CURAG_ERADIO_EBUSY_TIMEOUT));
  TEST_ASSERT_EQ_U64(1U, g_fake_sx1262_radio.calls[FAKE_RADIO_OP_SLEEP_COLD]);
  TEST_ASSERT(assert_diagnostic(
      &diagnostic, CURAG_OP_SLEEP, CURAG_RADIO_STATE_READY, 0x80U,
      CURAG_RADIO_STAGE_WAIT_BUSY, CURAG_RADIO_CONTEXT_HARDWARE_TOUCHED,
      CURAG_RADIO_BACKEND_STATUS_NONE, 0, 0U, 0U, 0U));

  TEST_ASSERT_EQ_U64(CURAG_OK, sx1262_radio_sleep(&diagnostic));
  TEST_ASSERT_EQ_U64(2U, g_fake_sx1262_radio.calls[FAKE_RADIO_OP_STANDBY]);
  TEST_ASSERT_EQ_U64(2U, g_fake_sx1262_radio.calls[FAKE_RADIO_OP_SLEEP_COLD]);
  return true;
}

static const test_case_t k_tests[] = {
    {"profile_and_lazy_initialization", test_profile_and_lazy_initialization},
    {"invalid_arguments_and_initial_deadline",
     test_invalid_arguments_and_initial_deadline},
    {"initialization_failure_is_cached", test_initialization_failure_is_cached},
    {"payload_boundaries_and_watchdog", test_payload_boundaries_and_watchdog},
    {"deadline_reached_before_set_tx", test_deadline_reached_before_set_tx},
    {"failure_before_set_tx_reports_no_attempt",
     test_failure_before_set_tx_reports_no_attempt},
    {"uncertain_set_tx_preserves_attempt",
     test_uncertain_set_tx_preserves_attempt},
    {"uncertain_command_before_set_tx_reports_no_attempt",
     test_uncertain_command_before_set_tx_reports_no_attempt},
    {"failure_after_set_tx_preserves_attempt",
     test_failure_after_set_tx_preserves_attempt},
    {"clear_failure_preserves_tx_done", test_clear_failure_preserves_tx_done},
    {"tx_timeout_and_unexpected_irq", test_tx_timeout_and_unexpected_irq},
    {"combined_tx_done_and_timeout_preserves_progress",
     test_combined_tx_done_and_timeout_preserves_progress},
    {"unexpected_irq_reports_device_error",
     test_unexpected_irq_reports_device_error},
    {"receive_exact_packet_and_iq_transition",
     test_receive_exact_packet_and_iq_transition},
    {"receive_pending_irq_at_deadline_wins",
     test_receive_pending_irq_at_deadline_wins},
    {"receive_snapshots_before_clear_and_rearms",
     test_receive_snapshots_before_clear_and_rearms},
    {"receive_deadline_keeps_rx_armed", test_receive_deadline_keeps_rx_armed},
    {"receive_discards_phy_errors", test_receive_discards_phy_errors},
    {"oversized_receive_is_bounded", test_oversized_receive_is_bounded},
    {"late_packet_is_discarded", test_late_packet_is_discarded},
    {"same_absolute_deadline_is_not_extended",
     test_same_absolute_deadline_is_not_extended},
    {"receive_local_error_is_distinct", test_receive_local_error_is_distinct},
    {"unexpected_receive_irq_fails", test_unexpected_receive_irq_fails},
    {"sleep_lifecycle", test_sleep_lifecycle},
    {"sleep_failure_retries_and_preserves_first_error",
     test_sleep_failure_retries_and_preserves_first_error},
};

int main(int argc, char **argv) {
  if (argc != 2) {
    fprintf(stderr, "usage: %s TEST_NAME\n", argv[0]);
    return 2;
  }
  for (size_t index = 0U; index < sizeof(k_tests) / sizeof(k_tests[0]);
       ++index) {
    if (strcmp(argv[1], k_tests[index].name) == 0) {
      if (!k_tests[index].function()) {
        return 1;
      }
      printf("PASS sx1262_radio/%s\n", k_tests[index].name);
      return 0;
    }
  }
  fprintf(stderr, "unknown test: %s\n", argv[1]);
  return 2;
}
