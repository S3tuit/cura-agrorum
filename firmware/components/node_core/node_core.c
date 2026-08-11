#include "node_core.h"

#include <limits.h>
#include <stdatomic.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "node_persistence.h"
#include "node_sensors.h"
#include "protocol_v2_lora_crypto.h"
#include "sx1262_radio.h"

typedef struct {
  uint32_t current_tx_attempts;
  uint64_t current_delivery_us;
  uint32_t cycle_tx_attempts;
  uint32_t accepted_readings;
  uint64_t charged_airtime_us;
  bool current_accepted;
} node_wake_metrics_t;

typedef struct {
  const node_platform_ports_t *platform;
  const node_identity_t *identity;
  uint64_t application_start_us;
  uint64_t radio_deadline_us;
  uint32_t cycle_sample_id;
  bool cycle_sample_id_valid;
  node_wake_metrics_t metrics;
} node_cycle_context_t;

typedef enum {
  ACK_VALIDATION_VALID = 0,
  ACK_VALIDATION_INVALID,
  ACK_VALIDATION_LOCAL_ERROR,
} ack_validation_kind_t;

typedef struct {
  ack_validation_kind_t kind;
  node_delivery_final_result_t result;
  err_curag_t error;
  curag_operation_t operation;
} ack_validation_result_t;

typedef struct {
  node_delivery_final_result_t result;
  uint32_t attempt_count;
} node_delivery_result_t;

static bool cycle_metrics_are_valid(const node_cycle_metrics_t *metrics) {
  if (metrics == NULL ||
      metrics->current_tx_attempts > metrics->cycle_tx_attempts ||
      metrics->accepted_readings > metrics->cycle_tx_attempts ||
      metrics->current_accepted != (metrics->accepted_readings != 0U) ||
      (!metrics->current_accepted && metrics->current_delivery_ms != 0U)) {
    return false;
  }
  return true;
}

void node_rtc_record_take(node_rtc_record_t *retained,
                          node_rtc_record_t *out_copy) {
  if (out_copy != NULL) {
    memset(out_copy, 0, sizeof(*out_copy));
  }
  if (retained == NULL || out_copy == NULL) {
    return;
  }
  *out_copy = *retained;
  retained->commit_marker = 0U;
  atomic_signal_fence(memory_order_seq_cst);
  atomic_thread_fence(memory_order_seq_cst);
}

bool node_rtc_record_validate_previous(const node_rtc_record_t *record,
                                       uint8_t reset_reason,
                                       uint32_t current_sample_id,
                                       node_cycle_metrics_t *out_metrics) {
  if (out_metrics != NULL) {
    memset(out_metrics, 0, sizeof(*out_metrics));
  }
  if (record == NULL || out_metrics == NULL ||
      record->commit_marker != NODE_RTC_COMMITTED_V1 ||
      reset_reason != CURA_LORA_V2_RESET_REASON_ESP_RST_DEEPSLEEP ||
      current_sample_id == 0U ||
      record->completed_sample_id != current_sample_id - 1U ||
      !cycle_metrics_are_valid(&record->metrics)) {
    return false;
  }
  *out_metrics = record->metrics;
  return true;
}

bool node_rtc_record_commit(node_rtc_record_t *record,
                            uint32_t completed_sample_id,
                            const node_cycle_metrics_t *metrics) {
  if (record == NULL) {
    return false;
  }
  record->commit_marker = 0U;
  atomic_signal_fence(memory_order_seq_cst);
  atomic_thread_fence(memory_order_seq_cst);
  if (!cycle_metrics_are_valid(metrics)) {
    return false;
  }
  record->completed_sample_id = completed_sample_id;
  record->metrics = *metrics;
  atomic_signal_fence(memory_order_seq_cst);
  atomic_thread_fence(memory_order_seq_cst);
  record->commit_marker = NODE_RTC_COMMITTED_V1;
  return true;
}

static uint64_t saturating_add_u64(uint64_t left, uint64_t right) {
  if (UINT64_MAX - left < right) {
    return UINT64_MAX;
  }
  return left + right;
}

static uint64_t elapsed_us(uint64_t start, uint64_t end) {
  return end >= start ? end - start : 0U;
}

static uint64_t elapsed_ms(uint64_t start, uint64_t end) {
  return elapsed_us(start, end) / UINT64_C(1000);
}

static uint64_t reading_airtime_charge_us(void) {
  const uint64_t airtime_us =
      sx1262_radio_airtime_us(CURA_LORA_V2_READING_FRAME_SIZE);
  if (airtime_us == UINT64_MAX) {
    return UINT64_MAX;
  }
  const uint64_t ten_percent_us =
      airtime_us / UINT64_C(10) + (airtime_us % UINT64_C(10) != 0U ? 1U : 0U);
  return saturating_add_u64(airtime_us, ten_percent_us);
}

bool node_core_attempt_fits(uint64_t now_us, uint64_t deadline_us,
                            uint64_t charged_airtime_us) {
  const uint64_t airtime_charge_us = reading_airtime_charge_us();
  const bool airtime_fits =
      airtime_charge_us <= NODE_CORE_TX_AIRTIME_BUDGET_US &&
      charged_airtime_us <= NODE_CORE_TX_AIRTIME_BUDGET_US - airtime_charge_us;
  const uint64_t minimum_window_us =
      sx1262_radio_min_tx_window_us(CURA_LORA_V2_READING_FRAME_SIZE);
  const bool time_fits = minimum_window_us != UINT64_MAX &&
                         now_us <= deadline_us &&
                         minimum_window_us <= deadline_us - now_us;
  return airtime_fits && time_fits;
}

static uint32_t application_offset_ms(uint64_t application_start_us,
                                      uint64_t now_us, bool *out_valid) {
  const uint64_t offset_ms = elapsed_ms(application_start_us, now_us);
  if (offset_ms > UINT32_MAX) {
    *out_valid = false;
    return 0U;
  }
  *out_valid = true;
  return (uint32_t)offset_ms;
}

static bool build_reading(const node_sensor_sample_t *sample,
                          const node_cycle_metrics_t *previous_metrics,
                          bool previous_metrics_valid, uint8_t reset_reason,
                          uint16_t run_ms,
                          cura_lora_v2_reading_t *out_reading) {
  if (sample == NULL || previous_metrics == NULL || out_reading == NULL) {
    return false;
  }

  cura_lora_v2_reading_t reading = {
      .run_ms = run_ms,
      .reset_reason = reset_reason,
  };
  if ((sample->validity & NODE_SENSOR_SOIL_0_VALID) != 0U) {
    reading.soil_0_mv = sample->soil_0_mv;
    reading.flags |= CURA_LORA_V2_FLAG_SOIL_0_VALID;
  }
  if ((sample->validity & NODE_SENSOR_SOIL_1_VALID) != 0U) {
    reading.soil_1_mv = sample->soil_1_mv;
    reading.flags |= CURA_LORA_V2_FLAG_SOIL_1_VALID;
  }
  if ((sample->validity & NODE_SENSOR_SOIL_TEMP_0_VALID) != 0U) {
    reading.soil_temp_0_centi_c = sample->soil_temp_0_centi_c;
    reading.flags |= CURA_LORA_V2_FLAG_SOIL_TEMP_0_VALID;
  }
  if ((sample->validity & NODE_SENSOR_SOIL_TEMP_1_VALID) != 0U) {
    reading.soil_temp_1_centi_c = sample->soil_temp_1_centi_c;
    reading.flags |= CURA_LORA_V2_FLAG_SOIL_TEMP_1_VALID;
  }
  if ((sample->validity & NODE_SENSOR_ENCLOSURE_ENV_VALID) != 0U) {
    reading.enclosure_centi_c = sample->enclosure_centi_c;
    reading.enclosure_pressure_pa = sample->enclosure_pressure_pa;
    reading.enclosure_humidity_centi_pct = sample->enclosure_humidity_centi_pct;
    reading.flags |= CURA_LORA_V2_FLAG_ENCLOSURE_TEMP_VALID |
                     CURA_LORA_V2_FLAG_ENCLOSURE_PRESSURE_VALID |
                     CURA_LORA_V2_FLAG_ENCLOSURE_HUMIDITY_VALID;
  }
  if (reset_reason == CURA_LORA_V2_RESET_REASON_ESP_RST_DEEPSLEEP) {
    reading.flags |= CURA_LORA_V2_FLAG_DEEP_SLEEP_BOOT;
  }
  if (previous_metrics_valid) {
    reading.previous_current_tx_attempts =
        previous_metrics->current_tx_attempts;
    reading.previous_awake_ms = previous_metrics->awake_ms;
    reading.previous_current_delivery_ms =
        previous_metrics->current_delivery_ms;
    reading.previous_cycle_tx_attempts = previous_metrics->cycle_tx_attempts;
    reading.previous_cycle_accepted_readings =
        previous_metrics->accepted_readings;
    reading.flags |= CURA_LORA_V2_FLAG_PREVIOUS_CYCLE_METRICS_VALID;
    if (previous_metrics->current_accepted) {
      reading.flags |= CURA_LORA_V2_FLAG_PREVIOUS_CURRENT_ACCEPTED;
    }
  }
  if (cura_lora_v2_validate_reading(&reading) != CURA_LORA_V2_CODEC_OK) {
    return false;
  }
  *out_reading = reading;
  return true;
}

static bool wake_metrics_encode(const node_wake_metrics_t *wake,
                                uint64_t awake_us,
                                node_cycle_metrics_t *out_metrics) {
  if (wake == NULL || out_metrics == NULL) {
    return false;
  }
  const uint64_t awake_ms = awake_us / UINT64_C(1000);
  const uint64_t delivery_ms = wake->current_delivery_us / UINT64_C(1000);
  if (wake->current_tx_attempts > UINT8_MAX || awake_ms > UINT16_MAX ||
      delivery_ms > UINT16_MAX || wake->cycle_tx_attempts > UINT8_MAX ||
      wake->accepted_readings > UINT8_MAX) {
    memset(out_metrics, 0, sizeof(*out_metrics));
    return false;
  }
  *out_metrics = (node_cycle_metrics_t){
      .current_tx_attempts = (uint8_t)wake->current_tx_attempts,
      .awake_ms = (uint16_t)awake_ms,
      .current_delivery_ms = (uint16_t)delivery_ms,
      .cycle_tx_attempts = (uint8_t)wake->cycle_tx_attempts,
      .accepted_readings = (uint8_t)wake->accepted_readings,
      .current_accepted = wake->current_accepted,
  };
  return cycle_metrics_are_valid(out_metrics);
}

static bool platform_is_valid(const node_platform_ports_t *platform) {
  return platform != NULL && platform->clock.monotonic_us != NULL &&
         platform->randomness.uniform_u32_inclusive != NULL &&
         platform->system.get_reset_reason != NULL &&
         platform->system.enter_deep_sleep_for != NULL;
}

static err_curag_t core_error(curag_error_code_t code) {
  return curag_error_make(CURAG_EDOM_CORE, code);
}

static void core_diagnostic_context(curag_operation_t operation,
                                    diagn_context_t *out_context) {
  curag_diagnostic_context_clear(out_context);
  out_context->operation = operation;
}

static void append_diagnostic(node_cycle_context_t *cycle, err_curag_t error,
                              const diagn_context_t *context) {
  if (cycle == NULL || error == CURAG_OK) {
    return;
  }
  const uint64_t now_us =
      cycle->platform->clock.monotonic_us(cycle->platform->clock.context);
  bool offset_valid = false;
  const uint32_t offset_ms =
      application_offset_ms(cycle->application_start_us, now_us, &offset_valid);
  node_diagnostic_event_t event = {
      .error = error,
      .flags = 0U,
      .application_offset_ms = offset_ms,
      .cycle_sample_id = 0U,
      .context = context,
  };
  if (offset_valid) {
    event.flags |= NODE_DIAGNOSTIC_APPLICATION_OFFSET_VALID;
  }
  if (cycle->cycle_sample_id_valid) {
    event.flags |= NODE_DIAGNOSTIC_CYCLE_SAMPLE_ID_VALID;
    event.cycle_sample_id = cycle->cycle_sample_id;
  }
  (void)node_persistence_append_diagnostic_event(&event, NULL);
}

static err_curag_t build_frame(const node_identity_t *identity,
                               uint32_t sample_id, cura_lora_v2_domain_t domain,
                               const cura_lora_v2_reading_t *reading,
                               uint8_t frame[CURA_LORA_V2_READING_FRAME_SIZE],
                               diagn_context_t *out_diag) {
  uint8_t body[CURA_LORA_V2_READING_BODY_SIZE];
  if (cura_lora_v2_encode_reading(body, sizeof(body), reading) !=
      CURA_LORA_V2_CODEC_OK) {
    core_diagnostic_context(CURAG_OP_ENCODE, out_diag);
    return core_error(CURAG_ECORE_ECODEC);
  }

  cura_lora_v2_clear_header_t header = {
      .control = CURA_LORA_V2_CONTROL,
      .domain = domain,
      .sample_id = sample_id,
  };
  memcpy(header.node_id, identity->node_id, sizeof(header.node_id));
  size_t frame_length = 0U;
  if (cura_lora_v2_seal_frame(frame, CURA_LORA_V2_READING_FRAME_SIZE,
                              &frame_length, identity->node_key, &header, body,
                              sizeof(body)) != CURA_LORA_V2_CRYPTO_OK ||
      frame_length != CURA_LORA_V2_READING_FRAME_SIZE) {
    core_diagnostic_context(CURAG_OP_ENCRYPT, out_diag);
    return core_error(CURAG_ECORE_ECRYPTO);
  }
  curag_diagnostic_context_clear(out_diag);
  return CURAG_OK;
}

static ack_validation_result_t invalid_ack(curag_error_code_t code,
                                           curag_operation_t operation) {
  return (ack_validation_result_t){
      .kind = ACK_VALIDATION_INVALID,
      .result = NODE_DELIVERY_RESULT_INVALID,
      .error = core_error(code),
      .operation = operation,
  };
}

static ack_validation_result_t
validate_ack(const node_identity_t *identity, uint32_t sample_id,
             const sx1262_radio_rx_result_t *received) {
  if (received->payload_length != CURA_LORA_V2_ACK_FRAME_SIZE) {
    return invalid_ack(CURAG_ECORE_EACK_LENGTH, CURAG_OP_DECODE);
  }

  cura_lora_v2_clear_header_t header;
  uint8_t body[CURA_LORA_V2_MAX_BODY_SIZE];
  size_t body_length = 0U;
  const cura_lora_v2_crypto_result_t crypto_result = cura_lora_v2_open_frame(
      &header, body, sizeof(body), &body_length, identity->node_key,
      received->payload, received->payload_length);
  if (crypto_result == CURA_LORA_V2_CRYPTO_AUTHENTICATION_FAILED) {
    return invalid_ack(CURAG_ECORE_EACK_AUTHENTICATION, CURAG_OP_DECRYPT);
  }
  if (crypto_result != CURA_LORA_V2_CRYPTO_OK) {
    return (ack_validation_result_t){
        .kind = ACK_VALIDATION_LOCAL_ERROR,
        .result = NODE_DELIVERY_RESULT_LOCAL_RADIO_ERROR,
        .error = core_error(CURAG_ECORE_ECRYPTO),
        .operation = CURAG_OP_DECRYPT,
    };
  }
  if (!cura_lora_v2_is_supported_control(header.control)) {
    return invalid_ack(CURAG_ECORE_EACK_CONTROL, CURAG_OP_VALIDATE);
  }
  if (memcmp(header.node_id, identity->node_id, sizeof(header.node_id)) != 0) {
    return invalid_ack(CURAG_ECORE_EACK_NODE_ID, CURAG_OP_VALIDATE);
  }
  if (header.sample_id != sample_id) {
    return invalid_ack(CURAG_ECORE_EACK_SAMPLE_ID, CURAG_OP_VALIDATE);
  }
  if (!cura_lora_v2_domain_is_ack(header.domain)) {
    return invalid_ack(CURAG_ECORE_EACK_DOMAIN, CURAG_OP_VALIDATE);
  }
  if (body_length != CURA_LORA_V2_ACK_BODY_SIZE) {
    return invalid_ack(CURAG_ECORE_EACK_BODY, CURAG_OP_DECODE);
  }
  cura_lora_v2_ack_t ack;
  const cura_lora_v2_codec_result_t decode_result =
      cura_lora_v2_decode_ack(&ack, body, body_length);
  if (decode_result == CURA_LORA_V2_CODEC_MALFORMED) {
    return invalid_ack(CURAG_ECORE_EACK_STATUS, CURAG_OP_VALIDATE);
  }
  if (decode_result != CURA_LORA_V2_CODEC_OK) {
    return invalid_ack(CURAG_ECORE_EACK_BODY, CURAG_OP_DECODE);
  }
  if (!cura_lora_v2_ack_status_matches_domain(header.domain, ack.status)) {
    return invalid_ack(CURAG_ECORE_EACK_STATUS, CURAG_OP_VALIDATE);
  }

  node_delivery_final_result_t result = NODE_DELIVERY_RESULT_INVALID;
  switch (ack.status) {
  case CURA_LORA_V2_ACK_STATUS_ACCEPTED:
    result = NODE_DELIVERY_RESULT_ACCEPTED;
    break;
  case CURA_LORA_V2_ACK_STATUS_RETRY_LATER:
    result = NODE_DELIVERY_RESULT_RETRY_LATER;
    break;
  case CURA_LORA_V2_ACK_STATUS_REJECTED_UNSUPPORTED:
    result = NODE_DELIVERY_RESULT_UNSUPPORTED;
    break;
  case CURA_LORA_V2_ACK_STATUS_REJECTED_MALFORMED:
    result = NODE_DELIVERY_RESULT_MALFORMED;
    break;
  default:
    return invalid_ack(CURAG_ECORE_EACK_STATUS, CURAG_OP_VALIDATE);
  }
  return (ack_validation_result_t){
      .kind = ACK_VALIDATION_VALID,
      .result = result,
      .error = CURAG_OK,
      .operation = CURAG_OP_NONE,
  };
}

static node_delivery_final_result_t limit_result(uint64_t now_us,
                                                 uint64_t deadline_us,
                                                 uint64_t charged_airtime_us) {
  const uint64_t airtime_charge_us = reading_airtime_charge_us();
  if (airtime_charge_us > NODE_CORE_TX_AIRTIME_BUDGET_US ||
      charged_airtime_us > NODE_CORE_TX_AIRTIME_BUDGET_US - airtime_charge_us) {
    return NODE_DELIVERY_RESULT_AIRTIME_BUDGET_END;
  }
  const uint64_t minimum_window_us =
      sx1262_radio_min_tx_window_us(CURA_LORA_V2_READING_FRAME_SIZE);
  if (minimum_window_us == UINT64_MAX || now_us > deadline_us ||
      minimum_window_us > deadline_us - now_us) {
    return NODE_DELIVERY_RESULT_RADIO_CYCLE_DEADLINE;
  }
  return NODE_DELIVERY_RESULT_INVALID;
}

static void append_delivery_boundary(node_cycle_context_t *cycle,
                                     const node_delivery_event_t *event) {
  diagn_context_t diagnostic;
  const err_curag_t result =
      node_persistence_append_delivery_event(event, &diagnostic);
  if (result != CURAG_OK) {
    append_diagnostic(cycle, result, &diagnostic);
  }
}

static node_delivery_result_t
deliver_reading(node_cycle_context_t *cycle, uint32_t sample_id,
                const cura_lora_v2_reading_t *reading,
                cura_lora_v2_domain_t domain, bool is_current) {
  const uint64_t delivery_start_us =
      cycle->platform->clock.monotonic_us(cycle->platform->clock.context);
  diagn_context_t diagnostic;
  bool start_offset_valid = false;
  uint32_t start_offset_ms = application_offset_ms(
      cycle->application_start_us, delivery_start_us, &start_offset_valid);
  if (!start_offset_valid) {
    start_offset_ms = UINT32_MAX;
    core_diagnostic_context(CURAG_OP_VALIDATE, &diagnostic);
    append_diagnostic(cycle, core_error(CURAG_ECORE_ETIME_RANGE), &diagnostic);
  }
  node_delivery_event_t started = {
      .type = NODE_DELIVERY_EVENT_STARTED,
      .cycle_sample_id = cycle->cycle_sample_id,
      .sample_id = sample_id,
      .domain = domain,
      .detail.started = {.start_offset_ms = start_offset_ms},
  };
  append_delivery_boundary(cycle, &started);

  uint8_t frame[CURA_LORA_V2_READING_FRAME_SIZE];
  const err_curag_t frame_result = build_frame(
      cycle->identity, sample_id, domain, reading, frame, &diagnostic);
  if (frame_result != CURAG_OK) {
    append_diagnostic(cycle, frame_result, &diagnostic);
    node_delivery_event_t failed = {
        .type = NODE_DELIVERY_EVENT_FINISHED,
        .cycle_sample_id = cycle->cycle_sample_id,
        .sample_id = sample_id,
        .domain = domain,
        .detail.finished =
            {
                .attempt_count = 0U,
                .final_result = NODE_DELIVERY_RESULT_LOCAL_RADIO_ERROR,
            },
    };
    append_delivery_boundary(cycle, &failed);
    return (node_delivery_result_t){
        .result = NODE_DELIVERY_RESULT_LOCAL_RADIO_ERROR,
        .attempt_count = 0U,
    };
  }

  node_delivery_final_result_t final_result = NODE_DELIVERY_RESULT_INVALID;
  uint32_t attempt_count = 0U;
  uint64_t first_set_tx_us = 0U;
  bool first_set_tx_valid = false;
  uint64_t accepted_at_us = 0U;

  while (final_result == NODE_DELIVERY_RESULT_INVALID) {
    const uint64_t before_tx_us =
        cycle->platform->clock.monotonic_us(cycle->platform->clock.context);
    if (!node_core_attempt_fits(before_tx_us, cycle->radio_deadline_us,
                                cycle->metrics.charged_airtime_us)) {
      final_result = limit_result(before_tx_us, cycle->radio_deadline_us,
                                  cycle->metrics.charged_airtime_us);
      break;
    }

    sx1262_radio_tx_result_t tx_result;
    const err_curag_t tx_error = sx1262_radio_transmit_uplink(
        frame, sizeof(frame), cycle->radio_deadline_us, &tx_result,
        &diagnostic);
    if (tx_result.tx_started) {
      attempt_count++;
      cycle->metrics.cycle_tx_attempts++;
      if (is_current) {
        cycle->metrics.current_tx_attempts++;
      }
      cycle->metrics.charged_airtime_us += reading_airtime_charge_us();
      if (!first_set_tx_valid) {
        first_set_tx_us = tx_result.set_tx_at_us;
        first_set_tx_valid = true;
      }
    }

    uint64_t retry_at_us = 0U;
    if (tx_result.tx_done) {
      const uint32_t jitter_us =
          cycle->platform->randomness.uniform_u32_inclusive(
              cycle->platform->randomness.context,
              NODE_CORE_RETRY_JITTER_MIN_US, NODE_CORE_RETRY_JITTER_MAX_US);
      retry_at_us = saturating_add_u64(
          saturating_add_u64(tx_result.tx_done_at_us, NODE_CORE_ACK_WAIT_US),
          jitter_us);
    }
    if (tx_error != CURAG_OK) {
      append_diagnostic(cycle, tx_error, &diagnostic);
      /* A deadline after SetTx is a charged radio failure, not admission. */
      if (!tx_result.tx_started &&
          curag_error_domain(tx_error) == CURAG_EDOM_RADIO &&
          curag_error_code(tx_error) == CURAG_ERADIO_EDEADLINE) {
        final_result = NODE_DELIVERY_RESULT_RADIO_CYCLE_DEADLINE;
      } else {
        final_result = NODE_DELIVERY_RESULT_LOCAL_RADIO_ERROR;
      }
      break;
    }
    if (!tx_result.tx_started || !tx_result.tx_done) {
      final_result = NODE_DELIVERY_RESULT_LOCAL_RADIO_ERROR;
      break;
    }
    const uint64_t receive_deadline_us = retry_at_us < cycle->radio_deadline_us
                                             ? retry_at_us
                                             : cycle->radio_deadline_us;

    bool receive_complete = false;
    while (!receive_complete) {
      sx1262_radio_rx_result_t rx_result;
      const err_curag_t rx_error = sx1262_radio_receive_downlink_until(
          receive_deadline_us, &rx_result, &diagnostic);
      if (rx_error != CURAG_OK) {
        append_diagnostic(cycle, rx_error, &diagnostic);
        final_result = NODE_DELIVERY_RESULT_LOCAL_RADIO_ERROR;
        receive_complete = true;
      } else if (rx_result.outcome == SX1262_RADIO_RX_DEADLINE) {
        if (cycle->radio_deadline_us <= retry_at_us) {
          final_result = NODE_DELIVERY_RESULT_RADIO_CYCLE_DEADLINE;
        }
        receive_complete = true;
      } else if (rx_result.outcome == SX1262_RADIO_RX_PACKET) {
        if (rx_result.rx_done_at_us > receive_deadline_us) {
          core_diagnostic_context(CURAG_OP_VALIDATE, &diagnostic);
          append_diagnostic(cycle, core_error(CURAG_ECORE_EACK_TIMESTAMP),
                            &diagnostic);
          continue;
        }
        const ack_validation_result_t validation =
            validate_ack(cycle->identity, sample_id, &rx_result);
        if (validation.kind == ACK_VALIDATION_VALID) {
          final_result = validation.result;
          accepted_at_us = rx_result.rx_done_at_us;
          receive_complete = true;
        } else {
          core_diagnostic_context(validation.operation, &diagnostic);
          append_diagnostic(cycle, validation.error, &diagnostic);
          if (validation.kind == ACK_VALIDATION_LOCAL_ERROR) {
            final_result = NODE_DELIVERY_RESULT_LOCAL_RADIO_ERROR;
            receive_complete = true;
          }
        }
      } else {
        core_diagnostic_context(CURAG_OP_RECEIVE, &diagnostic);
        append_diagnostic(cycle, core_error(CURAG_ECORE_EACK_BODY),
                          &diagnostic);
        final_result = NODE_DELIVERY_RESULT_LOCAL_RADIO_ERROR;
        receive_complete = true;
      }
    }
  }

  if (final_result == NODE_DELIVERY_RESULT_ACCEPTED) {
    cycle->metrics.accepted_readings++;
    if (is_current) {
      cycle->metrics.current_accepted = true;
      cycle->metrics.current_delivery_us =
          elapsed_us(first_set_tx_us, accepted_at_us);
    }
  }

  node_delivery_event_t finished = {
      .type = NODE_DELIVERY_EVENT_FINISHED,
      .cycle_sample_id = cycle->cycle_sample_id,
      .sample_id = sample_id,
      .domain = domain,
      .detail.finished =
          {
              .attempt_count = (uint8_t)attempt_count,
              .final_result = final_result,
          },
  };
  append_delivery_boundary(cycle, &finished);
  return (node_delivery_result_t){
      .result = final_result,
      .attempt_count = attempt_count,
  };
}

static bool result_is_permanent_rejection(node_delivery_final_result_t result) {
  return result == NODE_DELIVERY_RESULT_UNSUPPORTED ||
         result == NODE_DELIVERY_RESULT_MALFORMED;
}

static bool remove_pending(node_cycle_context_t *cycle, uint32_t sample_id) {
  /* Callers delete only at authenticated or permanent protocol boundaries. */
  diagn_context_t diagnostic;
  const err_curag_t result =
      node_persistence_remove_newest_reading(sample_id, &diagnostic);
  if (result != CURAG_OK) {
    append_diagnostic(cycle, result, &diagnostic);
    return false;
  }
  return true;
}

static bool quarantine_and_remove(node_cycle_context_t *cycle,
                                  uint32_t sample_id,
                                  const cura_lora_v2_reading_t *reading,
                                  bool remove_persisted_copy) {
  diagn_context_t diagnostic;
  const err_curag_t quarantine_result =
      node_persistence_quarantine_reading(sample_id, reading, &diagnostic);
  if (quarantine_result != CURAG_OK) {
    append_diagnostic(cycle, quarantine_result, &diagnostic);
  }
  if (!remove_persisted_copy) {
    return true;
  }
  /* A permanently rejected tail must not obstruct newer pending readings. */
  return remove_pending(cycle, sample_id);
}

static void drain_backlog(node_cycle_context_t *cycle) {
  while (true) {
    const uint64_t now_us =
        cycle->platform->clock.monotonic_us(cycle->platform->clock.context);
    if (!node_core_attempt_fits(now_us, cycle->radio_deadline_us,
                                cycle->metrics.charged_airtime_us)) {
      return;
    }

    uint32_t sample_id = 0U;
    cura_lora_v2_reading_t reading;
    bool found = false;
    diagn_context_t diagnostic;
    const err_curag_t peek_result = node_persistence_peek_most_recent_pending(
        &sample_id, &reading, &found, &diagnostic);
    if (peek_result != CURAG_OK) {
      append_diagnostic(cycle, peek_result, &diagnostic);
      return;
    }
    if (!found) {
      return;
    }

    const node_delivery_result_t delivery =
        deliver_reading(cycle, sample_id, &reading,
                        CURA_LORA_V2_DOMAIN_BACKLOG_READING_UPLINK, false);
    if (delivery.result == NODE_DELIVERY_RESULT_ACCEPTED) {
      if (!remove_pending(cycle, sample_id)) {
        return;
      }
      continue;
    }
    if (result_is_permanent_rejection(delivery.result)) {
      if (!quarantine_and_remove(cycle, sample_id, &reading, true)) {
        return;
      }
      continue;
    }
    return;
  }
}

static void run_claimed_cycle(node_cycle_context_t *cycle,
                              const node_rtc_record_t *incoming_rtc,
                              uint8_t reset_reason) {
  node_cycle_metrics_t previous_metrics = {0};
  const bool previous_metrics_valid = node_rtc_record_validate_previous(
      incoming_rtc, reset_reason, cycle->cycle_sample_id, &previous_metrics);

  node_sensor_sample_t sensor_sample;
  diagn_context_t diagnostic;
  const err_curag_t sensor_result =
      node_sensors_sample_all(&sensor_sample, &diagnostic);
  if (sensor_result != CURAG_OK) {
    append_diagnostic(cycle, sensor_result, &diagnostic);
  }

  const uint64_t body_finalized_us =
      cycle->platform->clock.monotonic_us(cycle->platform->clock.context);
  const uint64_t run_ms_wide =
      elapsed_ms(cycle->application_start_us, body_finalized_us);
  uint16_t run_ms = 0U;
  bool run_ms_overflow = false;
  if (run_ms_wide > UINT16_MAX) {
    run_ms = UINT16_MAX;
    run_ms_overflow = true;
  } else {
    run_ms = (uint16_t)run_ms_wide;
  }

  cura_lora_v2_reading_t current_reading;
  if (!build_reading(&sensor_sample, &previous_metrics, previous_metrics_valid,
                     reset_reason, run_ms, &current_reading)) {
    core_diagnostic_context(CURAG_OP_ENCODE, &diagnostic);
    append_diagnostic(cycle, core_error(CURAG_ECORE_ECODEC), &diagnostic);
    return;
  }

  bool current_persisted = false;
  const err_curag_t append_result = node_persistence_append_pending_reading(
      cycle->cycle_sample_id, &current_reading, &diagnostic);
  if (append_result == CURAG_OK) {
    current_persisted = true;
  } else {
    append_diagnostic(cycle, append_result, &diagnostic);
    /* Continue volatile; deep sleep may discard an unaccepted reading. */
  }
  if (run_ms_overflow) {
    core_diagnostic_context(CURAG_OP_VALIDATE, &diagnostic);
    append_diagnostic(cycle, core_error(CURAG_ECORE_ETIME_RANGE), &diagnostic);
  }

  const node_delivery_result_t current_delivery =
      deliver_reading(cycle, cycle->cycle_sample_id, &current_reading,
                      CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK, true);
  if (current_delivery.result == NODE_DELIVERY_RESULT_ACCEPTED) {
    if (current_persisted && !remove_pending(cycle, cycle->cycle_sample_id)) {
      return;
    }
    if (current_persisted) {
      drain_backlog(cycle);
    }
    return;
  }
  if (result_is_permanent_rejection(current_delivery.result)) {
    (void)quarantine_and_remove(cycle, cycle->cycle_sample_id, &current_reading,
                                current_persisted);
  }
}

static void finalize_cycle(node_cycle_context_t *cycle,
                           node_rtc_record_t *rtc_record) {
  diagn_context_t diagnostic;
  err_curag_t result = node_sensors_force_power_off(&diagnostic);
  if (result != CURAG_OK) {
    append_diagnostic(cycle, result, &diagnostic);
  }

  result = sx1262_radio_sleep(&diagnostic);
  if (result != CURAG_OK) {
    append_diagnostic(cycle, result, &diagnostic);
  }

  if (cycle->cycle_sample_id_valid) {
    const uint64_t before_sync_us =
        cycle->platform->clock.monotonic_us(cycle->platform->clock.context);
    node_cycle_metrics_t ignored;
    /* Report overflows still knowable before diagnostics are synchronized. */
    if (!wake_metrics_encode(
            &cycle->metrics,
            elapsed_us(cycle->application_start_us, before_sync_us),
            &ignored)) {
      core_diagnostic_context(CURAG_OP_VALIDATE, &diagnostic);
      append_diagnostic(cycle, core_error(CURAG_ECORE_EMETRICS_OVERFLOW),
                        &diagnostic);
    }
  }

  /* A failed final sync cannot be durably logged without a second sync. */
  (void)node_persistence_sync_all(&diagnostic);
  const uint64_t completed_us =
      cycle->platform->clock.monotonic_us(cycle->platform->clock.context);
  if (cycle->cycle_sample_id_valid) {
    node_cycle_metrics_t encoded_metrics;
    /* After the only sync, fail closed in RTC without a late diagnostic. */
    if (wake_metrics_encode(
            &cycle->metrics,
            elapsed_us(cycle->application_start_us, completed_us),
            &encoded_metrics)) {
      (void)node_rtc_record_commit(rtc_record, cycle->cycle_sample_id,
                                   &encoded_metrics);
    } else {
      rtc_record->commit_marker = 0U;
    }
  }

  cycle->platform->system.enter_deep_sleep_for(
      cycle->platform->system.context, NODE_CORE_DEEP_SLEEP_DURATION_US);
}

void node_cycle_run(const node_platform_ports_t *platform,
                    const node_identity_t *identity,
                    node_rtc_record_t *rtc_record) {
  if (!platform_is_valid(platform) || identity == NULL || rtc_record == NULL) {
    return;
  }

  node_cycle_context_t cycle = {
      .platform = platform,
      .identity = identity,
  };
  cycle.application_start_us =
      platform->clock.monotonic_us(platform->clock.context);
  cycle.radio_deadline_us = saturating_add_u64(cycle.application_start_us,
                                               NODE_CORE_RADIO_CYCLE_LIMIT_US);
  const uint8_t reset_reason =
      platform->system.get_reset_reason(platform->system.context);

  node_rtc_record_t incoming_rtc;
  node_rtc_record_take(rtc_record, &incoming_rtc);

  diagn_context_t diagnostic;
  uint32_t sample_id = 0U;
  const err_curag_t claim_result =
      node_persistence_claim_sample_id(&sample_id, &diagnostic);
  if (claim_result != CURAG_OK) {
    append_diagnostic(&cycle, claim_result, &diagnostic);
  } else {
    cycle.cycle_sample_id = sample_id;
    cycle.cycle_sample_id_valid = true;
    run_claimed_cycle(&cycle, &incoming_rtc, reset_reason);
  }
  finalize_cycle(&cycle, rtc_record);
}
