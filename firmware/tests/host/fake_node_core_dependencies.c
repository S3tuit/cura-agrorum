#include "fake_node_core_dependencies.h"

#include <assert.h>
#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "protocol_v2_lora_crypto.h"

fake_node_core_state_t fake_node_core;

static void trace_call(fake_node_core_trace_t value) {
  assert(fake_node_core.trace_count < FAKE_NODE_CORE_MAX_TRACE);
  if (fake_node_core.deep_sleep_call_count != 0U) {
    fake_node_core.calls_after_deep_sleep++;
  }
  fake_node_core.trace[fake_node_core.trace_count++] = value;
}

static void advance_time(uint64_t amount_us) {
  if (UINT64_MAX - fake_node_core.now_us < amount_us) {
    fake_node_core.now_us = UINT64_MAX;
  } else {
    fake_node_core.now_us += amount_us;
  }
}

static void advance_to(uint64_t time_us) {
  if (time_us > fake_node_core.now_us) {
    fake_node_core.now_us = time_us;
  }
}

static void make_context(diagn_context_t *context, curag_operation_t operation,
                         uint8_t schema, uint8_t length) {
  curag_diagnostic_context_clear(context);
  context->operation = operation;
  context->context_schema = schema;
  context->context_length = length;
}

void fake_node_core_reset(void) {
  memset(&fake_node_core, 0, sizeof(fake_node_core));
  fake_node_core.now_us = UINT64_C(1000000);
  fake_node_core.reset_reason = CURA_LORA_V2_RESET_REASON_ESP_RST_POWERON;
  fake_node_core.sensor_sample = (node_sensor_sample_t){
      .soil_0_mv = UINT16_C(101),
      .soil_1_mv = UINT16_C(202),
      .soil_temp_0_centi_c = INT16_C(303),
      .soil_temp_1_centi_c = INT16_C(404),
      .enclosure_centi_c = INT16_C(505),
      .enclosure_pressure_pa = UINT32_C(100600),
      .enclosure_humidity_centi_pct = UINT16_C(707),
      .validity = NODE_SENSOR_VALIDITY_ALL,
  };
  make_context(&fake_node_core.claim_diagnostic, CURAG_OP_READ,
               CURAG_PERSISTENCE_CONTEXT_V1, CURAG_PERSISTENCE_CONTEXT_V1_SIZE);
  make_context(&fake_node_core.message_claim_diagnostic, CURAG_OP_READ,
               CURAG_PERSISTENCE_CONTEXT_V1, CURAG_PERSISTENCE_CONTEXT_V1_SIZE);
  make_context(&fake_node_core.append_pending_diagnostic, CURAG_OP_APPEND,
               CURAG_PERSISTENCE_CONTEXT_V1, CURAG_PERSISTENCE_CONTEXT_V1_SIZE);
  make_context(&fake_node_core.persistence_diagnostic, CURAG_OP_READ,
               CURAG_PERSISTENCE_CONTEXT_V1, CURAG_PERSISTENCE_CONTEXT_V1_SIZE);
  make_context(&fake_node_core.sensor_diagnostic, CURAG_OP_READ,
               CURAG_SENSOR_CONTEXT_V1, CURAG_SENSOR_CONTEXT_V1_LENGTH);
  make_context(&fake_node_core.force_power_off_diagnostic, CURAG_OP_POWER_OFF,
               CURAG_SENSOR_CONTEXT_V1, CURAG_SENSOR_CONTEXT_V1_LENGTH);
  make_context(&fake_node_core.radio_sleep_diagnostic, CURAG_OP_SLEEP,
               CURAG_RADIO_CONTEXT_V1, CURAG_RADIO_CONTEXT_V1_LENGTH);
}

void fake_node_core_advance_us(uint64_t amount_us) { advance_time(amount_us); }

void fake_node_core_add_pending(uint32_t sample_id,
                                const cura_lora_v2_reading_t *reading) {
  assert(reading != NULL);
  assert(fake_node_core.pending_count < FAKE_NODE_CORE_MAX_PENDING);
  fake_node_core_pending_t *pending =
      &fake_node_core.pending[fake_node_core.pending_count++];
  pending->value.reading = *reading;
  pending->value.reading.sample_id = sample_id;
}

void fake_node_core_add_bound_pending(
    uint32_t message_id, const cura_lora_v2_reading_t *reading,
    const uint8_t frame[CURA_LORA_V2_READING_FRAME_SIZE]) {
  fake_node_core_add_pending(reading->sample_id, reading);
  fake_node_core_pending_t *pending =
      &fake_node_core.pending[fake_node_core.pending_count - 1U];
  pending->value.backlog_bound = true;
  pending->value.message_id = message_id;
  memcpy(pending->value.frame, frame, sizeof(pending->value.frame));
}

void fake_node_core_script_tx_done(uint64_t set_tx_at_us,
                                   uint64_t tx_done_at_us) {
  assert(fake_node_core.tx_script_count < FAKE_NODE_CORE_MAX_ITEMS);
  fake_node_core.tx_scripts[fake_node_core.tx_script_count++] =
      (fake_node_core_tx_script_t){
          .error = CURAG_OK,
          .result =
              {
                  .tx_started = true,
                  .tx_done = true,
                  .set_tx_at_us = set_tx_at_us,
                  .tx_done_at_us = tx_done_at_us,
              },
          .return_at_us = tx_done_at_us,
      };
}

void fake_node_core_script_tx_error(err_curag_t error, bool tx_started,
                                    bool tx_done, uint64_t set_tx_at_us,
                                    uint64_t tx_done_at_us,
                                    uint64_t return_at_us) {
  assert(fake_node_core.tx_script_count < FAKE_NODE_CORE_MAX_ITEMS);
  fake_node_core_tx_script_t *script =
      &fake_node_core.tx_scripts[fake_node_core.tx_script_count++];
  *script = (fake_node_core_tx_script_t){
      .error = error,
      .result =
          {
              .tx_started = tx_started,
              .tx_done = tx_done,
              .set_tx_at_us = set_tx_at_us,
              .tx_done_at_us = tx_done_at_us,
          },
      .return_at_us = return_at_us,
  };
  make_context(&script->diagnostic, CURAG_OP_TRANSMIT, CURAG_RADIO_CONTEXT_V1,
               CURAG_RADIO_CONTEXT_V1_LENGTH);
}

void fake_node_core_script_rx_deadline(uint64_t deadline_us) {
  assert(fake_node_core.rx_script_count < FAKE_NODE_CORE_MAX_ITEMS);
  fake_node_core.rx_scripts[fake_node_core.rx_script_count++] =
      (fake_node_core_rx_script_t){
          .error = CURAG_OK,
          .result = {.outcome = SX1262_RADIO_RX_DEADLINE},
          .return_at_us = deadline_us,
      };
}

void fake_node_core_script_rx_packet(const uint8_t *payload,
                                     size_t payload_length,
                                     uint64_t rx_done_at_us,
                                     uint64_t return_at_us) {
  assert(payload != NULL);
  assert(payload_length <= SX1262_RADIO_MAX_PAYLOAD_SIZE);
  assert(fake_node_core.rx_script_count < FAKE_NODE_CORE_MAX_ITEMS);
  fake_node_core_rx_script_t *script =
      &fake_node_core.rx_scripts[fake_node_core.rx_script_count++];
  *script = (fake_node_core_rx_script_t){
      .error = CURAG_OK,
      .result =
          {
              .outcome = SX1262_RADIO_RX_PACKET,
              .rx_done_at_us = rx_done_at_us,
              .payload_length = (uint8_t)payload_length,
          },
      .return_at_us = return_at_us,
  };
  memcpy(script->result.payload, payload, payload_length);
}

void fake_node_core_script_rx_error(err_curag_t error, uint64_t return_at_us) {
  assert(fake_node_core.rx_script_count < FAKE_NODE_CORE_MAX_ITEMS);
  fake_node_core_rx_script_t *script =
      &fake_node_core.rx_scripts[fake_node_core.rx_script_count++];
  *script = (fake_node_core_rx_script_t){
      .error = error,
      .return_at_us = return_at_us,
  };
  make_context(&script->diagnostic, CURAG_OP_RECEIVE, CURAG_RADIO_CONTEXT_V1,
               CURAG_RADIO_CONTEXT_V1_LENGTH);
}

bool fake_node_core_make_ack(uint8_t output[CURA_LORA_V2_ACK_FRAME_SIZE],
                             const uint8_t node_key[CURA_LORA_V2_KEY_SIZE],
                             const uint8_t node_id[8], uint32_t message_id,
                             uint8_t control, cura_lora_v2_domain_t domain,
                             cura_lora_v2_ack_status_t status) {
  cura_lora_v2_ack_t ack = {.status = status};
  uint8_t body[CURA_LORA_V2_ACK_BODY_SIZE];
  if (cura_lora_v2_encode_ack(body, sizeof(body), &ack) !=
      CURA_LORA_V2_CODEC_OK) {
    return false;
  }
  cura_lora_v2_clear_header_t header = {
      .control = control,
      .domain = domain,
      .message_id = message_id,
  };
  memcpy(header.node_id, node_id, sizeof(header.node_id));
  size_t output_length = 0U;
  return cura_lora_v2_seal_frame(output, CURA_LORA_V2_ACK_FRAME_SIZE,
                                 &output_length, node_key, &header, body,
                                 sizeof(body)) == CURA_LORA_V2_CRYPTO_OK &&
         output_length == CURA_LORA_V2_ACK_FRAME_SIZE;
}

size_t fake_node_core_trace_find(fake_node_core_trace_t value, size_t start) {
  for (size_t index = start; index < fake_node_core.trace_count; index++) {
    if (fake_node_core.trace[index] == value) {
      return index;
    }
  }
  return SIZE_MAX;
}

static uint64_t fake_monotonic_us(void *context) {
  (void)context;
  if (fake_node_core.deep_sleep_call_count != 0U) {
    fake_node_core.calls_after_deep_sleep++;
  }
  fake_node_core.clock_call_count++;
  return fake_node_core.now_us;
}

static uint32_t fake_uniform_u32_inclusive(void *context, uint32_t minimum,
                                           uint32_t maximum) {
  (void)context;
  trace_call(FAKE_CORE_TRACE_RANDOM);
  assert(fake_node_core.random_call_count < FAKE_NODE_CORE_MAX_ITEMS);
  const size_t call = fake_node_core.random_call_count++;
  fake_node_core.random_minimums[call] = minimum;
  fake_node_core.random_maximums[call] = maximum;
  uint32_t value = minimum;
  if (fake_node_core.random_value_index < fake_node_core.random_value_count) {
    value = fake_node_core.random_values[fake_node_core.random_value_index++];
  }
  assert(value >= minimum && value <= maximum);
  return value;
}

static uint8_t fake_get_reset_reason(void *context) {
  (void)context;
  trace_call(FAKE_CORE_TRACE_RESET_REASON);
  fake_node_core.reset_reason_call_count++;
  return fake_node_core.reset_reason;
}

static void fake_enter_deep_sleep_for(void *context, uint64_t duration_us) {
  (void)context;
  trace_call(FAKE_CORE_TRACE_DEEP_SLEEP);
  fake_node_core.deep_sleep_call_count++;
  fake_node_core.deep_sleep_duration_us = duration_us;
  if (fake_node_core.observed_rtc != NULL) {
    fake_node_core.rtc_at_sleep = *fake_node_core.observed_rtc;
  }
}

node_platform_ports_t fake_node_core_platform(void) {
  return (node_platform_ports_t){
      .clock = {.context = NULL, .monotonic_us = fake_monotonic_us},
      .randomness = {.context = NULL,
                     .uniform_u32_inclusive = fake_uniform_u32_inclusive},
      .system = {.context = NULL,
                 .get_reset_reason = fake_get_reset_reason,
                 .enter_deep_sleep_for = fake_enter_deep_sleep_for},
  };
}

static err_curag_t queued_error(const err_curag_t *errors, size_t count,
                                size_t *index) {
  if (*index >= count) {
    return CURAG_OK;
  }
  return errors[(*index)++];
}

static void return_diagnostic(err_curag_t error,
                              const diagn_context_t *configured,
                              diagn_context_t *out_diag) {
  curag_diagnostic_context_clear(out_diag);
  if (error != CURAG_OK && out_diag != NULL) {
    *out_diag = *configured;
  }
}

err_curag_t node_persistence_claim_sample_id(uint32_t *out_sample_id,
                                             diagn_context_t *out_diag) {
  trace_call(FAKE_CORE_TRACE_CLAIM_SAMPLE);
  fake_node_core.claim_call_count++;
  if (fake_node_core.observed_rtc != NULL) {
    fake_node_core.rtc_marker_at_claim =
        fake_node_core.observed_rtc->commit_marker;
  }
  advance_time(fake_node_core.claim_advance_us);
  return_diagnostic(fake_node_core.claim_error,
                    &fake_node_core.claim_diagnostic, out_diag);
  if (fake_node_core.claim_error == CURAG_OK) {
    assert(out_sample_id != NULL);
    *out_sample_id = fake_node_core.claimed_sample_id;
  }
  return fake_node_core.claim_error;
}

err_curag_t node_persistence_claim_message_id(uint32_t *out_message_id,
                                              diagn_context_t *out_diag) {
  trace_call(FAKE_CORE_TRACE_CLAIM_MESSAGE);
  fake_node_core.message_claim_call_count++;
  advance_time(fake_node_core.message_claim_advance_us);
  const err_curag_t error =
      queued_error(fake_node_core.message_claim_errors,
                   fake_node_core.message_claim_error_count,
                   &fake_node_core.message_claim_error_index);
  return_diagnostic(error, &fake_node_core.message_claim_diagnostic, out_diag);
  if (error == CURAG_OK) {
    assert(out_message_id != NULL);
    if (fake_node_core.claimed_message_id_index <
        fake_node_core.claimed_message_id_count) {
      *out_message_id =
          fake_node_core
              .claimed_message_ids[fake_node_core.claimed_message_id_index++];
    } else {
      *out_message_id =
          (uint32_t)(UINT32_C(1000) + fake_node_core.message_claim_call_count);
    }
  }
  return error;
}

err_curag_t
node_persistence_append_pending_reading(const cura_lora_v2_reading_t *reading,
                                        diagn_context_t *out_diag) {
  trace_call(FAKE_CORE_TRACE_APPEND_PENDING);
  assert(reading != NULL);
  assert(fake_node_core.appended_count < FAKE_NODE_CORE_MAX_ITEMS);
  fake_node_core.appended[fake_node_core.appended_count++].value.reading =
      *reading;
  advance_time(fake_node_core.append_pending_advance_us);
  return_diagnostic(fake_node_core.append_pending_error,
                    &fake_node_core.append_pending_diagnostic, out_diag);
  if (fake_node_core.append_pending_error == CURAG_OK) {
    fake_node_core_add_pending(reading->sample_id, reading);
  }
  return fake_node_core.append_pending_error;
}

err_curag_t node_persistence_bind_newest_backlog_frame(
    uint32_t expected_sample_id, uint32_t message_id,
    const uint8_t frame[CURA_LORA_V2_READING_FRAME_SIZE],
    diagn_context_t *out_diag) {
  trace_call(FAKE_CORE_TRACE_BIND_BACKLOG);
  fake_node_core.bind_backlog_call_count++;
  const err_curag_t error =
      queued_error(fake_node_core.bind_backlog_errors,
                   fake_node_core.bind_backlog_error_count,
                   &fake_node_core.bind_backlog_error_index);
  return_diagnostic(error, &fake_node_core.persistence_diagnostic, out_diag);
  if (error == CURAG_OK) {
    assert(fake_node_core.pending_count != 0U);
    fake_node_core_pending_t *pending =
        &fake_node_core.pending[fake_node_core.pending_count - 1U];
    assert(pending->value.reading.sample_id == expected_sample_id);
    pending->value.backlog_bound = true;
    pending->value.message_id = message_id;
    memcpy(pending->value.frame, frame, sizeof(pending->value.frame));
  }
  return error;
}

err_curag_t
node_persistence_peek_most_recent_pending(node_pending_reading_t *out_pending,
                                          bool *out_found,
                                          diagn_context_t *out_diag) {
  trace_call(FAKE_CORE_TRACE_PEEK);
  const err_curag_t error =
      queued_error(fake_node_core.peek_errors, fake_node_core.peek_error_count,
                   &fake_node_core.peek_error_index);
  return_diagnostic(error, &fake_node_core.persistence_diagnostic, out_diag);
  if (error != CURAG_OK) {
    return error;
  }
  assert(out_pending != NULL && out_found != NULL);
  if (fake_node_core.pending_count == 0U) {
    *out_found = false;
    return CURAG_OK;
  }
  const fake_node_core_pending_t *pending =
      &fake_node_core.pending[fake_node_core.pending_count - 1U];
  *out_pending = pending->value;
  *out_found = true;
  return CURAG_OK;
}

err_curag_t node_persistence_remove_newest_reading(uint32_t expected_sample_id,
                                                   diagn_context_t *out_diag) {
  trace_call(FAKE_CORE_TRACE_REMOVE);
  assert(fake_node_core.removed_count < FAKE_NODE_CORE_MAX_ITEMS);
  fake_node_core.removed_ids[fake_node_core.removed_count++] =
      expected_sample_id;
  err_curag_t error = queued_error(fake_node_core.remove_errors,
                                   fake_node_core.remove_error_count,
                                   &fake_node_core.remove_error_index);
  if (error == CURAG_OK &&
      (fake_node_core.pending_count == 0U ||
       fake_node_core.pending[fake_node_core.pending_count - 1U]
               .value.reading.sample_id != expected_sample_id)) {
    error = CURAG_ERECORD_MISMATCH;
  }
  return_diagnostic(error, &fake_node_core.persistence_diagnostic, out_diag);
  if (error == CURAG_OK) {
    fake_node_core.pending_count--;
  }
  return error;
}

err_curag_t
node_persistence_quarantine_reading(const cura_lora_v2_reading_t *reading,
                                    diagn_context_t *out_diag) {
  trace_call(FAKE_CORE_TRACE_QUARANTINE);
  assert(reading != NULL);
  assert(fake_node_core.quarantined_count < FAKE_NODE_CORE_MAX_ITEMS);
  fake_node_core.quarantined[fake_node_core.quarantined_count++].value.reading =
      *reading;
  const err_curag_t error = queued_error(
      fake_node_core.quarantine_errors, fake_node_core.quarantine_error_count,
      &fake_node_core.quarantine_error_index);
  return_diagnostic(error, &fake_node_core.persistence_diagnostic, out_diag);
  return error;
}

err_curag_t
node_persistence_append_diagnostic_event(const node_diagnostic_event_t *event,
                                         diagn_context_t *out_diag) {
  trace_call(FAKE_CORE_TRACE_DIAGNOSTIC);
  assert(event != NULL);
  assert(fake_node_core.diagnostic_event_count < FAKE_NODE_CORE_MAX_ITEMS);
  fake_node_core_captured_diagnostic_t *captured =
      &fake_node_core
           .diagnostic_events[fake_node_core.diagnostic_event_count++];
  captured->event = *event;
  captured->event.context = NULL;
  if (event->context != NULL) {
    captured->has_context = true;
    captured->context = *event->context;
  }
  advance_time(fake_node_core.diagnostic_advance_us);
  const err_curag_t error =
      queued_error(fake_node_core.diagnostic_event_errors,
                   fake_node_core.diagnostic_event_error_count,
                   &fake_node_core.diagnostic_event_error_index);
  return_diagnostic(error, &fake_node_core.persistence_diagnostic, out_diag);
  return error;
}

err_curag_t
node_persistence_append_delivery_event(const node_delivery_event_t *event,
                                       diagn_context_t *out_diag) {
  trace_call(FAKE_CORE_TRACE_DELIVERY_EVENT);
  assert(event != NULL);
  assert(fake_node_core.delivery_event_count < FAKE_NODE_CORE_MAX_ITEMS);
  fake_node_core.delivery_events[fake_node_core.delivery_event_count++] =
      *event;
  const err_curag_t error =
      queued_error(fake_node_core.delivery_event_errors,
                   fake_node_core.delivery_event_error_count,
                   &fake_node_core.delivery_event_error_index);
  return_diagnostic(error, &fake_node_core.persistence_diagnostic, out_diag);
  return error;
}

err_curag_t node_persistence_sync_all(diagn_context_t *out_diag) {
  trace_call(FAKE_CORE_TRACE_SYNC);
  fake_node_core.sync_call_count++;
  advance_time(fake_node_core.sync_advance_us);
  return_diagnostic(fake_node_core.sync_error,
                    &fake_node_core.persistence_diagnostic, out_diag);
  return fake_node_core.sync_error;
}

err_curag_t node_sensors_sample_all(node_sensor_sample_t *out_sample,
                                    diagn_context_t *out_diag) {
  trace_call(FAKE_CORE_TRACE_SAMPLE);
  fake_node_core.sample_call_count++;
  advance_time(fake_node_core.sensor_advance_us);
  assert(out_sample != NULL);
  *out_sample = fake_node_core.sensor_sample;
  return_diagnostic(fake_node_core.sensor_error,
                    &fake_node_core.sensor_diagnostic, out_diag);
  return fake_node_core.sensor_error;
}

err_curag_t node_sensors_force_power_off(diagn_context_t *out_diag) {
  trace_call(FAKE_CORE_TRACE_FORCE_POWER_OFF);
  fake_node_core.force_power_off_call_count++;
  advance_time(fake_node_core.force_power_off_advance_us);
  return_diagnostic(fake_node_core.force_power_off_error,
                    &fake_node_core.force_power_off_diagnostic, out_diag);
  return fake_node_core.force_power_off_error;
}

err_curag_t sx1262_radio_transmit_uplink(const uint8_t *payload,
                                         size_t payload_length,
                                         uint64_t deadline_monotonic_us,
                                         sx1262_radio_tx_result_t *out_result,
                                         diagn_context_t *out_diag) {
  trace_call(FAKE_CORE_TRACE_TRANSMIT);
  assert(payload != NULL && out_result != NULL);
  assert(fake_node_core.transmission_count < FAKE_NODE_CORE_MAX_ITEMS);
  assert(payload_length <= CURA_LORA_V2_READING_FRAME_SIZE);
  fake_node_core_captured_tx_t *captured =
      &fake_node_core.transmissions[fake_node_core.transmission_count++];
  memcpy(captured->payload, payload, payload_length);
  captured->payload_length = payload_length;
  captured->deadline_us = deadline_monotonic_us;
  assert(fake_node_core.tx_script_index < fake_node_core.tx_script_count);
  const fake_node_core_tx_script_t *script =
      &fake_node_core.tx_scripts[fake_node_core.tx_script_index++];
  *out_result = script->result;
  advance_to(script->return_at_us);
  return_diagnostic(script->error, &script->diagnostic, out_diag);
  return script->error;
}

err_curag_t
sx1262_radio_receive_downlink_until(uint64_t deadline_monotonic_us,
                                    sx1262_radio_rx_result_t *out_result,
                                    diagn_context_t *out_diag) {
  trace_call(FAKE_CORE_TRACE_RECEIVE);
  assert(out_result != NULL);
  assert(fake_node_core.receive_count < FAKE_NODE_CORE_MAX_ITEMS);
  fake_node_core.receive_deadlines[fake_node_core.receive_count++] =
      deadline_monotonic_us;
  assert(fake_node_core.rx_script_index < fake_node_core.rx_script_count);
  const fake_node_core_rx_script_t *script =
      &fake_node_core.rx_scripts[fake_node_core.rx_script_index++];
  *out_result = script->result;
  advance_to(script->return_at_us);
  return_diagnostic(script->error, &script->diagnostic, out_diag);
  return script->error;
}

err_curag_t sx1262_radio_sleep(diagn_context_t *out_diag) {
  trace_call(FAKE_CORE_TRACE_RADIO_SLEEP);
  fake_node_core.radio_sleep_call_count++;
  advance_time(fake_node_core.radio_sleep_advance_us);
  return_diagnostic(fake_node_core.radio_sleep_error,
                    &fake_node_core.radio_sleep_diagnostic, out_diag);
  return fake_node_core.radio_sleep_error;
}
