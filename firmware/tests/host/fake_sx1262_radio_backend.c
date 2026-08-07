#include "fake_sx1262_radio_backend.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

fake_sx1262_radio_backend_t g_fake_sx1262_radio;

void fake_sx1262_radio_backend_reset(void) {
  memset(&g_fake_sx1262_radio, 0, sizeof(g_fake_sx1262_radio));
  g_fake_sx1262_radio.now_us = UINT64_C(1000);
}

void fake_sx1262_radio_backend_add_irq(const fake_radio_irq_event_t *event) {
  assert(event != NULL);
  assert(g_fake_sx1262_radio.irq_event_count < FAKE_RADIO_MAX_IRQ_EVENTS);
  g_fake_sx1262_radio.irq_events[g_fake_sx1262_radio.irq_event_count++] =
      *event;
}

void fake_sx1262_radio_backend_fail(
    fake_radio_operation_t operation, size_t occurrence,
    const sx1262_radio_backend_error_t *detail) {
  fake_sx1262_radio_backend_set_command_result(operation, occurrence,
                                               SX1262_COMMAND_FAILED, detail);
}

void fake_sx1262_radio_backend_set_command_result(
    fake_radio_operation_t operation, size_t occurrence,
    sx1262_radio_backend_call_result_t result,
    const sx1262_radio_backend_error_t *detail) {
  assert(operation < FAKE_RADIO_OP_COUNT);
  assert(occurrence > 0U);
  assert(result != SX1262_COMMAND_CONFIRMED);
  assert(detail != NULL);
  g_fake_sx1262_radio.failure[operation].enabled = true;
  g_fake_sx1262_radio.failure[operation].occurrence = occurrence;
  g_fake_sx1262_radio.failure[operation].command_result = result;
  g_fake_sx1262_radio.failure[operation].detail = *detail;
}

static bool begin_operation(fake_radio_operation_t operation,
                            sx1262_radio_backend_error_t *error) {
  assert(operation < FAKE_RADIO_OP_COUNT);
  sx1262_radio_backend_error_clear(error);
  ++g_fake_sx1262_radio.calls[operation];
  assert(g_fake_sx1262_radio.trace_length < FAKE_RADIO_MAX_TRACE);
  g_fake_sx1262_radio.trace[g_fake_sx1262_radio.trace_length++] = operation;
  g_fake_sx1262_radio.now_us += g_fake_sx1262_radio.advance_us[operation];
  const fake_radio_failure_t *const failure =
      &g_fake_sx1262_radio.failure[operation];
  if (failure->enabled &&
      failure->occurrence == g_fake_sx1262_radio.calls[operation]) {
    if (error != NULL) {
      *error = failure->detail;
    }
    return false;
  }
  return true;
}

static sx1262_radio_backend_call_result_t
begin_command(fake_radio_operation_t operation,
              sx1262_radio_backend_error_t *error) {
  if (begin_operation(operation, error)) {
    return SX1262_COMMAND_CONFIRMED;
  }
  return g_fake_sx1262_radio.failure[operation].command_result;
}

uint64_t sx1262_radio_backend_monotonic_us(void) {
  return g_fake_sx1262_radio.now_us;
}

bool sx1262_radio_backend_initialize(const sx1262_radio_profile_t *profile,
                                     sx1262_radio_backend_error_t *error) {
  if (!begin_operation(FAKE_RADIO_OP_INITIALIZE, error)) {
    return false;
  }
  assert(profile != NULL);
  g_fake_sx1262_radio.initialized_profile = *profile;
  g_fake_sx1262_radio.initialized_profile_valid = true;
  return true;
}

sx1262_radio_backend_call_result_t
sx1262_radio_backend_set_standby(sx1262_radio_backend_error_t *error) {
  return begin_command(FAKE_RADIO_OP_STANDBY, error);
}

sx1262_radio_backend_call_result_t
sx1262_radio_backend_set_packet_params(uint8_t payload_length, bool inverted_iq,
                                       sx1262_radio_backend_error_t *error) {
  const sx1262_radio_backend_call_result_t result =
      begin_command(FAKE_RADIO_OP_PACKET_PARAMS, error);
  if (result == SX1262_COMMAND_FAILED) {
    return result;
  }
  assert(g_fake_sx1262_radio.packet_param_count <
         FAKE_RADIO_MAX_PACKET_PARAM_CALLS);
  const size_t index = g_fake_sx1262_radio.packet_param_count++;
  g_fake_sx1262_radio.packet_param_lengths[index] = payload_length;
  g_fake_sx1262_radio.packet_param_inverted[index] = inverted_iq;
  return result;
}

sx1262_radio_backend_call_result_t
sx1262_radio_backend_configure_irq(uint16_t irq_mask,
                                   sx1262_radio_backend_error_t *error) {
  const sx1262_radio_backend_call_result_t result =
      begin_command(FAKE_RADIO_OP_CONFIGURE_IRQ, error);
  if (result == SX1262_COMMAND_FAILED) {
    return result;
  }
  g_fake_sx1262_radio.configured_irq_mask = irq_mask;
  return result;
}

sx1262_radio_backend_call_result_t
sx1262_radio_backend_clear_irq(uint16_t irq_mask,
                               sx1262_radio_backend_error_t *error) {
  (void)irq_mask;
  return begin_command(FAKE_RADIO_OP_CLEAR_IRQ, error);
}

sx1262_radio_backend_call_result_t
sx1262_radio_backend_write_payload(const uint8_t *payload,
                                   uint8_t payload_length,
                                   sx1262_radio_backend_error_t *error) {
  const sx1262_radio_backend_call_result_t result =
      begin_command(FAKE_RADIO_OP_WRITE_PAYLOAD, error);
  if (result == SX1262_COMMAND_FAILED) {
    return result;
  }
  assert(payload != NULL);
  memcpy(g_fake_sx1262_radio.transmitted_payload, payload, payload_length);
  g_fake_sx1262_radio.transmitted_payload_length = payload_length;
  return result;
}

sx1262_radio_backend_call_result_t
sx1262_radio_backend_start_tx(uint32_t watchdog_rtc_steps,
                              sx1262_radio_backend_error_t *error) {
  const sx1262_radio_backend_call_result_t result =
      begin_command(FAKE_RADIO_OP_START_TX, error);
  if (result == SX1262_COMMAND_FAILED) {
    return result;
  }
  g_fake_sx1262_radio.last_tx_watchdog_steps = watchdog_rtc_steps;
  return result;
}

sx1262_radio_backend_call_result_t
sx1262_radio_backend_start_single_rx(sx1262_radio_backend_error_t *error) {
  return begin_command(FAKE_RADIO_OP_START_RX, error);
}

bool sx1262_radio_backend_wait_dio1(uint64_t deadline_monotonic_us,
                                    bool *out_observed, uint64_t *out_irq_at_us,
                                    sx1262_radio_backend_error_t *error) {
  if (!begin_operation(FAKE_RADIO_OP_WAIT_DIO1, error)) {
    return false;
  }
  assert(out_observed != NULL);
  assert(out_irq_at_us != NULL);
  *out_observed = false;
  *out_irq_at_us = 0U;
  g_fake_sx1262_radio.active_irq_event_valid = false;
  if (g_fake_sx1262_radio.next_irq_event >=
      g_fake_sx1262_radio.irq_event_count) {
    g_fake_sx1262_radio.now_us = deadline_monotonic_us;
    return true;
  }

  const size_t index = g_fake_sx1262_radio.next_irq_event;
  const fake_radio_irq_event_t *const event =
      &g_fake_sx1262_radio.irq_events[index];
  const bool already_pending = event->at_us <= g_fake_sx1262_radio.now_us;
  if (!already_pending && event->at_us > deadline_monotonic_us &&
      !event->surface_even_after_deadline) {
    g_fake_sx1262_radio.now_us = deadline_monotonic_us;
    return true;
  }
  ++g_fake_sx1262_radio.next_irq_event;
  g_fake_sx1262_radio.active_irq_event = index;
  g_fake_sx1262_radio.active_irq_event_valid = true;
  g_fake_sx1262_radio.now_us = event->at_us;
  *out_observed = true;
  *out_irq_at_us = event->at_us;
  return true;
}

static const fake_radio_irq_event_t *active_event(void) {
  assert(g_fake_sx1262_radio.active_irq_event_valid);
  return &g_fake_sx1262_radio.irq_events[g_fake_sx1262_radio.active_irq_event];
}

sx1262_radio_backend_call_result_t
sx1262_radio_backend_get_irq(uint16_t *out_irq_status,
                             sx1262_radio_backend_error_t *error) {
  const sx1262_radio_backend_call_result_t result =
      begin_command(FAKE_RADIO_OP_GET_IRQ, error);
  if (result != SX1262_COMMAND_CONFIRMED) {
    return result;
  }
  assert(out_irq_status != NULL);
  *out_irq_status = active_event()->irq_status;
  return SX1262_COMMAND_CONFIRMED;
}

sx1262_radio_backend_call_result_t
sx1262_radio_backend_handle_rx_done(sx1262_radio_backend_error_t *error) {
  return begin_command(FAKE_RADIO_OP_HANDLE_RX_DONE, error);
}

sx1262_radio_backend_call_result_t sx1262_radio_backend_get_rx_buffer_status(
    sx1262_radio_backend_rx_buffer_status_t *out_status,
    sx1262_radio_backend_error_t *error) {
  const sx1262_radio_backend_call_result_t result =
      begin_command(FAKE_RADIO_OP_GET_RX_BUFFER_STATUS, error);
  if (result != SX1262_COMMAND_CONFIRMED) {
    return result;
  }
  assert(out_status != NULL);
  const fake_radio_irq_event_t *const event = active_event();
  out_status->payload_length = event->payload_length;
  out_status->start_offset = event->start_offset;
  return SX1262_COMMAND_CONFIRMED;
}

sx1262_radio_backend_call_result_t
sx1262_radio_backend_read_buffer(uint8_t offset, uint8_t *output,
                                 uint8_t output_length,
                                 sx1262_radio_backend_error_t *error) {
  const sx1262_radio_backend_call_result_t result =
      begin_command(FAKE_RADIO_OP_READ_BUFFER, error);
  if (result != SX1262_COMMAND_CONFIRMED) {
    return result;
  }
  assert(output != NULL);
  const fake_radio_irq_event_t *const event = active_event();
  assert(offset == event->start_offset);
  assert((uint16_t)output_length <= event->payload_length);
  memcpy(output, event->payload, output_length);
  return SX1262_COMMAND_CONFIRMED;
}

sx1262_radio_backend_call_result_t sx1262_radio_backend_get_packet_status(
    sx1262_radio_backend_packet_status_t *out_status,
    sx1262_radio_backend_error_t *error) {
  const sx1262_radio_backend_call_result_t result =
      begin_command(FAKE_RADIO_OP_GET_PACKET_STATUS, error);
  if (result != SX1262_COMMAND_CONFIRMED) {
    return result;
  }
  assert(out_status != NULL);
  const fake_radio_irq_event_t *const event = active_event();
  out_status->rssi_dbm_x2 = event->rssi_dbm_x2;
  out_status->snr_db_x4 = event->snr_db_x4;
  return SX1262_COMMAND_CONFIRMED;
}

sx1262_radio_backend_call_result_t
sx1262_radio_backend_get_device_errors(uint16_t *out_device_errors,
                                       sx1262_radio_backend_error_t *error) {
  const sx1262_radio_backend_call_result_t result =
      begin_command(FAKE_RADIO_OP_GET_DEVICE_ERRORS, error);
  if (result != SX1262_COMMAND_CONFIRMED) {
    return result;
  }
  assert(out_device_errors != NULL);
  *out_device_errors = g_fake_sx1262_radio.device_errors;
  return SX1262_COMMAND_CONFIRMED;
}

sx1262_radio_backend_call_result_t
sx1262_radio_backend_set_sleep_cold(sx1262_radio_backend_error_t *error) {
  return begin_command(FAKE_RADIO_OP_SLEEP_COLD, error);
}
