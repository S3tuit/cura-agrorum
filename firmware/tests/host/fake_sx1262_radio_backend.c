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
  assert(operation < FAKE_RADIO_OP_COUNT);
  assert(occurrence > 0U);
  assert(detail != NULL);
  g_fake_sx1262_radio.failure[operation].enabled = true;
  g_fake_sx1262_radio.failure[operation].occurrence = occurrence;
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

bool sx1262_radio_backend_set_standby(sx1262_radio_backend_error_t *error) {
  return begin_operation(FAKE_RADIO_OP_STANDBY, error);
}

bool sx1262_radio_backend_set_packet_params(
    uint8_t payload_length, bool inverted_iq,
    sx1262_radio_backend_error_t *error) {
  if (!begin_operation(FAKE_RADIO_OP_PACKET_PARAMS, error)) {
    return false;
  }
  assert(g_fake_sx1262_radio.packet_param_count <
         FAKE_RADIO_MAX_PACKET_PARAM_CALLS);
  const size_t index = g_fake_sx1262_radio.packet_param_count++;
  g_fake_sx1262_radio.packet_param_lengths[index] = payload_length;
  g_fake_sx1262_radio.packet_param_inverted[index] = inverted_iq;
  return true;
}

bool sx1262_radio_backend_configure_irq(uint16_t irq_mask,
                                        sx1262_radio_backend_error_t *error) {
  if (!begin_operation(FAKE_RADIO_OP_CONFIGURE_IRQ, error)) {
    return false;
  }
  g_fake_sx1262_radio.configured_irq_mask = irq_mask;
  return true;
}

bool sx1262_radio_backend_clear_irq(uint16_t irq_mask,
                                    sx1262_radio_backend_error_t *error) {
  (void)irq_mask;
  return begin_operation(FAKE_RADIO_OP_CLEAR_IRQ, error);
}

bool sx1262_radio_backend_write_payload(const uint8_t *payload,
                                        uint8_t payload_length,
                                        sx1262_radio_backend_error_t *error) {
  if (!begin_operation(FAKE_RADIO_OP_WRITE_PAYLOAD, error)) {
    return false;
  }
  assert(payload != NULL);
  memcpy(g_fake_sx1262_radio.transmitted_payload, payload, payload_length);
  g_fake_sx1262_radio.transmitted_payload_length = payload_length;
  return true;
}

bool sx1262_radio_backend_start_tx(uint32_t watchdog_rtc_steps,
                                   sx1262_radio_backend_error_t *error) {
  if (!begin_operation(FAKE_RADIO_OP_START_TX, error)) {
    return false;
  }
  g_fake_sx1262_radio.last_tx_watchdog_steps = watchdog_rtc_steps;
  return true;
}

bool sx1262_radio_backend_start_continuous_rx(
    sx1262_radio_backend_error_t *error) {
  return begin_operation(FAKE_RADIO_OP_START_RX, error);
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
  if (event->at_us > deadline_monotonic_us &&
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

bool sx1262_radio_backend_get_irq(uint16_t *out_irq_status,
                                  sx1262_radio_backend_error_t *error) {
  if (!begin_operation(FAKE_RADIO_OP_GET_IRQ, error)) {
    return false;
  }
  assert(out_irq_status != NULL);
  *out_irq_status = active_event()->irq_status;
  return true;
}

bool sx1262_radio_backend_handle_rx_done(sx1262_radio_backend_error_t *error) {
  return begin_operation(FAKE_RADIO_OP_HANDLE_RX_DONE, error);
}

bool sx1262_radio_backend_get_rx_buffer_status(
    sx1262_radio_backend_rx_buffer_status_t *out_status,
    sx1262_radio_backend_error_t *error) {
  if (!begin_operation(FAKE_RADIO_OP_GET_RX_BUFFER_STATUS, error)) {
    return false;
  }
  assert(out_status != NULL);
  const fake_radio_irq_event_t *const event = active_event();
  out_status->payload_length = event->payload_length;
  out_status->start_offset = event->start_offset;
  return true;
}

bool sx1262_radio_backend_read_buffer(uint8_t offset, uint8_t *output,
                                      uint8_t output_length,
                                      sx1262_radio_backend_error_t *error) {
  if (!begin_operation(FAKE_RADIO_OP_READ_BUFFER, error)) {
    return false;
  }
  assert(output != NULL);
  const fake_radio_irq_event_t *const event = active_event();
  assert(offset == event->start_offset);
  assert((uint16_t)output_length <= event->payload_length);
  memcpy(output, event->payload, output_length);
  return true;
}

bool sx1262_radio_backend_get_packet_status(
    sx1262_radio_backend_packet_status_t *out_status,
    sx1262_radio_backend_error_t *error) {
  if (!begin_operation(FAKE_RADIO_OP_GET_PACKET_STATUS, error)) {
    return false;
  }
  assert(out_status != NULL);
  const fake_radio_irq_event_t *const event = active_event();
  out_status->rssi_dbm_x2 = event->rssi_dbm_x2;
  out_status->snr_db_x4 = event->snr_db_x4;
  return true;
}

bool sx1262_radio_backend_get_device_errors(
    uint16_t *out_device_errors, sx1262_radio_backend_error_t *error) {
  if (!begin_operation(FAKE_RADIO_OP_GET_DEVICE_ERRORS, error)) {
    return false;
  }
  assert(out_device_errors != NULL);
  *out_device_errors = g_fake_sx1262_radio.device_errors;
  return true;
}

bool sx1262_radio_backend_set_sleep_cold(sx1262_radio_backend_error_t *error) {
  return begin_operation(FAKE_RADIO_OP_SLEEP_COLD, error);
}
