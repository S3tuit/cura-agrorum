#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sx1262_radio_backend.h"

#define FAKE_RADIO_MAX_TRACE 128U
#define FAKE_RADIO_MAX_IRQ_EVENTS 16U
#define FAKE_RADIO_MAX_PACKET_PARAM_CALLS 16U

typedef enum {
  FAKE_RADIO_OP_INITIALIZE = 0,
  FAKE_RADIO_OP_STANDBY,
  FAKE_RADIO_OP_PACKET_PARAMS,
  FAKE_RADIO_OP_CONFIGURE_IRQ,
  FAKE_RADIO_OP_CLEAR_IRQ,
  FAKE_RADIO_OP_WRITE_PAYLOAD,
  FAKE_RADIO_OP_START_TX,
  FAKE_RADIO_OP_START_RX,
  FAKE_RADIO_OP_WAIT_DIO1,
  FAKE_RADIO_OP_GET_IRQ,
  FAKE_RADIO_OP_HANDLE_RX_DONE,
  FAKE_RADIO_OP_GET_RX_BUFFER_STATUS,
  FAKE_RADIO_OP_READ_BUFFER,
  FAKE_RADIO_OP_GET_PACKET_STATUS,
  FAKE_RADIO_OP_GET_DEVICE_ERRORS,
  FAKE_RADIO_OP_SLEEP_COLD,
  FAKE_RADIO_OP_COUNT,
} fake_radio_operation_t;

typedef struct {
  uint64_t at_us;
  uint16_t irq_status;
  uint16_t payload_length;
  uint8_t start_offset;
  int16_t rssi_dbm_x2;
  int16_t snr_db_x4;
  uint8_t payload[SX1262_RADIO_MAX_PAYLOAD_SIZE];
  bool surface_even_after_deadline;
} fake_radio_irq_event_t;

typedef struct {
  bool enabled;
  size_t occurrence;
  sx1262_radio_backend_call_result_t command_result;
  sx1262_radio_backend_error_t detail;
} fake_radio_failure_t;

typedef struct {
  uint64_t now_us;
  size_t calls[FAKE_RADIO_OP_COUNT];
  uint64_t advance_us[FAKE_RADIO_OP_COUNT];
  fake_radio_failure_t failure[FAKE_RADIO_OP_COUNT];
  fake_radio_operation_t trace[FAKE_RADIO_MAX_TRACE];
  size_t trace_length;

  sx1262_radio_profile_t initialized_profile;
  bool initialized_profile_valid;

  uint8_t packet_param_lengths[FAKE_RADIO_MAX_PACKET_PARAM_CALLS];
  bool packet_param_inverted[FAKE_RADIO_MAX_PACKET_PARAM_CALLS];
  size_t packet_param_count;
  uint16_t configured_irq_mask;
  uint32_t last_tx_watchdog_steps;
  uint8_t transmitted_payload[SX1262_RADIO_MAX_PAYLOAD_SIZE];
  uint8_t transmitted_payload_length;

  fake_radio_irq_event_t irq_events[FAKE_RADIO_MAX_IRQ_EVENTS];
  size_t irq_event_count;
  size_t next_irq_event;
  size_t active_irq_event;
  bool active_irq_event_valid;
  uint16_t device_errors;
} fake_sx1262_radio_backend_t;

extern fake_sx1262_radio_backend_t g_fake_sx1262_radio;

void fake_sx1262_radio_backend_reset(void);
void fake_sx1262_radio_backend_add_irq(const fake_radio_irq_event_t *event);
void fake_sx1262_radio_backend_fail(fake_radio_operation_t operation,
                                    size_t occurrence,
                                    const sx1262_radio_backend_error_t *detail);
void fake_sx1262_radio_backend_set_command_result(
    fake_radio_operation_t operation, size_t occurrence,
    sx1262_radio_backend_call_result_t result,
    const sx1262_radio_backend_error_t *detail);
