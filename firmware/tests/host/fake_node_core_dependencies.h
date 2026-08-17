#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "node_core.h"
#include "node_persistence.h"
#include "node_sensors.h"
#include "sx1262_radio.h"

#define FAKE_NODE_CORE_MAX_ITEMS 256U
#define FAKE_NODE_CORE_MAX_PENDING 128U
#define FAKE_NODE_CORE_MAX_TRACE 1024U

typedef enum {
  FAKE_CORE_TRACE_RESET_REASON = 1,
  FAKE_CORE_TRACE_CLAIM_SAMPLE,
  FAKE_CORE_TRACE_CLAIM_MESSAGE,
  FAKE_CORE_TRACE_DIAGNOSTIC,
  FAKE_CORE_TRACE_SAMPLE,
  FAKE_CORE_TRACE_APPEND_PENDING,
  FAKE_CORE_TRACE_BIND_BACKLOG,
  FAKE_CORE_TRACE_DELIVERY_EVENT,
  FAKE_CORE_TRACE_TRANSMIT,
  FAKE_CORE_TRACE_RANDOM,
  FAKE_CORE_TRACE_RECEIVE,
  FAKE_CORE_TRACE_PEEK,
  FAKE_CORE_TRACE_REMOVE,
  FAKE_CORE_TRACE_QUARANTINE,
  FAKE_CORE_TRACE_FORCE_POWER_OFF,
  FAKE_CORE_TRACE_RADIO_SLEEP,
  FAKE_CORE_TRACE_SYNC,
  FAKE_CORE_TRACE_DEEP_SLEEP,
} fake_node_core_trace_t;

typedef struct {
  node_pending_reading_t value;
} fake_node_core_pending_t;

typedef struct {
  err_curag_t error;
  sx1262_radio_tx_result_t result;
  diagn_context_t diagnostic;
  uint64_t return_at_us;
} fake_node_core_tx_script_t;

typedef struct {
  err_curag_t error;
  sx1262_radio_rx_result_t result;
  diagn_context_t diagnostic;
  uint64_t return_at_us;
} fake_node_core_rx_script_t;

typedef struct {
  node_diagnostic_event_t event;
  bool has_context;
  diagn_context_t context;
} fake_node_core_captured_diagnostic_t;

typedef struct {
  uint8_t payload[CURA_LORA_V2_READING_FRAME_SIZE];
  size_t payload_length;
  uint64_t deadline_us;
} fake_node_core_captured_tx_t;

typedef struct {
  uint64_t now_us;
  uint8_t reset_reason;
  node_rtc_record_t *observed_rtc;
  uint32_t rtc_marker_at_claim;
  node_rtc_record_t rtc_at_sleep;

  uint32_t claimed_sample_id;
  err_curag_t claim_error;
  diagn_context_t claim_diagnostic;
  uint64_t claim_advance_us;

  uint32_t claimed_message_ids[FAKE_NODE_CORE_MAX_ITEMS];
  size_t claimed_message_id_count;
  size_t claimed_message_id_index;
  err_curag_t message_claim_errors[FAKE_NODE_CORE_MAX_ITEMS];
  size_t message_claim_error_count;
  size_t message_claim_error_index;
  diagn_context_t message_claim_diagnostic;
  uint64_t message_claim_advance_us;

  node_sensor_sample_t sensor_sample;
  err_curag_t sensor_error;
  diagn_context_t sensor_diagnostic;
  uint64_t sensor_advance_us;
  err_curag_t force_power_off_error;
  diagn_context_t force_power_off_diagnostic;
  uint64_t force_power_off_advance_us;

  err_curag_t append_pending_error;
  diagn_context_t append_pending_diagnostic;
  uint64_t append_pending_advance_us;
  err_curag_t bind_backlog_errors[FAKE_NODE_CORE_MAX_ITEMS];
  size_t bind_backlog_error_count;
  size_t bind_backlog_error_index;
  err_curag_t peek_errors[FAKE_NODE_CORE_MAX_ITEMS];
  size_t peek_error_count;
  size_t peek_error_index;
  err_curag_t remove_errors[FAKE_NODE_CORE_MAX_ITEMS];
  size_t remove_error_count;
  size_t remove_error_index;
  err_curag_t quarantine_errors[FAKE_NODE_CORE_MAX_ITEMS];
  size_t quarantine_error_count;
  size_t quarantine_error_index;
  err_curag_t delivery_event_errors[FAKE_NODE_CORE_MAX_ITEMS];
  size_t delivery_event_error_count;
  size_t delivery_event_error_index;
  err_curag_t diagnostic_event_errors[FAKE_NODE_CORE_MAX_ITEMS];
  size_t diagnostic_event_error_count;
  size_t diagnostic_event_error_index;
  err_curag_t sync_error;
  diagn_context_t persistence_diagnostic;
  uint64_t diagnostic_advance_us;
  uint64_t sync_advance_us;

  fake_node_core_pending_t pending[FAKE_NODE_CORE_MAX_PENDING];
  size_t pending_count;
  fake_node_core_pending_t appended[FAKE_NODE_CORE_MAX_ITEMS];
  size_t appended_count;
  uint32_t removed_ids[FAKE_NODE_CORE_MAX_ITEMS];
  size_t removed_count;
  fake_node_core_pending_t quarantined[FAKE_NODE_CORE_MAX_ITEMS];
  size_t quarantined_count;
  node_delivery_event_t delivery_events[FAKE_NODE_CORE_MAX_ITEMS];
  size_t delivery_event_count;
  fake_node_core_captured_diagnostic_t
      diagnostic_events[FAKE_NODE_CORE_MAX_ITEMS];
  size_t diagnostic_event_count;

  fake_node_core_tx_script_t tx_scripts[FAKE_NODE_CORE_MAX_ITEMS];
  size_t tx_script_count;
  size_t tx_script_index;
  fake_node_core_rx_script_t rx_scripts[FAKE_NODE_CORE_MAX_ITEMS];
  size_t rx_script_count;
  size_t rx_script_index;
  fake_node_core_captured_tx_t transmissions[FAKE_NODE_CORE_MAX_ITEMS];
  size_t transmission_count;
  uint64_t receive_deadlines[FAKE_NODE_CORE_MAX_ITEMS];
  size_t receive_count;
  err_curag_t radio_sleep_error;
  diagn_context_t radio_sleep_diagnostic;
  uint64_t radio_sleep_advance_us;

  uint32_t random_values[FAKE_NODE_CORE_MAX_ITEMS];
  size_t random_value_count;
  size_t random_value_index;
  uint32_t random_minimums[FAKE_NODE_CORE_MAX_ITEMS];
  uint32_t random_maximums[FAKE_NODE_CORE_MAX_ITEMS];
  size_t random_call_count;

  fake_node_core_trace_t trace[FAKE_NODE_CORE_MAX_TRACE];
  size_t trace_count;
  size_t clock_call_count;
  size_t reset_reason_call_count;
  size_t claim_call_count;
  size_t message_claim_call_count;
  size_t bind_backlog_call_count;
  size_t sample_call_count;
  size_t force_power_off_call_count;
  size_t radio_sleep_call_count;
  size_t sync_call_count;
  size_t deep_sleep_call_count;
  uint64_t deep_sleep_duration_us;
  size_t calls_after_deep_sleep;
} fake_node_core_state_t;

extern fake_node_core_state_t fake_node_core;

void fake_node_core_reset(void);
node_platform_ports_t fake_node_core_platform(void);
void fake_node_core_advance_us(uint64_t amount_us);
void fake_node_core_add_pending(uint32_t sample_id,
                                const cura_lora_v2_reading_t *reading);
void fake_node_core_add_bound_pending(
    uint32_t message_id, const cura_lora_v2_reading_t *reading,
    const uint8_t frame[CURA_LORA_V2_READING_FRAME_SIZE]);
void fake_node_core_script_tx_done(uint64_t set_tx_at_us,
                                   uint64_t tx_done_at_us);
void fake_node_core_script_tx_error(err_curag_t error, bool tx_started,
                                    bool tx_done, uint64_t set_tx_at_us,
                                    uint64_t tx_done_at_us,
                                    uint64_t return_at_us);
void fake_node_core_script_rx_deadline(uint64_t deadline_us);
void fake_node_core_script_rx_packet(const uint8_t *payload,
                                     size_t payload_length,
                                     uint64_t rx_done_at_us,
                                     uint64_t return_at_us);
void fake_node_core_script_rx_error(err_curag_t error, uint64_t return_at_us);
bool fake_node_core_make_ack(uint8_t output[CURA_LORA_V2_ACK_FRAME_SIZE],
                             const uint8_t node_key[CURA_LORA_V2_KEY_SIZE],
                             const uint8_t node_id[8], uint32_t message_id,
                             uint8_t control, cura_lora_v2_domain_t domain,
                             cura_lora_v2_ack_status_t status);
size_t fake_node_core_trace_find(fake_node_core_trace_t value, size_t start);
