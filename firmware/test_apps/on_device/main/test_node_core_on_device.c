#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_bit_defs.h"
#include "esp_sleep.h"
#include "node_core.h"
#include "node_core_hardware_test_support.h"
#include "node_persistence.h"
#include "node_platform_esp.h"
#include "persistence_test_support.h"
#include "protocol_v2_lora_schema_generated.h"
#include "unity.h"

#define CORE_HWTEST_CURRENT_START_MS UINT32_C(5)
#define CORE_HWTEST_ACCEPTED_DELIVERY_MS UINT16_C(104)
#define CORE_HWTEST_ACCEPTED_AWAKE_MS UINT16_C(110)
#define CORE_HWTEST_TWO_ATTEMPT_DELIVERY_MS UINT16_C(808)
#define CORE_HWTEST_TWO_ATTEMPT_AWAKE_MS UINT16_C(814)
#define CORE_HWTEST_CURRENT_AND_BACKLOG_AWAKE_MS UINT16_C(216)

static void assert_deep_sleep_boot(void) {
  const node_platform_ports_t *const platform = node_platform_esp_ports();
  TEST_ASSERT_EQUAL_UINT8(
      CURA_LORA_V2_RESET_REASON_ESP_RST_DEEPSLEEP,
      platform->system.get_reset_reason(platform->system.context));
  TEST_ASSERT_EQUAL_HEX32(BIT(ESP_SLEEP_WAKEUP_TIMER),
                          esp_sleep_get_wakeup_causes());
}

static void assert_nvs_successors(uint32_t sample_id, uint32_t message_id) {
  node_persistence_test_reset();
  TEST_ASSERT_EQUAL_UINT32(sample_id, hwtest_read_next_sample_id());
  TEST_ASSERT_EQUAL_UINT32(message_id, hwtest_read_next_message_id());
}

static void assert_pending_empty(void) {
  node_pending_reading_t pending;
  bool found = true;
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(CURAG_OK, node_persistence_peek_most_recent_pending(
                                        &pending, &found, &diag));
  TEST_ASSERT_FALSE(found);
}

static void assert_pending_sample(uint32_t sample_id, bool expected_bound) {
  node_pending_reading_t pending;
  bool found = false;
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(CURAG_OK, node_persistence_peek_most_recent_pending(
                                        &pending, &found, &diag));
  TEST_ASSERT_TRUE(found);
  TEST_ASSERT_EQUAL_UINT32(sample_id, pending.reading.sample_id);
  TEST_ASSERT_EQUAL(expected_bound, pending.backlog_bound);
}

static cura_lora_v2_reading_t
assert_transmission(size_t index, uint32_t sample_id, uint32_t message_id,
                    cura_lora_v2_domain_t domain) {
  cura_lora_v2_clear_header_t header;
  cura_lora_v2_reading_t reading;
  TEST_ASSERT_TRUE(core_hwtest_decode_transmission(index, &header, &reading));
  TEST_ASSERT_EQUAL_HEX8(CURA_LORA_V2_CONTROL, header.control);
  TEST_ASSERT_EQUAL_HEX8(domain, header.domain);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(CORE_HWTEST_IDENTITY.node_id, header.node_id,
                               sizeof(header.node_id));
  TEST_ASSERT_EQUAL_UINT32(message_id, header.message_id);
  TEST_ASSERT_EQUAL_UINT32(sample_id, reading.sample_id);
  return reading;
}

static void assert_sensor_values(const cura_lora_v2_reading_t *reading,
                                 uint16_t marker) {
  TEST_ASSERT_EQUAL_UINT16((uint16_t)(1000U + marker), reading->soil_0_mv);
  TEST_ASSERT_EQUAL_UINT16((uint16_t)(1100U + marker), reading->soil_1_mv);
  TEST_ASSERT_EQUAL_INT16((int16_t)(2000 + marker),
                          reading->soil_temp_0_centi_c);
  TEST_ASSERT_EQUAL_INT16((int16_t)(1900 + marker),
                          reading->soil_temp_1_centi_c);
  TEST_ASSERT_EQUAL_INT16((int16_t)(2200 + marker), reading->enclosure_centi_c);
  TEST_ASSERT_EQUAL_UINT32(UINT32_C(100000) + marker,
                           reading->enclosure_pressure_pa);
  TEST_ASSERT_EQUAL_UINT16((uint16_t)(5000U + marker),
                           reading->enclosure_humidity_centi_pct);
  const uint16_t sensor_flags = CURA_LORA_V2_FLAG_SOIL_0_VALID |
                                CURA_LORA_V2_FLAG_SOIL_1_VALID |
                                CURA_LORA_V2_FLAG_SOIL_TEMP_0_VALID |
                                CURA_LORA_V2_FLAG_SOIL_TEMP_1_VALID |
                                CURA_LORA_V2_FLAG_ENCLOSURE_TEMP_VALID |
                                CURA_LORA_V2_FLAG_ENCLOSURE_PRESSURE_VALID |
                                CURA_LORA_V2_FLAG_ENCLOSURE_HUMIDITY_VALID;
  TEST_ASSERT_EQUAL_HEX16(sensor_flags, reading->flags & sensor_flags);
}

static node_delivery_event_t
started_event(uint32_t cycle_sample_id, uint32_t sample_id, uint32_t message_id,
              cura_lora_v2_domain_t domain, uint32_t start_offset_ms) {
  return (node_delivery_event_t){
      .type = NODE_DELIVERY_EVENT_STARTED,
      .cycle_sample_id = cycle_sample_id,
      .sample_id = sample_id,
      .message_id = message_id,
      .domain = domain,
      .detail.started = {.start_offset_ms = start_offset_ms},
  };
}

static node_delivery_event_t
finished_event(uint32_t cycle_sample_id, uint32_t sample_id,
               uint32_t message_id, cura_lora_v2_domain_t domain,
               uint8_t attempts, node_delivery_final_result_t result) {
  return (node_delivery_event_t){
      .type = NODE_DELIVERY_EVENT_FINISHED,
      .cycle_sample_id = cycle_sample_id,
      .sample_id = sample_id,
      .message_id = message_id,
      .domain = domain,
      .detail.finished = {.attempt_count = attempts, .final_result = result},
  };
}

static void assert_delivery_log(const node_delivery_event_t *events,
                                size_t event_count) {
  TEST_ASSERT_NOT_NULL(events);
  static hwtest_snapshot_t expected;
  static hwtest_snapshot_t actual;
  memset(&expected, 0, sizeof(expected));
  for (size_t index = 0U; index < event_count; ++index) {
    uint8_t encoded[NODE_PERSISTENCE_RECORD_MAX_SIZE];
    const size_t encoded_length =
        hwtest_encode_delivery_record(&events[index], encoded);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(sizeof(expected.bytes) - expected.length,
                                     encoded_length);
    memcpy(expected.bytes + expected.length, encoded, encoded_length);
    expected.length += encoded_length;
  }
  hwtest_snapshot(HWTEST_DELIVERY_PATH, &actual);
  hwtest_assert_snapshot_equal(&expected, &actual);
}

static void assert_accepted_rtc(uint32_t sample_id, uint8_t current_attempts,
                                uint8_t cycle_attempts,
                                uint8_t accepted_readings, uint16_t awake_ms,
                                uint16_t delivery_ms) {
  TEST_ASSERT_EQUAL_HEX32(NODE_RTC_COMMITTED_V1,
                          core_hwtest_rtc_record.commit_marker);
  TEST_ASSERT_EQUAL_UINT32(sample_id,
                           core_hwtest_rtc_record.completed_sample_id);
  TEST_ASSERT_EQUAL_UINT8(current_attempts,
                          core_hwtest_rtc_record.metrics.current_tx_attempts);
  TEST_ASSERT_EQUAL_UINT8(cycle_attempts,
                          core_hwtest_rtc_record.metrics.cycle_tx_attempts);
  TEST_ASSERT_EQUAL_UINT8(accepted_readings,
                          core_hwtest_rtc_record.metrics.accepted_readings);
  TEST_ASSERT_TRUE(core_hwtest_rtc_record.metrics.current_accepted);
  TEST_ASSERT_EQUAL_UINT16(awake_ms, core_hwtest_rtc_record.metrics.awake_ms);
  TEST_ASSERT_EQUAL_UINT16(delivery_ms,
                           core_hwtest_rtc_record.metrics.current_delivery_ms);
}

static void assert_unaccepted_rtc(uint32_t sample_id, uint8_t attempts,
                                  uint16_t awake_ms) {
  TEST_ASSERT_EQUAL_HEX32(NODE_RTC_COMMITTED_V1,
                          core_hwtest_rtc_record.commit_marker);
  TEST_ASSERT_EQUAL_UINT32(sample_id,
                           core_hwtest_rtc_record.completed_sample_id);
  TEST_ASSERT_EQUAL_UINT8(attempts,
                          core_hwtest_rtc_record.metrics.current_tx_attempts);
  TEST_ASSERT_EQUAL_UINT8(attempts,
                          core_hwtest_rtc_record.metrics.cycle_tx_attempts);
  TEST_ASSERT_EQUAL_UINT8(0U, core_hwtest_rtc_record.metrics.accepted_readings);
  TEST_ASSERT_FALSE(core_hwtest_rtc_record.metrics.current_accepted);
  TEST_ASSERT_EQUAL_UINT16(0U,
                           core_hwtest_rtc_record.metrics.current_delivery_ms);
  TEST_ASSERT_EQUAL_UINT16(awake_ms, core_hwtest_rtc_record.metrics.awake_ms);
}

static void current_accepted_stage_1(void) {
  core_hwtest_begin_case();
  const node_sensor_sample_t sensor = core_hwtest_sensor_sample(1U);
  const core_hwtest_response_t responses[] = {
      CORE_HWTEST_RESPONSE_ACCEPTED,
  };
  core_hwtest_run_cycle(&sensor, responses, 1U);
}

static void current_accepted_stage_2(void) {
  assert_deep_sleep_boot();
  TEST_ASSERT_EQUAL_UINT32(1U, core_hwtest_retained.transmission_count);
  const cura_lora_v2_reading_t reading = assert_transmission(
      0U, 0U, 0U, CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK);
  assert_sensor_values(&reading, 1U);
  assert_pending_empty();
  assert_accepted_rtc(0U, 1U, 1U, 1U, CORE_HWTEST_ACCEPTED_AWAKE_MS,
                      CORE_HWTEST_ACCEPTED_DELIVERY_MS);
  core_hwtest_assert_adapter_counts(1U, 1U);
  assert_nvs_successors(1U, 1U);
  const node_delivery_event_t events[] = {
      started_event(0U, 0U, 0U, CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK,
                    CORE_HWTEST_CURRENT_START_MS),
      finished_event(0U, 0U, 0U, CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK, 1U,
                     NODE_DELIVERY_RESULT_ACCEPTED),
  };
  assert_delivery_log(events, 2U);
  core_hwtest_finish_case();
}

TEST_CASE_MULTIPLE_STAGES("node core accepts current reading across deep sleep",
                          "[node_core][reset=DEEPSLEEP_RESET]",
                          current_accepted_stage_1, current_accepted_stage_2);

static void backlog_stage_1(void) {
  core_hwtest_begin_case();
  const node_sensor_sample_t sensor = core_hwtest_sensor_sample(2U);
  const core_hwtest_response_t responses[] = {
      CORE_HWTEST_RESPONSE_SILENCE,
  };
  core_hwtest_run_cycle(&sensor, responses, 1U);
}

static void backlog_stage_2(void) {
  assert_deep_sleep_boot();
  TEST_ASSERT_GREATER_THAN_UINT32(1U, core_hwtest_retained.transmission_count);
  core_hwtest_retained.checkpoint_transmission_count =
      core_hwtest_retained.transmission_count;
  assert_pending_sample(0U, false);
  assert_unaccepted_rtc(
      0U, (uint8_t)core_hwtest_retained.checkpoint_transmission_count,
      UINT16_C(30000));

  const node_sensor_sample_t sensor = core_hwtest_sensor_sample(3U);
  const core_hwtest_response_t responses[] = {
      CORE_HWTEST_RESPONSE_ACCEPTED,
      CORE_HWTEST_RESPONSE_ACCEPTED,
  };
  core_hwtest_run_cycle(&sensor, responses, 2U);
}

static void backlog_stage_3(void) {
  assert_deep_sleep_boot();
  const size_t checkpoint =
      (size_t)core_hwtest_retained.checkpoint_transmission_count;
  TEST_ASSERT_EQUAL_UINT32(checkpoint + 2U,
                           core_hwtest_retained.transmission_count);
  for (size_t index = 0U; index < checkpoint; ++index) {
    (void)assert_transmission(index, 0U, 0U,
                              CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(
        core_hwtest_retained.transmissions[0].bytes,
        core_hwtest_retained.transmissions[index].bytes,
        CURA_LORA_V2_READING_FRAME_SIZE);
  }
  const cura_lora_v2_reading_t current = assert_transmission(
      checkpoint, 1U, 1U, CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK);
  assert_sensor_values(&current, 3U);
  (void)assert_transmission(checkpoint + 1U, 0U, 2U,
                            CURA_LORA_V2_DOMAIN_BACKLOG_READING_UPLINK);
  TEST_ASSERT_TRUE(core_hwtest_retained.backlog_binding_verified_before_tx);
  assert_pending_empty();
  assert_accepted_rtc(1U, 1U, 2U, 2U, CORE_HWTEST_CURRENT_AND_BACKLOG_AWAKE_MS,
                      CORE_HWTEST_ACCEPTED_DELIVERY_MS);
  core_hwtest_assert_adapter_counts(2U, 2U);
  assert_nvs_successors(2U, 3U);

  const uint8_t first_attempts = (uint8_t)checkpoint;
  const node_delivery_event_t events[] = {
      started_event(0U, 0U, 0U, CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK,
                    CORE_HWTEST_CURRENT_START_MS),
      finished_event(0U, 0U, 0U, CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK,
                     first_attempts, NODE_DELIVERY_RESULT_RADIO_CYCLE_DEADLINE),
      started_event(1U, 1U, 1U, CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK,
                    CORE_HWTEST_CURRENT_START_MS),
      finished_event(1U, 1U, 1U, CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK, 1U,
                     NODE_DELIVERY_RESULT_ACCEPTED),
      started_event(1U, 0U, 2U, CURA_LORA_V2_DOMAIN_BACKLOG_READING_UPLINK,
                    UINT32_C(110)),
      finished_event(1U, 0U, 2U, CURA_LORA_V2_DOMAIN_BACKLOG_READING_UPLINK, 1U,
                     NODE_DELIVERY_RESULT_ACCEPTED),
  };
  assert_delivery_log(events, 6U);
  core_hwtest_finish_case();
}

TEST_CASE_MULTIPLE_STAGES(
    "node core converts unacknowledged current to drained backlog",
    "[node_core][reset=DEEPSLEEP_RESET,DEEPSLEEP_RESET]", backlog_stage_1,
    backlog_stage_2, backlog_stage_3);

static void rejected_stage_1(void) {
  core_hwtest_begin_case();
  const node_sensor_sample_t sensor = core_hwtest_sensor_sample(4U);
  const core_hwtest_response_t responses[] = {
      CORE_HWTEST_RESPONSE_REJECTED_MALFORMED,
  };
  core_hwtest_run_cycle(&sensor, responses, 1U);
}

static void rejected_stage_2(void) {
  assert_deep_sleep_boot();
  TEST_ASSERT_EQUAL_UINT32(1U, core_hwtest_retained.transmission_count);
  const cura_lora_v2_reading_t reading = assert_transmission(
      0U, 0U, 0U, CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK);
  assert_sensor_values(&reading, 4U);
  assert_pending_empty();
  assert_unaccepted_rtc(0U, 1U, CORE_HWTEST_ACCEPTED_AWAKE_MS);
  core_hwtest_assert_adapter_counts(1U, 1U);
  assert_nvs_successors(1U, 1U);

  static hwtest_snapshot_t expected_quarantine;
  static hwtest_snapshot_t actual_quarantine;
  expected_quarantine.length = hwtest_encode_reading_record(
      NODE_PERSISTENCE_RECORD_TYPE_QUARANTINED_READING, 0U, &reading,
      expected_quarantine.bytes);
  hwtest_snapshot(HWTEST_QUARANTINE_PATH, &actual_quarantine);
  hwtest_assert_snapshot_equal(&expected_quarantine, &actual_quarantine);
  const node_delivery_event_t events[] = {
      started_event(0U, 0U, 0U, CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK,
                    CORE_HWTEST_CURRENT_START_MS),
      finished_event(0U, 0U, 0U, CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK, 1U,
                     NODE_DELIVERY_RESULT_MALFORMED),
  };
  assert_delivery_log(events, 2U);
  core_hwtest_finish_case();
}

TEST_CASE_MULTIPLE_STAGES("node core quarantines permanently rejected current",
                          "[node_core][reset=DEEPSLEEP_RESET]",
                          rejected_stage_1, rejected_stage_2);

static void retry_later_stage_1(void) {
  core_hwtest_begin_case();
  const cura_lora_v2_reading_t older = hwtest_make_reading(0U);
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(
      CURAG_OK, node_persistence_append_pending_reading(&older, &diag));
  TEST_ASSERT_EQUAL_HEX32(CURAG_OK, node_persistence_sync_all(&diag));
  hwtest_simulate_component_restart();
  hwtest_seed_next_sample_id(1U);

  const node_sensor_sample_t sensor = core_hwtest_sensor_sample(5U);
  const core_hwtest_response_t responses[] = {
      CORE_HWTEST_RESPONSE_RETRY_LATER,
  };
  core_hwtest_run_cycle(&sensor, responses, 1U);
}

static void retry_later_stage_2(void) {
  assert_deep_sleep_boot();
  TEST_ASSERT_EQUAL_UINT32(1U, core_hwtest_retained.transmission_count);
  const cura_lora_v2_reading_t current = assert_transmission(
      0U, 1U, 0U, CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK);
  assert_sensor_values(&current, 5U);
  assert_unaccepted_rtc(1U, 1U, CORE_HWTEST_ACCEPTED_AWAKE_MS);
  core_hwtest_assert_adapter_counts(1U, 1U);
  assert_nvs_successors(2U, 1U);

  static hwtest_snapshot_t expected_pending;
  static hwtest_snapshot_t actual_pending;
  const cura_lora_v2_reading_t older = hwtest_make_reading(0U);
  expected_pending.length =
      hwtest_encode_reading_record(NODE_PERSISTENCE_RECORD_TYPE_PENDING_READING,
                                   0U, &older, expected_pending.bytes);
  expected_pending.length += hwtest_encode_reading_record(
      NODE_PERSISTENCE_RECORD_TYPE_PENDING_READING, 1U, &current,
      expected_pending.bytes + expected_pending.length);
  hwtest_snapshot(HWTEST_PENDING_PATH, &actual_pending);
  hwtest_assert_snapshot_equal(&expected_pending, &actual_pending);

  const node_delivery_event_t events[] = {
      started_event(1U, 1U, 0U, CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK,
                    CORE_HWTEST_CURRENT_START_MS),
      finished_event(1U, 1U, 0U, CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK, 1U,
                     NODE_DELIVERY_RESULT_RETRY_LATER),
  };
  assert_delivery_log(events, 2U);
  core_hwtest_finish_case();
}

TEST_CASE_MULTIPLE_STAGES("node core retry later retains current and backlog",
                          "[node_core][reset=DEEPSLEEP_RESET]",
                          retry_later_stage_1, retry_later_stage_2);

static void previous_metrics_stage_1(void) {
  core_hwtest_begin_case();
  const node_sensor_sample_t sensor = core_hwtest_sensor_sample(6U);
  const core_hwtest_response_t responses[] = {
      CORE_HWTEST_RESPONSE_SILENCE,
      CORE_HWTEST_RESPONSE_ACCEPTED,
  };
  core_hwtest_run_cycle(&sensor, responses, 2U);
}

static void previous_metrics_stage_2(void) {
  assert_deep_sleep_boot();
  TEST_ASSERT_EQUAL_UINT32(2U, core_hwtest_retained.transmission_count);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(core_hwtest_retained.transmissions[0].bytes,
                               core_hwtest_retained.transmissions[1].bytes,
                               CURA_LORA_V2_READING_FRAME_SIZE);
  assert_accepted_rtc(0U, 2U, 2U, 1U, CORE_HWTEST_TWO_ATTEMPT_AWAKE_MS,
                      CORE_HWTEST_TWO_ATTEMPT_DELIVERY_MS);
  core_hwtest_retained.checkpoint_transmission_count = 2U;

  const node_sensor_sample_t sensor = core_hwtest_sensor_sample(7U);
  const core_hwtest_response_t responses[] = {
      CORE_HWTEST_RESPONSE_ACCEPTED,
  };
  core_hwtest_run_cycle(&sensor, responses, 1U);
}

static void previous_metrics_stage_3(void) {
  assert_deep_sleep_boot();
  TEST_ASSERT_EQUAL_UINT32(3U, core_hwtest_retained.transmission_count);
  const cura_lora_v2_reading_t reading = assert_transmission(
      2U, 1U, 1U, CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK);
  assert_sensor_values(&reading, 7U);
  TEST_ASSERT_EQUAL_UINT8(2U, reading.previous_current_tx_attempts);
  TEST_ASSERT_EQUAL_UINT16(CORE_HWTEST_TWO_ATTEMPT_AWAKE_MS,
                           reading.previous_awake_ms);
  TEST_ASSERT_EQUAL_UINT16(CORE_HWTEST_TWO_ATTEMPT_DELIVERY_MS,
                           reading.previous_current_delivery_ms);
  TEST_ASSERT_EQUAL_UINT8(2U, reading.previous_cycle_tx_attempts);
  TEST_ASSERT_EQUAL_UINT8(1U, reading.previous_cycle_accepted_readings);
  TEST_ASSERT_BITS_HIGH(CURA_LORA_V2_FLAG_PREVIOUS_CYCLE_METRICS_VALID |
                            CURA_LORA_V2_FLAG_PREVIOUS_CURRENT_ACCEPTED |
                            CURA_LORA_V2_FLAG_DEEP_SLEEP_BOOT,
                        reading.flags);
  assert_pending_empty();
  assert_accepted_rtc(1U, 1U, 1U, 1U, CORE_HWTEST_ACCEPTED_AWAKE_MS,
                      CORE_HWTEST_ACCEPTED_DELIVERY_MS);
  core_hwtest_assert_adapter_counts(2U, 2U);
  assert_nvs_successors(2U, 2U);

  const node_delivery_event_t events[] = {
      started_event(0U, 0U, 0U, CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK,
                    CORE_HWTEST_CURRENT_START_MS),
      finished_event(0U, 0U, 0U, CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK, 2U,
                     NODE_DELIVERY_RESULT_ACCEPTED),
      started_event(1U, 1U, 1U, CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK,
                    CORE_HWTEST_CURRENT_START_MS),
      finished_event(1U, 1U, 1U, CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK, 1U,
                     NODE_DELIVERY_RESULT_ACCEPTED),
  };
  assert_delivery_log(events, 4U);
  core_hwtest_finish_case();
}

TEST_CASE_MULTIPLE_STAGES(
    "node core carries previous metrics into the next reading",
    "[node_core][reset=DEEPSLEEP_RESET,DEEPSLEEP_RESET]",
    previous_metrics_stage_1, previous_metrics_stage_2,
    previous_metrics_stage_3);

static void restart_after_consumption_stage_1(void) {
  core_hwtest_begin_case();
  const node_sensor_sample_t sensor = core_hwtest_sensor_sample(8U);
  const core_hwtest_response_t responses[] = {
      CORE_HWTEST_RESPONSE_ACCEPTED,
  };
  core_hwtest_run_cycle(&sensor, responses, 1U);
}

static void restart_after_consumption_stage_2(void) {
  assert_deep_sleep_boot();
  assert_accepted_rtc(0U, 1U, 1U, 1U, CORE_HWTEST_ACCEPTED_AWAKE_MS,
                      CORE_HWTEST_ACCEPTED_DELIVERY_MS);
  TEST_ASSERT_EQUAL_UINT32(1U, core_hwtest_retained.transmission_count);
  const cura_lora_v2_reading_t previous = assert_transmission(
      0U, 0U, 0U, CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK);
  assert_sensor_values(&previous, 8U);
  core_hwtest_arm_restart_after_rtc_take(0U);
  const node_sensor_sample_t sensor = core_hwtest_sensor_sample(9U);
  const core_hwtest_response_t responses[] = {
      CORE_HWTEST_RESPONSE_ACCEPTED,
  };
  core_hwtest_run_cycle(&sensor, responses, 1U);
}

static void restart_after_consumption_stage_3(void) {
  const node_platform_ports_t *const production = node_platform_esp_ports();
  TEST_ASSERT_EQUAL_UINT8(
      CURA_LORA_V2_RESET_REASON_ESP_RST_SW,
      production->system.get_reset_reason(production->system.context));
  TEST_ASSERT_TRUE(core_hwtest_restart_evidence.restart_hook_fired);
  TEST_ASSERT_TRUE(core_hwtest_restart_evidence.hook_saw_committed_copy);
  TEST_ASSERT_TRUE(core_hwtest_restart_evidence.hook_saw_invalid_retained);
  TEST_ASSERT_FALSE(core_hwtest_restart_evidence.hook_contract_failed);
  TEST_ASSERT_EQUAL_HEX32(0U, core_hwtest_rtc_record.commit_marker);
  assert_nvs_successors(1U, 1U);

  const node_sensor_sample_t sensor = core_hwtest_sensor_sample(10U);
  const core_hwtest_response_t responses[] = {
      CORE_HWTEST_RESPONSE_ACCEPTED,
  };
  core_hwtest_run_cycle(&sensor, responses, 1U);
}

static void restart_after_consumption_stage_4(void) {
  assert_deep_sleep_boot();
  TEST_ASSERT_EQUAL_UINT32(1U, core_hwtest_retained.transmission_count);
  const cura_lora_v2_reading_t reading = assert_transmission(
      0U, 1U, 1U, CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK);
  assert_sensor_values(&reading, 10U);
  TEST_ASSERT_EQUAL_UINT8(CURA_LORA_V2_RESET_REASON_ESP_RST_SW,
                          reading.reset_reason);
  TEST_ASSERT_EQUAL_UINT8(0U, reading.previous_current_tx_attempts);
  TEST_ASSERT_EQUAL_UINT16(0U, reading.previous_awake_ms);
  TEST_ASSERT_EQUAL_UINT16(0U, reading.previous_current_delivery_ms);
  TEST_ASSERT_EQUAL_UINT8(0U, reading.previous_cycle_tx_attempts);
  TEST_ASSERT_EQUAL_UINT8(0U, reading.previous_cycle_accepted_readings);
  TEST_ASSERT_EQUAL_HEX16(
      0U, reading.flags & (CURA_LORA_V2_FLAG_PREVIOUS_CYCLE_METRICS_VALID |
                           CURA_LORA_V2_FLAG_PREVIOUS_CURRENT_ACCEPTED));
  assert_pending_empty();
  assert_accepted_rtc(1U, 1U, 1U, 1U, CORE_HWTEST_ACCEPTED_AWAKE_MS,
                      CORE_HWTEST_ACCEPTED_DELIVERY_MS);
  core_hwtest_assert_adapter_counts(1U, 1U);
  TEST_ASSERT_EQUAL_UINT32(3U, core_hwtest_restart_evidence.hook_call_count);
  assert_nvs_successors(2U, 2U);

  const node_delivery_event_t events[] = {
      started_event(0U, 0U, 0U, CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK,
                    CORE_HWTEST_CURRENT_START_MS),
      finished_event(0U, 0U, 0U, CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK, 1U,
                     NODE_DELIVERY_RESULT_ACCEPTED),
      started_event(1U, 1U, 1U, CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK,
                    CORE_HWTEST_CURRENT_START_MS),
      finished_event(1U, 1U, 1U, CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK, 1U,
                     NODE_DELIVERY_RESULT_ACCEPTED),
  };
  assert_delivery_log(events, 4U);
  core_hwtest_finish_case();
}

TEST_CASE_MULTIPLE_STAGES(
    "node core restart after RTC consumption does not reuse metrics",
    "[node_core][reset=DEEPSLEEP_RESET,SW_CPU_RESET,DEEPSLEEP_RESET]",
    restart_after_consumption_stage_1, restart_after_consumption_stage_2,
    restart_after_consumption_stage_3, restart_after_consumption_stage_4);
