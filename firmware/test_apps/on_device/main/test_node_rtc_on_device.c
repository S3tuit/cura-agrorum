#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_bit_defs.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "node_core.h"
#include "node_persistence.h"
#include "node_platform_esp.h"
#include "persistence_test_support.h"
#include "protocol_v2_lora_schema_generated.h"
#include "unity.h"

#define RTC_HWTEST_DEEP_SLEEP_DURATION_US UINT64_C(250000)
#define RTC_HWTEST_ROUND_TRIPS 20U

RTC_DATA_ATTR static node_rtc_record_t s_rtc_record;
RTC_DATA_ATTR static uint32_t s_round_trip_iteration;

static node_cycle_metrics_t rtc_metrics(uint32_t marker) {
  return (node_cycle_metrics_t){
      .current_tx_attempts = (uint8_t)(marker % 3U + 1U),
      .awake_ms = (uint16_t)(300U + marker),
      .current_delivery_ms = (uint16_t)(80U + marker),
      .cycle_tx_attempts = (uint8_t)(marker % 3U + 3U),
      .accepted_readings = 1U,
      .current_accepted = true,
  };
}

static void assert_metrics_equal(const node_cycle_metrics_t *expected,
                                 const node_cycle_metrics_t *actual) {
  TEST_ASSERT_NOT_NULL(expected);
  TEST_ASSERT_NOT_NULL(actual);
  TEST_ASSERT_EQUAL_UINT8(expected->current_tx_attempts,
                          actual->current_tx_attempts);
  TEST_ASSERT_EQUAL_UINT16(expected->awake_ms, actual->awake_ms);
  TEST_ASSERT_EQUAL_UINT16(expected->current_delivery_ms,
                           actual->current_delivery_ms);
  TEST_ASSERT_EQUAL_UINT8(expected->cycle_tx_attempts,
                          actual->cycle_tx_attempts);
  TEST_ASSERT_EQUAL_UINT8(expected->accepted_readings,
                          actual->accepted_readings);
  TEST_ASSERT_EQUAL(expected->current_accepted, actual->current_accepted);
}

static void assert_record(uint32_t expected_sample_id,
                          const node_cycle_metrics_t *expected_metrics) {
  TEST_ASSERT_EQUAL_HEX32(NODE_RTC_COMMITTED_V1, s_rtc_record.commit_marker);
  TEST_ASSERT_EQUAL_UINT32(expected_sample_id,
                           s_rtc_record.completed_sample_id);
  assert_metrics_equal(expected_metrics, &s_rtc_record.metrics);
}

static void assert_deep_sleep_boot(void) {
  const node_platform_ports_t *const platform = node_platform_esp_ports();
  TEST_ASSERT_EQUAL_UINT8(
      CURA_LORA_V2_RESET_REASON_ESP_RST_DEEPSLEEP,
      platform->system.get_reset_reason(platform->system.context));
  TEST_ASSERT_EQUAL_HEX32(BIT(ESP_SLEEP_WAKEUP_TIMER),
                          esp_sleep_get_wakeup_causes());
}

static void rtc_case_begin(void) {
  hwtest_erase_state();
  memset(&s_rtc_record, 0, sizeof(s_rtc_record));
  s_round_trip_iteration = 0U;
}

static void rtc_case_finish(void) {
  memset(&s_rtc_record, 0, sizeof(s_rtc_record));
  s_round_trip_iteration = 0U;
  hwtest_finish_case();
}

static void enter_short_deep_sleep(void) {
  const node_platform_ports_t *const platform = node_platform_esp_ports();
  platform->system.enter_deep_sleep_for(platform->system.context,
                                        RTC_HWTEST_DEEP_SLEEP_DURATION_US);
  TEST_FAIL_MESSAGE("deep-sleep adapter returned");
}

static void rtc_survives_stage_1(void) {
  rtc_case_begin();
  const node_cycle_metrics_t metrics = rtc_metrics(7U);
  TEST_ASSERT_TRUE(
      node_rtc_record_commit(&s_rtc_record, UINT32_C(41), &metrics));
  enter_short_deep_sleep();
}

static void rtc_survives_stage_2(void) {
  assert_deep_sleep_boot();
  const node_cycle_metrics_t metrics = rtc_metrics(7U);
  assert_record(UINT32_C(41), &metrics);
  rtc_case_finish();
}

TEST_CASE_MULTIPLE_STAGES("RTC committed record survives timer deep sleep",
                          "[node_rtc][reset=DEEPSLEEP_RESET]",
                          rtc_survives_stage_1, rtc_survives_stage_2);

static void rtc_software_reset_stage_1(void) {
  rtc_case_begin();
  const node_cycle_metrics_t metrics = rtc_metrics(8U);
  TEST_ASSERT_TRUE(
      node_rtc_record_commit(&s_rtc_record, UINT32_C(9), &metrics));
  esp_restart();
  abort();
}

static void rtc_software_reset_stage_2(void) {
  const node_platform_ports_t *const platform = node_platform_esp_ports();
  TEST_ASSERT_EQUAL_UINT8(
      CURA_LORA_V2_RESET_REASON_ESP_RST_SW,
      platform->system.get_reset_reason(platform->system.context));
  node_cycle_metrics_t previous;
  memset(&previous, UINT8_C(0xa5), sizeof(previous));
  TEST_ASSERT_FALSE(node_rtc_record_validate_previous(
      &s_rtc_record,
      platform->system.get_reset_reason(platform->system.context), UINT32_C(10),
      &previous));
  const node_cycle_metrics_t zero = {0};
  assert_metrics_equal(&zero, &previous);
  rtc_case_finish();
}

TEST_CASE_MULTIPLE_STAGES(
    "RTC software reset is rejected and platform reports software reset",
    "[node_rtc][node_platform][reset=SW_CPU_RESET]", rtc_software_reset_stage_1,
    rtc_software_reset_stage_2);

static void rtc_consumed_invalid_stage_1(void) {
  rtc_case_begin();
  const node_cycle_metrics_t metrics = rtc_metrics(9U);
  TEST_ASSERT_TRUE(
      node_rtc_record_commit(&s_rtc_record, UINT32_C(5), &metrics));
  enter_short_deep_sleep();
}

static void rtc_consumed_invalid_stage_2(void) {
  assert_deep_sleep_boot();
  node_rtc_record_t copy;
  node_rtc_record_take(&s_rtc_record, &copy);
  TEST_ASSERT_EQUAL_HEX32(NODE_RTC_COMMITTED_V1, copy.commit_marker);
  TEST_ASSERT_EQUAL_UINT32(5U, copy.completed_sample_id);
  TEST_ASSERT_EQUAL_HEX32(0U, s_rtc_record.commit_marker);
  esp_restart();
  abort();
}

static void rtc_consumed_invalid_stage_3(void) {
  const node_platform_ports_t *const platform = node_platform_esp_ports();
  TEST_ASSERT_EQUAL_UINT8(
      CURA_LORA_V2_RESET_REASON_ESP_RST_SW,
      platform->system.get_reset_reason(platform->system.context));
  TEST_ASSERT_EQUAL_HEX32(0U, s_rtc_record.commit_marker);
  rtc_case_finish();
}

TEST_CASE_MULTIPLE_STAGES(
    "RTC consumed record stays invalid after software restart",
    "[node_rtc][reset=DEEPSLEEP_RESET,SW_CPU_RESET]",
    rtc_consumed_invalid_stage_1, rtc_consumed_invalid_stage_2,
    rtc_consumed_invalid_stage_3);

static void rtc_replaced_stage_1(void) {
  rtc_case_begin();
  const node_cycle_metrics_t metrics_a = rtc_metrics(10U);
  TEST_ASSERT_TRUE(
      node_rtc_record_commit(&s_rtc_record, UINT32_C(20), &metrics_a));
  enter_short_deep_sleep();
}

static void rtc_replaced_stage_2(void) {
  assert_deep_sleep_boot();
  node_rtc_record_t copy_a;
  node_rtc_record_take(&s_rtc_record, &copy_a);
  const node_cycle_metrics_t metrics_a = rtc_metrics(10U);
  node_cycle_metrics_t previous_a;
  TEST_ASSERT_TRUE(node_rtc_record_validate_previous(
      &copy_a, CURA_LORA_V2_RESET_REASON_ESP_RST_DEEPSLEEP, UINT32_C(21),
      &previous_a));
  assert_metrics_equal(&metrics_a, &previous_a);

  const node_cycle_metrics_t metrics_b = rtc_metrics(11U);
  TEST_ASSERT_TRUE(
      node_rtc_record_commit(&s_rtc_record, UINT32_C(21), &metrics_b));
  enter_short_deep_sleep();
}

static void rtc_replaced_stage_3(void) {
  assert_deep_sleep_boot();
  node_rtc_record_t copy_b;
  node_rtc_record_take(&s_rtc_record, &copy_b);
  TEST_ASSERT_EQUAL_UINT32(21U, copy_b.completed_sample_id);
  const node_cycle_metrics_t metrics_b = rtc_metrics(11U);
  node_cycle_metrics_t previous_b;
  TEST_ASSERT_TRUE(node_rtc_record_validate_previous(
      &copy_b, CURA_LORA_V2_RESET_REASON_ESP_RST_DEEPSLEEP, UINT32_C(22),
      &previous_b));
  assert_metrics_equal(&metrics_b, &previous_b);
  TEST_ASSERT_NOT_EQUAL_UINT32(20U, copy_b.completed_sample_id);
  TEST_ASSERT_EQUAL_HEX32(0U, s_rtc_record.commit_marker);
  rtc_case_finish();
}

TEST_CASE_MULTIPLE_STAGES("RTC new commit replaces the previous commit",
                          "[node_rtc][reset=DEEPSLEEP_RESET,DEEPSLEEP_RESET]",
                          rtc_replaced_stage_1, rtc_replaced_stage_2,
                          rtc_replaced_stage_3);

static void rtc_nvs_consecutive_stage_1(void) {
  rtc_case_begin();
  hwtest_seed_next_sample_id(UINT32_C(101));
  const node_cycle_metrics_t metrics = rtc_metrics(12U);
  TEST_ASSERT_TRUE(
      node_rtc_record_commit(&s_rtc_record, UINT32_C(100), &metrics));
  enter_short_deep_sleep();
}

static void rtc_nvs_consecutive_stage_2(void) {
  assert_deep_sleep_boot();
  node_rtc_record_t copy;
  node_rtc_record_take(&s_rtc_record, &copy);
  uint32_t sample_id = 0U;
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(CURAG_OK,
                          node_persistence_claim_sample_id(&sample_id, &diag));
  TEST_ASSERT_EQUAL_UINT32(101U, sample_id);
  node_cycle_metrics_t previous;
  TEST_ASSERT_TRUE(node_rtc_record_validate_previous(
      &copy, CURA_LORA_V2_RESET_REASON_ESP_RST_DEEPSLEEP, sample_id,
      &previous));
  const node_cycle_metrics_t metrics = rtc_metrics(12U);
  assert_metrics_equal(&metrics, &previous);
  TEST_ASSERT_EQUAL_HEX32(CURAG_OK, node_persistence_sync_all(&diag));
  node_persistence_test_reset();
  TEST_ASSERT_EQUAL_UINT32(102U, hwtest_read_next_sample_id());
  rtc_case_finish();
}

TEST_CASE_MULTIPLE_STAGES(
    "RTC accepts consecutive sample claimed from real NVS",
    "[node_rtc][node_persistence][reset=DEEPSLEEP_RESET]",
    rtc_nvs_consecutive_stage_1, rtc_nvs_consecutive_stage_2);

static void rtc_nvs_nonconsecutive_stage_1(void) {
  rtc_case_begin();
  hwtest_seed_next_sample_id(UINT32_C(105));
  const node_cycle_metrics_t metrics = rtc_metrics(13U);
  TEST_ASSERT_TRUE(
      node_rtc_record_commit(&s_rtc_record, UINT32_C(100), &metrics));
  enter_short_deep_sleep();
}

static void rtc_nvs_nonconsecutive_stage_2(void) {
  assert_deep_sleep_boot();
  node_rtc_record_t copy;
  node_rtc_record_take(&s_rtc_record, &copy);
  uint32_t sample_id = 0U;
  diagn_context_t diag;
  TEST_ASSERT_EQUAL_HEX32(CURAG_OK,
                          node_persistence_claim_sample_id(&sample_id, &diag));
  TEST_ASSERT_EQUAL_UINT32(105U, sample_id);
  node_cycle_metrics_t previous;
  memset(&previous, UINT8_C(0xa5), sizeof(previous));
  TEST_ASSERT_FALSE(node_rtc_record_validate_previous(
      &copy, CURA_LORA_V2_RESET_REASON_ESP_RST_DEEPSLEEP, sample_id,
      &previous));
  const node_cycle_metrics_t zero = {0};
  assert_metrics_equal(&zero, &previous);
  rtc_case_finish();
}

TEST_CASE_MULTIPLE_STAGES(
    "RTC rejects nonconsecutive sample claimed from real NVS",
    "[node_rtc][node_persistence][reset=DEEPSLEEP_RESET]",
    rtc_nvs_nonconsecutive_stage_1, rtc_nvs_nonconsecutive_stage_2);

static void rtc_repeated_round_trip_stage_1(void) {
  rtc_case_begin();
  const node_cycle_metrics_t metrics = rtc_metrics(0U);
  TEST_ASSERT_TRUE(
      node_rtc_record_commit(&s_rtc_record, UINT32_C(0x1000), &metrics));
  enter_short_deep_sleep();
}

static void rtc_repeated_round_trip_stage_2(void) {
  assert_deep_sleep_boot();
  const uint32_t iteration = s_round_trip_iteration;
  TEST_ASSERT_LESS_THAN_UINT32(RTC_HWTEST_ROUND_TRIPS, iteration);
  const node_cycle_metrics_t expected = rtc_metrics(iteration);
  assert_record(UINT32_C(0x1000) + iteration, &expected);

  ++s_round_trip_iteration;
  if (s_round_trip_iteration < RTC_HWTEST_ROUND_TRIPS) {
    const node_cycle_metrics_t next = rtc_metrics(s_round_trip_iteration);
    TEST_ASSERT_TRUE(node_rtc_record_commit(
        &s_rtc_record, UINT32_C(0x1000) + s_round_trip_iteration, &next));
    enter_short_deep_sleep();
  }
  rtc_case_finish();
}

TEST_CASE_MULTIPLE_STAGES(
    "RTC repeats exactly 20 retained deep-sleep round trips",
    "[node_rtc][slow][reset=DEEPSLEEP_RESET]", rtc_repeated_round_trip_stage_1,
    rtc_repeated_round_trip_stage_2);
