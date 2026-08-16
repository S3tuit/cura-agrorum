#include "node_core_test.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static bool script_current_ack(uint32_t sample_id,
                               cura_lora_v2_ack_status_t status) {
  const uint64_t set_tx = fake_node_core.now_us + UINT64_C(1000);
  const uint64_t tx_done = set_tx + core_test_reading_airtime_us();
  const uint64_t ack_at = tx_done + UINT64_C(20000);
  cura_lora_v2_domain_t domain = CURA_LORA_V2_DOMAIN_ACK_ACCEPTED_DOWNLINK;
  if (status == CURA_LORA_V2_ACK_STATUS_RETRY_LATER) {
    domain = CURA_LORA_V2_DOMAIN_ACK_RETRY_LATER_DOWNLINK;
  } else if (status == CURA_LORA_V2_ACK_STATUS_REJECTED_UNSUPPORTED) {
    domain = CURA_LORA_V2_DOMAIN_ACK_REJECTED_UNSUPPORTED_DOWNLINK;
  } else if (status == CURA_LORA_V2_ACK_STATUS_REJECTED_MALFORMED) {
    domain = CURA_LORA_V2_DOMAIN_ACK_REJECTED_MALFORMED_DOWNLINK;
  }
  return core_test_script_ack(sample_id, domain, status, set_tx, tx_done,
                              ack_at);
}

static bool first_attempt_success(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  fake_node_core.claimed_sample_id = 7U;
  CORE_TEST_ASSERT(script_current_ack(7U, CURA_LORA_V2_ACK_STATUS_ACCEPTED));

  core_test_run(&rtc, &platform);

  CORE_TEST_ASSERT_EQ_SIZE(1U, fake_node_core.appended_count);
  CORE_TEST_ASSERT_EQ_U32(7U, fake_node_core.appended[0].sample_id);
  const cura_lora_v2_reading_t *reading = &fake_node_core.appended[0].reading;
  CORE_TEST_ASSERT_EQ_U32(101U, reading->soil_0_mv);
  CORE_TEST_ASSERT_EQ_U32(202U, reading->soil_1_mv);
  CORE_TEST_ASSERT_EQ_U32(0U, reading->run_ms);
  CORE_TEST_ASSERT_EQ_SIZE(1U, fake_node_core.transmission_count);
  CORE_TEST_ASSERT_EQ_SIZE(1U, fake_node_core.removed_count);
  CORE_TEST_ASSERT_EQ_U32(7U, fake_node_core.removed_ids[0]);
  CORE_TEST_ASSERT(fake_node_core_trace_find(FAKE_CORE_TRACE_PEEK, 0U) !=
                   SIZE_MAX);
  CORE_TEST_ASSERT_EQ_U32(NODE_RTC_COMMITTED_V1, rtc.commit_marker);
  CORE_TEST_ASSERT_EQ_U32(7U, rtc.completed_sample_id);
  CORE_TEST_ASSERT_EQ_U32(1U, rtc.metrics.current_tx_attempts);
  CORE_TEST_ASSERT_EQ_U32(1U, rtc.metrics.cycle_tx_attempts);
  CORE_TEST_ASSERT_EQ_U32(1U, rtc.metrics.accepted_readings);
  CORE_TEST_ASSERT(rtc.metrics.current_accepted);
  CORE_TEST_ASSERT(core_test_cleanup_is_complete());
  return true;
}

static bool claim_failure_invalidates_and_cleans_up(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  rtc.commit_marker = NODE_RTC_COMMITTED_V1;
  rtc.completed_sample_id = 10U;
  fake_node_core.claim_error = CURAG_ENVS_ACCESS;

  core_test_run(&rtc, &platform);

  CORE_TEST_ASSERT_EQ_U32(0U, fake_node_core.rtc_marker_at_claim);
  CORE_TEST_ASSERT_EQ_U32(0U, rtc.commit_marker);
  CORE_TEST_ASSERT_EQ_SIZE(0U, fake_node_core.sample_call_count);
  CORE_TEST_ASSERT_EQ_SIZE(0U, fake_node_core.transmission_count);
  CORE_TEST_ASSERT_EQ_SIZE(0U, fake_node_core.receive_count);
  CORE_TEST_ASSERT_EQ_SIZE(1U, fake_node_core.diagnostic_event_count);
  CORE_TEST_ASSERT((fake_node_core.diagnostic_events[0].event.flags &
                    NODE_DIAGNOSTIC_CYCLE_SAMPLE_ID_VALID) == 0U);
  CORE_TEST_ASSERT(core_test_cleanup_is_complete());
  return true;
}

static bool valid_rtc_reaches_next_reading(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  const node_cycle_metrics_t previous = {
      .current_tx_attempts = 2U,
      .awake_ms = 345U,
      .current_delivery_ms = 67U,
      .cycle_tx_attempts = 4U,
      .accepted_readings = 2U,
      .current_accepted = true,
  };
  CORE_TEST_ASSERT(node_rtc_record_commit(&rtc, 41U, &previous));
  fake_node_core.reset_reason = CURA_LORA_V2_RESET_REASON_ESP_RST_DEEPSLEEP;
  fake_node_core.claimed_sample_id = 42U;
  CORE_TEST_ASSERT(script_current_ack(42U, CURA_LORA_V2_ACK_STATUS_ACCEPTED));

  core_test_run(&rtc, &platform);

  const cura_lora_v2_reading_t *reading = &fake_node_core.appended[0].reading;
  CORE_TEST_ASSERT_EQ_U32(2U, reading->previous_current_tx_attempts);
  CORE_TEST_ASSERT_EQ_U32(345U, reading->previous_awake_ms);
  CORE_TEST_ASSERT_EQ_U32(67U, reading->previous_current_delivery_ms);
  CORE_TEST_ASSERT_EQ_U32(4U, reading->previous_cycle_tx_attempts);
  CORE_TEST_ASSERT_EQ_U32(2U, reading->previous_cycle_accepted_readings);
  CORE_TEST_ASSERT(
      (reading->flags & CURA_LORA_V2_FLAG_PREVIOUS_CYCLE_METRICS_VALID) != 0U);
  CORE_TEST_ASSERT(
      (reading->flags & CURA_LORA_V2_FLAG_PREVIOUS_CURRENT_ACCEPTED) != 0U);
  CORE_TEST_ASSERT((reading->flags & CURA_LORA_V2_FLAG_DEEP_SLEEP_BOOT) != 0U);
  return true;
}

static bool rejected_rtc_is_zeroed(void) {
  for (size_t variant = 0U; variant < 4U; variant++) {
    node_rtc_record_t rtc;
    node_platform_ports_t platform;
    core_test_setup(&rtc, &platform);
    const node_cycle_metrics_t valid = {
        .current_tx_attempts = 1U,
        .awake_ms = 2U,
        .current_delivery_ms = 3U,
        .cycle_tx_attempts = 1U,
        .accepted_readings = 1U,
        .current_accepted = true,
    };
    CORE_TEST_ASSERT(node_rtc_record_commit(&rtc, 8U, &valid));
    fake_node_core.reset_reason = CURA_LORA_V2_RESET_REASON_ESP_RST_DEEPSLEEP;
    fake_node_core.claimed_sample_id = 9U;
    if (variant == 0U) {
      rtc.commit_marker ^= UINT32_C(1);
    } else if (variant == 1U) {
      fake_node_core.reset_reason = CURA_LORA_V2_RESET_REASON_ESP_RST_POWERON;
    } else if (variant == 2U) {
      rtc.completed_sample_id = 7U;
    } else {
      rtc.metrics.current_accepted = false;
    }
    CORE_TEST_ASSERT(script_current_ack(9U, CURA_LORA_V2_ACK_STATUS_ACCEPTED));
    core_test_run(&rtc, &platform);
    const cura_lora_v2_reading_t *reading = &fake_node_core.appended[0].reading;
    CORE_TEST_ASSERT_EQ_U32(0U, reading->previous_current_tx_attempts);
    CORE_TEST_ASSERT_EQ_U32(0U, reading->previous_awake_ms);
    CORE_TEST_ASSERT_EQ_U32(
        0U, reading->flags & (CURA_LORA_V2_FLAG_PREVIOUS_CYCLE_METRICS_VALID |
                              CURA_LORA_V2_FLAG_PREVIOUS_CURRENT_ACCEPTED));
  }
  return true;
}

static bool sensor_validity_maps_independently(void) {
  static const uint8_t validities[] = {
      NODE_SENSOR_SOIL_0_VALID,        NODE_SENSOR_SOIL_1_VALID,
      NODE_SENSOR_SOIL_TEMP_0_VALID,   NODE_SENSOR_SOIL_TEMP_1_VALID,
      NODE_SENSOR_ENCLOSURE_ENV_VALID, 0U,
  };
  for (size_t index = 0U; index < sizeof(validities); index++) {
    node_rtc_record_t rtc;
    node_platform_ports_t platform;
    core_test_setup(&rtc, &platform);
    fake_node_core.sensor_sample.validity = validities[index];
    fake_node_core.sensor_error = curag_error_make(
        CURAG_EDOM_SENSORS, validities[index] == 0U
                                ? CURAG_ESENSORS_ECOMPLETE_SAMPLE
                                : CURAG_ESENSORS_EPARTIAL_SAMPLE);
    CORE_TEST_ASSERT(script_current_ack(0U, CURA_LORA_V2_ACK_STATUS_ACCEPTED));
    core_test_run(&rtc, &platform);
    const cura_lora_v2_reading_t *reading = &fake_node_core.appended[0].reading;
    const bool soil_0 = (validities[index] & NODE_SENSOR_SOIL_0_VALID) != 0U;
    const bool soil_1 = (validities[index] & NODE_SENSOR_SOIL_1_VALID) != 0U;
    const bool soil_temp_0 =
        (validities[index] & NODE_SENSOR_SOIL_TEMP_0_VALID) != 0U;
    const bool soil_temp_1 =
        (validities[index] & NODE_SENSOR_SOIL_TEMP_1_VALID) != 0U;
    CORE_TEST_ASSERT(
        ((reading->flags & CURA_LORA_V2_FLAG_SOIL_0_VALID) != 0U) == soil_0);
    CORE_TEST_ASSERT(
        ((reading->flags & CURA_LORA_V2_FLAG_SOIL_1_VALID) != 0U) == soil_1);
    CORE_TEST_ASSERT(((reading->flags & CURA_LORA_V2_FLAG_SOIL_TEMP_0_VALID) !=
                      0U) == soil_temp_0);
    CORE_TEST_ASSERT(((reading->flags & CURA_LORA_V2_FLAG_SOIL_TEMP_1_VALID) !=
                      0U) == soil_temp_1);
    CORE_TEST_ASSERT_EQ_U32(soil_0 ? 101U : 0U, reading->soil_0_mv);
    CORE_TEST_ASSERT_EQ_U32(soil_1 ? 202U : 0U, reading->soil_1_mv);
    CORE_TEST_ASSERT(reading->soil_temp_0_centi_c ==
                     (soil_temp_0 ? INT16_C(303) : INT16_C(0)));
    CORE_TEST_ASSERT(reading->soil_temp_1_centi_c ==
                     (soil_temp_1 ? INT16_C(404) : INT16_C(0)));
    const bool enclosure =
        (validities[index] & NODE_SENSOR_ENCLOSURE_ENV_VALID) != 0U;
    CORE_TEST_ASSERT(
        ((reading->flags & CURA_LORA_V2_FLAG_ENCLOSURE_TEMP_VALID) != 0U) ==
        enclosure);
    CORE_TEST_ASSERT(
        ((reading->flags & CURA_LORA_V2_FLAG_ENCLOSURE_PRESSURE_VALID) != 0U) ==
        enclosure);
    CORE_TEST_ASSERT(
        ((reading->flags & CURA_LORA_V2_FLAG_ENCLOSURE_HUMIDITY_VALID) != 0U) ==
        enclosure);
    CORE_TEST_ASSERT(reading->enclosure_centi_c ==
                     (enclosure ? INT16_C(505) : INT16_C(0)));
    CORE_TEST_ASSERT_EQ_U32(enclosure ? 100600U : 0U,
                            reading->enclosure_pressure_pa);
    CORE_TEST_ASSERT_EQ_U32(enclosure ? 707U : 0U,
                            reading->enclosure_humidity_centi_pct);
    CORE_TEST_ASSERT_EQ_U32(CURA_LORA_V2_CODEC_OK,
                            cura_lora_v2_validate_reading(reading));
  }
  return true;
}

static bool run_time_and_sampling_deadline_boundaries(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  fake_node_core.sensor_advance_us = UINT64_C(123999);
  CORE_TEST_ASSERT(script_current_ack(0U, CURA_LORA_V2_ACK_STATUS_ACCEPTED));
  core_test_run(&rtc, &platform);
  CORE_TEST_ASSERT_EQ_U32(123U, fake_node_core.appended[0].reading.run_ms);

  core_test_setup(&rtc, &platform);
  fake_node_core.sensor_advance_us = UINT64_C(65536000);
  core_test_run(&rtc, &platform);
  CORE_TEST_ASSERT_EQ_U32(UINT16_MAX,
                          fake_node_core.appended[0].reading.run_ms);
  CORE_TEST_ASSERT_EQ_SIZE(0U, fake_node_core.transmission_count);
  CORE_TEST_ASSERT(
      core_test_has_diagnostic(CURAG_EDOM_CORE, CURAG_ECORE_ETIME_RANGE));

  core_test_setup(&rtc, &platform);
  fake_node_core.sensor_advance_us = NODE_CORE_RADIO_CYCLE_LIMIT_US -
                                     core_test_reading_min_tx_window_us() + 1U;
  core_test_run(&rtc, &platform);
  CORE_TEST_ASSERT_EQ_SIZE(0U, fake_node_core.transmission_count);
  CORE_TEST_ASSERT_EQ_SIZE(2U, fake_node_core.delivery_event_count);
  CORE_TEST_ASSERT_EQ_U32(
      NODE_DELIVERY_RESULT_RADIO_CYCLE_DEADLINE,
      fake_node_core.delivery_events[1].detail.finished.final_result);
  return true;
}

static bool pending_failure_allows_ram_only_delivery(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  fake_node_core.append_pending_error = CURAG_EIO;
  CORE_TEST_ASSERT(script_current_ack(0U, CURA_LORA_V2_ACK_STATUS_ACCEPTED));

  core_test_run(&rtc, &platform);

  CORE_TEST_ASSERT_EQ_SIZE(1U, fake_node_core.transmission_count);
  CORE_TEST_ASSERT_EQ_SIZE(0U, fake_node_core.removed_count);
  CORE_TEST_ASSERT(fake_node_core_trace_find(FAKE_CORE_TRACE_PEEK, 0U) ==
                   SIZE_MAX);
  CORE_TEST_ASSERT(core_test_has_diagnostic(CURAG_EDOM_PERSISTENCE,
                                            curag_error_code(CURAG_EIO)));
  CORE_TEST_ASSERT(rtc.metrics.current_accepted);
  return true;
}

static bool invalid_composition_returns_without_side_effects(void) {
  node_rtc_record_t rtc = {.commit_marker = NODE_RTC_COMMITTED_V1};
  node_platform_ports_t platform;
  fake_node_core_reset();
  platform = fake_node_core_platform();
  node_cycle_run(&platform, NULL, &rtc);
  CORE_TEST_ASSERT_EQ_U32(NODE_RTC_COMMITTED_V1, rtc.commit_marker);
  CORE_TEST_ASSERT_EQ_SIZE(0U, fake_node_core.trace_count);
  platform.clock.monotonic_us = NULL;
  node_cycle_run(&platform, &CORE_TEST_IDENTITY, &rtc);
  CORE_TEST_ASSERT_EQ_SIZE(0U, fake_node_core.trace_count);
  return true;
}

bool node_core_test_initialization(const char *name) {
  if (strcmp(name, "first_attempt_success") == 0) {
    return first_attempt_success();
  }
  if (strcmp(name, "claim_failure_invalidates_and_cleans_up") == 0) {
    return claim_failure_invalidates_and_cleans_up();
  }
  if (strcmp(name, "valid_rtc_reaches_next_reading") == 0) {
    return valid_rtc_reaches_next_reading();
  }
  if (strcmp(name, "rejected_rtc_is_zeroed") == 0) {
    return rejected_rtc_is_zeroed();
  }
  if (strcmp(name, "sensor_validity_maps_independently") == 0) {
    return sensor_validity_maps_independently();
  }
  if (strcmp(name, "run_time_and_sampling_deadline_boundaries") == 0) {
    return run_time_and_sampling_deadline_boundaries();
  }
  if (strcmp(name, "pending_failure_allows_ram_only_delivery") == 0) {
    return pending_failure_allows_ram_only_delivery();
  }
  if (strcmp(name, "invalid_composition_returns_without_side_effects") == 0) {
    return invalid_composition_returns_without_side_effects();
  }
  return false;
}
