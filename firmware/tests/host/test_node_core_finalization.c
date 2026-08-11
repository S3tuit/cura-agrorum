#include "node_core_test.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static bool script_status(uint32_t sample_id, cura_lora_v2_ack_status_t status,
                          uint64_t set_tx, uint64_t tx_done, uint64_t ack_at) {
  cura_lora_v2_domain_t domain = CURA_LORA_V2_DOMAIN_ACK_ACCEPTED_DOWNLINK;
  if (status == CURA_LORA_V2_ACK_STATUS_RETRY_LATER) {
    domain = CURA_LORA_V2_DOMAIN_ACK_RETRY_LATER_DOWNLINK;
  }
  return core_test_script_ack(sample_id, domain, status, set_tx, tx_done,
                              ack_at);
}

static bool delivery_events_bracket_retries_and_failures_are_best_effort(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  fake_node_core.claimed_sample_id = 22U;
  const uint64_t first_set = UINT64_C(1001000);
  const uint64_t first_done = first_set + core_test_reading_airtime_us();
  const uint64_t retry_at =
      first_done + NODE_CORE_ACK_WAIT_US + NODE_CORE_RETRY_JITTER_MIN_US;
  fake_node_core_script_tx_done(first_set, first_done);
  fake_node_core_script_rx_deadline(retry_at);
  CORE_TEST_ASSERT(script_status(
      22U, CURA_LORA_V2_ACK_STATUS_ACCEPTED, retry_at + UINT64_C(1000),
      retry_at + UINT64_C(1000) + core_test_reading_airtime_us(),
      retry_at + UINT64_C(110000)));
  core_test_run(&rtc, &platform);
  CORE_TEST_ASSERT_EQ_SIZE(2U, fake_node_core.delivery_event_count);
  const node_delivery_event_t *start = &fake_node_core.delivery_events[0];
  const node_delivery_event_t *finish = &fake_node_core.delivery_events[1];
  CORE_TEST_ASSERT_EQ_U32(NODE_DELIVERY_EVENT_STARTED, start->type);
  CORE_TEST_ASSERT_EQ_U32(NODE_DELIVERY_EVENT_FINISHED, finish->type);
  CORE_TEST_ASSERT_EQ_U32(22U, start->cycle_sample_id);
  CORE_TEST_ASSERT_EQ_U32(22U, start->sample_id);
  CORE_TEST_ASSERT_EQ_U32(2U, finish->detail.finished.attempt_count);
  CORE_TEST_ASSERT_EQ_U32(NODE_DELIVERY_RESULT_ACCEPTED,
                          finish->detail.finished.final_result);

  core_test_setup(&rtc, &platform);
  fake_node_core.delivery_event_errors[0] = CURAG_EIO;
  fake_node_core.delivery_event_errors[1] = CURAG_EIO;
  fake_node_core.delivery_event_error_count = 2U;
  CORE_TEST_ASSERT(script_status(0U, CURA_LORA_V2_ACK_STATUS_ACCEPTED,
                                 UINT64_C(1001000), UINT64_C(1098536),
                                 UINT64_C(1110000)));
  core_test_run(&rtc, &platform);
  CORE_TEST_ASSERT(rtc.metrics.current_accepted);
  CORE_TEST_ASSERT_EQ_SIZE(1U, fake_node_core.removed_count);
  CORE_TEST_ASSERT_EQ_SIZE(2U, fake_node_core.diagnostic_event_count);
  CORE_TEST_ASSERT(core_test_cleanup_is_complete());
  return true;
}

static bool diagnostic_failure_is_not_logged_recursively(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  fake_node_core.append_pending_error = CURAG_EIO;
  fake_node_core.diagnostic_event_errors[0] = CURAG_EIO;
  fake_node_core.diagnostic_event_error_count = 1U;
  CORE_TEST_ASSERT(script_status(0U, CURA_LORA_V2_ACK_STATUS_ACCEPTED,
                                 UINT64_C(1001000), UINT64_C(1098536),
                                 UINT64_C(1110000)));
  core_test_run(&rtc, &platform);
  CORE_TEST_ASSERT_EQ_SIZE(1U, fake_node_core.diagnostic_event_count);
  CORE_TEST_ASSERT(rtc.metrics.current_accepted);
  CORE_TEST_ASSERT(core_test_cleanup_is_complete());
  return true;
}

static bool current_and_backlog_metrics_accumulate_in_correct_domains(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  const cura_lora_v2_reading_t backlog = core_test_reading(4U);
  fake_node_core_add_pending(4U, &backlog);
  fake_node_core.claimed_sample_id = 5U;

  const uint64_t first_set = UINT64_C(1001000);
  const uint64_t first_done = first_set + core_test_reading_airtime_us();
  const uint64_t retry_at =
      first_done + NODE_CORE_ACK_WAIT_US + NODE_CORE_RETRY_JITTER_MIN_US;
  fake_node_core_script_tx_done(first_set, first_done);
  fake_node_core_script_rx_deadline(retry_at);
  const uint64_t current_second_set = retry_at + UINT64_C(1000);
  const uint64_t current_second_done =
      current_second_set + core_test_reading_airtime_us();
  const uint64_t current_ack = current_second_done + UINT64_C(2000);
  CORE_TEST_ASSERT(script_status(5U, CURA_LORA_V2_ACK_STATUS_ACCEPTED,
                                 current_second_set, current_second_done,
                                 current_ack));
  const uint64_t backlog_set = current_ack + UINT64_C(1000);
  const uint64_t backlog_done = backlog_set + core_test_reading_airtime_us();
  CORE_TEST_ASSERT(script_status(4U, CURA_LORA_V2_ACK_STATUS_ACCEPTED,
                                 backlog_set, backlog_done,
                                 backlog_done + UINT64_C(2000)));

  core_test_run(&rtc, &platform);

  CORE_TEST_ASSERT_EQ_U32(2U, rtc.metrics.current_tx_attempts);
  CORE_TEST_ASSERT_EQ_U32(3U, rtc.metrics.cycle_tx_attempts);
  CORE_TEST_ASSERT_EQ_U32(2U, rtc.metrics.accepted_readings);
  CORE_TEST_ASSERT(rtc.metrics.current_accepted);
  CORE_TEST_ASSERT_EQ_U32(
      (uint16_t)((current_ack - first_set) / UINT64_C(1000)),
      rtc.metrics.current_delivery_ms);
  return true;
}

static bool unaccepted_current_has_zero_delivery_metric(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  CORE_TEST_ASSERT(script_status(0U, CURA_LORA_V2_ACK_STATUS_RETRY_LATER,
                                 UINT64_C(1001000), UINT64_C(1098536),
                                 UINT64_C(1110000)));
  core_test_run(&rtc, &platform);
  CORE_TEST_ASSERT_EQ_U32(NODE_RTC_COMMITTED_V1, rtc.commit_marker);
  CORE_TEST_ASSERT(!rtc.metrics.current_accepted);
  CORE_TEST_ASSERT_EQ_U32(0U, rtc.metrics.current_delivery_ms);
  CORE_TEST_ASSERT_EQ_U32(1U, rtc.metrics.current_tx_attempts);
  CORE_TEST_ASSERT_EQ_U32(0U, rtc.metrics.accepted_readings);
  return true;
}

static bool awake_time_includes_cleanup_and_sync_and_sleep_is_terminal(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  const uint64_t ack_at = UINT64_C(1110000);
  CORE_TEST_ASSERT(script_status(0U, CURA_LORA_V2_ACK_STATUS_RETRY_LATER,
                                 UINT64_C(1001000), UINT64_C(1098536), ack_at));
  fake_node_core.force_power_off_advance_us = UINT64_C(5000);
  fake_node_core.radio_sleep_advance_us = UINT64_C(7000);
  fake_node_core.sync_advance_us = UINT64_C(11000);
  core_test_run(&rtc, &platform);
  CORE_TEST_ASSERT_EQ_U32(
      (uint16_t)((ack_at + UINT64_C(23000) - UINT64_C(1000000)) /
                 UINT64_C(1000)),
      rtc.metrics.awake_ms);
  const size_t force =
      fake_node_core_trace_find(FAKE_CORE_TRACE_FORCE_POWER_OFF, 0U);
  const size_t radio =
      fake_node_core_trace_find(FAKE_CORE_TRACE_RADIO_SLEEP, 0U);
  const size_t sync = fake_node_core_trace_find(FAKE_CORE_TRACE_SYNC, 0U);
  const size_t sleep =
      fake_node_core_trace_find(FAKE_CORE_TRACE_DEEP_SLEEP, 0U);
  CORE_TEST_ASSERT(force < radio && radio < sync && sync < sleep);
  CORE_TEST_ASSERT(core_test_cleanup_is_complete());
  return true;
}

static bool rtc_metric_boundaries_and_semantics_are_enforced(void) {
  node_rtc_record_t record = {0};
  const node_cycle_metrics_t maximum = {
      .current_tx_attempts = UINT8_MAX,
      .awake_ms = UINT16_MAX,
      .current_delivery_ms = UINT16_MAX,
      .cycle_tx_attempts = UINT8_MAX,
      .accepted_readings = UINT8_MAX,
      .current_accepted = true,
  };
  CORE_TEST_ASSERT(node_rtc_record_commit(&record, UINT32_MAX, &maximum));
  CORE_TEST_ASSERT_EQ_U32(NODE_RTC_COMMITTED_V1, record.commit_marker);
  node_cycle_metrics_t decoded;
  CORE_TEST_ASSERT(!node_rtc_record_validate_previous(
      &record, CURA_LORA_V2_RESET_REASON_ESP_RST_DEEPSLEEP, 0U, &decoded));
  CORE_TEST_ASSERT_EQ_U32(0U, decoded.cycle_tx_attempts);

  node_cycle_metrics_t invalid = maximum;
  invalid.current_tx_attempts = UINT8_MAX;
  invalid.cycle_tx_attempts = UINT8_MAX - 1U;
  CORE_TEST_ASSERT(!node_rtc_record_commit(&record, 1U, &invalid));
  CORE_TEST_ASSERT_EQ_U32(0U, record.commit_marker);
  invalid = maximum;
  invalid.current_accepted = false;
  CORE_TEST_ASSERT(!node_rtc_record_commit(&record, 1U, &invalid));
  CORE_TEST_ASSERT_EQ_U32(0U, record.commit_marker);

  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  fake_node_core.sensor_advance_us = UINT64_C(65535000);
  core_test_run(&rtc, &platform);
  CORE_TEST_ASSERT_EQ_U32(NODE_RTC_COMMITTED_V1, rtc.commit_marker);
  CORE_TEST_ASSERT_EQ_U32(UINT16_MAX, rtc.metrics.awake_ms);
  CORE_TEST_ASSERT_EQ_U32(0U, rtc.metrics.cycle_tx_attempts);

  core_test_setup(&rtc, &platform);
  fake_node_core.sensor_advance_us = UINT64_C(65536000);
  core_test_run(&rtc, &platform);
  CORE_TEST_ASSERT_EQ_U32(0U, rtc.commit_marker);
  CORE_TEST_ASSERT(
      core_test_has_diagnostic(CURAG_EDOM_CORE, CURAG_ECORE_EMETRICS_OVERFLOW));
  return true;
}

static bool sync_induced_metric_overflow_has_no_late_diagnostic(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  fake_node_core.sensor_advance_us = UINT64_C(65535000);
  fake_node_core.sync_advance_us = UINT64_C(1000);

  core_test_run(&rtc, &platform);

  CORE_TEST_ASSERT_EQ_U32(0U, rtc.commit_marker);
  CORE_TEST_ASSERT_EQ_SIZE(0U, fake_node_core.diagnostic_event_count);
  const size_t sync = fake_node_core_trace_find(FAKE_CORE_TRACE_SYNC, 0U);
  CORE_TEST_ASSERT(sync != SIZE_MAX);
  CORE_TEST_ASSERT(fake_node_core_trace_find(FAKE_CORE_TRACE_DIAGNOSTIC,
                                             sync + 1U) == SIZE_MAX);
  CORE_TEST_ASSERT(core_test_cleanup_is_complete());
  return true;
}

static bool component_diagnostics_are_copied_opaquely(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  fake_node_core.sensor_error =
      curag_error_make(CURAG_EDOM_SENSORS, CURAG_ESENSORS_ECLEANUP);
  fake_node_core.sensor_diagnostic.operation = CURAG_OP_CLEANUP;
  fake_node_core.sensor_diagnostic.context[0] = UINT8_C(0xa5);
  CORE_TEST_ASSERT(script_status(0U, CURA_LORA_V2_ACK_STATUS_ACCEPTED,
                                 UINT64_C(1001000), UINT64_C(1098536),
                                 UINT64_C(1110000)));
  core_test_run(&rtc, &platform);
  CORE_TEST_ASSERT(fake_node_core.diagnostic_event_count >= 1U);
  const fake_node_core_captured_diagnostic_t *captured =
      &fake_node_core.diagnostic_events[0];
  CORE_TEST_ASSERT(captured->has_context);
  CORE_TEST_ASSERT_EQ_U32(CURAG_OP_CLEANUP, captured->context.operation);
  CORE_TEST_ASSERT_EQ_U32(CURAG_SENSOR_CONTEXT_V1,
                          captured->context.context_schema);
  CORE_TEST_ASSERT_EQ_U32(CURAG_SENSOR_CONTEXT_V1_LENGTH,
                          captured->context.context_length);
  CORE_TEST_ASSERT_EQ_U32(UINT8_C(0xa5), captured->context.context[0]);
  CORE_TEST_ASSERT(
      (captured->event.flags & NODE_DIAGNOSTIC_CYCLE_SAMPLE_ID_VALID) != 0U);
  return true;
}

static bool cleanup_failures_do_not_prevent_sync_or_sleep(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  fake_node_core.force_power_off_error =
      curag_error_make(CURAG_EDOM_SENSORS, CURAG_ESENSORS_EPOWER_CONTROL);
  fake_node_core.radio_sleep_error =
      curag_error_make(CURAG_EDOM_RADIO, CURAG_ERADIO_EIO);
  fake_node_core.sync_error = CURAG_EIO;
  CORE_TEST_ASSERT(script_status(0U, CURA_LORA_V2_ACK_STATUS_RETRY_LATER,
                                 UINT64_C(1001000), UINT64_C(1098536),
                                 UINT64_C(1110000)));

  core_test_run(&rtc, &platform);

  CORE_TEST_ASSERT(core_test_has_diagnostic(CURAG_EDOM_SENSORS,
                                            CURAG_ESENSORS_EPOWER_CONTROL));
  CORE_TEST_ASSERT(
      core_test_has_diagnostic(CURAG_EDOM_RADIO, CURAG_ERADIO_EIO));
  CORE_TEST_ASSERT_EQ_SIZE(1U, fake_node_core.sync_call_count);
  CORE_TEST_ASSERT_EQ_SIZE(1U, fake_node_core.deep_sleep_call_count);
  CORE_TEST_ASSERT_EQ_U32(NODE_RTC_COMMITTED_V1, rtc.commit_marker);
  CORE_TEST_ASSERT(core_test_cleanup_is_complete());
  return true;
}

bool node_core_test_finalization(const char *name) {
  if (strcmp(name,
             "delivery_events_bracket_retries_and_failures_are_best_effort") ==
      0) {
    return delivery_events_bracket_retries_and_failures_are_best_effort();
  }
  if (strcmp(name, "diagnostic_failure_is_not_logged_recursively") == 0) {
    return diagnostic_failure_is_not_logged_recursively();
  }
  if (strcmp(name,
             "current_and_backlog_metrics_accumulate_in_correct_domains") ==
      0) {
    return current_and_backlog_metrics_accumulate_in_correct_domains();
  }
  if (strcmp(name, "unaccepted_current_has_zero_delivery_metric") == 0) {
    return unaccepted_current_has_zero_delivery_metric();
  }
  if (strcmp(name,
             "awake_time_includes_cleanup_and_sync_and_sleep_is_terminal") ==
      0) {
    return awake_time_includes_cleanup_and_sync_and_sleep_is_terminal();
  }
  if (strcmp(name, "rtc_metric_boundaries_and_semantics_are_enforced") == 0) {
    return rtc_metric_boundaries_and_semantics_are_enforced();
  }
  if (strcmp(name, "sync_induced_metric_overflow_has_no_late_diagnostic") ==
      0) {
    return sync_induced_metric_overflow_has_no_late_diagnostic();
  }
  if (strcmp(name, "component_diagnostics_are_copied_opaquely") == 0) {
    return component_diagnostics_are_copied_opaquely();
  }
  if (strcmp(name, "cleanup_failures_do_not_prevent_sync_or_sleep") == 0) {
    return cleanup_failures_do_not_prevent_sync_or_sleep();
  }
  return false;
}
