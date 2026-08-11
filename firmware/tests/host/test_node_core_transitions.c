#include "node_core_test.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static cura_lora_v2_domain_t ack_domain(cura_lora_v2_ack_status_t status) {
  switch (status) {
  case CURA_LORA_V2_ACK_STATUS_ACCEPTED:
    return CURA_LORA_V2_DOMAIN_ACK_ACCEPTED_DOWNLINK;
  case CURA_LORA_V2_ACK_STATUS_RETRY_LATER:
    return CURA_LORA_V2_DOMAIN_ACK_RETRY_LATER_DOWNLINK;
  case CURA_LORA_V2_ACK_STATUS_REJECTED_UNSUPPORTED:
    return CURA_LORA_V2_DOMAIN_ACK_REJECTED_UNSUPPORTED_DOWNLINK;
  default:
    return CURA_LORA_V2_DOMAIN_ACK_REJECTED_MALFORMED_DOWNLINK;
  }
}

static bool script_ack_after(uint32_t sample_id,
                             cura_lora_v2_ack_status_t status,
                             uint64_t *time_us) {
  const uint64_t set_tx = *time_us + UINT64_C(1000);
  const uint64_t tx_done = set_tx + core_test_reading_airtime_us();
  const uint64_t ack_at = tx_done + UINT64_C(1000);
  *time_us = ack_at;
  return core_test_script_ack(sample_id, ack_domain(status), status, set_tx,
                              tx_done, ack_at);
}

static bool current_ack_outcomes_apply_policy(void) {
  static const cura_lora_v2_ack_status_t statuses[] = {
      CURA_LORA_V2_ACK_STATUS_ACCEPTED,
      CURA_LORA_V2_ACK_STATUS_RETRY_LATER,
      CURA_LORA_V2_ACK_STATUS_REJECTED_UNSUPPORTED,
      CURA_LORA_V2_ACK_STATUS_REJECTED_MALFORMED,
  };
  static const node_delivery_final_result_t results[] = {
      NODE_DELIVERY_RESULT_ACCEPTED,
      NODE_DELIVERY_RESULT_RETRY_LATER,
      NODE_DELIVERY_RESULT_UNSUPPORTED,
      NODE_DELIVERY_RESULT_MALFORMED,
  };
  for (size_t index = 0U; index < sizeof(statuses); index++) {
    node_rtc_record_t rtc;
    node_platform_ports_t platform;
    core_test_setup(&rtc, &platform);
    fake_node_core.claimed_sample_id = 5U;
    if (index == 2U) {
      fake_node_core.quarantine_errors[0] = CURAG_ELOG_FULL;
      fake_node_core.quarantine_error_count = 1U;
    } else if (index == 3U) {
      fake_node_core.append_pending_error = CURAG_EIO;
    }
    uint64_t time_us = fake_node_core.now_us;
    CORE_TEST_ASSERT(script_ack_after(5U, statuses[index], &time_us));
    core_test_run(&rtc, &platform);
    CORE_TEST_ASSERT_EQ_U32(
        results[index],
        fake_node_core.delivery_events[1].detail.finished.final_result);
    if (statuses[index] == CURA_LORA_V2_ACK_STATUS_ACCEPTED) {
      CORE_TEST_ASSERT_EQ_SIZE(1U, fake_node_core.removed_count);
      CORE_TEST_ASSERT_EQ_SIZE(0U, fake_node_core.quarantined_count);
    } else if (statuses[index] == CURA_LORA_V2_ACK_STATUS_RETRY_LATER) {
      CORE_TEST_ASSERT_EQ_SIZE(0U, fake_node_core.removed_count);
      CORE_TEST_ASSERT_EQ_SIZE(0U, fake_node_core.quarantined_count);
      CORE_TEST_ASSERT(fake_node_core_trace_find(FAKE_CORE_TRACE_PEEK, 0U) ==
                       SIZE_MAX);
    } else {
      CORE_TEST_ASSERT_EQ_SIZE(1U, fake_node_core.quarantined_count);
      CORE_TEST_ASSERT_EQ_SIZE(index == 3U ? 0U : 1U,
                               fake_node_core.removed_count);
      CORE_TEST_ASSERT(fake_node_core_trace_find(FAKE_CORE_TRACE_PEEK, 0U) ==
                       SIZE_MAX);
      if (index == 2U) {
        CORE_TEST_ASSERT(core_test_has_diagnostic(
            CURAG_EDOM_PERSISTENCE, curag_error_code(CURAG_ELOG_FULL)));
      }
    }
  }
  return true;
}

static bool current_removal_failure_prevents_backlog(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  const cura_lora_v2_reading_t old = core_test_reading(9U);
  fake_node_core_add_pending(9U, &old);
  fake_node_core.claimed_sample_id = 10U;
  fake_node_core.remove_errors[0] = CURAG_EIO;
  fake_node_core.remove_error_count = 1U;
  uint64_t time_us = fake_node_core.now_us;
  CORE_TEST_ASSERT(
      script_ack_after(10U, CURA_LORA_V2_ACK_STATUS_ACCEPTED, &time_us));

  core_test_run(&rtc, &platform);

  CORE_TEST_ASSERT_EQ_SIZE(1U, fake_node_core.removed_count);
  CORE_TEST_ASSERT(fake_node_core_trace_find(FAKE_CORE_TRACE_PEEK, 0U) ==
                   SIZE_MAX);
  CORE_TEST_ASSERT(core_test_has_diagnostic(CURAG_EDOM_PERSISTENCE,
                                            curag_error_code(CURAG_EIO)));
  return true;
}

static bool backlog_is_drained_in_persistence_order_with_unchanged_body(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  const cura_lora_v2_reading_t reading10 = core_test_reading(10U);
  const cura_lora_v2_reading_t reading11 = core_test_reading(11U);
  fake_node_core_add_pending(10U, &reading10);
  fake_node_core_add_pending(11U, &reading11);
  fake_node_core.claimed_sample_id = 12U;
  uint64_t time_us = fake_node_core.now_us;
  CORE_TEST_ASSERT(
      script_ack_after(12U, CURA_LORA_V2_ACK_STATUS_ACCEPTED, &time_us));
  CORE_TEST_ASSERT(
      script_ack_after(11U, CURA_LORA_V2_ACK_STATUS_ACCEPTED, &time_us));
  CORE_TEST_ASSERT(
      script_ack_after(10U, CURA_LORA_V2_ACK_STATUS_ACCEPTED, &time_us));

  core_test_run(&rtc, &platform);

  CORE_TEST_ASSERT_EQ_SIZE(3U, fake_node_core.transmission_count);
  static const uint32_t expected_ids[] = {12U, 11U, 10U};
  for (size_t index = 0U; index < 3U; index++) {
    cura_lora_v2_clear_header_t header;
    cura_lora_v2_reading_t reading;
    CORE_TEST_ASSERT(core_test_decode_transmission(index, &header, &reading));
    CORE_TEST_ASSERT_EQ_U32(expected_ids[index], header.sample_id);
    CORE_TEST_ASSERT_EQ_U32(index == 0U
                                ? CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK
                                : CURA_LORA_V2_DOMAIN_BACKLOG_READING_UPLINK,
                            header.domain);
    if (index == 1U) {
      CORE_TEST_ASSERT(memcmp(&reading, &reading11, sizeof(reading)) == 0);
    } else if (index == 2U) {
      CORE_TEST_ASSERT(memcmp(&reading, &reading10, sizeof(reading)) == 0);
    }
  }
  CORE_TEST_ASSERT_EQ_SIZE(0U, fake_node_core.pending_count);
  CORE_TEST_ASSERT_EQ_U32(3U, rtc.metrics.cycle_tx_attempts);
  CORE_TEST_ASSERT_EQ_U32(1U, rtc.metrics.current_tx_attempts);
  CORE_TEST_ASSERT_EQ_U32(3U, rtc.metrics.accepted_readings);
  return true;
}

static bool backlog_retry_later_and_lookup_failure_stop(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  const cura_lora_v2_reading_t backlog = core_test_reading(1U);
  fake_node_core_add_pending(1U, &backlog);
  fake_node_core.claimed_sample_id = 2U;
  uint64_t time_us = fake_node_core.now_us;
  CORE_TEST_ASSERT(
      script_ack_after(2U, CURA_LORA_V2_ACK_STATUS_ACCEPTED, &time_us));
  CORE_TEST_ASSERT(
      script_ack_after(1U, CURA_LORA_V2_ACK_STATUS_RETRY_LATER, &time_us));
  core_test_run(&rtc, &platform);
  CORE_TEST_ASSERT_EQ_SIZE(2U, fake_node_core.transmission_count);
  CORE_TEST_ASSERT_EQ_SIZE(1U, fake_node_core.pending_count);

  core_test_setup(&rtc, &platform);
  fake_node_core.peek_errors[0] = CURAG_EIO;
  fake_node_core.peek_error_count = 1U;
  time_us = fake_node_core.now_us;
  CORE_TEST_ASSERT(
      script_ack_after(0U, CURA_LORA_V2_ACK_STATUS_ACCEPTED, &time_us));
  core_test_run(&rtc, &platform);
  CORE_TEST_ASSERT_EQ_SIZE(1U, fake_node_core.transmission_count);
  CORE_TEST_ASSERT(core_test_has_diagnostic(CURAG_EDOM_PERSISTENCE,
                                            curag_error_code(CURAG_EIO)));
  return true;
}

static bool backlog_quarantine_removal_combinations(void) {
  for (size_t variant = 0U; variant < 4U; variant++) {
    node_rtc_record_t rtc;
    node_platform_ports_t platform;
    core_test_setup(&rtc, &platform);
    const cura_lora_v2_reading_t backlog = core_test_reading(3U);
    fake_node_core_add_pending(3U, &backlog);
    fake_node_core.claimed_sample_id = 4U;
    fake_node_core.remove_errors[0] = CURAG_OK;
    fake_node_core.remove_errors[1] =
        (variant & 2U) != 0U ? CURAG_EIO : CURAG_OK;
    fake_node_core.remove_error_count = 2U;
    fake_node_core.quarantine_errors[0] =
        (variant & 1U) != 0U ? CURAG_ELOG_FULL : CURAG_OK;
    fake_node_core.quarantine_error_count = 1U;
    uint64_t time_us = fake_node_core.now_us;
    CORE_TEST_ASSERT(
        script_ack_after(4U, CURA_LORA_V2_ACK_STATUS_ACCEPTED, &time_us));
    const cura_lora_v2_ack_status_t rejection =
        (variant & 1U) != 0U ? CURA_LORA_V2_ACK_STATUS_REJECTED_UNSUPPORTED
                             : CURA_LORA_V2_ACK_STATUS_REJECTED_MALFORMED;
    CORE_TEST_ASSERT(script_ack_after(3U, rejection, &time_us));
    core_test_run(&rtc, &platform);
    CORE_TEST_ASSERT_EQ_SIZE(1U, fake_node_core.quarantined_count);
    CORE_TEST_ASSERT_EQ_SIZE(2U, fake_node_core.removed_count);
    if ((variant & 1U) != 0U) {
      CORE_TEST_ASSERT(core_test_has_diagnostic(
          CURAG_EDOM_PERSISTENCE, curag_error_code(CURAG_ELOG_FULL)));
    }
    if ((variant & 2U) == 0U) {
      CORE_TEST_ASSERT_EQ_SIZE(0U, fake_node_core.pending_count);
      CORE_TEST_ASSERT(fake_node_core_trace_find(
                           FAKE_CORE_TRACE_PEEK,
                           fake_node_core_trace_find(FAKE_CORE_TRACE_PEEK, 0U) +
                               1U) != SIZE_MAX);
    } else {
      CORE_TEST_ASSERT_EQ_SIZE(1U, fake_node_core.pending_count);
    }
  }
  return true;
}

static bool accepted_backlog_removal_failure_stops(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  const cura_lora_v2_reading_t backlog = core_test_reading(6U);
  fake_node_core_add_pending(6U, &backlog);
  fake_node_core.claimed_sample_id = 7U;
  fake_node_core.remove_errors[0] = CURAG_OK;
  fake_node_core.remove_errors[1] = CURAG_EIO;
  fake_node_core.remove_error_count = 2U;
  uint64_t time_us = fake_node_core.now_us;
  CORE_TEST_ASSERT(
      script_ack_after(7U, CURA_LORA_V2_ACK_STATUS_ACCEPTED, &time_us));
  CORE_TEST_ASSERT(
      script_ack_after(6U, CURA_LORA_V2_ACK_STATUS_ACCEPTED, &time_us));
  core_test_run(&rtc, &platform);
  CORE_TEST_ASSERT_EQ_SIZE(2U, fake_node_core.transmission_count);
  CORE_TEST_ASSERT_EQ_SIZE(1U, fake_node_core.pending_count);
  CORE_TEST_ASSERT_EQ_SIZE(2U, fake_node_core.removed_count);
  return true;
}

static bool airtime_budget_is_shared_across_backlog(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  for (uint32_t id = 0U; id < 73U; id++) {
    const cura_lora_v2_reading_t reading = core_test_reading((uint16_t)id);
    fake_node_core_add_pending(id, &reading);
  }
  fake_node_core.claimed_sample_id = 73U;
  uint64_t time_us = fake_node_core.now_us;
  for (uint32_t count = 0U; count < 73U; count++) {
    const uint32_t id = count == 0U ? 73U : 73U - count;
    CORE_TEST_ASSERT(
        script_ack_after(id, CURA_LORA_V2_ACK_STATUS_ACCEPTED, &time_us));
  }
  const uint64_t target_set_us = time_us + UINT64_C(1000);
  const uint64_t target_done_us =
      target_set_us + core_test_reading_airtime_us();
  const uint64_t retry_at_us =
      target_done_us + NODE_CORE_ACK_WAIT_US + NODE_CORE_RETRY_JITTER_MIN_US;
  fake_node_core_script_tx_done(target_set_us, target_done_us);
  fake_node_core_script_rx_deadline(retry_at_us);

  core_test_run(&rtc, &platform);

  CORE_TEST_ASSERT_EQ_SIZE(74U, fake_node_core.transmission_count);
  CORE_TEST_ASSERT_EQ_SIZE(1U, fake_node_core.pending_count);
  CORE_TEST_ASSERT_EQ_U32(0U, fake_node_core.pending[0].sample_id);
  CORE_TEST_ASSERT_EQ_U32(74U, rtc.metrics.cycle_tx_attempts);
  CORE_TEST_ASSERT_EQ_U32(73U, rtc.metrics.accepted_readings);
  CORE_TEST_ASSERT_EQ_U32(
      NODE_DELIVERY_RESULT_AIRTIME_BUDGET_END,
      fake_node_core.delivery_events[fake_node_core.delivery_event_count - 1U]
          .detail.finished.final_result);
  CORE_TEST_ASSERT_EQ_U32(
      1U,
      fake_node_core.delivery_events[fake_node_core.delivery_event_count - 1U]
          .detail.finished.attempt_count);
  return true;
}

static bool backlog_silence_retries_the_same_frame(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  const cura_lora_v2_reading_t backlog = core_test_reading(30U);
  fake_node_core_add_pending(30U, &backlog);
  fake_node_core.claimed_sample_id = 31U;
  uint64_t time_us = fake_node_core.now_us;
  CORE_TEST_ASSERT(
      script_ack_after(31U, CURA_LORA_V2_ACK_STATUS_ACCEPTED, &time_us));

  const uint64_t first_set = time_us + UINT64_C(1000);
  const uint64_t first_done = first_set + core_test_reading_airtime_us();
  const uint64_t retry_at =
      first_done + NODE_CORE_ACK_WAIT_US + NODE_CORE_RETRY_JITTER_MIN_US;
  fake_node_core_script_tx_done(first_set, first_done);
  fake_node_core_script_rx_deadline(retry_at);
  time_us = retry_at;
  CORE_TEST_ASSERT(
      script_ack_after(30U, CURA_LORA_V2_ACK_STATUS_ACCEPTED, &time_us));

  core_test_run(&rtc, &platform);

  CORE_TEST_ASSERT_EQ_SIZE(3U, fake_node_core.transmission_count);
  CORE_TEST_ASSERT(memcmp(fake_node_core.transmissions[1].payload,
                          fake_node_core.transmissions[2].payload,
                          CURA_LORA_V2_READING_FRAME_SIZE) == 0);
  CORE_TEST_ASSERT_EQ_U32(
      2U, fake_node_core.delivery_events[3].detail.finished.attempt_count);
  CORE_TEST_ASSERT_EQ_U32(1U, rtc.metrics.current_tx_attempts);
  CORE_TEST_ASSERT_EQ_U32(3U, rtc.metrics.cycle_tx_attempts);
  return true;
}

bool node_core_test_transitions(const char *name) {
  if (strcmp(name, "current_ack_outcomes_apply_policy") == 0) {
    return current_ack_outcomes_apply_policy();
  }
  if (strcmp(name, "current_removal_failure_prevents_backlog") == 0) {
    return current_removal_failure_prevents_backlog();
  }
  if (strcmp(name,
             "backlog_is_drained_in_persistence_order_with_unchanged_body") ==
      0) {
    return backlog_is_drained_in_persistence_order_with_unchanged_body();
  }
  if (strcmp(name, "backlog_retry_later_and_lookup_failure_stop") == 0) {
    return backlog_retry_later_and_lookup_failure_stop();
  }
  if (strcmp(name, "backlog_quarantine_removal_combinations") == 0) {
    return backlog_quarantine_removal_combinations();
  }
  if (strcmp(name, "accepted_backlog_removal_failure_stops") == 0) {
    return accepted_backlog_removal_failure_stops();
  }
  if (strcmp(name, "airtime_budget_is_shared_across_backlog") == 0) {
    return airtime_budget_is_shared_across_backlog();
  }
  if (strcmp(name, "backlog_silence_retries_the_same_frame") == 0) {
    return backlog_silence_retries_the_same_frame();
  }
  return false;
}
