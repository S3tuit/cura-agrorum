#include "node_core_test.h"

#include <string.h>

static void observations(size_t call, uint16_t header, uint16_t payload,
                         uint64_t first) {
  fake_node_core.rx_scripts[call].result.header_crc_count = header;
  fake_node_core.rx_scripts[call].result.payload_crc_count = payload;
  fake_node_core.rx_scripts[call].result.first_rejection_at_us = first;
}

static uint64_t little(const uint8_t *bytes, unsigned count) {
  uint64_t result = 0U;
  for (unsigned i = 0U; i < count; ++i)
    result |= (uint64_t)bytes[i] << (8U * i);
  return result;
}

static bool check_summary(size_t index, uint8_t attempt, bool ack,
                          uint16_t header, uint16_t payload, uint64_t first) {
  CORE_TEST_ASSERT(index < fake_node_core.diagnostic_event_count);
  const fake_node_core_captured_diagnostic_t *record =
      &fake_node_core.diagnostic_events[index];
  CORE_TEST_ASSERT_EQ_U32(curag_error_make(CURAG_EDOM_CORE, 15U),
                          record->event.error);
  CORE_TEST_ASSERT_EQ_U32(CORE_TEST_FIRST_MESSAGE_ID, record->event.message_id);
  CORE_TEST_ASSERT_EQ_U32(0U, record->event.cycle_sample_id);
  CORE_TEST_ASSERT_EQ_U32(17U, record->context.operation);
  CORE_TEST_ASSERT_EQ_U32(1U, record->context.context_schema);
  CORE_TEST_ASSERT_EQ_U32(14U, record->context.context_length);
  const uint8_t *context = record->context.context;
  CORE_TEST_ASSERT_EQ_U32(attempt, context[0]);
  CORE_TEST_ASSERT_EQ_U32(ack ? 1U : 0U, context[1]);
  CORE_TEST_ASSERT_EQ_U64(first, little(context + 2U, 8U));
  CORE_TEST_ASSERT_EQ_U32(header, little(context + 10U, 2U));
  CORE_TEST_ASSERT_EQ_U32(payload, little(context + 12U, 2U));
  return true;
}

static bool valid_statuses_and_logging_failure(void) {
  for (uint8_t status = 0U; status < 4U; ++status) {
    node_rtc_record_t rtc;
    node_platform_ports_t platform;
    core_test_setup(&rtc, &platform);
    const uint64_t set = fake_node_core.now_us;
    const uint64_t done = set + core_test_reading_airtime_us();
    CORE_TEST_ASSERT(core_test_script_ack(CORE_TEST_FIRST_MESSAGE_ID,
                                          (cura_lora_v2_domain_t)(3U + status),
                                          status, set, done, done + 1000U));
    observations(0U, 2U, 1U, done + 10U);
    fake_node_core.diagnostic_event_errors[0] =
        curag_error_make(CURAG_EDOM_PERSISTENCE, 3U);
    fake_node_core.diagnostic_event_error_count = 1U;
    core_test_run(&rtc, &platform);
    CORE_TEST_ASSERT_EQ_SIZE(1U, fake_node_core.diagnostic_event_count);
    CORE_TEST_ASSERT(check_summary(0U, 1U, true, 2U, 1U, done + 10U));
    CORE_TEST_ASSERT_EQ_U32(
        status + 1U,
        fake_node_core.delivery_events[1].detail.finished.final_result);
    CORE_TEST_ASSERT_EQ_SIZE(1U, fake_node_core.transmission_count);
    CORE_TEST_ASSERT_EQ_U32(status == 0U, rtc.metrics.current_accepted);
    const size_t first =
        fake_node_core_trace_find(FAKE_CORE_TRACE_DELIVERY_EVENT, 0U);
    const size_t finished =
        fake_node_core_trace_find(FAKE_CORE_TRACE_DELIVERY_EVENT, first + 1U);
    CORE_TEST_ASSERT(finished <
                     fake_node_core_trace_find(FAKE_CORE_TRACE_DIAGNOSTIC, 0U));
    CORE_TEST_ASSERT(core_test_cleanup_is_complete());
  }
  return true;
}

static bool merge_calls_and_saturate(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  const uint64_t set = fake_node_core.now_us;
  const uint64_t done = set + core_test_reading_airtime_us();
  uint8_t invalid[] = {0U};
  fake_node_core_script_rx_packet(invalid, sizeof(invalid), done + 100U,
                                  done + 100U);
  CORE_TEST_ASSERT(core_test_script_ack(CORE_TEST_FIRST_MESSAGE_ID, 3U, 0U, set,
                                        done, done + 200U));
  observations(0U, 60000U, 2U, done + 10U);
  observations(1U, 6000U, 4U, done + 110U);
  core_test_run(&rtc, &platform);
  CORE_TEST_ASSERT_EQ_SIZE(2U, fake_node_core.diagnostic_event_count);
  CORE_TEST_ASSERT(check_summary(1U, 1U, true, UINT16_MAX, 6U, done + 10U));
  CORE_TEST_ASSERT_EQ_U64(fake_node_core.receive_deadlines[0],
                          fake_node_core.receive_deadlines[1]);
  CORE_TEST_ASSERT_EQ_SIZE(1U, fake_node_core.transmission_count);
  return true;
}

static bool retry_and_exhaustion(void) {
  for (unsigned exhaust = 0U; exhaust < 2U; ++exhaust) {
    node_rtc_record_t rtc;
    node_platform_ports_t platform;
    core_test_setup(&rtc, &platform);
    const uint64_t set = fake_node_core.now_us;
    const uint64_t done = set + core_test_reading_airtime_us();
    const uint64_t retry =
        done + NODE_CORE_ACK_WAIT_US + NODE_CORE_RETRY_JITTER_MIN_US;
    fake_node_core_script_tx_done(set, done);
    fake_node_core_script_rx_deadline(retry);
    observations(0U, 2U, 0U, done + 10U);
    const uint64_t done2 = retry + core_test_reading_airtime_us();
    if (exhaust) {
      fake_node_core_script_tx_done(retry, done2);
      fake_node_core_script_rx_deadline(done2 + NODE_CORE_ACK_WAIT_US);
      observations(1U, 0U, 3U, done2 + 10U);
    } else {
      CORE_TEST_ASSERT(core_test_script_ack(CORE_TEST_FIRST_MESSAGE_ID, 3U, 0U,
                                            retry, done2, done2 + 1000U));
    }
    /* New diagnostic writes must not postpone the second transmission. */
    fake_node_core.diagnostic_advance_us = UINT64_C(5000000);
    core_test_run(&rtc, &platform);
    CORE_TEST_ASSERT_EQ_SIZE(2U, fake_node_core.transmission_count);
    CORE_TEST_ASSERT_EQ_U64(retry,
                            fake_node_core.transmissions[1].called_at_us);
    CORE_TEST_ASSERT_EQ_SIZE(exhaust ? 2U : 1U,
                             fake_node_core.diagnostic_event_count);
    CORE_TEST_ASSERT(check_summary(0U, 1U, false, 2U, 0U, done + 10U));
    if (exhaust)
      CORE_TEST_ASSERT(check_summary(1U, 2U, false, 0U, 3U, done2 + 10U));
    CORE_TEST_ASSERT_EQ_U32(
        exhaust ? NODE_DELIVERY_RESULT_NO_ACK_ATTEMPT_LIMIT
                : NODE_DELIVERY_RESULT_ACCEPTED,
        fake_node_core.delivery_events[1].detail.finished.final_result);
    CORE_TEST_ASSERT(memcmp(fake_node_core.transmissions[0].payload,
                            fake_node_core.transmissions[1].payload,
                            CURA_LORA_V2_READING_FRAME_SIZE) == 0);
  }
  return true;
}

static bool local_failure_preserves_summary(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  const uint64_t set = fake_node_core.now_us;
  const uint64_t done = set + core_test_reading_airtime_us();
  fake_node_core_script_tx_done(set, done);
  fake_node_core_script_rx_error(
      curag_error_make(CURAG_EDOM_RADIO, CURAG_ERADIO_EIO), done + 100U);
  observations(0U, 1U, 0U, done + 10U);
  core_test_run(&rtc, &platform);
  CORE_TEST_ASSERT_EQ_SIZE(2U, fake_node_core.diagnostic_event_count);
  CORE_TEST_ASSERT(check_summary(1U, 1U, false, 1U, 0U, done + 10U));
  CORE_TEST_ASSERT_EQ_U32(
      NODE_DELIVERY_RESULT_LOCAL_RADIO_ERROR,
      fake_node_core.delivery_events[1].detail.finished.final_result);
  CORE_TEST_ASSERT_EQ_SIZE(1U, fake_node_core.transmission_count);
  return true;
}

bool node_core_test_phy(const char *name) {
  if (!strcmp(name, "phy_valid_statuses_and_logging_failure"))
    return valid_statuses_and_logging_failure();
  if (!strcmp(name, "phy_merge_calls_and_saturate"))
    return merge_calls_and_saturate();
  if (!strcmp(name, "phy_retry_and_exhaustion"))
    return retry_and_exhaustion();
  if (!strcmp(name, "phy_local_failure_preserves_summary"))
    return local_failure_preserves_summary();
  return false;
}
