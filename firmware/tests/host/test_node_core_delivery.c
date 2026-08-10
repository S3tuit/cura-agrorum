#include "node_core_test.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "protocol_v2_lora_crypto.h"

static bool make_raw_ack(uint8_t output[CURA_LORA_V2_ACK_FRAME_SIZE],
                         const uint8_t node_id[8], uint32_t sample_id,
                         uint8_t control, cura_lora_v2_domain_t domain,
                         uint8_t status) {
  const cura_lora_v2_clear_header_t initial = {
      .control = control,
      .domain = domain,
      .sample_id = sample_id,
  };
  cura_lora_v2_clear_header_t header = initial;
  memcpy(header.node_id, node_id, sizeof(header.node_id));
  size_t length = 0U;
  return cura_lora_v2_seal_frame(output, CURA_LORA_V2_ACK_FRAME_SIZE, &length,
                                 CORE_TEST_IDENTITY.node_key, &header, &status,
                                 sizeof(status)) == CURA_LORA_V2_CRYPTO_OK &&
         length == CURA_LORA_V2_ACK_FRAME_SIZE;
}

static bool retries_reuse_identical_frame(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  const uint64_t tx1_set = UINT64_C(1001000);
  const uint64_t tx1_done = tx1_set + NODE_CORE_READING_AIRTIME_US;
  const uint64_t retry_at =
      tx1_done + NODE_CORE_ACK_WAIT_US + NODE_CORE_RETRY_JITTER_MIN_US;
  fake_node_core_script_tx_done(tx1_set, tx1_done);
  fake_node_core_script_rx_deadline(retry_at);
  CORE_TEST_ASSERT(core_test_script_ack(
      0U, CURA_LORA_V2_DOMAIN_ACK_ACCEPTED_DOWNLINK,
      CURA_LORA_V2_ACK_STATUS_ACCEPTED, retry_at + UINT64_C(1000),
      retry_at + UINT64_C(1000) + NODE_CORE_READING_AIRTIME_US,
      retry_at + UINT64_C(120000)));

  core_test_run(&rtc, &platform);

  CORE_TEST_ASSERT_EQ_SIZE(2U, fake_node_core.transmission_count);
  CORE_TEST_ASSERT(memcmp(fake_node_core.transmissions[0].payload,
                          fake_node_core.transmissions[1].payload,
                          CURA_LORA_V2_READING_FRAME_SIZE) == 0);
  CORE_TEST_ASSERT_EQ_SIZE(2U, fake_node_core.random_call_count);
  CORE_TEST_ASSERT_EQ_U32(NODE_CORE_RETRY_JITTER_MIN_US,
                          fake_node_core.random_minimums[0]);
  CORE_TEST_ASSERT_EQ_U32(NODE_CORE_RETRY_JITTER_MAX_US,
                          fake_node_core.random_maximums[0]);
  CORE_TEST_ASSERT_EQ_U32(2U, rtc.metrics.current_tx_attempts);
  CORE_TEST_ASSERT_EQ_SIZE(2U, fake_node_core.delivery_event_count);
  CORE_TEST_ASSERT_EQ_U32(
      2U, fake_node_core.delivery_events[1].detail.finished.attempt_count);
  return true;
}

static bool invalid_acks_share_one_receive_interval(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  const uint64_t tx_set = UINT64_C(1001000);
  const uint64_t tx_done = tx_set + NODE_CORE_READING_AIRTIME_US;
  const uint64_t retry_at =
      tx_done + NODE_CORE_ACK_WAIT_US + NODE_CORE_RETRY_JITTER_MIN_US;
  fake_node_core_script_tx_done(tx_set, tx_done);

  uint8_t valid[CURA_LORA_V2_ACK_FRAME_SIZE];
  CORE_TEST_ASSERT(fake_node_core_make_ack(
      valid, CORE_TEST_IDENTITY.node_key, CORE_TEST_IDENTITY.node_id, 0U,
      CURA_LORA_V2_CONTROL, CURA_LORA_V2_DOMAIN_ACK_ACCEPTED_DOWNLINK,
      CURA_LORA_V2_ACK_STATUS_ACCEPTED));
  fake_node_core_script_rx_packet(valid, sizeof(valid) - 1U,
                                  tx_done + UINT64_C(1000),
                                  tx_done + UINT64_C(1000));

  uint8_t bad_tag[sizeof(valid)];
  memcpy(bad_tag, valid, sizeof(bad_tag));
  bad_tag[sizeof(bad_tag) - 1U] ^= UINT8_C(1);
  fake_node_core_script_rx_packet(bad_tag, sizeof(bad_tag),
                                  tx_done + UINT64_C(2000),
                                  tx_done + UINT64_C(2000));

  uint8_t foreign_id[8];
  memcpy(foreign_id, CORE_TEST_IDENTITY.node_id, sizeof(foreign_id));
  foreign_id[0] ^= UINT8_C(0x80);
  uint8_t packet[CURA_LORA_V2_ACK_FRAME_SIZE];
  CORE_TEST_ASSERT(make_raw_ack(packet, foreign_id, 0U, CURA_LORA_V2_CONTROL,
                                CURA_LORA_V2_DOMAIN_ACK_ACCEPTED_DOWNLINK,
                                CURA_LORA_V2_ACK_STATUS_ACCEPTED));
  fake_node_core_script_rx_packet(packet, sizeof(packet),
                                  tx_done + UINT64_C(3000),
                                  tx_done + UINT64_C(3000));

  CORE_TEST_ASSERT(make_raw_ack(packet, CORE_TEST_IDENTITY.node_id, 1U,
                                CURA_LORA_V2_CONTROL,
                                CURA_LORA_V2_DOMAIN_ACK_ACCEPTED_DOWNLINK,
                                CURA_LORA_V2_ACK_STATUS_ACCEPTED));
  fake_node_core_script_rx_packet(packet, sizeof(packet),
                                  tx_done + UINT64_C(4000),
                                  tx_done + UINT64_C(4000));

  CORE_TEST_ASSERT(make_raw_ack(packet, CORE_TEST_IDENTITY.node_id, 0U,
                                UINT8_C(0x21),
                                CURA_LORA_V2_DOMAIN_ACK_ACCEPTED_DOWNLINK,
                                CURA_LORA_V2_ACK_STATUS_ACCEPTED));
  fake_node_core_script_rx_packet(packet, sizeof(packet),
                                  tx_done + UINT64_C(5000),
                                  tx_done + UINT64_C(5000));

  CORE_TEST_ASSERT(make_raw_ack(packet, CORE_TEST_IDENTITY.node_id, 0U,
                                CURA_LORA_V2_CONTROL,
                                CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK,
                                CURA_LORA_V2_ACK_STATUS_ACCEPTED));
  fake_node_core_script_rx_packet(packet, sizeof(packet),
                                  tx_done + UINT64_C(6000),
                                  tx_done + UINT64_C(6000));

  CORE_TEST_ASSERT(make_raw_ack(packet, CORE_TEST_IDENTITY.node_id, 0U,
                                CURA_LORA_V2_CONTROL,
                                CURA_LORA_V2_DOMAIN_ACK_ACCEPTED_DOWNLINK,
                                CURA_LORA_V2_ACK_STATUS_RETRY_LATER));
  fake_node_core_script_rx_packet(packet, sizeof(packet),
                                  tx_done + UINT64_C(7000),
                                  tx_done + UINT64_C(7000));

  CORE_TEST_ASSERT(
      make_raw_ack(packet, CORE_TEST_IDENTITY.node_id, 0U, CURA_LORA_V2_CONTROL,
                   CURA_LORA_V2_DOMAIN_ACK_ACCEPTED_DOWNLINK, UINT8_C(99)));
  fake_node_core_script_rx_packet(packet, sizeof(packet),
                                  tx_done + UINT64_C(8000),
                                  tx_done + UINT64_C(8000));
  fake_node_core_script_rx_packet(
      valid, sizeof(valid), tx_done + UINT64_C(9000), tx_done + UINT64_C(9000));

  core_test_run(&rtc, &platform);

  CORE_TEST_ASSERT_EQ_SIZE(9U, fake_node_core.receive_count);
  for (size_t index = 0U; index < fake_node_core.receive_count; index++) {
    CORE_TEST_ASSERT_EQ_U64(retry_at, fake_node_core.receive_deadlines[index]);
  }
  CORE_TEST_ASSERT_EQ_SIZE(1U, fake_node_core.transmission_count);
  CORE_TEST_ASSERT(rtc.metrics.current_accepted);
  CORE_TEST_ASSERT(
      core_test_has_diagnostic(CURAG_EDOM_CORE, CURAG_ECORE_EACK_LENGTH));
  CORE_TEST_ASSERT(core_test_has_diagnostic(CURAG_EDOM_CORE,
                                            CURAG_ECORE_EACK_AUTHENTICATION));
  CORE_TEST_ASSERT(
      core_test_has_diagnostic(CURAG_EDOM_CORE, CURAG_ECORE_EACK_NODE_ID));
  CORE_TEST_ASSERT(
      core_test_has_diagnostic(CURAG_EDOM_CORE, CURAG_ECORE_EACK_SAMPLE_ID));
  CORE_TEST_ASSERT(
      core_test_has_diagnostic(CURAG_EDOM_CORE, CURAG_ECORE_EACK_CONTROL));
  CORE_TEST_ASSERT(
      core_test_has_diagnostic(CURAG_EDOM_CORE, CURAG_ECORE_EACK_DOMAIN));
  CORE_TEST_ASSERT(
      core_test_has_diagnostic(CURAG_EDOM_CORE, CURAG_ECORE_EACK_STATUS));
  return true;
}

static bool radio_progress_and_rx_errors_are_accounted(void) {
  for (size_t variant = 0U; variant < 5U; variant++) {
    node_rtc_record_t rtc;
    node_platform_ports_t platform;
    core_test_setup(&rtc, &platform);
    const err_curag_t radio_error =
        curag_error_make(CURAG_EDOM_RADIO, CURAG_ERADIO_EIO);
    if (variant == 0U) {
      fake_node_core_script_tx_error(radio_error, false, false, 0U, 0U,
                                     UINT64_C(1002000));
    } else if (variant == 1U) {
      fake_node_core_script_tx_error(radio_error, true, false,
                                     UINT64_C(1001000), 0U, UINT64_C(1002000));
    } else if (variant == 2U) {
      fake_node_core_script_tx_error(radio_error, true, false,
                                     UINT64_C(1001000), 0U, UINT64_C(1100000));
    } else if (variant == 3U) {
      fake_node_core_script_tx_done(UINT64_C(1001000), UINT64_C(1098536));
      fake_node_core_script_rx_error(radio_error, UINT64_C(1100000));
    } else {
      fake_node_core_script_tx_error(radio_error, true, true, UINT64_C(1001000),
                                     UINT64_C(1098536), UINT64_C(1100000));
    }
    core_test_run(&rtc, &platform);
    const uint8_t expected_attempts = variant == 0U ? UINT8_C(0) : UINT8_C(1);
    CORE_TEST_ASSERT_EQ_U32(expected_attempts, rtc.metrics.current_tx_attempts);
    CORE_TEST_ASSERT_EQ_SIZE(variant == 3U ? 1U : 0U,
                             fake_node_core.receive_count);
    CORE_TEST_ASSERT_EQ_SIZE(variant >= 3U ? 1U : 0U,
                             fake_node_core.random_call_count);
    CORE_TEST_ASSERT_EQ_U32(
        NODE_DELIVERY_RESULT_LOCAL_RADIO_ERROR,
        fake_node_core.delivery_events[1].detail.finished.final_result);
    CORE_TEST_ASSERT(
        core_test_has_diagnostic(CURAG_EDOM_RADIO, CURAG_ERADIO_EIO));
  }
  return true;
}

static bool ack_retry_timestamp_boundary_is_enforced(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  const uint64_t tx_done = UINT64_C(1098536);
  const uint64_t retry_at =
      tx_done + NODE_CORE_ACK_WAIT_US + NODE_CORE_RETRY_JITTER_MIN_US;
  CORE_TEST_ASSERT(core_test_script_ack(
      0U, CURA_LORA_V2_DOMAIN_ACK_ACCEPTED_DOWNLINK,
      CURA_LORA_V2_ACK_STATUS_ACCEPTED, UINT64_C(1001000), tx_done, retry_at));
  core_test_run(&rtc, &platform);
  CORE_TEST_ASSERT_EQ_SIZE(1U, fake_node_core.transmission_count);
  CORE_TEST_ASSERT(rtc.metrics.current_accepted);

  core_test_setup(&rtc, &platform);
  uint8_t accepted[CURA_LORA_V2_ACK_FRAME_SIZE];
  CORE_TEST_ASSERT(fake_node_core_make_ack(
      accepted, CORE_TEST_IDENTITY.node_key, CORE_TEST_IDENTITY.node_id, 0U,
      CURA_LORA_V2_CONTROL, CURA_LORA_V2_DOMAIN_ACK_ACCEPTED_DOWNLINK,
      CURA_LORA_V2_ACK_STATUS_ACCEPTED));
  fake_node_core_script_tx_done(UINT64_C(1001000), tx_done);
  fake_node_core_script_rx_packet(accepted, sizeof(accepted), retry_at + 1U,
                                  retry_at + 1U);
  fake_node_core_script_rx_deadline(retry_at);
  CORE_TEST_ASSERT(core_test_script_ack(
      0U, CURA_LORA_V2_DOMAIN_ACK_ACCEPTED_DOWNLINK,
      CURA_LORA_V2_ACK_STATUS_ACCEPTED, retry_at + UINT64_C(1000),
      retry_at + UINT64_C(1000) + NODE_CORE_READING_AIRTIME_US,
      retry_at + UINT64_C(120000)));
  core_test_run(&rtc, &platform);
  CORE_TEST_ASSERT_EQ_SIZE(2U, fake_node_core.transmission_count);
  CORE_TEST_ASSERT(
      core_test_has_diagnostic(CURAG_EDOM_CORE, CURAG_ECORE_EACK_TIMESTAMP));
  return true;
}

static bool budget_boundaries_are_inclusive_and_independent(void) {
  CORE_TEST_ASSERT(node_core_attempt_fits(
      UINT64_C(1000), UINT64_C(1000) + NODE_CORE_READING_AIRTIME_US,
      NODE_CORE_TX_AIRTIME_BUDGET_US - NODE_CORE_READING_AIRTIME_CHARGE_US));
  CORE_TEST_ASSERT(!node_core_attempt_fits(
      UINT64_C(1000), UINT64_C(1000) + NODE_CORE_READING_AIRTIME_US,
      NODE_CORE_TX_AIRTIME_BUDGET_US - NODE_CORE_READING_AIRTIME_CHARGE_US +
          1U));
  CORE_TEST_ASSERT(!node_core_attempt_fits(
      UINT64_C(1001), UINT64_C(1000) + NODE_CORE_READING_AIRTIME_US, 0U));
  CORE_TEST_ASSERT(node_core_attempt_fits(
      UINT64_C(1000), UINT64_C(1000) + NODE_CORE_READING_AIRTIME_US, 0U));
  CORE_TEST_ASSERT(!node_core_attempt_fits(UINT64_MAX, UINT64_MAX, 0U));
  return true;
}

static bool zero_epoch_first_attempt_timestamp_is_retained(void) {
  node_rtc_record_t rtc;
  node_platform_ports_t platform;
  core_test_setup(&rtc, &platform);
  fake_node_core.now_us = 0U;
  const uint64_t first_done = NODE_CORE_READING_AIRTIME_US;
  const uint64_t retry_at =
      first_done + NODE_CORE_ACK_WAIT_US + NODE_CORE_RETRY_JITTER_MIN_US;
  fake_node_core_script_tx_done(0U, first_done);
  fake_node_core_script_rx_deadline(retry_at);
  const uint64_t second_set = retry_at + UINT64_C(1000);
  const uint64_t second_done = second_set + NODE_CORE_READING_AIRTIME_US;
  const uint64_t accepted_at = second_done + UINT64_C(2000);
  CORE_TEST_ASSERT(core_test_script_ack(
      0U, CURA_LORA_V2_DOMAIN_ACK_ACCEPTED_DOWNLINK,
      CURA_LORA_V2_ACK_STATUS_ACCEPTED, second_set, second_done, accepted_at));
  core_test_run(&rtc, &platform);
  CORE_TEST_ASSERT_EQ_U32((uint16_t)(accepted_at / UINT64_C(1000)),
                          rtc.metrics.current_delivery_ms);
  return true;
}

bool node_core_test_delivery(const char *name) {
  if (strcmp(name, "retries_reuse_identical_frame") == 0) {
    return retries_reuse_identical_frame();
  }
  if (strcmp(name, "invalid_acks_share_one_receive_interval") == 0) {
    return invalid_acks_share_one_receive_interval();
  }
  if (strcmp(name, "radio_progress_and_rx_errors_are_accounted") == 0) {
    return radio_progress_and_rx_errors_are_accounted();
  }
  if (strcmp(name, "ack_retry_timestamp_boundary_is_enforced") == 0) {
    return ack_retry_timestamp_boundary_is_enforced();
  }
  if (strcmp(name, "budget_boundaries_are_inclusive_and_independent") == 0) {
    return budget_boundaries_are_inclusive_and_independent();
  }
  if (strcmp(name, "zero_epoch_first_attempt_timestamp_is_retained") == 0) {
    return zero_epoch_first_attempt_timestamp_is_retained();
  }
  return false;
}
