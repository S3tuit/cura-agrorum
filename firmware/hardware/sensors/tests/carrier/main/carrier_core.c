#include "carrier_core.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_random.h"
#include "esp_timer.h"
#include "node_core.h"
#include "node_platform_esp.h"
#include "persistence_test_support.h"
#include "protocol_v2_lora_crypto.h"
#include "sx1262_radio.h"
#include "unity.h"

static struct {
  carrier_core_observation_t *observation;
  unsigned ds_mask, samples, tx_calls, rx_calls, radio_sleeps, terminal_calls;
  bool bme_present, failed;
  uint64_t sleep_us;
  uint8_t frame[CURA_LORA_V2_READING_FRAME_SIZE];
} s_cycle;

err_curag_t __real_node_sensors_sample_all(node_sensor_sample_t *, diagn_context_t *);

err_curag_t __wrap_node_sensors_sample_all(node_sensor_sample_t *sample,
                                         diagn_context_t *diagnostic) {
  if (s_cycle.observation == NULL) {
    return __real_node_sensors_sample_all(sample, diagnostic);
  }
  const int64_t start = esp_timer_get_time();
  const err_curag_t result = __real_node_sensors_sample_all(sample, diagnostic);
  const int64_t duration = esp_timer_get_time() - start;
  ++s_cycle.samples;
  if (sample == NULL || diagnostic == NULL) {
    s_cycle.failed = true;
    return result;
  }
  carrier_core_observation_t *const out = s_cycle.observation;
  out->sample = *sample;
  out->diagnostic = *diagnostic;
  out->result = result;
  out->duration_us = duration;
  out->acquisition = carrier_observer_snapshot();
  out->acquisition_valid =
      carrier_observer_sample_valid(s_cycle.ds_mask, s_cycle.bme_present);
  return result;
}

/* Radio is outside this test's sensor/core/persistence layer. Never touch SPI
 * or radio GPIOs, construct an ACK, or let a retry add another acquisition. */
static err_curag_t local_radio_error(diagn_context_t *diagnostic,
                                     curag_operation_t operation) {
  curag_diagnostic_context_clear(diagnostic);
  if (diagnostic != NULL) {
    diagnostic->operation = operation;
    diagnostic->context_schema = CURAG_RADIO_CONTEXT_V1;
    diagnostic->context_length = CURAG_RADIO_CONTEXT_V1_LENGTH;
    diagnostic->context[0] = CURAG_RADIO_STATE_FAILED;
    diagnostic->context[2] = CURAG_RADIO_STAGE_CONFIGURE_GPIO;
    diagnostic->context[4] = CURAG_RADIO_BACKEND_STATUS_ESP_ERR;
    /* Synthetic dependency failure, ESP_FAIL = -1, encoded LE i32. No
     * hardware-touch/status flags are set; no real radio result is claimed. */
    memset(&diagnostic->context[5], 0xff, 4);
  }
  return curag_error_make(CURAG_EDOM_RADIO, CURAG_ERADIO_EIO);
}

err_curag_t sx1262_radio_transmit_uplink(const uint8_t *payload, size_t length,
                                        uint64_t deadline,
                                        sx1262_radio_tx_result_t *result,
                                        diagn_context_t *diagnostic) {
  curag_diagnostic_context_clear(diagnostic);
  if (result != NULL) {
    memset(result, 0, sizeof(*result));
  }
  ++s_cycle.tx_calls;
  if (s_cycle.observation == NULL || payload == NULL || result == NULL ||
      length != sizeof(s_cycle.frame) || s_cycle.tx_calls != 1U ||
      (uint64_t)esp_timer_get_time() >= deadline) {
    s_cycle.failed = true;
  } else {
    memcpy(s_cycle.frame, payload, length);
  }
  return local_radio_error(diagnostic, CURAG_OP_INITIALIZE);
}

err_curag_t sx1262_radio_receive_downlink_until(uint64_t deadline,
                                              sx1262_radio_rx_result_t *result,
                                              diagn_context_t *diagnostic) {
  (void)deadline;
  curag_diagnostic_context_clear(diagnostic);
  if (result != NULL) {
    memset(result, 0, sizeof(*result));
  }
  ++s_cycle.rx_calls;
  s_cycle.failed = true;
  return local_radio_error(diagnostic, CURAG_OP_RECEIVE);
}

err_curag_t sx1262_radio_sleep(diagn_context_t *diagnostic) {
  curag_diagnostic_context_clear(diagnostic);
  ++s_cycle.radio_sleeps;
  return CURAG_OK;
}

static void observe_terminal(void *context, uint64_t duration_us) {
  (void)context;
  ++s_cycle.terminal_calls;
  s_cycle.sleep_us = duration_us;
}

static void print_body(const char *name, const uint8_t *body) {
  printf(" %s=", name);
  for (size_t index = 0; index < CURA_LORA_V2_READING_BODY_SIZE; ++index) {
    printf("%02x", body[index]);
  }
}

void carrier_core_run(const uint64_t roms[2], unsigned ds_mask, bool bme_present,
                      carrier_core_observation_t *out) {
  /* Test labels overlap production flash. Erasing them destroys any prior
   * production counters and logs; the disposable identity below cannot protect
   * that earlier lifetime. Production reuse requires a new node ID AND key. */
  puts("CARRIER_STORAGE_ERASE WARNING: reading erases nvs_test and storage_test "
       "before and after this case, including any prior production counters "
       "and logs. Before production reuse, provision a NEW node ID AND key; "
       "never restore the old identity or counter backup.");
  fflush(stdout);
  hwtest_erase_state();
  memset(out, 0, sizeof(*out));
  memset(&s_cycle, 0, sizeof(s_cycle));
  s_cycle.observation = out;
  s_cycle.ds_mask = ds_mask;
  s_cycle.bme_present = bme_present;
  /* Disposable local crypto fixture, never provisioned to a receiver or RF
   * device. Every erased counter set gets a new ID AND key. Never log the key. */
  node_identity_t identity;
  esp_fill_random(&identity, sizeof(identity));
  node_rtc_record_t rtc = {0};
  node_platform_ports_t platform = *node_platform_esp_ports();
  platform.system.enter_deep_sleep_for = observe_terminal;
  carrier_observer_begin(roms[0], roms[1]);
  node_cycle_run(&platform, &identity, &rtc);
  s_cycle.observation = NULL;

  cura_lora_v2_clear_header_t header;
  size_t length = 0;
  const cura_lora_v2_crypto_result_t opened = cura_lora_v2_open_frame(
      &header, out->body, sizeof(out->body), &length, identity.node_key,
      s_cycle.frame, sizeof(s_cycle.frame));
  const bool node_id_matches = opened == CURA_LORA_V2_CRYPTO_OK &&
      memcmp(header.node_id, identity.node_id, sizeof(identity.node_id)) == 0;
  memset(&identity, 0, sizeof(identity));

  TEST_ASSERT_FALSE_MESSAGE(s_cycle.failed, "local core adapter contract failed");
  TEST_ASSERT_EQUAL_UINT(1, s_cycle.samples);
  TEST_ASSERT_EQUAL_UINT(1, s_cycle.tx_calls);
  TEST_ASSERT_EQUAL_UINT(0, s_cycle.rx_calls);
  TEST_ASSERT_EQUAL_UINT(1, s_cycle.radio_sleeps);
  TEST_ASSERT_EQUAL_UINT(1, s_cycle.terminal_calls);
  TEST_ASSERT_EQUAL_UINT64(NODE_CORE_DEEP_SLEEP_DURATION_US, s_cycle.sleep_us);
  TEST_ASSERT_EQUAL(CURA_LORA_V2_CRYPTO_OK, opened);
  TEST_ASSERT_TRUE(node_id_matches);
  TEST_ASSERT_EQUAL_UINT(CURA_LORA_V2_READING_BODY_SIZE, length);
  TEST_ASSERT_EQUAL_HEX8(CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK, header.domain);
  TEST_ASSERT_EQUAL_HEX8(CURA_LORA_V2_CONTROL, header.control);
  TEST_ASSERT_EQUAL_UINT32(0, header.message_id);
  TEST_ASSERT_EQUAL(CURA_LORA_V2_CODEC_OK,
                    cura_lora_v2_decode_reading(&out->reading, out->body, length));
  TEST_ASSERT_EQUAL_UINT32(0, out->reading.sample_id);
  TEST_ASSERT_EQUAL_UINT8(platform.system.get_reset_reason(platform.system.context),
                         out->reading.reset_reason);
  TEST_ASSERT_EQUAL_HEX16(0, out->reading.flags &
      (CURA_LORA_V2_FLAG_PREVIOUS_CYCLE_METRICS_VALID |
       CURA_LORA_V2_FLAG_PREVIOUS_CURRENT_ACCEPTED));

  /* Close/reopen real storage before inspecting; do not re-encode a synthetic
   * reading or use an append observer as proof of a durable canonical body. */
  hwtest_simulate_component_restart();
  TEST_ASSERT_EQUAL_UINT32(1, hwtest_read_next_sample_id());
  TEST_ASSERT_EQUAL_UINT32(1, hwtest_read_next_message_id());
  hwtest_snapshot_t pending;
  hwtest_snapshot(HWTEST_PENDING_PATH, &pending);
  TEST_ASSERT_EQUAL_UINT(NODE_PERSISTENCE_RECORD_OVERHEAD +
                        CURA_LORA_V2_READING_BODY_SIZE, pending.length);
  TEST_ASSERT_EQUAL(NODE_PERSISTENCE_RECORD_VALID,
      node_persistence_record_validate(node_persistence_backend(),
          NODE_PERSISTENCE_LOG_PENDING, pending.bytes, pending.length));
  TEST_ASSERT_TRUE(node_persistence_record_decode_reading(
      pending.bytes, pending.length, out->pending_body));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(out->body, out->pending_body, sizeof(out->body));

  printf("CARRIER_READING samples=%u tx_calls=%u rx_calls=%u terminal_calls=%u"
         " sleep_us=%" PRIu64, s_cycle.samples, s_cycle.tx_calls,
         s_cycle.rx_calls, s_cycle.terminal_calls, s_cycle.sleep_us);
  print_body("body", out->body);
  print_body("pending_body", out->pending_body);
  putchar('\n');
  hwtest_finish_case();
}
