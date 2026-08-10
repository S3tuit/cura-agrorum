#include "node_core_test.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "protocol_v2_lora_crypto.h"

const node_identity_t CORE_TEST_IDENTITY = {
    .node_id = {UINT8_C(0x01), UINT8_C(0x02), UINT8_C(0x03), UINT8_C(0x04),
                UINT8_C(0x05), UINT8_C(0x06), UINT8_C(0x07), UINT8_C(0x08)},
    .node_key = {UINT8_C(0xc0), UINT8_C(0xf9), UINT8_C(0xa1), UINT8_C(0xa0),
                 UINT8_C(0xf3), UINT8_C(0x86), UINT8_C(0x69), UINT8_C(0x2e),
                 UINT8_C(0x01), UINT8_C(0x02), UINT8_C(0x80), UINT8_C(0x82),
                 UINT8_C(0xbe), UINT8_C(0x92), UINT8_C(0x33), UINT8_C(0x0e)},
};

void core_test_setup(node_rtc_record_t *rtc,
                     node_platform_ports_t *out_platform) {
  fake_node_core_reset();
  memset(rtc, 0, sizeof(*rtc));
  fake_node_core.observed_rtc = rtc;
  *out_platform = fake_node_core_platform();
}

void core_test_run(node_rtc_record_t *rtc,
                   const node_platform_ports_t *platform) {
  node_cycle_run(platform, &CORE_TEST_IDENTITY, rtc);
}

cura_lora_v2_reading_t core_test_reading(uint16_t marker) {
  return (cura_lora_v2_reading_t){
      .run_ms = marker,
      .soil_0_mv = (uint16_t)(marker + 1U),
      .flags = CURA_LORA_V2_FLAG_SOIL_0_VALID,
  };
}

bool core_test_script_ack(uint32_t sample_id, cura_lora_v2_domain_t domain,
                          cura_lora_v2_ack_status_t status,
                          uint64_t set_tx_at_us, uint64_t tx_done_at_us,
                          uint64_t ack_at_us) {
  uint8_t ack[CURA_LORA_V2_ACK_FRAME_SIZE];
  if (!fake_node_core_make_ack(ack, CORE_TEST_IDENTITY.node_key,
                               CORE_TEST_IDENTITY.node_id, sample_id,
                               CURA_LORA_V2_CONTROL, domain, status)) {
    return false;
  }
  fake_node_core_script_tx_done(set_tx_at_us, tx_done_at_us);
  fake_node_core_script_rx_packet(ack, sizeof(ack), ack_at_us, ack_at_us);
  return true;
}

bool core_test_decode_transmission(size_t index,
                                   cura_lora_v2_clear_header_t *out_header,
                                   cura_lora_v2_reading_t *out_reading) {
  if (index >= fake_node_core.transmission_count || out_header == NULL ||
      out_reading == NULL) {
    return false;
  }
  uint8_t body[CURA_LORA_V2_READING_BODY_SIZE];
  size_t body_length = 0U;
  const fake_node_core_captured_tx_t *transmission =
      &fake_node_core.transmissions[index];
  return cura_lora_v2_open_frame(
             out_header, body, sizeof(body), &body_length,
             CORE_TEST_IDENTITY.node_key, transmission->payload,
             transmission->payload_length) == CURA_LORA_V2_CRYPTO_OK &&
         body_length == sizeof(body) &&
         cura_lora_v2_decode_reading(out_reading, body, sizeof(body)) ==
             CURA_LORA_V2_CODEC_OK;
}

bool core_test_has_diagnostic(curag_error_domain_t domain,
                              curag_error_code_t code) {
  for (size_t index = 0U; index < fake_node_core.diagnostic_event_count;
       index++) {
    const err_curag_t error =
        fake_node_core.diagnostic_events[index].event.error;
    if (curag_error_domain(error) == domain &&
        curag_error_code(error) == code) {
      return true;
    }
  }
  return false;
}

bool core_test_cleanup_is_complete(void) {
  return fake_node_core.force_power_off_call_count == 1U &&
         fake_node_core.radio_sleep_call_count == 1U &&
         fake_node_core.sync_call_count == 1U &&
         fake_node_core.deep_sleep_call_count == 1U &&
         fake_node_core.deep_sleep_duration_us ==
             NODE_CORE_DEEP_SLEEP_DURATION_US &&
         fake_node_core.calls_after_deep_sleep == 0U;
}
