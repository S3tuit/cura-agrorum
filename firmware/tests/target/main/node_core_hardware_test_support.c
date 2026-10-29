#include "node_core_hardware_test_support.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_system.h"
#include "node_common.h"
#include "node_platform_esp.h"
#include "persistence_test_support.h"
#include "protocol_v2_lora_crypto.h"
#include "sx1262_radio.h"
#include "unity.h"

#define CORE_HWTEST_SENSOR_DURATION_US UINT64_C(5000)
#define CORE_HWTEST_RADIO_SETUP_US UINT64_C(1000)
#define CORE_HWTEST_ACK_DELAY_US UINT64_C(2000)

RTC_DATA_ATTR node_rtc_record_t core_hwtest_rtc_record;
RTC_DATA_ATTR core_hwtest_retained_state_t core_hwtest_retained;
RTC_NOINIT_ATTR core_hwtest_restart_evidence_t core_hwtest_restart_evidence;

const node_identity_t CORE_HWTEST_IDENTITY = {
    .node_id = {0x43, 0x55, 0x52, 0x41, 0x48, 0x57, 0x54, 0x31},
    .node_key = {0x10, 0x32, 0x54, 0x76, 0x98, 0xba, 0xdc, 0xfe, 0x01, 0x23,
                 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef},
};

typedef struct {
  uint64_t now_us;
  node_sensor_sample_t sensor_sample;
  core_hwtest_response_t responses[CORE_HWTEST_MAX_RESPONSES];
  size_t response_count;
  size_t response_index;
} core_hwtest_boot_state_t;

static core_hwtest_boot_state_t s_boot;

static void clear_diag(diagn_context_t *diag) {
  if (diag != NULL) {
    curag_diagnostic_context_clear(diag);
  }
}

void core_hwtest_reset_retained(void) {
  memset(&core_hwtest_rtc_record, 0, sizeof(core_hwtest_rtc_record));
  memset(&core_hwtest_retained, 0, sizeof(core_hwtest_retained));
  memset(&core_hwtest_restart_evidence, 0,
         sizeof(core_hwtest_restart_evidence));
}

void core_hwtest_begin_case(void) {
  hwtest_erase_state();
  core_hwtest_reset_retained();
}

void core_hwtest_finish_case(void) {
  hwtest_finish_case();
  core_hwtest_reset_retained();
}

node_sensor_sample_t core_hwtest_sensor_sample(uint16_t marker) {
  return (node_sensor_sample_t){
      .soil_0_mv = (uint16_t)(1000U + marker),
      .soil_1_mv = (uint16_t)(1100U + marker),
      .soil_temp_0_centi_c = (int16_t)(2000 + marker),
      .soil_temp_1_centi_c = (int16_t)(1900 + marker),
      .enclosure_centi_c = (int16_t)(2200 + marker),
      .enclosure_pressure_pa = UINT32_C(100000) + marker,
      .enclosure_humidity_centi_pct = (uint16_t)(5000U + marker),
      .validity = NODE_SENSOR_VALIDITY_ALL,
  };
}

void core_hwtest_configure_boot(const node_sensor_sample_t *sensor_sample,
                                const core_hwtest_response_t *responses,
                                size_t response_count) {
  memset(&s_boot, 0, sizeof(s_boot));
  if (sensor_sample != NULL) {
    s_boot.sensor_sample = *sensor_sample;
  }
  if (responses == NULL || response_count > CORE_HWTEST_MAX_RESPONSES) {
    if (response_count != 0U) {
      core_hwtest_retained.adapter_contract_failed = true;
    }
    return;
  }
  memcpy(s_boot.responses, responses,
         response_count * sizeof(s_boot.responses[0]));
  s_boot.response_count = response_count;
}

static uint64_t monotonic_us(void *context) {
  (void)context;
  return s_boot.now_us;
}

static uint32_t uniform_u32_inclusive(void *context, uint32_t minimum,
                                      uint32_t maximum) {
  (void)context;
  if (minimum > maximum) {
    core_hwtest_retained.adapter_contract_failed = true;
    return minimum;
  }
  return minimum;
}

node_platform_ports_t core_hwtest_platform(void) {
  const node_platform_ports_t *const production = node_platform_esp_ports();
  return (node_platform_ports_t){
      .clock = {.context = NULL, .monotonic_us = monotonic_us},
      .randomness = {.context = NULL,
                     .uniform_u32_inclusive = uniform_u32_inclusive},
      .system = production->system,
  };
}

void core_hwtest_run_cycle(const node_sensor_sample_t *sensor_sample,
                           const core_hwtest_response_t *responses,
                           size_t response_count) {
  core_hwtest_configure_boot(sensor_sample, responses, response_count);
  const node_platform_ports_t platform = core_hwtest_platform();
  node_cycle_run(&platform, &CORE_HWTEST_IDENTITY, &core_hwtest_rtc_record);
  core_hwtest_retained.adapter_contract_failed = true;
  TEST_FAIL_MESSAGE("node_cycle_run returned instead of entering deep sleep");
}

bool core_hwtest_decode_transmission(size_t index,
                                     cura_lora_v2_clear_header_t *out_header,
                                     cura_lora_v2_reading_t *out_reading) {
  if (index >= core_hwtest_retained.transmission_count || out_header == NULL ||
      out_reading == NULL) {
    return false;
  }
  uint8_t body[CURA_LORA_V2_READING_BODY_SIZE];
  size_t body_length = 0U;
  const cura_lora_v2_authenticated_reading_frame_t *const transmission =
      &core_hwtest_retained.transmissions[index];
  return cura_lora_v2_open_frame(
             out_header, body, sizeof(body), &body_length,
             CORE_HWTEST_IDENTITY.node_key, transmission->bytes,
             sizeof(transmission->bytes)) == CURA_LORA_V2_CRYPTO_OK &&
         body_length == sizeof(body) &&
         cura_lora_v2_decode_reading(out_reading, body, sizeof(body)) ==
             CURA_LORA_V2_CODEC_OK;
}

void core_hwtest_arm_restart_after_rtc_take(uint32_t expected_sample_id) {
  core_hwtest_restart_evidence.restart_hook_expected_sample_id =
      expected_sample_id;
  core_hwtest_restart_evidence.restart_hook_armed = true;
}

void core_hwtest_assert_adapter_counts(uint32_t expected_sensor_calls,
                                       uint32_t expected_cleanup_calls) {
  TEST_ASSERT_FALSE(core_hwtest_retained.adapter_contract_failed);
  TEST_ASSERT_FALSE(core_hwtest_restart_evidence.hook_contract_failed);
  TEST_ASSERT_EQUAL_UINT32(expected_sensor_calls,
                           core_hwtest_retained.sensor_call_count);
  TEST_ASSERT_EQUAL_UINT32(expected_cleanup_calls,
                           core_hwtest_retained.force_power_off_call_count);
  TEST_ASSERT_EQUAL_UINT32(expected_cleanup_calls,
                           core_hwtest_retained.radio_sleep_call_count);
}

err_curag_t node_sensors_sample_all(node_sensor_sample_t *out_sample,
                                    diagn_context_t *out_diag) {
  clear_diag(out_diag);
  if (out_sample == NULL) {
    core_hwtest_retained.adapter_contract_failed = true;
    return CURAG_ESENSORS_EINVALID_ARGUMENT;
  }
  ++core_hwtest_retained.sensor_call_count;
  *out_sample = s_boot.sensor_sample;
  s_boot.now_us += CORE_HWTEST_SENSOR_DURATION_US;
  return CURAG_OK;
}

err_curag_t node_sensors_force_power_off(diagn_context_t *out_diag) {
  clear_diag(out_diag);
  ++core_hwtest_retained.force_power_off_call_count;
  return CURAG_OK;
}

err_curag_t sx1262_radio_transmit_uplink(const uint8_t *payload,
                                         size_t payload_length,
                                         uint64_t deadline_monotonic_us,
                                         sx1262_radio_tx_result_t *out_result,
                                         diagn_context_t *out_diag) {
  clear_diag(out_diag);
  if (payload == NULL || out_result == NULL ||
      payload_length != CURA_LORA_V2_READING_FRAME_SIZE ||
      core_hwtest_retained.transmission_count >=
          CORE_HWTEST_MAX_TRANSMISSIONS) {
    core_hwtest_retained.adapter_contract_failed = true;
    return CURAG_ERADIO_EINVALID_ARGUMENT;
  }
  const uint64_t airtime_us = sx1262_radio_airtime_us(payload_length);
  const uint64_t set_tx_at_us = s_boot.now_us + CORE_HWTEST_RADIO_SETUP_US;
  const uint64_t tx_done_at_us = set_tx_at_us + airtime_us;
  if (airtime_us == UINT64_MAX || tx_done_at_us > deadline_monotonic_us) {
    core_hwtest_retained.adapter_contract_failed = true;
    return CURAG_ERADIO_EDEADLINE;
  }

  cura_lora_v2_authenticated_reading_frame_t *const captured =
      &core_hwtest_retained
           .transmissions[core_hwtest_retained.transmission_count++];
  memcpy(captured->bytes, payload, sizeof(captured->bytes));
  cura_lora_v2_clear_header_t header;
  if (cura_lora_v2_decode_clear_header(&header, captured->bytes,
                                       CURA_LORA_V2_CLEAR_HEADER_SIZE) !=
      CURA_LORA_V2_CODEC_OK) {
    core_hwtest_retained.adapter_contract_failed = true;
    return CURAG_ERADIO_EIO;
  }
  if (header.domain == CURA_LORA_V2_DOMAIN_BACKLOG_READING_UPLINK) {
    const bool binding_matches =
        hwtest_pending_tail_matches_binding(header.message_id, captured);
    core_hwtest_retained.backlog_binding_verified_before_tx |= binding_matches;
    if (!binding_matches) {
      core_hwtest_retained.adapter_contract_failed = true;
      return CURAG_ERADIO_EIO;
    }
  }
  *out_result = (sx1262_radio_tx_result_t){
      .tx_started = true,
      .tx_done = true,
      .set_tx_at_us = set_tx_at_us,
      .tx_done_at_us = tx_done_at_us,
  };
  s_boot.now_us = tx_done_at_us;
  return CURAG_OK;
}

static cura_lora_v2_domain_t ack_domain(core_hwtest_response_t response) {
  switch (response) {
  case CORE_HWTEST_RESPONSE_ACCEPTED:
    return CURA_LORA_V2_DOMAIN_ACK_ACCEPTED_DOWNLINK;
  case CORE_HWTEST_RESPONSE_RETRY_LATER:
    return CURA_LORA_V2_DOMAIN_ACK_RETRY_LATER_DOWNLINK;
  case CORE_HWTEST_RESPONSE_REJECTED_UNSUPPORTED:
    return CURA_LORA_V2_DOMAIN_ACK_REJECTED_UNSUPPORTED_DOWNLINK;
  case CORE_HWTEST_RESPONSE_REJECTED_MALFORMED:
    return CURA_LORA_V2_DOMAIN_ACK_REJECTED_MALFORMED_DOWNLINK;
  case CORE_HWTEST_RESPONSE_SILENCE:
  default:
    return 0U;
  }
}

static cura_lora_v2_ack_status_t ack_status(core_hwtest_response_t response) {
  switch (response) {
  case CORE_HWTEST_RESPONSE_ACCEPTED:
    return CURA_LORA_V2_ACK_STATUS_ACCEPTED;
  case CORE_HWTEST_RESPONSE_RETRY_LATER:
    return CURA_LORA_V2_ACK_STATUS_RETRY_LATER;
  case CORE_HWTEST_RESPONSE_REJECTED_UNSUPPORTED:
    return CURA_LORA_V2_ACK_STATUS_REJECTED_UNSUPPORTED;
  case CORE_HWTEST_RESPONSE_REJECTED_MALFORMED:
    return CURA_LORA_V2_ACK_STATUS_REJECTED_MALFORMED;
  case CORE_HWTEST_RESPONSE_SILENCE:
  default:
    return CURA_LORA_V2_ACK_STATUS_ACCEPTED;
  }
}

err_curag_t
sx1262_radio_receive_downlink_until(uint64_t deadline_monotonic_us,
                                    sx1262_radio_rx_result_t *out_result,
                                    diagn_context_t *out_diag) {
  clear_diag(out_diag);
  if (out_result == NULL || core_hwtest_retained.transmission_count == 0U) {
    core_hwtest_retained.adapter_contract_failed = true;
    return CURAG_ERADIO_EINVALID_ARGUMENT;
  }

  core_hwtest_response_t response = CORE_HWTEST_RESPONSE_SILENCE;
  if (s_boot.response_index < s_boot.response_count) {
    response = s_boot.responses[s_boot.response_index++];
  }
  if (response == CORE_HWTEST_RESPONSE_SILENCE) {
    s_boot.now_us = deadline_monotonic_us;
    *out_result = (sx1262_radio_rx_result_t){
        .outcome = SX1262_RADIO_RX_DEADLINE,
    };
    return CURAG_OK;
  }

  const cura_lora_v2_authenticated_reading_frame_t *const transmitted =
      &core_hwtest_retained
           .transmissions[core_hwtest_retained.transmission_count - 1U];
  cura_lora_v2_clear_header_t uplink_header;
  if (cura_lora_v2_decode_clear_header(&uplink_header, transmitted->bytes,
                                       CURA_LORA_V2_CLEAR_HEADER_SIZE) !=
      CURA_LORA_V2_CODEC_OK) {
    core_hwtest_retained.adapter_contract_failed = true;
    return CURAG_ERADIO_EIO;
  }

  const cura_lora_v2_ack_t ack = {.status = ack_status(response)};
  uint8_t ack_body[CURA_LORA_V2_ACK_BODY_SIZE];
  if (cura_lora_v2_encode_ack(ack_body, sizeof(ack_body), &ack) !=
      CURA_LORA_V2_CODEC_OK) {
    core_hwtest_retained.adapter_contract_failed = true;
    return CURAG_ERADIO_EIO;
  }
  cura_lora_v2_clear_header_t ack_header = {
      .control = CURA_LORA_V2_CONTROL,
      .domain = ack_domain(response),
      .message_id = uplink_header.message_id,
  };
  memcpy(ack_header.node_id, CORE_HWTEST_IDENTITY.node_id,
         sizeof(ack_header.node_id));
  size_t frame_length = 0U;
  uint8_t ack_frame[CURA_LORA_V2_ACK_FRAME_SIZE];
  if (cura_lora_v2_seal_frame(ack_frame, sizeof(ack_frame), &frame_length,
                              CORE_HWTEST_IDENTITY.node_key, &ack_header,
                              ack_body,
                              sizeof(ack_body)) != CURA_LORA_V2_CRYPTO_OK ||
      frame_length != sizeof(ack_frame)) {
    core_hwtest_retained.adapter_contract_failed = true;
    return CURAG_ERADIO_EIO;
  }

  s_boot.now_us += CORE_HWTEST_ACK_DELAY_US;
  *out_result = (sx1262_radio_rx_result_t){
      .outcome = SX1262_RADIO_RX_PACKET,
      .rx_done_at_us = s_boot.now_us,
      .payload_length = (uint8_t)sizeof(ack_frame),
  };
  memcpy(out_result->payload, ack_frame, sizeof(ack_frame));
  return CURAG_OK;
}

err_curag_t sx1262_radio_sleep(diagn_context_t *out_diag) {
  clear_diag(out_diag);
  ++core_hwtest_retained.radio_sleep_call_count;
  return CURAG_OK;
}

void node_core_test_observe_rtc_precommit(const node_rtc_record_t *record) {
  if (record == NULL || record->commit_marker != 0U) {
    core_hwtest_retained.adapter_contract_failed = true;
  }
}

void node_core_test_after_rtc_take(const node_rtc_record_t *retained,
                                   const node_rtc_record_t *incoming_copy) {
  ++core_hwtest_restart_evidence.hook_call_count;
  core_hwtest_restart_evidence.hook_saw_invalid_retained |=
      retained != NULL && retained->commit_marker == 0U;
  core_hwtest_restart_evidence.hook_saw_committed_copy |=
      incoming_copy != NULL &&
      incoming_copy->commit_marker == NODE_RTC_COMMITTED_V1;
  if (retained == NULL || incoming_copy == NULL) {
    core_hwtest_restart_evidence.hook_contract_failed = true;
  }
  if (!core_hwtest_restart_evidence.restart_hook_armed) {
    return;
  }

  core_hwtest_restart_evidence.restart_hook_armed = false;
  core_hwtest_restart_evidence.restart_hook_fired = true;
  if (retained == NULL || retained->commit_marker != 0U ||
      incoming_copy == NULL ||
      incoming_copy->commit_marker != NODE_RTC_COMMITTED_V1 ||
      incoming_copy->completed_sample_id !=
          core_hwtest_restart_evidence.restart_hook_expected_sample_id) {
    core_hwtest_restart_evidence.hook_contract_failed = true;
  }
  esp_restart();
  abort();
}
