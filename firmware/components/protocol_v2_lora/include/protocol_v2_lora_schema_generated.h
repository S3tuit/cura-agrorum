/* Generated from protocol/protocol-v2-lora/schemas/protocol_v2_lora.json by protocol/protocol-v2-lora/tools/generate.py.
 * Schema SHA-256: 12f34aea53874681f3ac8db6978f6e202301f57c92682b3080a5e082d51bf333. Do not edit by hand. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CURA_LORA_V2_SCHEMA_SHA256 "12f34aea53874681f3ac8db6978f6e202301f57c92682b3080a5e082d51bf333"
#define CURA_LORA_V2_PROTOCOL_VERSION UINT8_C(2)
#define CURA_LORA_V2_CONTROL UINT8_C(0x20)
#define CURA_LORA_V2_KEY_SIZE 16u
#define CURA_LORA_V2_NONCE_SIZE 13u
#define CURA_LORA_V2_TAG_SIZE 8u
#define CURA_LORA_V2_CLEAR_HEADER_SIZE 14u
#define CURA_LORA_V2_READING_BODY_SIZE 28u
#define CURA_LORA_V2_ACK_BODY_SIZE 1u
#define CURA_LORA_V2_READING_FRAME_SIZE 50u
#define CURA_LORA_V2_ACK_FRAME_SIZE 23u

#define CURA_LORA_V2_CLEAR_HEADER_CONTROL_OFFSET 0u
#define CURA_LORA_V2_CLEAR_HEADER_DOMAIN_OFFSET 1u
#define CURA_LORA_V2_CLEAR_HEADER_NODE_ID_OFFSET 2u
#define CURA_LORA_V2_CLEAR_HEADER_SAMPLE_ID_OFFSET 10u

#define CURA_LORA_V2_NONCE_NODE_ID_OFFSET 0u
#define CURA_LORA_V2_NONCE_SAMPLE_ID_OFFSET 8u
#define CURA_LORA_V2_NONCE_DOMAIN_OFFSET 12u

#define CURA_LORA_V2_READING_RUN_MS_OFFSET 0u
#define CURA_LORA_V2_READING_SOIL_0_MV_OFFSET 2u
#define CURA_LORA_V2_READING_SOIL_1_MV_OFFSET 4u
#define CURA_LORA_V2_READING_SOIL_TEMP_0_CENTI_C_OFFSET 6u
#define CURA_LORA_V2_READING_SOIL_TEMP_1_CENTI_C_OFFSET 8u
#define CURA_LORA_V2_READING_ENCLOSURE_CENTI_C_OFFSET 10u
#define CURA_LORA_V2_READING_ENCLOSURE_PRESSURE_PA_OFFSET 12u
#define CURA_LORA_V2_READING_ENCLOSURE_HUMIDITY_CENTI_PCT_OFFSET 16u
#define CURA_LORA_V2_READING_RESET_REASON_OFFSET 18u
#define CURA_LORA_V2_READING_PREVIOUS_CURRENT_TX_ATTEMPTS_OFFSET 19u
#define CURA_LORA_V2_READING_PREVIOUS_AWAKE_MS_OFFSET 20u
#define CURA_LORA_V2_READING_PREVIOUS_CURRENT_DELIVERY_MS_OFFSET 22u
#define CURA_LORA_V2_READING_PREVIOUS_CYCLE_TX_ATTEMPTS_OFFSET 24u
#define CURA_LORA_V2_READING_PREVIOUS_CYCLE_ACCEPTED_READINGS_OFFSET 25u
#define CURA_LORA_V2_READING_FLAGS_OFFSET 26u

#define CURA_LORA_V2_ACK_STATUS_OFFSET 0u

typedef uint8_t cura_lora_v2_domain_t;
#define CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK UINT8_C(0x01)
#define CURA_LORA_V2_DOMAIN_BACKLOG_READING_UPLINK UINT8_C(0x02)
#define CURA_LORA_V2_DOMAIN_ACK_ACCEPTED_DOWNLINK UINT8_C(0x03)
#define CURA_LORA_V2_DOMAIN_ACK_RETRY_LATER_DOWNLINK UINT8_C(0x04)
#define CURA_LORA_V2_DOMAIN_ACK_REJECTED_UNSUPPORTED_DOWNLINK UINT8_C(0x05)
#define CURA_LORA_V2_DOMAIN_ACK_REJECTED_MALFORMED_DOWNLINK UINT8_C(0x06)

typedef uint8_t cura_lora_v2_ack_status_t;
#define CURA_LORA_V2_ACK_STATUS_ACCEPTED UINT8_C(0)
#define CURA_LORA_V2_ACK_STATUS_RETRY_LATER UINT8_C(1)
#define CURA_LORA_V2_ACK_STATUS_REJECTED_UNSUPPORTED UINT8_C(2)
#define CURA_LORA_V2_ACK_STATUS_REJECTED_MALFORMED UINT8_C(3)

typedef uint8_t cura_lora_v2_reset_reason_t;
#define CURA_LORA_V2_RESET_REASON_ESP_RST_UNKNOWN UINT8_C(0)
#define CURA_LORA_V2_RESET_REASON_ESP_RST_POWERON UINT8_C(1)
#define CURA_LORA_V2_RESET_REASON_ESP_RST_EXT UINT8_C(2)
#define CURA_LORA_V2_RESET_REASON_ESP_RST_SW UINT8_C(3)
#define CURA_LORA_V2_RESET_REASON_ESP_RST_PANIC UINT8_C(4)
#define CURA_LORA_V2_RESET_REASON_ESP_RST_INT_WDT UINT8_C(5)
#define CURA_LORA_V2_RESET_REASON_ESP_RST_TASK_WDT UINT8_C(6)
#define CURA_LORA_V2_RESET_REASON_ESP_RST_WDT UINT8_C(7)
#define CURA_LORA_V2_RESET_REASON_ESP_RST_DEEPSLEEP UINT8_C(8)
#define CURA_LORA_V2_RESET_REASON_ESP_RST_BROWNOUT UINT8_C(9)
#define CURA_LORA_V2_RESET_REASON_ESP_RST_SDIO UINT8_C(10)
#define CURA_LORA_V2_RESET_REASON_ESP_RST_USB UINT8_C(11)
#define CURA_LORA_V2_RESET_REASON_ESP_RST_JTAG UINT8_C(12)
#define CURA_LORA_V2_RESET_REASON_ESP_RST_EFUSE UINT8_C(13)
#define CURA_LORA_V2_RESET_REASON_ESP_RST_PWR_GLITCH UINT8_C(14)
#define CURA_LORA_V2_RESET_REASON_ESP_RST_CPU_LOCKUP UINT8_C(15)

typedef uint16_t cura_lora_v2_reading_flags_t;
#define CURA_LORA_V2_FLAG_DEEP_SLEEP_BOOT UINT16_C(0x0001)
#define CURA_LORA_V2_FLAG_SOIL_0_VALID UINT16_C(0x0002)
#define CURA_LORA_V2_FLAG_SOIL_1_VALID UINT16_C(0x0004)
#define CURA_LORA_V2_FLAG_SOIL_TEMP_0_VALID UINT16_C(0x0008)
#define CURA_LORA_V2_FLAG_SOIL_TEMP_1_VALID UINT16_C(0x0010)
#define CURA_LORA_V2_FLAG_ENCLOSURE_TEMP_VALID UINT16_C(0x0020)
#define CURA_LORA_V2_FLAG_ENCLOSURE_PRESSURE_VALID UINT16_C(0x0040)
#define CURA_LORA_V2_FLAG_ENCLOSURE_HUMIDITY_VALID UINT16_C(0x0080)
#define CURA_LORA_V2_FLAG_PREVIOUS_CYCLE_METRICS_VALID UINT16_C(0x0100)
#define CURA_LORA_V2_FLAG_PREVIOUS_CURRENT_ACCEPTED UINT16_C(0x0200)
#define CURA_LORA_V2_READING_RESERVED_FLAGS_MASK UINT16_C(0xfc00)

typedef enum {
  CURA_LORA_V2_CODEC_OK = 0,
  CURA_LORA_V2_CODEC_INVALID_ARGUMENT,
  CURA_LORA_V2_CODEC_BUFFER_TOO_SMALL,
  CURA_LORA_V2_CODEC_INVALID_LENGTH,
  CURA_LORA_V2_CODEC_MALFORMED,
} cura_lora_v2_codec_result_t;

typedef struct {
  uint8_t control;
  uint8_t domain;
  uint8_t node_id[8];
  uint32_t sample_id;
} cura_lora_v2_clear_header_t;

typedef struct {
  uint16_t run_ms;
  uint16_t soil_0_mv;
  uint16_t soil_1_mv;
  int16_t soil_temp_0_centi_c;
  int16_t soil_temp_1_centi_c;
  int16_t enclosure_centi_c;
  uint32_t enclosure_pressure_pa;
  uint16_t enclosure_humidity_centi_pct;
  uint8_t reset_reason;
  uint8_t previous_current_tx_attempts;
  uint16_t previous_awake_ms;
  uint16_t previous_current_delivery_ms;
  uint8_t previous_cycle_tx_attempts;
  uint8_t previous_cycle_accepted_readings;
  uint16_t flags;
} cura_lora_v2_reading_t;

typedef struct {
  uint8_t status;
} cura_lora_v2_ack_t;

bool cura_lora_v2_is_supported_control(uint8_t control);
bool cura_lora_v2_domain_is_reading(uint8_t domain);
bool cura_lora_v2_domain_is_ack(uint8_t domain);
bool cura_lora_v2_ack_status_matches_domain(
    uint8_t domain, uint8_t status);

cura_lora_v2_codec_result_t cura_lora_v2_validate_reading(
    const cura_lora_v2_reading_t *reading);

cura_lora_v2_codec_result_t cura_lora_v2_encode_clear_header(
    uint8_t *output, size_t output_size,
    const cura_lora_v2_clear_header_t *header);
cura_lora_v2_codec_result_t cura_lora_v2_decode_clear_header(
    cura_lora_v2_clear_header_t *header,
    const uint8_t *input, size_t input_size);

cura_lora_v2_codec_result_t cura_lora_v2_build_nonce(
    uint8_t *output, size_t output_size,
    const cura_lora_v2_clear_header_t *header);

cura_lora_v2_codec_result_t cura_lora_v2_encode_reading(
    uint8_t *output, size_t output_size,
    const cura_lora_v2_reading_t *reading);
cura_lora_v2_codec_result_t cura_lora_v2_decode_reading(
    cura_lora_v2_reading_t *reading,
    const uint8_t *input, size_t input_size);

cura_lora_v2_codec_result_t cura_lora_v2_encode_ack(
    uint8_t *output, size_t output_size,
    const cura_lora_v2_ack_t *ack);
cura_lora_v2_codec_result_t cura_lora_v2_decode_ack(
    cura_lora_v2_ack_t *ack,
    const uint8_t *input, size_t input_size);

#ifdef __cplusplus
}
#endif
