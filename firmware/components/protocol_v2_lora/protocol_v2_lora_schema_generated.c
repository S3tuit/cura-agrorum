/* Generated from protocol/protocol-v2-lora/schemas/protocol_v2_lora.json by protocol/protocol-v2-lora/tools/generate.py.
 * Schema SHA-256: ece4ec58bad29ddad03be41ccb257ffef960fa11a2df1d9f8650f4eb499de009. Do not edit by hand. */
#include "protocol_v2_lora_schema_generated.h"

#include <limits.h>
#include <string.h>

_Static_assert(CHAR_BIT == 8, "protocol requires 8-bit bytes");
_Static_assert(sizeof(uint16_t) == 2, "protocol requires 16-bit uint16_t");
_Static_assert(sizeof(uint32_t) == 4, "protocol requires 32-bit uint32_t");
_Static_assert(INT16_MIN == -32768, "protocol requires 16-bit int16_t");
_Static_assert(INT16_MAX == 32767, "protocol requires 16-bit int16_t");

static void write_u16_le(uint8_t *output, uint16_t value) {
  output[0] = (uint8_t)value;
  output[1] = (uint8_t)(value >> 8);
}

static void write_u32_le(uint8_t *output, uint32_t value) {
  output[0] = (uint8_t)value;
  output[1] = (uint8_t)(value >> 8);
  output[2] = (uint8_t)(value >> 16);
  output[3] = (uint8_t)(value >> 24);
}

static uint16_t read_u16_le(const uint8_t *input) {
  return (uint16_t)input[0] | ((uint16_t)input[1] << 8);
}

static uint32_t read_u32_le(const uint8_t *input) {
  return (uint32_t)input[0] | ((uint32_t)input[1] << 8) |
         ((uint32_t)input[2] << 16) | ((uint32_t)input[3] << 24);
}

static int16_t read_i16_le(const uint8_t *input) {
  const uint16_t raw = read_u16_le(input);
  if (raw <= (uint16_t)INT16_MAX) {
    return (int16_t)raw;
  }
  return (int16_t)(-1 - (int32_t)(UINT16_MAX - raw));
}

bool cura_lora_v2_is_supported_control(uint8_t control) {
  return control == CURA_LORA_V2_CONTROL;
}

bool cura_lora_v2_domain_is_reading(uint8_t domain) {
  switch (domain) {
  case CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK:
  case CURA_LORA_V2_DOMAIN_BACKLOG_READING_UPLINK:
    return true;
  default:
    return false;
  }
}

bool cura_lora_v2_domain_is_ack(uint8_t domain) {
  switch (domain) {
  case CURA_LORA_V2_DOMAIN_ACK_ACCEPTED_DOWNLINK:
  case CURA_LORA_V2_DOMAIN_ACK_RETRY_LATER_DOWNLINK:
  case CURA_LORA_V2_DOMAIN_ACK_REJECTED_UNSUPPORTED_DOWNLINK:
  case CURA_LORA_V2_DOMAIN_ACK_REJECTED_MALFORMED_DOWNLINK:
    return true;
  default:
    return false;
  }
}

bool cura_lora_v2_ack_status_matches_domain(
    uint8_t domain, uint8_t status) {
  switch (domain) {
  case CURA_LORA_V2_DOMAIN_ACK_ACCEPTED_DOWNLINK:
    return status == CURA_LORA_V2_ACK_STATUS_ACCEPTED;
  case CURA_LORA_V2_DOMAIN_ACK_RETRY_LATER_DOWNLINK:
    return status == CURA_LORA_V2_ACK_STATUS_RETRY_LATER;
  case CURA_LORA_V2_DOMAIN_ACK_REJECTED_UNSUPPORTED_DOWNLINK:
    return status == CURA_LORA_V2_ACK_STATUS_REJECTED_UNSUPPORTED;
  case CURA_LORA_V2_DOMAIN_ACK_REJECTED_MALFORMED_DOWNLINK:
    return status == CURA_LORA_V2_ACK_STATUS_REJECTED_MALFORMED;
  default:
    return false;
  }
}

static bool ack_status_is_known(uint8_t status) {
  switch (status) {
  case CURA_LORA_V2_ACK_STATUS_ACCEPTED:
  case CURA_LORA_V2_ACK_STATUS_RETRY_LATER:
  case CURA_LORA_V2_ACK_STATUS_REJECTED_UNSUPPORTED:
  case CURA_LORA_V2_ACK_STATUS_REJECTED_MALFORMED:
    return true;
  default:
    return false;
  }
}

cura_lora_v2_codec_result_t cura_lora_v2_validate_reading(
    const cura_lora_v2_reading_t *reading) {
  if (reading == NULL) {
    return CURA_LORA_V2_CODEC_INVALID_ARGUMENT;
  }
  if ((reading->flags & CURA_LORA_V2_FLAG_SOIL_0_VALID) == 0u &&
      reading->soil_0_mv != 0) {
    return CURA_LORA_V2_CODEC_MALFORMED;
  }
  if ((reading->flags & CURA_LORA_V2_FLAG_SOIL_1_VALID) == 0u &&
      reading->soil_1_mv != 0) {
    return CURA_LORA_V2_CODEC_MALFORMED;
  }
  if ((reading->flags & CURA_LORA_V2_FLAG_SOIL_TEMP_0_VALID) == 0u &&
      reading->soil_temp_0_centi_c != 0) {
    return CURA_LORA_V2_CODEC_MALFORMED;
  }
  if ((reading->flags & CURA_LORA_V2_FLAG_SOIL_TEMP_1_VALID) == 0u &&
      reading->soil_temp_1_centi_c != 0) {
    return CURA_LORA_V2_CODEC_MALFORMED;
  }
  if ((reading->flags & CURA_LORA_V2_FLAG_ENCLOSURE_TEMP_VALID) == 0u &&
      reading->enclosure_centi_c != 0) {
    return CURA_LORA_V2_CODEC_MALFORMED;
  }
  if ((reading->flags & CURA_LORA_V2_FLAG_ENCLOSURE_PRESSURE_VALID) == 0u &&
      reading->enclosure_pressure_pa != 0) {
    return CURA_LORA_V2_CODEC_MALFORMED;
  }
  if ((reading->flags & CURA_LORA_V2_FLAG_ENCLOSURE_HUMIDITY_VALID) == 0u &&
      reading->enclosure_humidity_centi_pct != 0) {
    return CURA_LORA_V2_CODEC_MALFORMED;
  }
  if (((reading->flags & CURA_LORA_V2_FLAG_DEEP_SLEEP_BOOT) != 0u) !=
      (reading->reset_reason == 8u)) {
    return CURA_LORA_V2_CODEC_MALFORMED;
  }
  if ((reading->flags & CURA_LORA_V2_FLAG_PREVIOUS_CYCLE_METRICS_VALID) == 0u &&
      (reading->previous_current_tx_attempts != 0 ||
       reading->previous_awake_ms != 0 ||
       reading->previous_current_delivery_ms != 0 ||
       reading->previous_cycle_tx_attempts != 0 ||
       reading->previous_cycle_accepted_readings != 0)) {
    return CURA_LORA_V2_CODEC_MALFORMED;
  }
  if ((reading->flags & CURA_LORA_V2_FLAG_PREVIOUS_CURRENT_ACCEPTED) != 0u &&
      (reading->flags & CURA_LORA_V2_FLAG_PREVIOUS_CYCLE_METRICS_VALID) == 0u) {
    return CURA_LORA_V2_CODEC_MALFORMED;
  }
  if ((reading->flags & CURA_LORA_V2_FLAG_PREVIOUS_CURRENT_ACCEPTED) == 0u &&
      reading->previous_current_delivery_ms != 0) {
    return CURA_LORA_V2_CODEC_MALFORMED;
  }
  if ((reading->flags &
       CURA_LORA_V2_READING_RESERVED_FLAGS_MASK) != 0u) {
    return CURA_LORA_V2_CODEC_MALFORMED;
  }
  return CURA_LORA_V2_CODEC_OK;
}

cura_lora_v2_codec_result_t cura_lora_v2_encode_clear_header(
    uint8_t *output, size_t output_size,
    const cura_lora_v2_clear_header_t *value) {
  if (output == NULL || value == NULL) {
    return CURA_LORA_V2_CODEC_INVALID_ARGUMENT;
  }
  if (output_size < CURA_LORA_V2_CLEAR_HEADER_SIZE) {
    return CURA_LORA_V2_CODEC_BUFFER_TOO_SMALL;
  }
  output[CURA_LORA_V2_CLEAR_HEADER_CONTROL_OFFSET] = value->control;
  output[CURA_LORA_V2_CLEAR_HEADER_DOMAIN_OFFSET] = value->domain;
  memcpy(&output[CURA_LORA_V2_CLEAR_HEADER_NODE_ID_OFFSET], value->node_id, sizeof(value->node_id));
  write_u32_le(&output[CURA_LORA_V2_CLEAR_HEADER_MESSAGE_ID_OFFSET], value->message_id);
  return CURA_LORA_V2_CODEC_OK;
}

cura_lora_v2_codec_result_t cura_lora_v2_decode_clear_header(
    cura_lora_v2_clear_header_t *value,
    const uint8_t *input, size_t input_size) {
  if (value == NULL || input == NULL) {
    return CURA_LORA_V2_CODEC_INVALID_ARGUMENT;
  }
  if (input_size != CURA_LORA_V2_CLEAR_HEADER_SIZE) {
    return CURA_LORA_V2_CODEC_INVALID_LENGTH;
  }
  value->control = input[CURA_LORA_V2_CLEAR_HEADER_CONTROL_OFFSET];
  value->domain = input[CURA_LORA_V2_CLEAR_HEADER_DOMAIN_OFFSET];
  memcpy(value->node_id, &input[CURA_LORA_V2_CLEAR_HEADER_NODE_ID_OFFSET], sizeof(value->node_id));
  value->message_id = read_u32_le(&input[CURA_LORA_V2_CLEAR_HEADER_MESSAGE_ID_OFFSET]);
  return CURA_LORA_V2_CODEC_OK;
}

cura_lora_v2_codec_result_t cura_lora_v2_build_nonce(
    uint8_t *output, size_t output_size,
    const cura_lora_v2_clear_header_t *header) {
  if (output == NULL || header == NULL) {
    return CURA_LORA_V2_CODEC_INVALID_ARGUMENT;
  }
  if (output_size < CURA_LORA_V2_NONCE_SIZE) {
    return CURA_LORA_V2_CODEC_BUFFER_TOO_SMALL;
  }
  memcpy(&output[CURA_LORA_V2_NONCE_NODE_ID_OFFSET], header->node_id, sizeof(header->node_id));
  write_u32_le(&output[CURA_LORA_V2_NONCE_MESSAGE_ID_OFFSET], header->message_id);
  output[CURA_LORA_V2_NONCE_DOMAIN_OFFSET] = header->domain;
  return CURA_LORA_V2_CODEC_OK;
}

cura_lora_v2_codec_result_t cura_lora_v2_encode_reading(
    uint8_t *output, size_t output_size,
    const cura_lora_v2_reading_t *value) {
  if (output == NULL || value == NULL) {
    return CURA_LORA_V2_CODEC_INVALID_ARGUMENT;
  }
  if (output_size < CURA_LORA_V2_READING_BODY_SIZE) {
    return CURA_LORA_V2_CODEC_BUFFER_TOO_SMALL;
  }
  const cura_lora_v2_codec_result_t validation = cura_lora_v2_validate_reading(value);
  if (validation != CURA_LORA_V2_CODEC_OK) {
    return validation;
  }
  write_u32_le(&output[CURA_LORA_V2_READING_SAMPLE_ID_OFFSET], value->sample_id);
  write_u16_le(&output[CURA_LORA_V2_READING_RUN_MS_OFFSET], value->run_ms);
  write_u16_le(&output[CURA_LORA_V2_READING_SOIL_0_MV_OFFSET], value->soil_0_mv);
  write_u16_le(&output[CURA_LORA_V2_READING_SOIL_1_MV_OFFSET], value->soil_1_mv);
  write_u16_le(&output[CURA_LORA_V2_READING_SOIL_TEMP_0_CENTI_C_OFFSET], (uint16_t)value->soil_temp_0_centi_c);
  write_u16_le(&output[CURA_LORA_V2_READING_SOIL_TEMP_1_CENTI_C_OFFSET], (uint16_t)value->soil_temp_1_centi_c);
  write_u16_le(&output[CURA_LORA_V2_READING_ENCLOSURE_CENTI_C_OFFSET], (uint16_t)value->enclosure_centi_c);
  write_u32_le(&output[CURA_LORA_V2_READING_ENCLOSURE_PRESSURE_PA_OFFSET], value->enclosure_pressure_pa);
  write_u16_le(&output[CURA_LORA_V2_READING_ENCLOSURE_HUMIDITY_CENTI_PCT_OFFSET], value->enclosure_humidity_centi_pct);
  output[CURA_LORA_V2_READING_RESET_REASON_OFFSET] = value->reset_reason;
  output[CURA_LORA_V2_READING_PREVIOUS_CURRENT_TX_ATTEMPTS_OFFSET] = value->previous_current_tx_attempts;
  write_u16_le(&output[CURA_LORA_V2_READING_PREVIOUS_AWAKE_MS_OFFSET], value->previous_awake_ms);
  write_u16_le(&output[CURA_LORA_V2_READING_PREVIOUS_CURRENT_DELIVERY_MS_OFFSET], value->previous_current_delivery_ms);
  output[CURA_LORA_V2_READING_PREVIOUS_CYCLE_TX_ATTEMPTS_OFFSET] = value->previous_cycle_tx_attempts;
  output[CURA_LORA_V2_READING_PREVIOUS_CYCLE_ACCEPTED_READINGS_OFFSET] = value->previous_cycle_accepted_readings;
  write_u16_le(&output[CURA_LORA_V2_READING_FLAGS_OFFSET], value->flags);
  return CURA_LORA_V2_CODEC_OK;
}

cura_lora_v2_codec_result_t cura_lora_v2_decode_reading(
    cura_lora_v2_reading_t *value,
    const uint8_t *input, size_t input_size) {
  if (value == NULL || input == NULL) {
    return CURA_LORA_V2_CODEC_INVALID_ARGUMENT;
  }
  if (input_size != CURA_LORA_V2_READING_BODY_SIZE) {
    return CURA_LORA_V2_CODEC_INVALID_LENGTH;
  }
  value->sample_id = read_u32_le(&input[CURA_LORA_V2_READING_SAMPLE_ID_OFFSET]);
  value->run_ms = read_u16_le(&input[CURA_LORA_V2_READING_RUN_MS_OFFSET]);
  value->soil_0_mv = read_u16_le(&input[CURA_LORA_V2_READING_SOIL_0_MV_OFFSET]);
  value->soil_1_mv = read_u16_le(&input[CURA_LORA_V2_READING_SOIL_1_MV_OFFSET]);
  value->soil_temp_0_centi_c = read_i16_le(&input[CURA_LORA_V2_READING_SOIL_TEMP_0_CENTI_C_OFFSET]);
  value->soil_temp_1_centi_c = read_i16_le(&input[CURA_LORA_V2_READING_SOIL_TEMP_1_CENTI_C_OFFSET]);
  value->enclosure_centi_c = read_i16_le(&input[CURA_LORA_V2_READING_ENCLOSURE_CENTI_C_OFFSET]);
  value->enclosure_pressure_pa = read_u32_le(&input[CURA_LORA_V2_READING_ENCLOSURE_PRESSURE_PA_OFFSET]);
  value->enclosure_humidity_centi_pct = read_u16_le(&input[CURA_LORA_V2_READING_ENCLOSURE_HUMIDITY_CENTI_PCT_OFFSET]);
  value->reset_reason = input[CURA_LORA_V2_READING_RESET_REASON_OFFSET];
  value->previous_current_tx_attempts = input[CURA_LORA_V2_READING_PREVIOUS_CURRENT_TX_ATTEMPTS_OFFSET];
  value->previous_awake_ms = read_u16_le(&input[CURA_LORA_V2_READING_PREVIOUS_AWAKE_MS_OFFSET]);
  value->previous_current_delivery_ms = read_u16_le(&input[CURA_LORA_V2_READING_PREVIOUS_CURRENT_DELIVERY_MS_OFFSET]);
  value->previous_cycle_tx_attempts = input[CURA_LORA_V2_READING_PREVIOUS_CYCLE_TX_ATTEMPTS_OFFSET];
  value->previous_cycle_accepted_readings = input[CURA_LORA_V2_READING_PREVIOUS_CYCLE_ACCEPTED_READINGS_OFFSET];
  value->flags = read_u16_le(&input[CURA_LORA_V2_READING_FLAGS_OFFSET]);
  return cura_lora_v2_validate_reading(value);
}

cura_lora_v2_codec_result_t cura_lora_v2_encode_ack(
    uint8_t *output, size_t output_size,
    const cura_lora_v2_ack_t *value) {
  if (output == NULL || value == NULL) {
    return CURA_LORA_V2_CODEC_INVALID_ARGUMENT;
  }
  if (output_size < CURA_LORA_V2_ACK_BODY_SIZE) {
    return CURA_LORA_V2_CODEC_BUFFER_TOO_SMALL;
  }
  const cura_lora_v2_codec_result_t validation = ack_status_is_known(value->status) ? CURA_LORA_V2_CODEC_OK : CURA_LORA_V2_CODEC_MALFORMED;
  if (validation != CURA_LORA_V2_CODEC_OK) {
    return validation;
  }
  output[CURA_LORA_V2_ACK_STATUS_OFFSET] = value->status;
  return CURA_LORA_V2_CODEC_OK;
}

cura_lora_v2_codec_result_t cura_lora_v2_decode_ack(
    cura_lora_v2_ack_t *value,
    const uint8_t *input, size_t input_size) {
  if (value == NULL || input == NULL) {
    return CURA_LORA_V2_CODEC_INVALID_ARGUMENT;
  }
  if (input_size != CURA_LORA_V2_ACK_BODY_SIZE) {
    return CURA_LORA_V2_CODEC_INVALID_LENGTH;
  }
  value->status = input[CURA_LORA_V2_ACK_STATUS_OFFSET];
  return ack_status_is_known(value->status) ? CURA_LORA_V2_CODEC_OK : CURA_LORA_V2_CODEC_MALFORMED;
}
