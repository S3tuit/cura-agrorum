#include "node_persistence_record.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "node_common.h"
#include "node_persistence.h"
#include "protocol_v2_lora_schema_generated.h"

#define RECORD_FORMAT_VERSION_OFFSET 4U
#define RECORD_TYPE_OFFSET 5U
#define RECORD_PAYLOAD_LENGTH_OFFSET 6U
#define RECORD_PAYLOAD_OFFSET 8U

void node_persistence_store_le16(uint8_t *output, uint16_t value) {
  output[0] = (uint8_t)value;
  output[1] = (uint8_t)(value >> 8U);
}

void node_persistence_store_le32(uint8_t *output, uint32_t value) {
  output[0] = (uint8_t)value;
  output[1] = (uint8_t)(value >> 8U);
  output[2] = (uint8_t)(value >> 16U);
  output[3] = (uint8_t)(value >> 24U);
}

uint16_t node_persistence_load_le16(const uint8_t *input) {
  return (uint16_t)((uint16_t)input[0] | ((uint16_t)input[1] << 8U));
}

uint32_t node_persistence_load_le32(const uint8_t *input) {
  return (uint32_t)input[0] | ((uint32_t)input[1] << 8U) |
         ((uint32_t)input[2] << 16U) | ((uint32_t)input[3] << 24U);
}

bool node_persistence_record_encode(
    const node_persistence_backend_t *backend, uint8_t record_type,
    const uint8_t *payload, size_t payload_length,
    uint8_t output[NODE_PERSISTENCE_RECORD_MAX_SIZE], size_t *out_length) {
  if (backend == NULL || backend->crc32_iso_hdlc == NULL || output == NULL ||
      out_length == NULL || (payload == NULL && payload_length != 0U) ||
      payload_length > NODE_PERSISTENCE_RECORD_MAX_PAYLOAD_SIZE) {
    return false;
  }

  const size_t total_length = payload_length + NODE_PERSISTENCE_RECORD_OVERHEAD;
  node_persistence_store_le32(output, NODE_PERSISTENCE_RECORD_MAGIC);
  output[RECORD_FORMAT_VERSION_OFFSET] = NODE_PERSISTENCE_RECORD_FORMAT_VERSION;
  output[RECORD_TYPE_OFFSET] = record_type;
  node_persistence_store_le16(output + RECORD_PAYLOAD_LENGTH_OFFSET,
                              (uint16_t)payload_length);
  if (payload_length != 0U) {
    memcpy(output + RECORD_PAYLOAD_OFFSET, payload, payload_length);
  }

  const size_t footer_offset = RECORD_PAYLOAD_OFFSET + payload_length;
  node_persistence_store_le16(output + footer_offset, (uint16_t)total_length);
  const uint32_t crc = backend->crc32_iso_hdlc(output, total_length - 4U);
  node_persistence_store_le32(output + total_length - 4U, crc);
  *out_length = total_length;
  return true;
}

node_persistence_record_result_t node_persistence_record_validate_structural(
    const node_persistence_backend_t *backend, const uint8_t *record,
    size_t record_length) {
  if (backend == NULL || backend->crc32_iso_hdlc == NULL || record == NULL ||
      record_length < NODE_PERSISTENCE_RECORD_OVERHEAD ||
      record_length > NODE_PERSISTENCE_RECORD_MAX_SIZE) {
    return NODE_PERSISTENCE_RECORD_INVALID_FRAMING;
  }
  if (node_persistence_load_le32(record) != NODE_PERSISTENCE_RECORD_MAGIC) {
    return NODE_PERSISTENCE_RECORD_INVALID_FRAMING;
  }

  const size_t payload_length =
      node_persistence_load_le16(record + RECORD_PAYLOAD_LENGTH_OFFSET);
  if (payload_length > NODE_PERSISTENCE_RECORD_MAX_PAYLOAD_SIZE ||
      payload_length + NODE_PERSISTENCE_RECORD_OVERHEAD != record_length) {
    return NODE_PERSISTENCE_RECORD_INVALID_FRAMING;
  }

  const size_t footer_offset = RECORD_PAYLOAD_OFFSET + payload_length;
  if (node_persistence_load_le16(record + footer_offset) != record_length) {
    return NODE_PERSISTENCE_RECORD_INVALID_FRAMING;
  }

  const uint32_t stored_crc =
      node_persistence_load_le32(record + record_length - 4U);
  const uint32_t calculated_crc =
      backend->crc32_iso_hdlc(record, record_length - 4U);
  if (stored_crc != calculated_crc) {
    return NODE_PERSISTENCE_RECORD_INVALID_FRAMING;
  }
  return NODE_PERSISTENCE_RECORD_VALID;
}

static bool record_type_allowed(node_persistence_log_kind_t log_kind,
                                uint8_t record_type) {
  switch (log_kind) {
  case NODE_PERSISTENCE_LOG_PENDING:
    return record_type == NODE_PERSISTENCE_RECORD_TYPE_PENDING_READING ||
           record_type == NODE_PERSISTENCE_RECORD_TYPE_PENDING_BACKLOG_BINDING;
  case NODE_PERSISTENCE_LOG_QUARANTINE:
    return record_type == NODE_PERSISTENCE_RECORD_TYPE_QUARANTINED_READING;
  case NODE_PERSISTENCE_LOG_DIAGNOSTIC:
    return record_type == NODE_PERSISTENCE_RECORD_TYPE_DIAGNOSTIC_EVENT;
  case NODE_PERSISTENCE_LOG_DELIVERY:
    return record_type == NODE_PERSISTENCE_RECORD_TYPE_DELIVERY_STARTED ||
           record_type == NODE_PERSISTENCE_RECORD_TYPE_DELIVERY_FINISHED;
  default:
    return false;
  }
}

static bool validate_reading_payload(const uint8_t *payload,
                                     size_t payload_length) {
  if (payload_length != NODE_PERSISTENCE_READING_PAYLOAD_SIZE) {
    return false;
  }
  cura_lora_v2_reading_t reading;
  return cura_lora_v2_decode_reading(&reading, payload,
                                     CURA_LORA_V2_READING_BODY_SIZE) ==
         CURA_LORA_V2_CODEC_OK;
}

static bool validate_backlog_binding_payload(const uint8_t *payload,
                                             size_t payload_length) {
  if (payload_length != NODE_PERSISTENCE_BACKLOG_BINDING_PAYLOAD_SIZE) {
    return false;
  }
  const uint32_t message_id = node_persistence_load_le32(payload + 4U);
  const uint8_t *frame = payload + 8U;
  return frame[CURA_LORA_V2_CLEAR_HEADER_CONTROL_OFFSET] ==
             CURA_LORA_V2_CONTROL &&
         frame[CURA_LORA_V2_CLEAR_HEADER_DOMAIN_OFFSET] ==
             CURA_LORA_V2_DOMAIN_BACKLOG_READING_UPLINK &&
         node_persistence_load_le32(
             frame + CURA_LORA_V2_CLEAR_HEADER_MESSAGE_ID_OFFSET) == message_id;
}

static bool validate_diagnostic_context(uint16_t error_domain,
                                        uint16_t operation,
                                        uint8_t context_length,
                                        uint8_t context_schema) {
  if (operation > CURAG_OP_CLEANUP) {
    return false;
  }
  if ((context_schema == CURAG_CONTEXT_SCHEMA_NONE) != (context_length == 0U)) {
    return false;
  }
  if (context_length != 0U && operation == CURAG_OP_NONE) {
    return false;
  }
  if (context_schema != UINT8_C(1)) {
    return true;
  }

  switch (error_domain) {
  case CURAG_EDOM_PERSISTENCE:
    return context_length == CURAG_PERSISTENCE_CONTEXT_V1_SIZE;
  case CURAG_EDOM_RADIO:
    return context_length == 14U;
  case CURAG_EDOM_SENSORS:
    return context_length == 48U;
  default:
    /* Schema numbers are scoped by domain. Preserve unknown domains. */
    return true;
  }
}

static bool validate_diagnostic_payload(const uint8_t *payload,
                                        size_t payload_length) {
  if (payload_length < NODE_PERSISTENCE_DIAGNOSTIC_PREFIX_SIZE ||
      payload_length > NODE_PERSISTENCE_DIAGNOSTIC_MAX_PAYLOAD_SIZE) {
    return false;
  }

  const uint16_t error_domain = node_persistence_load_le16(payload);
  const uint16_t error_code = node_persistence_load_le16(payload + 2U);
  const uint16_t flags = node_persistence_load_le16(payload + 4U);
  const uint32_t application_offset_ms =
      node_persistence_load_le32(payload + 6U);
  const uint32_t cycle_sample_id = node_persistence_load_le32(payload + 10U);
  const uint32_t message_id = node_persistence_load_le32(payload + 14U);
  const uint16_t operation = node_persistence_load_le16(payload + 18U);
  const uint8_t context_length = payload[20U];
  const uint8_t context_schema = payload[21U];

  if (error_domain == CURAG_EDOM_NONE || error_code == CURAG_ECODE_NONE ||
      (flags & NODE_DIAGNOSTIC_RESERVED_FLAGS_MASK) != 0U ||
      ((flags & NODE_DIAGNOSTIC_APPLICATION_OFFSET_VALID) == 0U &&
       application_offset_ms != 0U) ||
      ((flags & NODE_DIAGNOSTIC_CYCLE_SAMPLE_ID_VALID) == 0U &&
       cycle_sample_id != 0U) ||
      ((flags & NODE_DIAGNOSTIC_MESSAGE_ID_VALID) == 0U && message_id != 0U) ||
      (size_t)context_length + NODE_PERSISTENCE_DIAGNOSTIC_PREFIX_SIZE !=
          payload_length) {
    return false;
  }
  return validate_diagnostic_context(error_domain, operation, context_length,
                                     context_schema);
}

static bool domain_is_reading(uint8_t domain) {
  return domain == CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK ||
         domain == CURA_LORA_V2_DOMAIN_BACKLOG_READING_UPLINK;
}

static bool validate_delivery_started_payload(const uint8_t *payload,
                                              size_t payload_length) {
  return payload_length == NODE_PERSISTENCE_DELIVERY_STARTED_PAYLOAD_SIZE &&
         domain_is_reading(payload[12U]);
}

static bool validate_delivery_finished_payload(const uint8_t *payload,
                                               size_t payload_length) {
  if (payload_length != NODE_PERSISTENCE_DELIVERY_FINISHED_PAYLOAD_SIZE ||
      !domain_is_reading(payload[12U])) {
    return false;
  }
  const uint8_t final_result = payload[14U];
  return final_result >= NODE_DELIVERY_RESULT_ACCEPTED &&
         final_result <= NODE_DELIVERY_RESULT_LOCAL_RADIO_ERROR;
}

node_persistence_record_result_t
node_persistence_record_validate(const node_persistence_backend_t *backend,
                                 node_persistence_log_kind_t log_kind,
                                 const uint8_t *record, size_t record_length) {
  const node_persistence_record_result_t structural =
      node_persistence_record_validate_structural(backend, record,
                                                  record_length);
  if (structural != NODE_PERSISTENCE_RECORD_VALID) {
    return structural;
  }
  if (record[RECORD_FORMAT_VERSION_OFFSET] !=
      NODE_PERSISTENCE_RECORD_FORMAT_VERSION) {
    return NODE_PERSISTENCE_RECORD_UNSUPPORTED;
  }

  const uint8_t record_type = record[RECORD_TYPE_OFFSET];
  if (!record_type_allowed(log_kind, record_type)) {
    return NODE_PERSISTENCE_RECORD_UNSUPPORTED;
  }

  const uint8_t *payload = record + RECORD_PAYLOAD_OFFSET;
  const size_t payload_length =
      node_persistence_load_le16(record + RECORD_PAYLOAD_LENGTH_OFFSET);
  bool valid_payload = false;
  switch (record_type) {
  case NODE_PERSISTENCE_RECORD_TYPE_PENDING_READING:
  case NODE_PERSISTENCE_RECORD_TYPE_QUARANTINED_READING:
    valid_payload = validate_reading_payload(payload, payload_length);
    break;
  case NODE_PERSISTENCE_RECORD_TYPE_PENDING_BACKLOG_BINDING:
    valid_payload = validate_backlog_binding_payload(payload, payload_length);
    break;
  case NODE_PERSISTENCE_RECORD_TYPE_DIAGNOSTIC_EVENT:
    valid_payload = validate_diagnostic_payload(payload, payload_length);
    break;
  case NODE_PERSISTENCE_RECORD_TYPE_DELIVERY_STARTED:
    valid_payload = validate_delivery_started_payload(payload, payload_length);
    break;
  case NODE_PERSISTENCE_RECORD_TYPE_DELIVERY_FINISHED:
    valid_payload = validate_delivery_finished_payload(payload, payload_length);
    break;
  default:
    return NODE_PERSISTENCE_RECORD_UNSUPPORTED;
  }
  return valid_payload ? NODE_PERSISTENCE_RECORD_VALID
                       : NODE_PERSISTENCE_RECORD_INVALID_PAYLOAD;
}

bool node_persistence_record_decode_reading(
    const uint8_t *record, size_t record_length,
    uint8_t out_reading_body[CURA_LORA_V2_READING_BODY_SIZE]) {
  if (record == NULL || out_reading_body == NULL ||
      record_length != NODE_PERSISTENCE_READING_PAYLOAD_SIZE +
                           NODE_PERSISTENCE_RECORD_OVERHEAD) {
    return false;
  }
  const uint8_t record_type = record[RECORD_TYPE_OFFSET];
  if (record_type != NODE_PERSISTENCE_RECORD_TYPE_PENDING_READING &&
      record_type != NODE_PERSISTENCE_RECORD_TYPE_QUARANTINED_READING) {
    return false;
  }
  memcpy(out_reading_body, record + RECORD_PAYLOAD_OFFSET,
         CURA_LORA_V2_READING_BODY_SIZE);
  return true;
}

bool node_persistence_record_decode_backlog_binding(
    const uint8_t *record, size_t record_length, uint32_t *out_sample_id,
    uint32_t *out_message_id,
    cura_lora_v2_authenticated_reading_frame_t *out_frame) {
  if (record == NULL || out_sample_id == NULL || out_message_id == NULL ||
      out_frame == NULL ||
      record_length != NODE_PERSISTENCE_BACKLOG_BINDING_PAYLOAD_SIZE +
                           NODE_PERSISTENCE_RECORD_OVERHEAD ||
      record[RECORD_TYPE_OFFSET] !=
          NODE_PERSISTENCE_RECORD_TYPE_PENDING_BACKLOG_BINDING) {
    return false;
  }
  const uint8_t *payload = record + RECORD_PAYLOAD_OFFSET;
  *out_sample_id = node_persistence_load_le32(payload);
  *out_message_id = node_persistence_load_le32(payload + 4U);
  memcpy(out_frame->bytes, payload + 8U, sizeof(out_frame->bytes));
  return true;
}
