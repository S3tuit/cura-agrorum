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

void node_persistence_store_le64(uint8_t *output, uint64_t value) {
  node_persistence_store_le32(output, (uint32_t)value);
  node_persistence_store_le32(output + 4U, (uint32_t)(value >> 32U));
}

uint16_t node_persistence_load_le16(const uint8_t *input) {
  return (uint16_t)((uint16_t)input[0] | ((uint16_t)input[1] << 8U));
}

uint32_t node_persistence_load_le32(const uint8_t *input) {
  return (uint32_t)input[0] | ((uint32_t)input[1] << 8U) |
         ((uint32_t)input[2] << 16U) | ((uint32_t)input[3] << 24U);
}

uint64_t node_persistence_load_le64(const uint8_t *input) {
  return (uint64_t)node_persistence_load_le32(input) |
         ((uint64_t)node_persistence_load_le32(input + 4U) << 32U);
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

/* Stores each field only under its validity condition; see node_persistence.h. */
static void encode_tx_call(const node_delivery_tx_call_t *call,
                           uint8_t *slot) {
  const bool ack = call->outcome == NODE_DELIVERY_TX_OUTCOME_ACK_RECEIVED;
  slot[0U] = (uint8_t)((call->tx_started
                            ? NODE_PERSISTENCE_DELIVERY_TX_STARTED_FLAG
                            : 0U) |
                       (call->tx_done ? NODE_PERSISTENCE_DELIVERY_TX_DONE_FLAG
                                      : 0U));
  slot[1U] = call->outcome;
  node_persistence_store_le64(slot + 2U,
                              call->tx_started ? call->set_tx_at_us : 0U);
  node_persistence_store_le64(slot + 10U,
                              call->tx_done ? call->tx_done_at_us : 0U);
  node_persistence_store_le64(slot + 18U, ack ? call->ack_rx_done_at_us : 0U);
  node_persistence_store_le16(slot + 26U,
                              (uint16_t)(ack ? call->ack_rssi_dbm_x2 : 0));
  node_persistence_store_le16(slot + 28U,
                              (uint16_t)(ack ? call->ack_snr_db_x4 : 0));
}

bool node_persistence_record_encode_delivery(
    const node_persistence_backend_t *backend,
    const node_delivery_event_t *event,
    uint8_t output[NODE_PERSISTENCE_RECORD_MAX_SIZE], size_t *out_length) {
  if (event == NULL || (event->type != NODE_DELIVERY_EVENT_STARTED &&
                        event->type != NODE_DELIVERY_EVENT_FINISHED)) {
    return false;
  }
  uint8_t payload[NODE_PERSISTENCE_DELIVERY_FINISHED_PAYLOAD_SIZE] = {0};
  node_persistence_store_le32(payload, event->cycle_sample_id);
  node_persistence_store_le32(payload + 4U, event->sample_id);
  node_persistence_store_le32(payload + 8U, event->message_id);
  payload[12U] = event->domain;
  if (event->type == NODE_DELIVERY_EVENT_STARTED) {
    node_persistence_store_le32(payload + 13U,
                                event->detail.started.start_offset_ms);
    return node_persistence_record_encode(
        backend, NODE_PERSISTENCE_RECORD_TYPE_DELIVERY_STARTED, payload,
        NODE_PERSISTENCE_DELIVERY_STARTED_PAYLOAD_SIZE, output, out_length);
  }
  if (event->detail.finished.tx_call_count > NODE_DELIVERY_TX_CALL_SLOTS) {
    return false;
  }
  payload[13U] = event->detail.finished.attempt_count;
  payload[14U] = event->detail.finished.final_result;
  node_persistence_store_le64(payload + 15U,
                              event->detail.finished.application_start_us);
  payload[23U] = event->detail.finished.tx_call_count;
  for (uint8_t index = 0U; index < event->detail.finished.tx_call_count;
       ++index) {
    encode_tx_call(&event->detail.finished.tx_calls[index],
                   payload + NODE_PERSISTENCE_DELIVERY_TX_CALL_OFFSET +
                       index * NODE_PERSISTENCE_DELIVERY_TX_CALL_SIZE);
  }
  return node_persistence_record_encode(
      backend, NODE_PERSISTENCE_RECORD_TYPE_DELIVERY_FINISHED, payload,
      NODE_PERSISTENCE_DELIVERY_FINISHED_PAYLOAD_SIZE, output, out_length);
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

static bool result_is_ack(uint8_t final_result) {
  return final_result >= NODE_DELIVERY_RESULT_ACCEPTED &&
         final_result <= NODE_DELIVERY_RESULT_MALFORMED;
}

static bool all_zero(const uint8_t *bytes, size_t length) {
  for (size_t index = 0U; index < length; ++index) {
    if (bytes[index] != 0U) {
      return false;
    }
  }
  return true;
}

/*
 * One used transmit-call slot is canonical: known flags/outcome, TX_DONE only
 * after a start, zero for every field outside its validity condition. Only
 * the last call may end in an ACK, and earlier calls can only have ended in
 * an ordinary ACK timeout after TX_DONE, because nothing else retries.
 */
static bool validate_tx_call(const uint8_t *slot, bool last,
                             uint8_t final_result) {
  const uint8_t flags = slot[0U];
  const uint8_t outcome = slot[1U];
  const bool started = (flags & NODE_PERSISTENCE_DELIVERY_TX_STARTED_FLAG) != 0U;
  const bool done = (flags & NODE_PERSISTENCE_DELIVERY_TX_DONE_FLAG) != 0U;
  const bool ack = outcome == NODE_DELIVERY_TX_OUTCOME_ACK_RECEIVED;
  if ((flags & (uint8_t)~(NODE_PERSISTENCE_DELIVERY_TX_STARTED_FLAG |
                          NODE_PERSISTENCE_DELIVERY_TX_DONE_FLAG)) != 0U ||
      outcome < NODE_DELIVERY_TX_OUTCOME_ACK_RECEIVED ||
      outcome > NODE_DELIVERY_TX_OUTCOME_DEADLINE_EXPIRED ||
      (done && !started) || (ack && !done) ||
      (!started && !all_zero(slot + 2U, 8U)) ||
      (!done && !all_zero(slot + 10U, 8U)) ||
      (!ack && !all_zero(slot + 18U, 12U))) {
    return false;
  }
  if (!last) {
    return outcome == NODE_DELIVERY_TX_OUTCOME_ACK_TIMEOUT && done;
  }
  return ack == result_is_ack(final_result);
}

static bool validate_delivery_finished_payload(const uint8_t *payload,
                                               size_t payload_length) {
  if (payload_length != NODE_PERSISTENCE_DELIVERY_FINISHED_PAYLOAD_SIZE ||
      !domain_is_reading(payload[12U])) {
    return false;
  }
  const uint8_t attempt_count = payload[13U];
  const uint8_t final_result = payload[14U];
  const uint8_t call_count = payload[23U];
  if (final_result < NODE_DELIVERY_RESULT_ACCEPTED ||
      final_result > NODE_DELIVERY_RESULT_NO_ACK_ATTEMPT_LIMIT ||
      call_count > NODE_DELIVERY_TX_CALL_SLOTS ||
      (call_count == 0U && result_is_ack(final_result))) {
    return false;
  }
  uint8_t started_count = 0U;
  for (uint8_t index = 0U; index < NODE_DELIVERY_TX_CALL_SLOTS; ++index) {
    const uint8_t *slot = payload + NODE_PERSISTENCE_DELIVERY_TX_CALL_OFFSET +
                          index * NODE_PERSISTENCE_DELIVERY_TX_CALL_SIZE;
    if (index >= call_count) {
      if (!all_zero(slot, NODE_PERSISTENCE_DELIVERY_TX_CALL_SIZE)) {
        return false;
      }
      continue;
    }
    if (!validate_tx_call(slot, index + 1U == call_count, final_result)) {
      return false;
    }
    if ((slot[0U] & NODE_PERSISTENCE_DELIVERY_TX_STARTED_FLAG) != 0U) {
      started_count++;
    }
  }
  return attempt_count == started_count;
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
