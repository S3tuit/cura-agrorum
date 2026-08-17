#pragma once

#include <stddef.h>
#include <stdint.h>

#include "node_persistence_backend.h"
#include "protocol_v2_lora_schema_generated.h"

#define NODE_PERSISTENCE_RECORD_MAGIC UINT32_C(0x756fec23)
#define NODE_PERSISTENCE_RECORD_FORMAT_VERSION UINT8_C(2)

#define NODE_PERSISTENCE_RECORD_TYPE_PENDING_READING UINT8_C(1)
#define NODE_PERSISTENCE_RECORD_TYPE_QUARANTINED_READING UINT8_C(2)
#define NODE_PERSISTENCE_RECORD_TYPE_DIAGNOSTIC_EVENT UINT8_C(3)
#define NODE_PERSISTENCE_RECORD_TYPE_DELIVERY_STARTED UINT8_C(4)
#define NODE_PERSISTENCE_RECORD_TYPE_DELIVERY_FINISHED UINT8_C(5)
#define NODE_PERSISTENCE_RECORD_TYPE_PENDING_BACKLOG_BINDING UINT8_C(6)

#define NODE_PERSISTENCE_RECORD_HEADER_SIZE 8U
#define NODE_PERSISTENCE_RECORD_FOOTER_SIZE 6U
#define NODE_PERSISTENCE_RECORD_OVERHEAD 14U
#define NODE_PERSISTENCE_RECORD_MAX_PAYLOAD_SIZE 498U
#define NODE_PERSISTENCE_RECORD_MAX_SIZE 512U

#define NODE_PERSISTENCE_READING_PAYLOAD_SIZE 32U
#define NODE_PERSISTENCE_BACKLOG_BINDING_PAYLOAD_SIZE 62U
#define NODE_PERSISTENCE_DELIVERY_STARTED_PAYLOAD_SIZE 17U
#define NODE_PERSISTENCE_DELIVERY_FINISHED_PAYLOAD_SIZE 15U
#define NODE_PERSISTENCE_DIAGNOSTIC_PREFIX_SIZE 22U
#define NODE_PERSISTENCE_DIAGNOSTIC_MAX_PAYLOAD_SIZE 274U

typedef uint8_t node_persistence_log_kind_t;
#define NODE_PERSISTENCE_LOG_PENDING UINT8_C(0)
#define NODE_PERSISTENCE_LOG_QUARANTINE UINT8_C(1)
#define NODE_PERSISTENCE_LOG_DIAGNOSTIC UINT8_C(2)
#define NODE_PERSISTENCE_LOG_DELIVERY UINT8_C(3)
#define NODE_PERSISTENCE_LOG_COUNT 4U

typedef enum {
  NODE_PERSISTENCE_RECORD_VALID = 0,
  NODE_PERSISTENCE_RECORD_INVALID_FRAMING,
  NODE_PERSISTENCE_RECORD_UNSUPPORTED,
  NODE_PERSISTENCE_RECORD_INVALID_PAYLOAD,
} node_persistence_record_result_t;

void node_persistence_store_le16(uint8_t *output, uint16_t value);
void node_persistence_store_le32(uint8_t *output, uint32_t value);
uint16_t node_persistence_load_le16(const uint8_t *input);
uint32_t node_persistence_load_le32(const uint8_t *input);

bool node_persistence_record_encode(
    const node_persistence_backend_t *backend, uint8_t record_type,
    const uint8_t *payload, size_t payload_length,
    uint8_t output[NODE_PERSISTENCE_RECORD_MAX_SIZE], size_t *out_length);

node_persistence_record_result_t node_persistence_record_validate_structural(
    const node_persistence_backend_t *backend, const uint8_t *record,
    size_t record_length);

node_persistence_record_result_t
node_persistence_record_validate(const node_persistence_backend_t *backend,
                                 node_persistence_log_kind_t log_kind,
                                 const uint8_t *record, size_t record_length);

bool node_persistence_record_decode_reading(
    const uint8_t *record, size_t record_length,
    uint8_t out_reading_body[CURA_LORA_V2_READING_BODY_SIZE]);

bool node_persistence_record_decode_backlog_binding(
    const uint8_t *record, size_t record_length, uint32_t *out_sample_id,
    uint32_t *out_message_id,
    uint8_t out_frame[CURA_LORA_V2_READING_FRAME_SIZE]);

#ifdef NODE_PERSISTENCE_TESTING
void node_persistence_test_reset(void);
#endif
