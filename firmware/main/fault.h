/* Generated C header from protocol/wifi-protocol-v1/schemas/fault_v1.json by protocol/wifi-protocol-v1/tools/generate.py. Do not edit by hand. */
#pragma once

#include <stdint.h>

#define FAULT_SCHEMA_VERSION 1
#define FAULT_RECORD_TYPE 3

typedef int32_t fault_operation_t;
#define CURA_FAULT_NVS_INIT ((fault_operation_t)1)
#define CURA_FAULT_LITTLEFS_MOUNT ((fault_operation_t)2)
#define CURA_FAULT_QUEUE_METADATA_OPEN ((fault_operation_t)3)
#define CURA_FAULT_QUEUE_SEGMENT_READ ((fault_operation_t)4)
#define CURA_FAULT_QUEUE_SEGMENT_WRITE ((fault_operation_t)5)
#define CURA_FAULT_QUEUE_SEGMENT_DELETE ((fault_operation_t)6)

typedef struct __attribute__((packed)) {
  uint8_t node_uuid[16]; // Node UUID bytes in RFC 4122 byte order.
  uint8_t fault_id[8]; // Random identifier used to deduplicate retransmitted faults.
  uint32_t sample_id; // Reading sample id associated with the fault, or UINT32_MAX when unavailable.
  uint32_t bootno; // Boot number on which the fault was first observed.
  fault_operation_t operation; // Stable Cura operation that observed the failure.
  int32_t esp_err; // ESP-IDF error code, or ESP_OK when not applicable.
  int32_t posix_errno; // POSIX errno captured immediately after failure, or 0 when not applicable.
} fault_t;

_Static_assert(sizeof(fault_t) == 44, "unexpected fault_t size");
_Static_assert(FAULT_SCHEMA_VERSION <= UINT8_MAX, "TCP frame schema version is one byte");
_Static_assert(FAULT_RECORD_TYPE <= UINT8_MAX, "TCP frame record type is one byte");
#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__)
_Static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "fault_t wire schema requires little-endian target");
#endif
