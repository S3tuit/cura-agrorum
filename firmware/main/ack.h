/* Generated C header from protocol/wifi-protocol-v1/schemas/ack_v1.json by protocol/wifi-protocol-v1/tools/generate.py. Do not edit by hand. */
#pragma once

#include <stdint.h>

#define ACK_SCHEMA_VERSION 1
#define ACK_RECORD_TYPE 2

typedef struct __attribute__((packed)) {
  uint32_t status; // Frame persistence result: 0 means all events persisted, nonzero means error.
} ack_t;

_Static_assert(sizeof(ack_t) == 4, "unexpected ack_t size");
_Static_assert(ACK_SCHEMA_VERSION <= UINT8_MAX, "TCP frame schema version is one byte");
_Static_assert(ACK_RECORD_TYPE <= UINT8_MAX, "TCP frame record type is one byte");
#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__)
_Static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "ack_t wire schema requires little-endian target");
#endif
