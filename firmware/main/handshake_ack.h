/* Generated C header from protocol/schemas/handshake_ack_v1.json by protocol/tools/generate.py. Do not edit by hand. */
#pragma once

#include <stdint.h>

#define HANDSHAKE_ACK_SCHEMA_VERSION 1
#define HANDSHAKE_ACK_RECORD_TYPE 3

typedef struct __attribute__((packed)) {
  uint32_t status; // Handshake result: 0 means accepted, nonzero means rejected.
} handshake_ack_t;

_Static_assert(sizeof(handshake_ack_t) == 4, "unexpected handshake_ack_t size");
_Static_assert(HANDSHAKE_ACK_SCHEMA_VERSION <= UINT8_MAX, "TCP frame schema version is one byte");
_Static_assert(HANDSHAKE_ACK_RECORD_TYPE <= UINT8_MAX, "TCP frame record type is one byte");
#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__)
_Static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "handshake_ack_t wire schema requires little-endian target");
#endif
