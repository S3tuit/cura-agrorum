/* Generated C header from protocol/schemas/config_ack_v1.json by protocol/tools/generate.py. Do not edit by hand. */
#pragma once

#include <stdint.h>

#define CONFIG_ACK_SCHEMA_VERSION 1
#define CONFIG_ACK_RECORD_TYPE 3

typedef struct __attribute__((packed)) {
  uint32_t status; // Config batch result: 0 means accepted, nonzero means rejected.
} config_ack_t;

_Static_assert(sizeof(config_ack_t) == 4, "unexpected config_ack_t size");
_Static_assert(CONFIG_ACK_SCHEMA_VERSION <= UINT8_MAX, "TCP frame schema version is one byte");
_Static_assert(CONFIG_ACK_RECORD_TYPE <= UINT8_MAX, "TCP frame record type is one byte");
#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__)
_Static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "config_ack_t wire schema requires little-endian target");
#endif
