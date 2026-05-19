/* Generated C header from protocol/schemas/node_config_v1.json by protocol/tools/generate.py. Do not edit by hand. */
#pragma once

#include <stdint.h>

#define NODE_CONFIG_SCHEMA_VERSION 1
#define NODE_CONFIG_RECORD_TYPE 2

typedef struct __attribute__((packed)) {
  uint8_t node_uuid[16]; // Node UUID bytes in RFC 4122 byte order.
  uint16_t soil_sensor_id; // Numeric identifier for the soil sensor on this node.
  uint16_t ds18b20_sensor_id; // Numeric identifier for the DS18B20 sensor on this node.
  uint16_t env280_sensor_id; // Numeric identifier for the BME280 sensor on this node.
  uint16_t soil_dry_mv; // Soil sensor mV when reading air.
  uint16_t soil_wet_mv; // Soil sensor mV when reading water.
} node_config_t;

_Static_assert(sizeof(node_config_t) == 26, "unexpected node_config_t size");
_Static_assert(NODE_CONFIG_SCHEMA_VERSION <= UINT8_MAX, "TCP frame schema version is one byte");
_Static_assert(NODE_CONFIG_RECORD_TYPE <= UINT8_MAX, "TCP frame record type is one byte");
#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__)
_Static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "node_config_t wire schema requires little-endian target");
#endif
