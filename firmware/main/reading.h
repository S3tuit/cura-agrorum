/* Generated C header from protocol/wifi-protocol-v1/schemas/reading_v1.json by protocol/wifi-protocol-v1/tools/generate.py. Do not edit by hand. */
#pragma once

#include <stdint.h>

#define FILE_SCHEMA_VERSION 1
#define CURA_RECORD_TYPE 1

// Bitmask of which sensor fields are valid in a reading_t instance.
#define READING_SOIL_MV_OK (1u << 0)
#define READING_DS18B20_TEMP_OK (1u << 1)
#define READING_ENV280_TEMP_OK (1u << 2)
#define READING_ENV280_PRESSURE_OK (1u << 3)
#define READING_ENV280_HUMIDITY_OK (1u << 4)

typedef struct __attribute__((packed)) {
  uint8_t node_uuid[16]; // Node UUID bytes in RFC 4122 byte order.
  uint32_t sample_id; // Monotonically increasing reading id scoped to node_uuid.
  uint32_t bootno; // Monotonic across deep-sleep wakes, reset on cold boot.
  uint32_t wake_causes; // Bitmask returned by esp_sleep_get_wakeup_causes().
  uint16_t run_ms; // How long this wake cycle took before sleep.
  uint16_t soil_mv; // ADC reading converted to millivolts.
  int16_t ds18b20_centi_c; // DS18B20 temperature in 0.01 degree Celsius.
  int16_t env280_centi_c; // BME280 temperature in 0.01 degree Celsius.
  uint32_t env280_pressure_pa; // BME280 pressure in pascals.
  uint16_t env280_humidity_centi_pct; // BME280 relative humidity in 0.01 percent.
  uint8_t flags; // Bitmask of valid fields.
  uint8_t padding[1]; // Used for padding.
} reading_t;

_Static_assert(sizeof(reading_t) == 44, "unexpected reading_t size");
_Static_assert(FILE_SCHEMA_VERSION <= UINT8_MAX, "TCP frame schema version is one byte");
_Static_assert(CURA_RECORD_TYPE <= UINT8_MAX, "TCP frame record type is one byte");
#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__)
_Static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "reading_t wire schema requires little-endian target");
#endif
