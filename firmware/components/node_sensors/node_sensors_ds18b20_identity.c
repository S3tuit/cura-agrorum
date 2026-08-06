#include "node_sensors_ds18b20_identity.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "node_sensors.h"

#define DS18B20_CHANNEL_COUNT 2U

static int hex_nibble(char character) {
  if (character >= '0' && character <= '9') {
    return character - '0';
  }
  if (character >= 'a' && character <= 'f') {
    return character - 'a' + 10;
  }
  if (character >= 'A' && character <= 'F') {
    return character - 'A' + 10;
  }
  return -1;
}

static bool parse_rom(const char *text, uint64_t *out_rom) {
  if (text == NULL || out_rom == NULL || strlen(text) != 16U) {
    return false;
  }
  uint64_t value = 0U;
  for (size_t index = 0U; index < 16U; ++index) {
    const int nibble = hex_nibble(text[index]);
    if (nibble < 0) {
      return false;
    }
    value = (value << 4U) | (uint64_t)(unsigned int)nibble;
  }
  *out_rom = value;
  return value != 0U;
}

static node_sensors_backend_result_t identity_error(int32_t status) {
  return (node_sensors_backend_result_t){
      .kind = NODE_SENSOR_BACKEND_STATUS_INTERNAL,
      .status = status,
      .operation = CURAG_OP_VALIDATE,
  };
}

bool node_sensors_ds18b20_resolve_identities(
    const char *const identities[2], uint64_t out_roms[2],
    node_sensors_backend_result_t out_channel[2]) {
  if (identities == NULL || out_roms == NULL || out_channel == NULL) {
    return false;
  }
  memset(out_roms, 0, sizeof(uint64_t) * DS18B20_CHANNEL_COUNT);
  memset(out_channel, 0,
         sizeof(node_sensors_backend_result_t) * DS18B20_CHANNEL_COUNT);

  bool needs_bus = false;
  for (size_t channel = 0U; channel < DS18B20_CHANNEL_COUNT; ++channel) {
    if (!parse_rom(identities[channel], &out_roms[channel])) {
      out_channel[channel] =
          identity_error(NODE_SENSORS_INTERNAL_UNPROVISIONED_IDENTITY);
    } else {
      needs_bus = true;
    }
  }

  if (out_roms[0] != 0U && out_roms[0] == out_roms[1]) {
    out_channel[0] = identity_error(NODE_SENSORS_INTERNAL_DUPLICATE_IDENTITY);
    out_channel[1] = identity_error(NODE_SENSORS_INTERNAL_DUPLICATE_IDENTITY);
    return false;
  }
  return needs_bus;
}
