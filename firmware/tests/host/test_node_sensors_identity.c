#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "node_sensors.h"
#include "node_sensors_ds18b20_identity.h"

#define TEST_ASSERT(expression)                                                \
  do {                                                                         \
    if (!(expression)) {                                                       \
      fprintf(stderr, "%s:%d: assertion failed: %s\n", __FILE__, __LINE__,     \
              #expression);                                                    \
      return false;                                                            \
    }                                                                          \
  } while (0)

static bool result_is_success(node_sensors_backend_result_t result) {
  return result.kind == NODE_SENSOR_BACKEND_STATUS_NONE && result.status == 0 &&
         result.operation == CURAG_OP_NONE;
}

static bool result_is_identity_error(node_sensors_backend_result_t result,
                                     int32_t status) {
  return result.kind == NODE_SENSOR_BACKEND_STATUS_INTERNAL &&
         result.status == status && result.operation == CURAG_OP_VALIDATE;
}

static bool test_distinct_identities_are_resolved(void) {
  const char *identities[2] = {
      "1122334455667788",
      "A1b2C3d4E5f60718",
  };
  uint64_t roms[2] = {0U, 0U};
  node_sensors_backend_result_t channel[2] = {0};
  TEST_ASSERT(
      node_sensors_ds18b20_resolve_identities(identities, roms, channel));
  TEST_ASSERT(roms[0] == UINT64_C(0x1122334455667788));
  TEST_ASSERT(roms[1] == UINT64_C(0xa1b2c3d4e5f60718));
  TEST_ASSERT(result_is_success(channel[0]));
  TEST_ASSERT(result_is_success(channel[1]));
  return true;
}

static bool test_duplicate_identities_forbid_bus_access(void) {
  const char *identities[2] = {
      "1122334455667788",
      "1122334455667788",
  };
  uint64_t roms[2] = {0U, 0U};
  node_sensors_backend_result_t channel[2] = {0};
  TEST_ASSERT(
      !node_sensors_ds18b20_resolve_identities(identities, roms, channel));
  TEST_ASSERT(result_is_identity_error(
      channel[0], NODE_SENSORS_INTERNAL_DUPLICATE_IDENTITY));
  TEST_ASSERT(result_is_identity_error(
      channel[1], NODE_SENSORS_INTERNAL_DUPLICATE_IDENTITY));
  return true;
}

static bool test_invalid_identities_are_reported_independently(void) {
  const char *identities[2] = {
      "0000000000000000",
      "not-a-rom",
  };
  uint64_t roms[2] = {UINT64_MAX, UINT64_MAX};
  node_sensors_backend_result_t channel[2] = {0};
  TEST_ASSERT(
      !node_sensors_ds18b20_resolve_identities(identities, roms, channel));
  TEST_ASSERT(roms[0] == 0U);
  TEST_ASSERT(roms[1] == 0U);
  TEST_ASSERT(result_is_identity_error(
      channel[0], NODE_SENSORS_INTERNAL_UNPROVISIONED_IDENTITY));
  TEST_ASSERT(result_is_identity_error(
      channel[1], NODE_SENSORS_INTERNAL_UNPROVISIONED_IDENTITY));
  return true;
}

static bool test_one_valid_identity_still_requires_bus(void) {
  const char *identities[2] = {
      "0000000000000000",
      "0123456789abcdef",
  };
  uint64_t roms[2] = {0U, 0U};
  node_sensors_backend_result_t channel[2] = {0};
  TEST_ASSERT(
      node_sensors_ds18b20_resolve_identities(identities, roms, channel));
  TEST_ASSERT(result_is_identity_error(
      channel[0], NODE_SENSORS_INTERNAL_UNPROVISIONED_IDENTITY));
  TEST_ASSERT(result_is_success(channel[1]));
  TEST_ASSERT(roms[1] == UINT64_C(0x0123456789abcdef));
  return true;
}

int main(void) {
  if (!test_distinct_identities_are_resolved() ||
      !test_duplicate_identities_forbid_bus_access() ||
      !test_invalid_identities_are_reported_independently() ||
      !test_one_valid_identity_still_requires_bus()) {
    return 1;
  }
  puts("PASS node_sensors_identity");
  return 0;
}
