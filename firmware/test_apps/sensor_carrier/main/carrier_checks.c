#include "carrier_checks.h"
#include "unity.h"
#include "esp_err.h"

void carrier_assert_inventory(const uint64_t configured[2], const uint64_t *roms,
                              size_t count, size_t ds_count,
                              unsigned expected_ds_mask) {
  TEST_ASSERT_TRUE(expected_ds_mask >= 1 && expected_ds_mask <= 3);
  TEST_ASSERT_TRUE(configured[0] && configured[1] && configured[0] != configured[1]);
  const size_t expected_count = expected_ds_mask == 3 ? 2 : 1;
  TEST_ASSERT_EQUAL_MESSAGE(expected_count, count, "declared ROM inventory mismatch");
  TEST_ASSERT_EQUAL(expected_count, ds_count);
  unsigned seen = 0;
  for (size_t i = 0; i < count; ++i) {
    const unsigned bit = roms[i] == configured[0] ? 1 : roms[i] == configured[1] ? 2 : 0;
    TEST_ASSERT_TRUE_MESSAGE(bit && (expected_ds_mask & bit) && !(seen & bit),
                             "wrong, unexpected or repeated configured ROM");
    seen |= bit;
  }
  TEST_ASSERT_EQUAL_HEX8(expected_ds_mask, seen);
}

static uint32_t read_u32_le(const uint8_t *bytes) {
  return (uint32_t)bytes[0] | (uint32_t)bytes[1] << 8 |
         (uint32_t)bytes[2] << 16 | (uint32_t)bytes[3] << 24;
}

void carrier_assert_sample(const node_sensor_sample_t *sample,
                           const diagn_context_t *diagnostic, err_curag_t result,
                           unsigned expected_ds_mask, bool air_soil, bool bme_present) {
  TEST_ASSERT_TRUE(expected_ds_mask >= 1 && expected_ds_mask <= 3);
  const bool nominal = expected_ds_mask == 3 && bme_present;
  TEST_ASSERT_EQUAL_HEX32(nominal ? CURAG_OK :
      curag_error_make(CURAG_EDOM_SENSORS, CURAG_ESENSORS_EPARTIAL_SAMPLE), result);
  TEST_ASSERT_EQUAL_HEX8(0x03U | (expected_ds_mask << 2) | (bme_present ? 0x10U : 0U), sample->validity);
  if (!(expected_ds_mask & 1)) TEST_ASSERT_EQUAL_INT16(0, sample->soil_temp_0_centi_c);
  if (!(expected_ds_mask & 2)) TEST_ASSERT_EQUAL_INT16(0, sample->soil_temp_1_centi_c);
  if (!bme_present) {
    TEST_ASSERT_EQUAL_INT16(0, sample->enclosure_centi_c);
    TEST_ASSERT_EQUAL_UINT32(0, sample->enclosure_pressure_pa);
    TEST_ASSERT_EQUAL_UINT32(0, sample->enclosure_humidity_centi_pct);
  }
  if (air_soil) {
    TEST_ASSERT_TRUE_MESSAGE(sample->soil_0_mv >= 2000U && sample->soil_0_mv <= 2700U,
                             "soil0 outside air-probe range");
    TEST_ASSERT_TRUE_MESSAGE(sample->soil_1_mv >= 2000U && sample->soil_1_mv <= 2700U,
                             "soil1 outside air-probe range");
  }
  TEST_ASSERT_EQUAL(nominal ? CURAG_OP_NONE : CURAG_OP_INITIALIZE, diagnostic->operation);
  TEST_ASSERT_EQUAL(nominal ? 0 : CURAG_SENSOR_CONTEXT_V1, diagnostic->context_schema);
  TEST_ASSERT_EQUAL(nominal ? 0 : CURAG_SENSOR_CONTEXT_V1_LENGTH, diagnostic->context_length);
  /* INTERFACE.md: six LE kind/status pairs. The selected backend marks an
   * absent configured device as DRIVER / NOT_FOUND at INITIALIZE, not READ. */
  for (unsigned pair = 0; pair < NODE_SENSOR_CONTEXT_PAIR_COUNT; ++pair) {
    const bool missing = (pair == NODE_SENSOR_CONTEXT_SOIL_TEMP_0 && !(expected_ds_mask & 1)) ||
                         (pair == NODE_SENSOR_CONTEXT_SOIL_TEMP_1 && !(expected_ds_mask & 2));
    const bool missing_bme = pair == NODE_SENSOR_CONTEXT_ENCLOSURE_ENV && !bme_present;
    TEST_ASSERT_EQUAL_UINT32(missing ? NODE_SENSOR_BACKEND_STATUS_DRIVER :
                             missing_bme ? NODE_SENSOR_BACKEND_STATUS_ESP_ERR : 0,
                             read_u32_le(&diagnostic->context[pair * 8]));
    TEST_ASSERT_EQUAL_UINT32(missing ? ESP_ERR_NOT_FOUND :
                             missing_bme ? ESP_ERR_INVALID_RESPONSE : 0,
                             read_u32_le(&diagnostic->context[pair * 8 + 4]));
  }
  for (size_t i = CURAG_SENSOR_CONTEXT_V1_LENGTH; i < sizeof(diagnostic->context); ++i) {
    TEST_ASSERT_EQUAL_HEX8(0, diagnostic->context[i]);
  }
}

void carrier_assert_bme_sleep(uint8_t status, uint8_t control) {
  TEST_ASSERT_EQUAL_HEX8(0, control & 3U);
  TEST_ASSERT_EQUAL_HEX8(0, status & 9U);
}
