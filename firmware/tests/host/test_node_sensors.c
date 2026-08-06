#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "fake_esp_gpio.h"
#include "fake_node_sensors_backend.h"
#include "node_common.h"
#include "node_sensors.h"
#include "node_sensors_power_gate.h"
#include "sdkconfig.h"

#define TEST_ASSERT(expression)                                                \
  do {                                                                         \
    if (!(expression)) {                                                       \
      fprintf(stderr, "%s:%d: assertion failed: %s\n", __FILE__, __LINE__,     \
              #expression);                                                    \
      return false;                                                            \
    }                                                                          \
  } while (0)

#define TEST_ASSERT_EQ_U32(expected, actual)                                   \
  do {                                                                         \
    const uint32_t test_expected_ = (uint32_t)(expected);                      \
    const uint32_t test_actual_ = (uint32_t)(actual);                          \
    if (test_expected_ != test_actual_) {                                      \
      fprintf(stderr, "%s:%d: expected 0x%08lx, got 0x%08lx: %s\n", __FILE__,  \
              __LINE__, (unsigned long)test_expected_,                         \
              (unsigned long)test_actual_, #actual);                           \
      return false;                                                            \
    }                                                                          \
  } while (0)

typedef bool (*test_function_t)(void);

typedef struct {
  const char *name;
  test_function_t function;
} test_case_t;

typedef struct {
  uint32_t kind;
  int32_t status;
} decoded_status_t;

static uint32_t read_u32_le(const uint8_t input[4]) {
  return (uint32_t)input[0] | ((uint32_t)input[1] << 8U) |
         ((uint32_t)input[2] << 16U) | ((uint32_t)input[3] << 24U);
}

static decoded_status_t diagnostic_pair(const diagn_context_t *diagnostic,
                                        node_sensor_context_pair_t pair) {
  const uint8_t *const input = &diagnostic->context[(size_t)pair * 8U];
  return (decoded_status_t){
      .kind = read_u32_le(input),
      .status = (int32_t)read_u32_le(&input[4]),
  };
}

static bool assert_error(err_curag_t result, uint16_t code) {
  TEST_ASSERT_EQ_U32(CURAG_EDOM_SENSORS, curag_error_domain(result));
  TEST_ASSERT_EQ_U32(code, curag_error_code(result));
  return true;
}

static bool assert_diagnostic_header(const diagn_context_t *diagnostic,
                                     curag_operation_t operation) {
  TEST_ASSERT_EQ_U32(operation, diagnostic->operation);
  TEST_ASSERT_EQ_U32(CURAG_SENSOR_CONTEXT_V1, diagnostic->context_schema);
  TEST_ASSERT_EQ_U32(CURAG_SENSOR_CONTEXT_V1_LENGTH,
                     diagnostic->context_length);
  return true;
}

static bool assert_pair(const diagn_context_t *diagnostic,
                        node_sensor_context_pair_t pair, uint32_t kind,
                        int32_t status) {
  const decoded_status_t decoded = diagnostic_pair(diagnostic, pair);
  TEST_ASSERT_EQ_U32(kind, decoded.kind);
  TEST_ASSERT(decoded.status == status);
  return true;
}

static bool sample_is_default(const node_sensor_sample_t *sample) {
  TEST_ASSERT_EQ_U32(NODE_SENSOR_VALIDITY_ALL, sample->validity);
  TEST_ASSERT_EQ_U32(1234U, sample->soil_0_mv);
  TEST_ASSERT_EQ_U32(2345U, sample->soil_1_mv);
  TEST_ASSERT(sample->soil_temp_0_centi_c == 2150);
  TEST_ASSERT(sample->soil_temp_1_centi_c == 1875);
  TEST_ASSERT(sample->enclosure_centi_c == 2675);
  TEST_ASSERT_EQ_U32(101325U, sample->enclosure_pressure_pa);
  TEST_ASSERT_EQ_U32(5432U, sample->enclosure_humidity_centi_pct);
  return true;
}

static bool test_success_sequence_and_values(void) {
  fake_node_sensors_reset();
  node_sensor_sample_t sample = {0};
  diagn_context_t diagnostic;
  memset(&diagnostic, 0xa5, sizeof(diagnostic));

  TEST_ASSERT_EQ_U32(CURAG_OK, node_sensors_sample_all(&sample, &diagnostic));
  TEST_ASSERT(sample_is_default(&sample));
  const diagn_context_t empty = {0};
  TEST_ASSERT(memcmp(&diagnostic, &empty, sizeof(empty)) == 0);
  TEST_ASSERT_EQ_U32(200U, fake_node_sensors_last_delay_ms());

  static const fake_sensor_operation_t expected[] = {
      FAKE_SENSOR_OP_POWER_ON, FAKE_SENSOR_OP_DELAY,   FAKE_SENSOR_OP_SOIL_0,
      FAKE_SENSOR_OP_SOIL_1,   FAKE_SENSOR_OP_DS18B20, FAKE_SENSOR_OP_POWER_OFF,
      FAKE_SENSOR_OP_BME280,
  };
  TEST_ASSERT_EQ_U32(sizeof(expected) / sizeof(expected[0]),
                     fake_node_sensors_trace_count());
  for (size_t index = 0U; index < sizeof(expected) / sizeof(expected[0]);
       ++index) {
    TEST_ASSERT_EQ_U32(expected[index], fake_node_sensors_trace_at(index));
  }
  return true;
}

static bool test_null_sample_is_rejected_without_hardware(void) {
  fake_node_sensors_reset();
  diagn_context_t diagnostic;
  const err_curag_t result = node_sensors_sample_all(NULL, &diagnostic);
  TEST_ASSERT(assert_error(result, CURAG_ESENSORS_EINVALID_ARGUMENT));
  TEST_ASSERT(assert_diagnostic_header(&diagnostic, CURAG_OP_VALIDATE));
  TEST_ASSERT_EQ_U32(0U, fake_node_sensors_trace_count());
  for (size_t pair = 0U; pair < NODE_SENSOR_CONTEXT_PAIR_COUNT; ++pair) {
    TEST_ASSERT(assert_pair(&diagnostic, (node_sensor_context_pair_t)pair,
                            NODE_SENSOR_BACKEND_STATUS_NONE, 0));
  }
  return true;
}

static bool test_each_group_failure_is_isolated(void) {
  for (size_t failing = 0U; failing < 5U; ++failing) {
    fake_node_sensors_reset();
    const node_sensors_backend_result_t failure =
        fake_node_sensors_result(NODE_SENSOR_BACKEND_STATUS_DRIVER,
                                 (int32_t)(100 + failing), CURAG_OP_READ);
    if (failing < 2U) {
      fake_node_sensors_set_soil(failing, UINT16_MAX, failure);
    } else if (failing < 4U) {
      fake_node_sensors_set_ds(failing - 2U, INT16_MAX, failure);
    } else {
      fake_node_sensors_set_bme(
          (node_sensors_backend_enclosure_t){
              .temperature_centi_c = INT16_MAX,
              .pressure_pa = UINT32_MAX,
              .humidity_centi_pct = UINT16_MAX,
          },
          failure);
    }

    node_sensor_sample_t sample;
    memset(&sample, 0xa5, sizeof(sample));
    diagn_context_t diagnostic;
    const err_curag_t result = node_sensors_sample_all(&sample, &diagnostic);
    TEST_ASSERT(assert_error(result, CURAG_ESENSORS_EPARTIAL_SAMPLE));
    TEST_ASSERT(assert_diagnostic_header(&diagnostic, CURAG_OP_READ));
    const uint8_t failed_bit = (uint8_t)(UINT8_C(1) << failing);
    TEST_ASSERT_EQ_U32((uint8_t)(NODE_SENSOR_VALIDITY_ALL & ~failed_bit),
                       sample.validity);
    TEST_ASSERT(assert_pair(&diagnostic,
                            (node_sensor_context_pair_t)(failing + 1U),
                            failure.kind, failure.status));
    if (failing == 0U) {
      TEST_ASSERT_EQ_U32(0U, sample.soil_0_mv);
    } else if (failing == 1U) {
      TEST_ASSERT_EQ_U32(0U, sample.soil_1_mv);
    } else if (failing == 2U) {
      TEST_ASSERT(sample.soil_temp_0_centi_c == 0);
    } else if (failing == 3U) {
      TEST_ASSERT(sample.soil_temp_1_centi_c == 0);
    } else {
      TEST_ASSERT(sample.enclosure_centi_c == 0);
      TEST_ASSERT_EQ_U32(0U, sample.enclosure_pressure_pa);
      TEST_ASSERT_EQ_U32(0U, sample.enclosure_humidity_centi_pct);
    }
    TEST_ASSERT_EQ_U32(
        1U, fake_node_sensors_operation_count(FAKE_SENSOR_OP_POWER_OFF));
  }
  return true;
}

static bool test_all_groups_failed_is_complete_sample_error(void) {
  fake_node_sensors_reset();
  const node_sensors_backend_result_t failure = fake_node_sensors_result(
      NODE_SENSOR_BACKEND_STATUS_DRIVER, -20, CURAG_OP_READ);
  fake_node_sensors_set_soil(0U, UINT16_MAX, failure);
  fake_node_sensors_set_soil(1U, UINT16_MAX, failure);
  fake_node_sensors_set_ds(0U, INT16_MAX, failure);
  fake_node_sensors_set_ds(1U, INT16_MAX, failure);
  fake_node_sensors_set_bme(
      (node_sensors_backend_enclosure_t){INT16_MAX, UINT32_MAX, UINT16_MAX},
      failure);

  node_sensor_sample_t sample;
  memset(&sample, 0xa5, sizeof(sample));
  diagn_context_t diagnostic;
  const err_curag_t result = node_sensors_sample_all(&sample, &diagnostic);
  TEST_ASSERT(assert_error(result, CURAG_ESENSORS_ECOMPLETE_SAMPLE));
  TEST_ASSERT_EQ_U32(0U, sample.validity);
  TEST_ASSERT_EQ_U32(0U, sample.soil_0_mv);
  TEST_ASSERT_EQ_U32(0U, sample.soil_1_mv);
  TEST_ASSERT(sample.soil_temp_0_centi_c == 0);
  TEST_ASSERT(sample.soil_temp_1_centi_c == 0);
  TEST_ASSERT(sample.enclosure_centi_c == 0);
  TEST_ASSERT_EQ_U32(0U, sample.enclosure_pressure_pa);
  TEST_ASSERT_EQ_U32(0U, sample.enclosure_humidity_centi_pct);
  return true;
}

static bool test_multiple_failures_remain_in_fixed_slots(void) {
  fake_node_sensors_reset();
  const node_sensors_backend_result_t soil_failure = fake_node_sensors_result(
      NODE_SENSOR_BACKEND_STATUS_ESP_ERR, -31, CURAG_OP_READ);
  const node_sensors_backend_result_t temperature_failure =
      fake_node_sensors_result(NODE_SENSOR_BACKEND_STATUS_DRIVER, -32,
                               CURAG_OP_READ);
  const node_sensors_backend_result_t bme_failure = fake_node_sensors_result(
      NODE_SENSOR_BACKEND_STATUS_DRIVER, -33, CURAG_OP_READ);
  fake_node_sensors_set_soil(0U, UINT16_MAX, soil_failure);
  fake_node_sensors_set_ds(1U, INT16_MAX, temperature_failure);
  fake_node_sensors_set_bme(
      (node_sensors_backend_enclosure_t){INT16_MAX, UINT32_MAX, UINT16_MAX},
      bme_failure);

  node_sensor_sample_t sample;
  diagn_context_t diagnostic;
  const err_curag_t result = node_sensors_sample_all(&sample, &diagnostic);
  TEST_ASSERT(assert_error(result, CURAG_ESENSORS_EPARTIAL_SAMPLE));
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_SOIL_0,
                          soil_failure.kind, soil_failure.status));
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_SOIL_TEMP_1,
                          temperature_failure.kind,
                          temperature_failure.status));
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_ENCLOSURE_ENV,
                          bme_failure.kind, bme_failure.status));
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_SOIL_1,
                          NODE_SENSOR_BACKEND_STATUS_NONE, 0));
  return true;
}

static bool test_shared_ds_failure_marks_both_channels_blocked(void) {
  fake_node_sensors_reset();
  const node_sensors_backend_result_t shared = fake_node_sensors_result(
      NODE_SENSOR_BACKEND_STATUS_ESP_ERR, -40, CURAG_OP_INITIALIZE);
  fake_node_sensors_set_ds_shared(shared);

  node_sensor_sample_t sample;
  diagn_context_t diagnostic;
  const err_curag_t result = node_sensors_sample_all(&sample, &diagnostic);
  TEST_ASSERT(assert_error(result, CURAG_ESENSORS_EPARTIAL_SAMPLE));
  TEST_ASSERT(assert_diagnostic_header(&diagnostic, CURAG_OP_INITIALIZE));
  TEST_ASSERT_EQ_U32(NODE_SENSOR_SOIL_0_VALID | NODE_SENSOR_SOIL_1_VALID |
                         NODE_SENSOR_ENCLOSURE_ENV_VALID,
                     sample.validity);
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_COMPONENT,
                          shared.kind, shared.status));
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_SOIL_TEMP_0,
                          NODE_SENSOR_BACKEND_STATUS_INTERNAL,
                          NODE_SENSORS_INTERNAL_BLOCKED_BY_SHARED_FAILURE));
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_SOIL_TEMP_1,
                          NODE_SENSOR_BACKEND_STATUS_INTERNAL,
                          NODE_SENSORS_INTERNAL_BLOCKED_BY_SHARED_FAILURE));
  return true;
}

static bool test_power_on_failure_blocks_gated_groups_but_samples_bme(void) {
  fake_node_sensors_reset();
  const node_sensors_backend_result_t failure = fake_node_sensors_result(
      NODE_SENSOR_BACKEND_STATUS_ESP_ERR, -50, CURAG_OP_POWER_ON);
  fake_node_sensors_set_power_on(failure);

  node_sensor_sample_t sample;
  diagn_context_t diagnostic;
  const err_curag_t result = node_sensors_sample_all(&sample, &diagnostic);
  TEST_ASSERT(assert_error(result, CURAG_ESENSORS_EPOWER_CONTROL));
  TEST_ASSERT(assert_diagnostic_header(&diagnostic, CURAG_OP_POWER_ON));
  TEST_ASSERT_EQ_U32(NODE_SENSOR_ENCLOSURE_ENV_VALID, sample.validity);
  TEST_ASSERT_EQ_U32(0U,
                     fake_node_sensors_operation_count(FAKE_SENSOR_OP_DELAY));
  TEST_ASSERT_EQ_U32(0U,
                     fake_node_sensors_operation_count(FAKE_SENSOR_OP_DS18B20));
  TEST_ASSERT_EQ_U32(1U,
                     fake_node_sensors_operation_count(FAKE_SENSOR_OP_BME280));
  TEST_ASSERT_EQ_U32(
      1U, fake_node_sensors_operation_count(FAKE_SENSOR_OP_POWER_OFF));
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_COMPONENT,
                          failure.kind, failure.status));
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_SOIL_0,
                          NODE_SENSOR_BACKEND_STATUS_INTERNAL,
                          NODE_SENSORS_INTERNAL_BLOCKED_BY_SHARED_FAILURE));
  return true;
}

static bool test_power_off_failure_takes_precedence_and_preserves_sample(void) {
  fake_node_sensors_reset();
  const node_sensors_backend_result_t failure = fake_node_sensors_result(
      NODE_SENSOR_BACKEND_STATUS_ESP_ERR, -60, CURAG_OP_POWER_OFF);
  fake_node_sensors_set_power_off(failure);

  node_sensor_sample_t sample;
  diagn_context_t diagnostic;
  const err_curag_t result = node_sensors_sample_all(&sample, &diagnostic);
  TEST_ASSERT(assert_error(result, CURAG_ESENSORS_EPOWER_CONTROL));
  TEST_ASSERT(sample_is_default(&sample));
  TEST_ASSERT(assert_diagnostic_header(&diagnostic, CURAG_OP_POWER_OFF));
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_COMPONENT,
                          failure.kind, failure.status));
  return true;
}

static bool test_power_cleanup_and_sensor_failure_are_both_retained(void) {
  fake_node_sensors_reset();
  const node_sensors_backend_result_t sensor_failure = fake_node_sensors_result(
      NODE_SENSOR_BACKEND_STATUS_DRIVER, -70, CURAG_OP_READ);
  const node_sensors_backend_result_t power_failure = fake_node_sensors_result(
      NODE_SENSOR_BACKEND_STATUS_ESP_ERR, -71, CURAG_OP_POWER_OFF);
  fake_node_sensors_set_bme(
      (node_sensors_backend_enclosure_t){INT16_MAX, UINT32_MAX, UINT16_MAX},
      sensor_failure);
  fake_node_sensors_set_power_off(power_failure);

  node_sensor_sample_t sample;
  diagn_context_t diagnostic;
  const err_curag_t result = node_sensors_sample_all(&sample, &diagnostic);
  TEST_ASSERT(assert_error(result, CURAG_ESENSORS_EPOWER_CONTROL));
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_COMPONENT,
                          power_failure.kind, power_failure.status));
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_ENCLOSURE_ENV,
                          sensor_failure.kind, sensor_failure.status));
  return true;
}

static bool test_force_power_off_is_untouched_noop_then_idempotent(void) {
  fake_node_sensors_reset();
  diagn_context_t diagnostic;
  TEST_ASSERT_EQ_U32(CURAG_OK, node_sensors_force_power_off(&diagnostic));
  TEST_ASSERT_EQ_U32(0U, fake_node_sensors_trace_count());

  node_sensor_sample_t sample;
  TEST_ASSERT_EQ_U32(CURAG_OK, node_sensors_sample_all(&sample, NULL));
  TEST_ASSERT_EQ_U32(CURAG_OK, node_sensors_force_power_off(&diagnostic));
  TEST_ASSERT_EQ_U32(CURAG_OK, node_sensors_force_power_off(&diagnostic));
  TEST_ASSERT_EQ_U32(
      3U, fake_node_sensors_operation_count(FAKE_SENSOR_OP_POWER_OFF));
  TEST_ASSERT_EQ_U32(
      1U, fake_node_sensors_operation_count(FAKE_SENSOR_OP_POWER_ON));
  return true;
}

static bool test_force_power_off_failure_has_component_diagnostic(void) {
  fake_node_sensors_reset();
  node_sensor_sample_t sample;
  TEST_ASSERT_EQ_U32(CURAG_OK, node_sensors_sample_all(&sample, NULL));
  const node_sensors_backend_result_t failure = fake_node_sensors_result(
      NODE_SENSOR_BACKEND_STATUS_ESP_ERR, -80, CURAG_OP_POWER_OFF);
  fake_node_sensors_set_power_off(failure);

  diagn_context_t diagnostic;
  const err_curag_t result = node_sensors_force_power_off(&diagnostic);
  TEST_ASSERT(assert_error(result, CURAG_ESENSORS_EPOWER_CONTROL));
  TEST_ASSERT(assert_diagnostic_header(&diagnostic, CURAG_OP_POWER_OFF));
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_COMPONENT,
                          failure.kind, failure.status));
  return true;
}

static bool test_null_diagnostic_is_supported_on_failure(void) {
  fake_node_sensors_reset();
  fake_node_sensors_set_soil(
      0U, UINT16_MAX,
      fake_node_sensors_result(NODE_SENSOR_BACKEND_STATUS_DRIVER, -90,
                               CURAG_OP_READ));
  node_sensor_sample_t sample;
  const err_curag_t result = node_sensors_sample_all(&sample, NULL);
  TEST_ASSERT(assert_error(result, CURAG_ESENSORS_EPARTIAL_SAMPLE));
  TEST_ASSERT_EQ_U32(0U, sample.soil_0_mv);
  TEST_ASSERT_EQ_U32(
      1U, fake_node_sensors_operation_count(FAKE_SENSOR_OP_POWER_OFF));
  return true;
}

static bool test_ds_cleanup_failure_preserves_successful_values(void) {
  fake_node_sensors_reset();
  const node_sensors_backend_result_t cleanup = fake_node_sensors_result(
      NODE_SENSOR_BACKEND_STATUS_ESP_ERR, -100, CURAG_OP_CLEANUP);
  fake_node_sensors_set_ds_cleanup(cleanup);

  node_sensor_sample_t sample;
  diagn_context_t diagnostic;
  const err_curag_t result = node_sensors_sample_all(&sample, &diagnostic);
  TEST_ASSERT(assert_error(result, CURAG_ESENSORS_ECLEANUP));
  TEST_ASSERT(sample_is_default(&sample));
  TEST_ASSERT(assert_diagnostic_header(&diagnostic, CURAG_OP_CLEANUP));
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_COMPONENT,
                          cleanup.kind, cleanup.status));
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_SOIL_TEMP_0,
                          NODE_SENSOR_BACKEND_STATUS_NONE, 0));
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_SOIL_TEMP_1,
                          NODE_SENSOR_BACKEND_STATUS_NONE, 0));
  return true;
}

static bool test_ds_channel_and_cleanup_failures_preserve_other_channel(void) {
  fake_node_sensors_reset();
  const node_sensors_backend_result_t channel_failure =
      fake_node_sensors_result(NODE_SENSOR_BACKEND_STATUS_DRIVER, -110,
                               CURAG_OP_READ);
  const node_sensors_backend_result_t cleanup = fake_node_sensors_result(
      NODE_SENSOR_BACKEND_STATUS_ESP_ERR, -111, CURAG_OP_CLEANUP);
  fake_node_sensors_set_ds(0U, INT16_MAX, channel_failure);
  fake_node_sensors_set_ds_cleanup(cleanup);

  node_sensor_sample_t sample;
  diagn_context_t diagnostic;
  const err_curag_t result = node_sensors_sample_all(&sample, &diagnostic);
  TEST_ASSERT(assert_error(result, CURAG_ESENSORS_ECLEANUP));
  TEST_ASSERT(assert_diagnostic_header(&diagnostic, CURAG_OP_CLEANUP));
  TEST_ASSERT_EQ_U32(
      (uint8_t)(NODE_SENSOR_VALIDITY_ALL & ~NODE_SENSOR_SOIL_TEMP_0_VALID),
      sample.validity);
  TEST_ASSERT(sample.soil_temp_0_centi_c == 0);
  TEST_ASSERT(sample.soil_temp_1_centi_c == 1875);
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_SOIL_TEMP_0,
                          channel_failure.kind, channel_failure.status));
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_COMPONENT,
                          cleanup.kind, cleanup.status));
  return true;
}

static bool test_ds_shared_read_operation_is_preserved(void) {
  fake_node_sensors_reset();
  const node_sensors_backend_result_t conversion = fake_node_sensors_result(
      NODE_SENSOR_BACKEND_STATUS_DRIVER, -120, CURAG_OP_READ);
  fake_node_sensors_set_ds_shared(conversion);

  node_sensor_sample_t sample;
  diagn_context_t diagnostic;
  const err_curag_t result = node_sensors_sample_all(&sample, &diagnostic);
  TEST_ASSERT(assert_error(result, CURAG_ESENSORS_EPARTIAL_SAMPLE));
  TEST_ASSERT(assert_diagnostic_header(&diagnostic, CURAG_OP_READ));
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_COMPONENT,
                          conversion.kind, conversion.status));
  return true;
}

static bool test_validation_precedes_initialize_and_read(void) {
  fake_node_sensors_reset();
  const node_sensors_backend_result_t validation = fake_node_sensors_result(
      NODE_SENSOR_BACKEND_STATUS_INTERNAL, -130, CURAG_OP_VALIDATE);
  const node_sensors_backend_result_t initialization = fake_node_sensors_result(
      NODE_SENSOR_BACKEND_STATUS_DRIVER, -131, CURAG_OP_INITIALIZE);
  const node_sensors_backend_result_t read = fake_node_sensors_result(
      NODE_SENSOR_BACKEND_STATUS_DRIVER, -132, CURAG_OP_READ);
  fake_node_sensors_set_ds(0U, INT16_MAX, validation);
  fake_node_sensors_set_ds(1U, INT16_MAX, initialization);
  fake_node_sensors_set_bme((node_sensors_backend_enclosure_t){0}, read);

  node_sensor_sample_t sample;
  diagn_context_t diagnostic;
  const err_curag_t result = node_sensors_sample_all(&sample, &diagnostic);
  TEST_ASSERT(assert_error(result, CURAG_ESENSORS_EPARTIAL_SAMPLE));
  TEST_ASSERT(assert_diagnostic_header(&diagnostic, CURAG_OP_VALIDATE));
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_SOIL_TEMP_0,
                          validation.kind, validation.status));
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_SOIL_TEMP_1,
                          initialization.kind, initialization.status));
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_ENCLOSURE_ENV,
                          read.kind, read.status));
  return true;
}

static bool test_bme_stage_operation_is_preserved(void) {
  static const curag_operation_t operations[] = {
      CURAG_OP_INITIALIZE,
      CURAG_OP_READ,
  };
  for (size_t index = 0U; index < sizeof(operations) / sizeof(operations[0]);
       ++index) {
    fake_node_sensors_reset();
    const node_sensors_backend_result_t failure = fake_node_sensors_result(
        NODE_SENSOR_BACKEND_STATUS_DRIVER, (int32_t)(-140 - (int32_t)index),
        operations[index]);
    fake_node_sensors_set_bme((node_sensors_backend_enclosure_t){0}, failure);

    node_sensor_sample_t sample;
    diagn_context_t diagnostic;
    const err_curag_t result = node_sensors_sample_all(&sample, &diagnostic);
    TEST_ASSERT(assert_error(result, CURAG_ESENSORS_EPARTIAL_SAMPLE));
    TEST_ASSERT(assert_diagnostic_header(&diagnostic, operations[index]));
    TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_ENCLOSURE_ENV,
                            failure.kind, failure.status));
  }
  return true;
}

static bool test_shared_ds_then_power_off_uses_accepted_precedence(void) {
  fake_node_sensors_reset();
  const node_sensors_backend_result_t shared = fake_node_sensors_result(
      NODE_SENSOR_BACKEND_STATUS_ESP_ERR, -150, CURAG_OP_INITIALIZE);
  const node_sensors_backend_result_t power_off = fake_node_sensors_result(
      NODE_SENSOR_BACKEND_STATUS_ESP_ERR, -151, CURAG_OP_POWER_OFF);
  fake_node_sensors_set_ds_shared(shared);
  fake_node_sensors_set_power_off(power_off);

  node_sensor_sample_t sample;
  diagn_context_t diagnostic;
  const err_curag_t result = node_sensors_sample_all(&sample, &diagnostic);
  TEST_ASSERT(assert_error(result, CURAG_ESENSORS_EPOWER_CONTROL));
  TEST_ASSERT(assert_diagnostic_header(&diagnostic, CURAG_OP_POWER_OFF));
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_COMPONENT,
                          power_off.kind, power_off.status));
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_SOIL_TEMP_0,
                          NODE_SENSOR_BACKEND_STATUS_INTERNAL,
                          NODE_SENSORS_INTERNAL_BLOCKED_BY_SHARED_FAILURE));
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_SOIL_TEMP_1,
                          NODE_SENSOR_BACKEND_STATUS_INTERNAL,
                          NODE_SENSORS_INTERNAL_BLOCKED_BY_SHARED_FAILURE));
  return true;
}

static bool test_cleanup_precedes_shared_acquisition_failure(void) {
  fake_node_sensors_reset();
  const node_sensors_backend_result_t shared = fake_node_sensors_result(
      NODE_SENSOR_BACKEND_STATUS_ESP_ERR, -155, CURAG_OP_INITIALIZE);
  const node_sensors_backend_result_t cleanup = fake_node_sensors_result(
      NODE_SENSOR_BACKEND_STATUS_ESP_ERR, -156, CURAG_OP_CLEANUP);
  fake_node_sensors_set_ds_shared(shared);
  fake_node_sensors_set_ds_cleanup(cleanup);

  node_sensor_sample_t sample;
  diagn_context_t diagnostic;
  const err_curag_t result = node_sensors_sample_all(&sample, &diagnostic);
  TEST_ASSERT(assert_error(result, CURAG_ESENSORS_ECLEANUP));
  TEST_ASSERT(assert_diagnostic_header(&diagnostic, CURAG_OP_CLEANUP));
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_COMPONENT,
                          cleanup.kind, cleanup.status));
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_SOIL_TEMP_0,
                          NODE_SENSOR_BACKEND_STATUS_INTERNAL,
                          NODE_SENSORS_INTERNAL_BLOCKED_BY_SHARED_FAILURE));
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_SOIL_TEMP_1,
                          NODE_SENSOR_BACKEND_STATUS_INTERNAL,
                          NODE_SENSORS_INTERNAL_BLOCKED_BY_SHARED_FAILURE));
  return true;
}

static bool test_power_off_precedes_cleanup(void) {
  fake_node_sensors_reset();
  const node_sensors_backend_result_t cleanup = fake_node_sensors_result(
      NODE_SENSOR_BACKEND_STATUS_ESP_ERR, -157, CURAG_OP_CLEANUP);
  const node_sensors_backend_result_t power_off = fake_node_sensors_result(
      NODE_SENSOR_BACKEND_STATUS_ESP_ERR, -158, CURAG_OP_POWER_OFF);
  fake_node_sensors_set_ds_cleanup(cleanup);
  fake_node_sensors_set_power_off(power_off);

  node_sensor_sample_t sample;
  diagn_context_t diagnostic;
  const err_curag_t result = node_sensors_sample_all(&sample, &diagnostic);
  TEST_ASSERT(assert_error(result, CURAG_ESENSORS_EPOWER_CONTROL));
  TEST_ASSERT(sample_is_default(&sample));
  TEST_ASSERT(assert_diagnostic_header(&diagnostic, CURAG_OP_POWER_OFF));
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_COMPONENT,
                          power_off.kind, power_off.status));
  return true;
}

static bool test_power_off_precedes_power_on(void) {
  fake_node_sensors_reset();
  const node_sensors_backend_result_t power_on = fake_node_sensors_result(
      NODE_SENSOR_BACKEND_STATUS_ESP_ERR, -160, CURAG_OP_POWER_ON);
  const node_sensors_backend_result_t power_off = fake_node_sensors_result(
      NODE_SENSOR_BACKEND_STATUS_ESP_ERR, -161, CURAG_OP_POWER_OFF);
  fake_node_sensors_set_power_on(power_on);
  fake_node_sensors_set_power_off(power_off);

  node_sensor_sample_t sample;
  diagn_context_t diagnostic;
  const err_curag_t result = node_sensors_sample_all(&sample, &diagnostic);
  TEST_ASSERT(assert_error(result, CURAG_ESENSORS_EPOWER_CONTROL));
  TEST_ASSERT(assert_diagnostic_header(&diagnostic, CURAG_OP_POWER_OFF));
  TEST_ASSERT(assert_pair(&diagnostic, NODE_SENSOR_CONTEXT_COMPONENT,
                          power_off.kind, power_off.status));
  return true;
}

static bool test_zero_values_can_be_valid(void) {
  fake_node_sensors_reset();
  const node_sensors_backend_result_t success = {0};
  fake_node_sensors_set_soil(0U, 0U, success);
  fake_node_sensors_set_soil(1U, 0U, success);
  fake_node_sensors_set_ds(0U, 0, success);
  fake_node_sensors_set_ds(1U, 0, success);
  fake_node_sensors_set_bme((node_sensors_backend_enclosure_t){0}, success);

  node_sensor_sample_t sample;
  diagn_context_t diagnostic;
  TEST_ASSERT_EQ_U32(CURAG_OK, node_sensors_sample_all(&sample, &diagnostic));
  TEST_ASSERT_EQ_U32(NODE_SENSOR_VALIDITY_ALL, sample.validity);
  TEST_ASSERT_EQ_U32(0U, sample.soil_0_mv);
  TEST_ASSERT_EQ_U32(0U, sample.soil_1_mv);
  TEST_ASSERT(sample.soil_temp_0_centi_c == 0);
  TEST_ASSERT(sample.soil_temp_1_centi_c == 0);
  TEST_ASSERT(sample.enclosure_centi_c == 0);
  TEST_ASSERT_EQ_U32(0U, sample.enclosure_pressure_pa);
  TEST_ASSERT_EQ_U32(0U, sample.enclosure_humidity_centi_pct);
  return true;
}

static bool test_power_gate_on_is_glitch_safe_and_active_low(void) {
  fake_esp_gpio_reset();
  TEST_ASSERT(node_sensors_power_gate_on() == ESP_OK);
  TEST_ASSERT_EQ_U32(3U, fake_esp_gpio_call_count());

  const fake_esp_gpio_call_t *call = fake_esp_gpio_call_at(0U);
  TEST_ASSERT_EQ_U32(FAKE_ESP_GPIO_SET_LEVEL, call->operation);
  TEST_ASSERT(call->gpio_num == CONFIG_CURA_SENSOR_POWER_GATE_GPIO);
  TEST_ASSERT_EQ_U32(1U, call->level);

  call = fake_esp_gpio_call_at(1U);
  TEST_ASSERT_EQ_U32(FAKE_ESP_GPIO_CONFIG, call->operation);
  TEST_ASSERT_EQ_U32(UINT64_C(1) << CONFIG_CURA_SENSOR_POWER_GATE_GPIO,
                     call->configuration.pin_bit_mask);
  TEST_ASSERT(call->configuration.mode == GPIO_MODE_OUTPUT_OD);
  TEST_ASSERT(call->configuration.pull_up_en == GPIO_PULLUP_DISABLE);
  TEST_ASSERT(call->configuration.pull_down_en == GPIO_PULLDOWN_DISABLE);
  TEST_ASSERT(call->configuration.intr_type == GPIO_INTR_DISABLE);

  call = fake_esp_gpio_call_at(2U);
  TEST_ASSERT_EQ_U32(FAKE_ESP_GPIO_SET_LEVEL, call->operation);
  TEST_ASSERT(call->gpio_num == CONFIG_CURA_SENSOR_POWER_GATE_GPIO);
  TEST_ASSERT_EQ_U32(0U, call->level);
  return true;
}

static bool test_power_gate_off_releases_before_floating_input(void) {
  fake_esp_gpio_reset();
  TEST_ASSERT(node_sensors_power_gate_off() == ESP_OK);
  TEST_ASSERT_EQ_U32(3U, fake_esp_gpio_call_count());

  const fake_esp_gpio_call_t *call = fake_esp_gpio_call_at(0U);
  TEST_ASSERT_EQ_U32(FAKE_ESP_GPIO_SET_LEVEL, call->operation);
  TEST_ASSERT(call->gpio_num == CONFIG_CURA_SENSOR_POWER_GATE_GPIO);
  TEST_ASSERT_EQ_U32(1U, call->level);

  call = fake_esp_gpio_call_at(1U);
  TEST_ASSERT_EQ_U32(FAKE_ESP_GPIO_SET_DIRECTION, call->operation);
  TEST_ASSERT(call->gpio_num == CONFIG_CURA_SENSOR_POWER_GATE_GPIO);
  TEST_ASSERT(call->mode == GPIO_MODE_INPUT);

  call = fake_esp_gpio_call_at(2U);
  TEST_ASSERT_EQ_U32(FAKE_ESP_GPIO_SET_PULL_MODE, call->operation);
  TEST_ASSERT(call->gpio_num == CONFIG_CURA_SENSOR_POWER_GATE_GPIO);
  TEST_ASSERT(call->pull_mode == GPIO_FLOATING);
  return true;
}

static bool test_power_gate_off_is_repeatable(void) {
  fake_esp_gpio_reset();
  TEST_ASSERT(node_sensors_power_gate_off() == ESP_OK);
  TEST_ASSERT(node_sensors_power_gate_off() == ESP_OK);
  TEST_ASSERT_EQ_U32(6U, fake_esp_gpio_call_count());
  for (size_t base = 0U; base < 6U; base += 3U) {
    TEST_ASSERT_EQ_U32(FAKE_ESP_GPIO_SET_LEVEL,
                       fake_esp_gpio_call_at(base)->operation);
    TEST_ASSERT_EQ_U32(1U, fake_esp_gpio_call_at(base)->level);
    TEST_ASSERT_EQ_U32(FAKE_ESP_GPIO_SET_DIRECTION,
                       fake_esp_gpio_call_at(base + 1U)->operation);
    TEST_ASSERT_EQ_U32(FAKE_ESP_GPIO_SET_PULL_MODE,
                       fake_esp_gpio_call_at(base + 2U)->operation);
  }
  return true;
}

static bool test_power_gate_on_reconfigures_after_every_off(void) {
  fake_esp_gpio_reset();
  TEST_ASSERT(node_sensors_power_gate_on() == ESP_OK);
  TEST_ASSERT(node_sensors_power_gate_off() == ESP_OK);
  TEST_ASSERT(node_sensors_power_gate_on() == ESP_OK);
  TEST_ASSERT_EQ_U32(9U, fake_esp_gpio_call_count());

  const fake_esp_gpio_call_t *call = fake_esp_gpio_call_at(6U);
  TEST_ASSERT_EQ_U32(FAKE_ESP_GPIO_SET_LEVEL, call->operation);
  TEST_ASSERT_EQ_U32(1U, call->level);
  TEST_ASSERT_EQ_U32(FAKE_ESP_GPIO_CONFIG,
                     fake_esp_gpio_call_at(7U)->operation);
  call = fake_esp_gpio_call_at(8U);
  TEST_ASSERT_EQ_U32(FAKE_ESP_GPIO_SET_LEVEL, call->operation);
  TEST_ASSERT_EQ_U32(0U, call->level);
  return true;
}

static bool test_power_gate_on_never_asserts_after_setup_failure(void) {
  for (size_t failing_call = 0U; failing_call < 3U; ++failing_call) {
    fake_esp_gpio_reset();
    const esp_err_t expected = (esp_err_t)(-100 - (int32_t)failing_call);
    fake_esp_gpio_fail_call(failing_call, expected);
    TEST_ASSERT(node_sensors_power_gate_on() == expected);
    TEST_ASSERT_EQ_U32(failing_call + 1U, fake_esp_gpio_call_count());
    if (failing_call < 2U) {
      for (size_t index = 0U; index < fake_esp_gpio_call_count(); ++index) {
        const fake_esp_gpio_call_t *call = fake_esp_gpio_call_at(index);
        TEST_ASSERT(call->operation != FAKE_ESP_GPIO_SET_LEVEL ||
                    call->level != 0U);
      }
    }
  }
  return true;
}

static bool test_power_gate_off_attempts_all_steps_and_keeps_first_error(void) {
  for (size_t failing_call = 0U; failing_call < 3U; ++failing_call) {
    fake_esp_gpio_reset();
    const esp_err_t expected = (esp_err_t)(-200 - (int32_t)failing_call);
    fake_esp_gpio_fail_call(failing_call, expected);
    TEST_ASSERT(node_sensors_power_gate_off() == expected);
    TEST_ASSERT_EQ_U32(3U, fake_esp_gpio_call_count());
  }
  return true;
}

int main(void) {
  static const test_case_t cases[] = {
      {"success sequence and values", test_success_sequence_and_values},
      {"null sample rejected", test_null_sample_is_rejected_without_hardware},
      {"each group failure isolated", test_each_group_failure_is_isolated},
      {"all groups failed", test_all_groups_failed_is_complete_sample_error},
      {"multiple failures retained",
       test_multiple_failures_remain_in_fixed_slots},
      {"shared DS failure", test_shared_ds_failure_marks_both_channels_blocked},
      {"power-on failure",
       test_power_on_failure_blocks_gated_groups_but_samples_bme},
      {"power-off precedence",
       test_power_off_failure_takes_precedence_and_preserves_sample},
      {"cleanup and sensor diagnostics",
       test_power_cleanup_and_sensor_failure_are_both_retained},
      {"force power off idempotent",
       test_force_power_off_is_untouched_noop_then_idempotent},
      {"force power off failure",
       test_force_power_off_failure_has_component_diagnostic},
      {"null diagnostic", test_null_diagnostic_is_supported_on_failure},
      {"DS cleanup preserves values",
       test_ds_cleanup_failure_preserves_successful_values},
      {"DS channel and cleanup",
       test_ds_channel_and_cleanup_failures_preserve_other_channel},
      {"DS shared read operation", test_ds_shared_read_operation_is_preserved},
      {"diagnostic operation precedence",
       test_validation_precedes_initialize_and_read},
      {"BME stage operation", test_bme_stage_operation_is_preserved},
      {"DS and power-off precedence",
       test_shared_ds_then_power_off_uses_accepted_precedence},
      {"cleanup over shared failure",
       test_cleanup_precedes_shared_acquisition_failure},
      {"power-off over cleanup", test_power_off_precedes_cleanup},
      {"power-off over power-on", test_power_off_precedes_power_on},
      {"valid zero values", test_zero_values_can_be_valid},
      {"power gate active-low on",
       test_power_gate_on_is_glitch_safe_and_active_low},
      {"power gate released off",
       test_power_gate_off_releases_before_floating_input},
      {"power gate repeatable off", test_power_gate_off_is_repeatable},
      {"power gate reconfigured",
       test_power_gate_on_reconfigures_after_every_off},
      {"power gate safe setup failure",
       test_power_gate_on_never_asserts_after_setup_failure},
      {"power gate best-effort off",
       test_power_gate_off_attempts_all_steps_and_keeps_first_error},
  };

  size_t failures = 0U;
  for (size_t index = 0U; index < sizeof(cases) / sizeof(cases[0]); ++index) {
    fake_node_sensors_reset();
    if (cases[index].function()) {
      printf("PASS node_sensors/%s\n", cases[index].name);
    } else {
      printf("FAIL node_sensors/%s\n", cases[index].name);
      ++failures;
    }
  }
  if (failures != 0U) {
    fprintf(stderr, "%zu node_sensors test(s) failed\n", failures);
    return 1;
  }
  return 0;
}
