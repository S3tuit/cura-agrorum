#include "node_sensors.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "node_sensors_backend.h"

#ifndef CONFIG_CURA_SENSOR_POWER_STABILIZATION_MS
#define CONFIG_CURA_SENSOR_POWER_STABILIZATION_MS 200
#endif

#define SENSOR_CONTEXT_PAIR_SIZE 8U

typedef struct {
  node_sensors_backend_result_t pairs[NODE_SENSOR_CONTEXT_PAIR_COUNT];
  curag_operation_t primary_operation;
} sensor_diagnostic_state_t;

static bool s_power_gate_touched;

static node_sensors_backend_result_t result_blocked(void) {
  return (node_sensors_backend_result_t){
      .kind = NODE_SENSOR_BACKEND_STATUS_INTERNAL,
      .status = NODE_SENSORS_INTERNAL_BLOCKED_BY_SHARED_FAILURE,
      .operation = CURAG_OP_NONE,
  };
}

static bool result_is_success(node_sensors_backend_result_t result) {
  return result.kind == NODE_SENSOR_BACKEND_STATUS_NONE && result.status == 0 &&
         result.operation == CURAG_OP_NONE;
}

static unsigned int operation_priority(curag_operation_t operation) {
  switch (operation) {
  case CURAG_OP_POWER_OFF:
    return 6U;
  case CURAG_OP_POWER_ON:
    return 5U;
  case CURAG_OP_CLEANUP:
    return 4U;
  case CURAG_OP_VALIDATE:
    return 3U;
  case CURAG_OP_INITIALIZE:
    return 2U;
  case CURAG_OP_READ:
    return 1U;
  default:
    return 0U;
  }
}

static void select_primary_operation(sensor_diagnostic_state_t *diagnostic,
                                     node_sensors_backend_result_t result) {
  if (result_is_success(result)) {
    return;
  }
  if (operation_priority(result.operation) >
      operation_priority(diagnostic->primary_operation)) {
    diagnostic->primary_operation = result.operation;
  }
}

static void record_component_failure(sensor_diagnostic_state_t *diagnostic,
                                     node_sensors_backend_result_t result) {
  if (result_is_success(result)) {
    return;
  }
  node_sensors_backend_result_t *const current =
      &diagnostic->pairs[NODE_SENSOR_CONTEXT_COMPONENT];
  if (result_is_success(*current) ||
      operation_priority(result.operation) >
          operation_priority(current->operation)) {
    *current = result;
  }
  select_primary_operation(diagnostic, result);
}

static void write_u32_le(uint8_t output[4], uint32_t value) {
  output[0] = (uint8_t)value;
  output[1] = (uint8_t)(value >> 8U);
  output[2] = (uint8_t)(value >> 16U);
  output[3] = (uint8_t)(value >> 24U);
}

static void populate_diagnostic(diagn_context_t *out_diag,
                                curag_operation_t operation,
                                const sensor_diagnostic_state_t *state) {
  if (out_diag == NULL) {
    return;
  }
  curag_diagnostic_context_clear(out_diag);
  out_diag->operation = operation;
  out_diag->context_schema = CURAG_SENSOR_CONTEXT_V1;
  out_diag->context_length = CURAG_SENSOR_CONTEXT_V1_LENGTH;
  for (size_t index = 0U; index < NODE_SENSOR_CONTEXT_PAIR_COUNT; ++index) {
    uint8_t *const pair = &out_diag->context[index * SENSOR_CONTEXT_PAIR_SIZE];
    write_u32_le(pair, state->pairs[index].kind);
    write_u32_le(&pair[4], (uint32_t)state->pairs[index].status);
  }
}

static err_curag_t sensor_error(uint16_t code) {
  return curag_error_make(CURAG_EDOM_SENSORS, code);
}

static void record_group_result(node_sensor_sample_t *sample,
                                sensor_diagnostic_state_t *diagnostic,
                                node_sensor_context_pair_t pair,
                                uint8_t validity_bit,
                                node_sensors_backend_result_t result) {
  diagnostic->pairs[pair] = result;
  if (result_is_success(result)) {
    sample->validity = (uint8_t)(sample->validity | validity_bit);
  } else {
    select_primary_operation(diagnostic, result);
  }
}

static size_t valid_group_count(uint8_t validity) {
  size_t count = 0U;
  for (uint8_t mask = UINT8_C(1); mask <= NODE_SENSOR_ENCLOSURE_ENV_VALID;
       mask = (uint8_t)(mask << 1U)) {
    if ((validity & mask) != 0U) {
      ++count;
    }
  }
  return count;
}

err_curag_t node_sensors_sample_all(node_sensor_sample_t *out_sample,
                                    diagn_context_t *out_diag) {
  sensor_diagnostic_state_t diagnostic = {0};
  curag_diagnostic_context_clear(out_diag);
  if (out_sample == NULL) {
    populate_diagnostic(out_diag, CURAG_OP_VALIDATE, &diagnostic);
    return sensor_error(CURAG_ESENSORS_EINVALID_ARGUMENT);
  }
  memset(out_sample, 0, sizeof(*out_sample));

  bool power_on_failed = false;
  bool power_off_failed = false;
  bool cleanup_failed = false;

  s_power_gate_touched = true;
  const node_sensors_backend_result_t power_on =
      node_sensors_backend_power_on();
  if (!result_is_success(power_on)) {
    power_on_failed = true;
    record_component_failure(&diagnostic, power_on);
    const node_sensors_backend_result_t blocked = result_blocked();
    diagnostic.pairs[NODE_SENSOR_CONTEXT_SOIL_0] = blocked;
    diagnostic.pairs[NODE_SENSOR_CONTEXT_SOIL_1] = blocked;
    diagnostic.pairs[NODE_SENSOR_CONTEXT_SOIL_TEMP_0] = blocked;
    diagnostic.pairs[NODE_SENSOR_CONTEXT_SOIL_TEMP_1] = blocked;
  } else {
    node_sensors_backend_delay_ms(CONFIG_CURA_SENSOR_POWER_STABILIZATION_MS);

    node_sensors_backend_result_t result =
        node_sensors_backend_read_soil(0U, &out_sample->soil_0_mv);
    record_group_result(out_sample, &diagnostic, NODE_SENSOR_CONTEXT_SOIL_0,
                        NODE_SENSOR_SOIL_0_VALID, result);
    if (!result_is_success(result)) {
      out_sample->soil_0_mv = 0U;
    }

    result = node_sensors_backend_read_soil(1U, &out_sample->soil_1_mv);
    record_group_result(out_sample, &diagnostic, NODE_SENSOR_CONTEXT_SOIL_1,
                        NODE_SENSOR_SOIL_1_VALID, result);
    if (!result_is_success(result)) {
      out_sample->soil_1_mv = 0U;
    }

    node_sensors_ds18b20_result_t ds = {0};
    node_sensors_backend_sample_ds18b20(&ds);
    if (!result_is_success(ds.shared_acquisition)) {
      record_component_failure(&diagnostic, ds.shared_acquisition);
      const node_sensors_backend_result_t blocked = result_blocked();
      diagnostic.pairs[NODE_SENSOR_CONTEXT_SOIL_TEMP_0] = blocked;
      diagnostic.pairs[NODE_SENSOR_CONTEXT_SOIL_TEMP_1] = blocked;
    } else {
      out_sample->soil_temp_0_centi_c = ds.centi_c[0];
      record_group_result(out_sample, &diagnostic,
                          NODE_SENSOR_CONTEXT_SOIL_TEMP_0,
                          NODE_SENSOR_SOIL_TEMP_0_VALID, ds.channel[0]);
      if (!result_is_success(ds.channel[0])) {
        out_sample->soil_temp_0_centi_c = 0;
      }

      out_sample->soil_temp_1_centi_c = ds.centi_c[1];
      record_group_result(out_sample, &diagnostic,
                          NODE_SENSOR_CONTEXT_SOIL_TEMP_1,
                          NODE_SENSOR_SOIL_TEMP_1_VALID, ds.channel[1]);
      if (!result_is_success(ds.channel[1])) {
        out_sample->soil_temp_1_centi_c = 0;
      }
    }
    if (!result_is_success(ds.cleanup)) {
      cleanup_failed = true;
      record_component_failure(&diagnostic, ds.cleanup);
    }
  }

  const node_sensors_backend_result_t power_off =
      node_sensors_backend_power_off();
  if (!result_is_success(power_off)) {
    power_off_failed = true;
    record_component_failure(&diagnostic, power_off);
  }

  node_sensors_backend_enclosure_t enclosure = {0};
  const node_sensors_backend_result_t bme_result =
      node_sensors_backend_sample_bme280(&enclosure);
  diagnostic.pairs[NODE_SENSOR_CONTEXT_ENCLOSURE_ENV] = bme_result;
  if (result_is_success(bme_result)) {
    out_sample->enclosure_centi_c = enclosure.temperature_centi_c;
    out_sample->enclosure_pressure_pa = enclosure.pressure_pa;
    out_sample->enclosure_humidity_centi_pct = enclosure.humidity_centi_pct;
    out_sample->validity =
        (uint8_t)(out_sample->validity | NODE_SENSOR_ENCLOSURE_ENV_VALID);
  } else {
    select_primary_operation(&diagnostic, bme_result);
  }

  if (power_on_failed || power_off_failed) {
    populate_diagnostic(out_diag, diagnostic.primary_operation, &diagnostic);
    return sensor_error(CURAG_ESENSORS_EPOWER_CONTROL);
  }

  if (cleanup_failed) {
    populate_diagnostic(out_diag, diagnostic.primary_operation, &diagnostic);
    return sensor_error(CURAG_ESENSORS_ECLEANUP);
  }

  const size_t valid_count = valid_group_count(out_sample->validity);
  if (valid_count == NODE_SENSOR_CONTEXT_PAIR_COUNT - 1U) {
    return CURAG_OK;
  }

  populate_diagnostic(out_diag, diagnostic.primary_operation, &diagnostic);
  return sensor_error(valid_count == 0U ? CURAG_ESENSORS_ECOMPLETE_SAMPLE
                                        : CURAG_ESENSORS_EPARTIAL_SAMPLE);
}

err_curag_t node_sensors_force_power_off(diagn_context_t *out_diag) {
  curag_diagnostic_context_clear(out_diag);
  if (!s_power_gate_touched) {
    return CURAG_OK;
  }

  const node_sensors_backend_result_t result = node_sensors_backend_power_off();
  if (result_is_success(result)) {
    return CURAG_OK;
  }

  sensor_diagnostic_state_t diagnostic = {0};
  record_component_failure(&diagnostic, result);
  populate_diagnostic(out_diag, diagnostic.primary_operation, &diagnostic);
  return sensor_error(CURAG_ESENSORS_EPOWER_CONTROL);
}

#ifdef NODE_SENSORS_TESTING
void node_sensors_test_reset_state(void) { s_power_gate_touched = false; }
#endif
