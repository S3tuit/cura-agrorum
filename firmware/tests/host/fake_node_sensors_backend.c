#include "fake_node_sensors_backend.h"

#include <stdlib.h>

#include "node_sensors.h"

#define TRACE_CAPACITY 32U

typedef struct {
  node_sensors_backend_result_t power_on;
  node_sensors_backend_result_t power_off;
  uint16_t soil_values[2];
  node_sensors_backend_result_t soil_results[2];
  node_sensors_ds18b20_result_t ds;
  node_sensors_backend_enclosure_t bme_value;
  node_sensors_backend_result_t bme_result;
  fake_sensor_operation_t trace[TRACE_CAPACITY];
  size_t trace_count;
  uint32_t last_delay_ms;
} fake_sensor_state_t;

static fake_sensor_state_t s_fake;

static void trace(fake_sensor_operation_t operation) {
  if (s_fake.trace_count >= TRACE_CAPACITY) {
    abort();
  }
  s_fake.trace[s_fake.trace_count++] = operation;
}

node_sensors_backend_result_t
fake_node_sensors_result(uint32_t kind, int32_t status,
                         curag_operation_t operation) {
  return (node_sensors_backend_result_t){
      .kind = kind,
      .status = status,
      .operation = operation,
  };
}

void fake_node_sensors_reset(void) {
  s_fake = (fake_sensor_state_t){
      .soil_values = {1234U, 2345U},
      .ds = {.centi_c = {2150, 1875}},
      .bme_value =
          {
              .temperature_centi_c = 2675,
              .pressure_pa = 101325U,
              .humidity_centi_pct = 5432U,
          },
  };
  node_sensors_test_reset_state();
}

void fake_node_sensors_set_power_on(node_sensors_backend_result_t result) {
  s_fake.power_on = result;
}

void fake_node_sensors_set_power_off(node_sensors_backend_result_t result) {
  s_fake.power_off = result;
}

void fake_node_sensors_set_soil(size_t channel, uint16_t value,
                                node_sensors_backend_result_t result) {
  if (channel >= 2U) {
    abort();
  }
  s_fake.soil_values[channel] = value;
  s_fake.soil_results[channel] = result;
}

void fake_node_sensors_set_ds_shared(node_sensors_backend_result_t result) {
  s_fake.ds.shared_acquisition = result;
}

void fake_node_sensors_set_ds(size_t channel, int16_t value,
                              node_sensors_backend_result_t result) {
  if (channel >= 2U) {
    abort();
  }
  s_fake.ds.centi_c[channel] = value;
  s_fake.ds.channel[channel] = result;
}

void fake_node_sensors_set_ds_cleanup(node_sensors_backend_result_t result) {
  s_fake.ds.cleanup = result;
}

void fake_node_sensors_set_bme(node_sensors_backend_enclosure_t value,
                               node_sensors_backend_result_t result) {
  s_fake.bme_value = value;
  s_fake.bme_result = result;
}

size_t fake_node_sensors_trace_count(void) { return s_fake.trace_count; }

fake_sensor_operation_t fake_node_sensors_trace_at(size_t index) {
  if (index >= s_fake.trace_count) {
    abort();
  }
  return s_fake.trace[index];
}

uint32_t fake_node_sensors_last_delay_ms(void) { return s_fake.last_delay_ms; }

size_t fake_node_sensors_operation_count(fake_sensor_operation_t operation) {
  size_t count = 0U;
  for (size_t index = 0U; index < s_fake.trace_count; ++index) {
    if (s_fake.trace[index] == operation) {
      ++count;
    }
  }
  return count;
}

node_sensors_backend_result_t node_sensors_backend_power_on(void) {
  trace(FAKE_SENSOR_OP_POWER_ON);
  return s_fake.power_on;
}

node_sensors_backend_result_t node_sensors_backend_power_off(void) {
  trace(FAKE_SENSOR_OP_POWER_OFF);
  return s_fake.power_off;
}

void node_sensors_backend_delay_ms(uint32_t duration_ms) {
  trace(FAKE_SENSOR_OP_DELAY);
  s_fake.last_delay_ms = duration_ms;
}

node_sensors_backend_result_t node_sensors_backend_read_soil(size_t channel,
                                                             uint16_t *out_mv) {
  if (channel >= 2U || out_mv == NULL) {
    abort();
  }
  trace(channel == 0U ? FAKE_SENSOR_OP_SOIL_0 : FAKE_SENSOR_OP_SOIL_1);
  *out_mv = s_fake.soil_values[channel];
  return s_fake.soil_results[channel];
}

void node_sensors_backend_sample_ds18b20(
    node_sensors_ds18b20_result_t *out_result) {
  if (out_result == NULL) {
    abort();
  }
  trace(FAKE_SENSOR_OP_DS18B20);
  *out_result = s_fake.ds;
}

node_sensors_backend_result_t node_sensors_backend_sample_bme280(
    node_sensors_backend_enclosure_t *out_enclosure) {
  if (out_enclosure == NULL) {
    abort();
  }
  trace(FAKE_SENSOR_OP_BME280);
  *out_enclosure = s_fake.bme_value;
  return s_fake.bme_result;
}
