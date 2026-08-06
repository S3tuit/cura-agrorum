#pragma once

#include <stddef.h>
#include <stdint.h>

#include "node_sensors_backend.h"

typedef enum {
  FAKE_SENSOR_OP_POWER_ON = 1,
  FAKE_SENSOR_OP_DELAY,
  FAKE_SENSOR_OP_SOIL_0,
  FAKE_SENSOR_OP_SOIL_1,
  FAKE_SENSOR_OP_DS18B20,
  FAKE_SENSOR_OP_BME280,
  FAKE_SENSOR_OP_POWER_OFF,
} fake_sensor_operation_t;

void fake_node_sensors_reset(void);
void fake_node_sensors_set_power_on(node_sensors_backend_result_t result);
void fake_node_sensors_set_power_off(node_sensors_backend_result_t result);
void fake_node_sensors_set_soil(size_t channel, uint16_t value,
                                node_sensors_backend_result_t result);
void fake_node_sensors_set_ds_shared(node_sensors_backend_result_t result);
void fake_node_sensors_set_ds(size_t channel, int16_t value,
                              node_sensors_backend_result_t result);
void fake_node_sensors_set_ds_cleanup(node_sensors_backend_result_t result);
void fake_node_sensors_set_bme(node_sensors_backend_enclosure_t value,
                               node_sensors_backend_result_t result);

node_sensors_backend_result_t
fake_node_sensors_result(uint32_t kind, int32_t status,
                         curag_operation_t operation);
size_t fake_node_sensors_trace_count(void);
fake_sensor_operation_t fake_node_sensors_trace_at(size_t index);
uint32_t fake_node_sensors_last_delay_ms(void);
size_t fake_node_sensors_operation_count(fake_sensor_operation_t operation);
