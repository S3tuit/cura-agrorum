#pragma once

#include <stddef.h>
#include <stdint.h>

#include "node_common.h"

typedef struct {
  uint32_t kind;
  int32_t status;
  curag_operation_t operation;
} node_sensors_backend_result_t;

typedef struct {
  int16_t centi_c[2];
  node_sensors_backend_result_t shared_acquisition;
  node_sensors_backend_result_t channel[2];
  node_sensors_backend_result_t cleanup;
} node_sensors_ds18b20_result_t;

typedef struct {
  int16_t temperature_centi_c;
  uint32_t pressure_pa;
  uint16_t humidity_centi_pct;
} node_sensors_backend_enclosure_t;

node_sensors_backend_result_t node_sensors_backend_power_on(void);
node_sensors_backend_result_t node_sensors_backend_power_off(void);
void node_sensors_backend_delay_ms(uint32_t duration_ms);
node_sensors_backend_result_t node_sensors_backend_read_soil(size_t channel,
                                                             uint16_t *out_mv);

/*
 * Samples both configured ROM identities after one bus-wide conversion.
 * Shared acquisition, per-channel acquisition and cleanup remain separate so
 * cleanup cannot invalidate values already read successfully.
 */
void node_sensors_backend_sample_ds18b20(
    node_sensors_ds18b20_result_t *out_result);

node_sensors_backend_result_t node_sensors_backend_sample_bme280(
    node_sensors_backend_enclosure_t *out_enclosure);

#ifdef NODE_SENSORS_TESTING
void node_sensors_test_reset_state(void);
#endif
