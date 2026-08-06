#pragma once

#include <stdint.h>

#include "node_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  NODE_SENSOR_SOIL_0_VALID = 1U << 0,
  NODE_SENSOR_SOIL_1_VALID = 1U << 1,
  NODE_SENSOR_SOIL_TEMP_0_VALID = 1U << 2,
  NODE_SENSOR_SOIL_TEMP_1_VALID = 1U << 3,
  NODE_SENSOR_ENCLOSURE_ENV_VALID = 1U << 4,
} node_sensor_validity_t;

#define NODE_SENSOR_VALIDITY_ALL UINT8_C(0x1f)

typedef struct {
  uint16_t soil_0_mv;
  uint16_t soil_1_mv;
  int16_t soil_temp_0_centi_c;
  int16_t soil_temp_1_centi_c;
  int16_t enclosure_centi_c;
  uint32_t enclosure_pressure_pa;
  uint16_t enclosure_humidity_centi_pct;
  uint8_t validity;
} node_sensor_sample_t;

#define CURAG_ESENSORS_EINVALID_ARGUMENT UINT16_C(1)
#define CURAG_ESENSORS_EPARTIAL_SAMPLE UINT16_C(2)
#define CURAG_ESENSORS_ECOMPLETE_SAMPLE UINT16_C(3)
#define CURAG_ESENSORS_EPOWER_CONTROL UINT16_C(4)
#define CURAG_ESENSORS_ECLEANUP UINT16_C(5)

#define CURAG_SENSOR_CONTEXT_V1 UINT8_C(1)
#define CURAG_SENSOR_CONTEXT_V1_LENGTH UINT8_C(48)

typedef enum {
  NODE_SENSOR_CONTEXT_COMPONENT = 0,
  NODE_SENSOR_CONTEXT_SOIL_0 = 1,
  NODE_SENSOR_CONTEXT_SOIL_1 = 2,
  NODE_SENSOR_CONTEXT_SOIL_TEMP_0 = 3,
  NODE_SENSOR_CONTEXT_SOIL_TEMP_1 = 4,
  NODE_SENSOR_CONTEXT_ENCLOSURE_ENV = 5,
  NODE_SENSOR_CONTEXT_PAIR_COUNT = 6,
} node_sensor_context_pair_t;

#define NODE_SENSOR_BACKEND_STATUS_NONE UINT32_C(0)
#define NODE_SENSOR_BACKEND_STATUS_ESP_ERR UINT32_C(1)
#define NODE_SENSOR_BACKEND_STATUS_DRIVER UINT32_C(2)
#define NODE_SENSOR_BACKEND_STATUS_INTERNAL UINT32_C(3)

#define NODE_SENSORS_INTERNAL_NONE INT32_C(0)
#define NODE_SENSORS_INTERNAL_BLOCKED_BY_SHARED_FAILURE INT32_C(1)
#define NODE_SENSORS_INTERNAL_UNPROVISIONED_IDENTITY INT32_C(2)
#define NODE_SENSORS_INTERNAL_DUPLICATE_IDENTITY INT32_C(3)

/*
 * Samples every configured sensor group. Invalid groups remain zero while
 * independent successful groups are preserved. The shared switched rail is
 * always driven off before this function returns after it may have been
 * enabled.
 */
err_curag_t node_sensors_sample_all(node_sensor_sample_t *out_sample,
                                    diagn_context_t *out_diag);

/*
 * Best-effort, idempotent enforcement of the switched sensor rail's off
 * state. This is a successful no-op if the component has not touched the gate
 * during the current wake.
 */
err_curag_t node_sensors_force_power_off(diagn_context_t *out_diag);

#ifdef __cplusplus
}
#endif
