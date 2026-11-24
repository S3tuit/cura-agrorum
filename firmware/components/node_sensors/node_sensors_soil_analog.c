#include "node_sensors_backend.h"
#include "node_sensors.h"
#include <string.h>

void node_sensors_backend_sample_soil(node_sensors_soil_result_t *out) {
  memset(out, 0, sizeof(*out));
  const uint8_t selection = node_sensors_backend_selection();
  for (unsigned slot = 0; slot < 2; ++slot) {
    if ((selection & (1U << slot)) != 0)
      out->channel[slot] = node_sensors_backend_read_soil(slot, &out->soil[slot]);
  }
  node_sensors_ds18b20_result_t ds = {0};
  node_sensors_backend_sample_ds18b20(&ds);
  for (unsigned slot = 0; slot < 2; ++slot) {
    out->centi_c[slot] = ds.centi_c[slot];
    /* DS bus failure does not erase successful ADC measurements. */
    out->temperature[slot] = ds.shared_acquisition.kind == NODE_SENSOR_BACKEND_STATUS_NONE
                                ? ds.channel[slot]
                                : (node_sensors_backend_result_t){
                                      .kind = NODE_SENSOR_BACKEND_STATUS_INTERNAL,
                                      .status = NODE_SENSORS_INTERNAL_BLOCKED_BY_SHARED_FAILURE,
                                      .operation = CURAG_OP_NONE};
  }
  out->component_error = ds.shared_acquisition;
  out->cleanup = ds.cleanup;
}
