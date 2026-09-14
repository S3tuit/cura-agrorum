#pragma once

#include "carrier_observer.h"
#include "node_sensors.h"
#include "protocol_v2_lora_schema_generated.h"

typedef struct {
  node_sensor_sample_t sample;
  diagn_context_t diagnostic;
  err_curag_t result;
  int64_t duration_us;
  carrier_observation_t acquisition;
  bool acquisition_valid;
  uint8_t body[CURA_LORA_V2_READING_BODY_SIZE];
  uint8_t pending_body[CURA_LORA_V2_READING_BODY_SIZE];
  cura_lora_v2_reading_t reading;
} carrier_core_observation_t;

/* One real core wake; local adapters terminate delivery and observe sleep. */
void carrier_core_run(const uint64_t roms[2], unsigned ds_mask, bool bme_present,
                      carrier_core_observation_t *observation);
