#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  unsigned gate_on, gate_off, gate_low, adc_new, adc_del, bus_new, bus_del;
  unsigned iter_new, iter_del, ds_new, ds_del, i2c_new, bme_new, bme_forced, bme_init;
  unsigned conversions, reads;
  bool invalid_sequence;
  uint64_t roms[2];
  int64_t conversion_us, minimum_wait_us;
  unsigned pending_read;
  unsigned stabilization_calls, stabilization_ms;
  unsigned soil_reads[2], soil_delays[2], soil_calibrations[2];
  int soil_mv[2];
  bool power_released;
} carrier_observation_t;

void carrier_observer_begin(uint64_t rom0, uint64_t rom1);
carrier_observation_t carrier_observer_snapshot(void);
bool carrier_observer_sample_valid(unsigned expected_ds_mask);
bool carrier_observer_cleanup_valid(unsigned expected_gate_off);
