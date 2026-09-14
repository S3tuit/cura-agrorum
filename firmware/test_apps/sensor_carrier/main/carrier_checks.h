#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "node_sensors.h"

/* App-local acceptance checks; mask bits identify configured DS0 and DS1. */
void carrier_assert_inventory(const uint64_t configured[2], const uint64_t *roms,
                              size_t count, size_t ds_count,
                              unsigned expected_ds_mask);
void carrier_assert_sample(const node_sensor_sample_t *sample,
                           const diagn_context_t *diagnostic, err_curag_t result,
                           unsigned expected_ds_mask, bool air_soil, bool bme_present);

void carrier_assert_bme_sleep(uint8_t status, uint8_t control);
