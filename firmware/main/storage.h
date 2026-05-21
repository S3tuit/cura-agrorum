#pragma once

#include <stdint.h>

#include "esp_err.h"

/* Initializes the firmware storage layer backed by ESP-IDF NVS.
 * Calling this more than once is safe.
 */
esp_err_t cura_storage_init(void);

/* Claims the next per-node sample id and persists the incremented counter.
 *
 * sample_id starts at 0 on a fresh node and increases by one for every
 * reading_t the firmware creates. The increment is committed before the value
 * is returned, so a reset may skip an id but should not reuse one.
 */
esp_err_t cura_storage_next_sample_id(uint32_t *sample_id);
