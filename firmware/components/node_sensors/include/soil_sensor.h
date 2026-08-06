#pragma once

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Reads an already-powered soil sensor connected to an ADC-capable GPIO.
 * On success, soil_mv contains the calibrated voltage. On failure it is zero.
 * This low-level compatibility API never controls the shared sensor rail.
 */
esp_err_t soil_sensor_read_mv(int gpio_num, uint16_t *soil_mv);

#ifdef __cplusplus
}
#endif
