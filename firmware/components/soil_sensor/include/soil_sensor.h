#pragma once

#include <stdint.h>

#include "esp_err.h"

/**
 * Read a soil sensor connected to an ADC-capable GPIO.
 *
 * On success, soil_mv contains the calibrated voltage. On failure, soil_mv is
 * set to zero.
 */
esp_err_t soil_sensor_read_mv(int gpio_num, uint16_t *soil_mv);
