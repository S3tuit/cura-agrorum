#pragma once

#include <stdint.h>

#include "esp_err.h"

esp_err_t soil_sensor_read_mv(uint16_t *soil_mv);
