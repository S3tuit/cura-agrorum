#pragma once

#include "esp_err.h"

/* Release the 1-Wire pad without an internal pull feeding the switched rail. */
esp_err_t node_sensors_ds18b20_release_gpio(void);
