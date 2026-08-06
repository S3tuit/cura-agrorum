#pragma once

#include "esp_err.h"

/*
 * Controls the active-low, open-drain P-MOSFET gate. Power-on configures the
 * GPIO on every call because power-off deliberately returns it to a floating
 * input state.
 */
esp_err_t node_sensors_power_gate_on(void);
esp_err_t node_sensors_power_gate_off(void);
