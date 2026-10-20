#pragma once

#include "esp_err.h"

esp_err_t sx1262_radio_rf_path_on(void);
/* Also establishes the inactive output mode after partial initialization. */
esp_err_t sx1262_radio_rf_path_off(void);
