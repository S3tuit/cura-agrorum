#pragma once
#include <stdbool.h>
#include "esp_err.h"

/* Fixture-only identification/absence; always release preflight resources. */
esp_err_t carrier_bme_preflight(bool missing);
