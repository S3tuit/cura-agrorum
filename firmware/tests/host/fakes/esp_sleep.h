#pragma once

#include <stdint.h>

#include "esp_err.h"

esp_err_t esp_sleep_enable_timer_wakeup(uint64_t time_in_us);
_Noreturn void esp_deep_sleep_start(void);
