#pragma once
#include <stdint.h>
typedef uint32_t TickType_t;
#define configTICK_RATE_HZ 100
#define pdMS_TO_TICKS(ms) ((TickType_t)((ms) / 10U))
