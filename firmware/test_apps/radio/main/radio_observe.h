#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  unsigned calls, initializes, starts, sleeps, overflows;
} radio_observation_t;
radio_observation_t radio_observation(void);
void radio_observation_dump(void);
