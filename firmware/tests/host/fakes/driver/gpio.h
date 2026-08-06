#pragma once

#include <stdint.h>

#include "esp_err.h"

typedef int32_t gpio_num_t;

typedef enum {
  GPIO_MODE_INPUT = 1,
  GPIO_MODE_OUTPUT_OD = 2,
} gpio_mode_t;

typedef enum {
  GPIO_PULLUP_DISABLE = 0,
  GPIO_PULLUP_ENABLE = 1,
} gpio_pullup_t;

typedef enum {
  GPIO_PULLDOWN_DISABLE = 0,
  GPIO_PULLDOWN_ENABLE = 1,
} gpio_pulldown_t;

typedef enum {
  GPIO_INTR_DISABLE = 0,
} gpio_int_type_t;

typedef enum {
  GPIO_FLOATING = 0,
} gpio_pull_mode_t;

typedef struct {
  uint64_t pin_bit_mask;
  gpio_mode_t mode;
  gpio_pullup_t pull_up_en;
  gpio_pulldown_t pull_down_en;
  gpio_int_type_t intr_type;
} gpio_config_t;

esp_err_t gpio_set_level(gpio_num_t gpio_num, uint32_t level);
esp_err_t gpio_config(const gpio_config_t *configuration);
esp_err_t gpio_set_direction(gpio_num_t gpio_num, gpio_mode_t mode);
esp_err_t gpio_set_pull_mode(gpio_num_t gpio_num, gpio_pull_mode_t pull);
