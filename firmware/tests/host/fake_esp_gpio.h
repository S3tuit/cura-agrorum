#pragma once

#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"

typedef enum {
  FAKE_ESP_GPIO_SET_LEVEL = 1,
  FAKE_ESP_GPIO_CONFIG,
  FAKE_ESP_GPIO_SET_DIRECTION,
  FAKE_ESP_GPIO_SET_PULL_MODE,
} fake_esp_gpio_operation_t;

typedef struct {
  fake_esp_gpio_operation_t operation;
  gpio_num_t gpio_num;
  uint32_t level;
  gpio_config_t configuration;
  gpio_mode_t mode;
  gpio_pull_mode_t pull_mode;
} fake_esp_gpio_call_t;

void fake_esp_gpio_reset(void);
void fake_esp_gpio_fail_call(size_t index, esp_err_t result);
size_t fake_esp_gpio_call_count(void);
const fake_esp_gpio_call_t *fake_esp_gpio_call_at(size_t index);
