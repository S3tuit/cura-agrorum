#include "fake_esp_gpio.h"

#include <stdbool.h>
#include <stdlib.h>

#define FAKE_ESP_GPIO_CALL_CAPACITY 32U

typedef struct {
  fake_esp_gpio_call_t calls[FAKE_ESP_GPIO_CALL_CAPACITY];
  size_t call_count;
  bool failure_enabled;
  size_t failing_call;
  esp_err_t failure_result;
} fake_esp_gpio_state_t;

static fake_esp_gpio_state_t s_fake_gpio;

static esp_err_t record_call(fake_esp_gpio_call_t call) {
  if (s_fake_gpio.call_count >= FAKE_ESP_GPIO_CALL_CAPACITY) {
    abort();
  }
  const size_t index = s_fake_gpio.call_count;
  s_fake_gpio.calls[s_fake_gpio.call_count++] = call;
  if (s_fake_gpio.failure_enabled && index == s_fake_gpio.failing_call) {
    return s_fake_gpio.failure_result;
  }
  return ESP_OK;
}

void fake_esp_gpio_reset(void) { s_fake_gpio = (fake_esp_gpio_state_t){0}; }

void fake_esp_gpio_fail_call(size_t index, esp_err_t result) {
  s_fake_gpio.failure_enabled = true;
  s_fake_gpio.failing_call = index;
  s_fake_gpio.failure_result = result;
}

size_t fake_esp_gpio_call_count(void) { return s_fake_gpio.call_count; }

const fake_esp_gpio_call_t *fake_esp_gpio_call_at(size_t index) {
  if (index >= s_fake_gpio.call_count) {
    abort();
  }
  return &s_fake_gpio.calls[index];
}

esp_err_t gpio_set_level(gpio_num_t gpio_num, uint32_t level) {
  return record_call((fake_esp_gpio_call_t){
      .operation = FAKE_ESP_GPIO_SET_LEVEL,
      .gpio_num = gpio_num,
      .level = level,
  });
}

esp_err_t gpio_config(const gpio_config_t *configuration) {
  if (configuration == NULL) {
    abort();
  }
  return record_call((fake_esp_gpio_call_t){
      .operation = FAKE_ESP_GPIO_CONFIG,
      .configuration = *configuration,
  });
}

esp_err_t gpio_set_direction(gpio_num_t gpio_num, gpio_mode_t mode) {
  return record_call((fake_esp_gpio_call_t){
      .operation = FAKE_ESP_GPIO_SET_DIRECTION,
      .gpio_num = gpio_num,
      .mode = mode,
  });
}

esp_err_t gpio_set_pull_mode(gpio_num_t gpio_num, gpio_pull_mode_t pull) {
  return record_call((fake_esp_gpio_call_t){
      .operation = FAKE_ESP_GPIO_SET_PULL_MODE,
      .gpio_num = gpio_num,
      .pull_mode = pull,
  });
}
