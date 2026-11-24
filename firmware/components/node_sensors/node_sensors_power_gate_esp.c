#include "node_sensors_power_gate.h"

#include <stdint.h>

#include "driver/gpio.h"
#include "sdkconfig.h"

#if CONFIG_CURA_SEN0600_0
#define GATE_OFF_LEVEL 0U
#define GATE_ON_LEVEL 1U
#define GATE_OUTPUT_MODE GPIO_MODE_OUTPUT
#else
#define GATE_OFF_LEVEL 1U
#define GATE_ON_LEVEL 0U
#define GATE_OUTPUT_MODE GPIO_MODE_OUTPUT_OD
#endif

static void retain_first_error(esp_err_t candidate, esp_err_t *result) {
  if (*result == ESP_OK && candidate != ESP_OK) {
    *result = candidate;
  }
}

esp_err_t node_sensors_power_gate_on(void) {
  const gpio_num_t gate = (gpio_num_t)CONFIG_CURA_SENSOR_POWER_GATE_GPIO;

  /* Load the inactive latch before enabling the output. The analog fixture
   * releases the P-MOSFET gate; the RS485 fixture drives its NPN base low. */
  esp_err_t result = gpio_set_level(gate, GATE_OFF_LEVEL);
  if (result != ESP_OK) {
    return result;
  }

  const gpio_config_t configuration = {
      .pin_bit_mask = UINT64_C(1) << (uint32_t)gate,
      .mode = GATE_OUTPUT_MODE,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  result = gpio_config(&configuration);
  if (result != ESP_OK) {
    return result;
  }

  /* Polarity follows the selected circuit, not a separate user setting. */
  return gpio_set_level(gate, GATE_ON_LEVEL);
}

esp_err_t node_sensors_power_gate_off(void) {
  const gpio_num_t gate = (gpio_num_t)CONFIG_CURA_SENSOR_POWER_GATE_GPIO;

  /* Release first, then leave the pad as a floating input. Attempt every
   * safety step even when an earlier GPIO operation reports a failure. */
  esp_err_t result = gpio_set_level(gate, GATE_OFF_LEVEL);
  retain_first_error(gpio_set_direction(gate, GPIO_MODE_INPUT), &result);
  retain_first_error(gpio_set_pull_mode(gate, GPIO_FLOATING), &result);
  return result;
}
