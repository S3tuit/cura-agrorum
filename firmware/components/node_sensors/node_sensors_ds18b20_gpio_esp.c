#include "node_sensors_ds18b20_gpio.h"

#include <stdint.h>

#include "driver/gpio.h"
#include "sdkconfig.h"

esp_err_t node_sensors_ds18b20_release_gpio(void) {
  /* gpio_reset_pin() enables a pull-up in ESP-IDF. Explicitly disconnect both
   * input and output with both pulls disabled before removing sensor power. */
  const gpio_config_t configuration = {
      .pin_bit_mask = UINT64_C(1) << CONFIG_CURA_DS18B20_GPIO,
      .mode = GPIO_MODE_DISABLE,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  return gpio_config(&configuration);
}
