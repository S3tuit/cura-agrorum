#include "sx1262_radio_rf_path.h"

#include <stdint.h>

#include "driver/gpio.h"
#include "sdkconfig.h"

esp_err_t sx1262_radio_rf_path_on(void) {
  const esp_err_t released = gpio_hold_dis(CONFIG_CURA_SX1262_RF_ENABLE_GPIO);
  if (released != ESP_OK) {
    return released;
  }
  return gpio_set_level(CONFIG_CURA_SX1262_RF_ENABLE_GPIO, 1U);
}

esp_err_t sx1262_radio_rf_path_off(void) {
  /* Load the inactive latch before enabling output. Retry both steps during
   * cleanup: a failed configuration must not prevent an attempt to lower RF_SW. */
  const esp_err_t level = gpio_set_level(CONFIG_CURA_SX1262_RF_ENABLE_GPIO, 0U);
  const gpio_config_t configuration = {
      .pin_bit_mask = UINT64_C(1) << CONFIG_CURA_SX1262_RF_ENABLE_GPIO,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  const esp_err_t mode = gpio_config(&configuration);
  const esp_err_t held = gpio_hold_en(CONFIG_CURA_SX1262_RF_ENABLE_GPIO);
  gpio_deep_sleep_hold_en();
  return level != ESP_OK ? level : mode != ESP_OK ? mode : held;
}
