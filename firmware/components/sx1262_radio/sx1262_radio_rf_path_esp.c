#include "sx1262_radio_rf_path.h"

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "sdkconfig.h"

/* gpio_config() reserves the output in IDF. Ordinary RAM keeps that reservation
 * wake-local: cleanup reuses it, while a deep-sleep reset requires setup again. */
static bool s_rf_path_configured;

esp_err_t sx1262_radio_rf_path_on(void) {
  const esp_err_t released = gpio_hold_dis(CONFIG_CURA_SX1262_RF_ENABLE_GPIO);
  if (released != ESP_OK) {
    return released;
  }
  return gpio_set_level(CONFIG_CURA_SX1262_RF_ENABLE_GPIO, 1U);
}

esp_err_t sx1262_radio_rf_path_off(void) {
  /* Load the inactive latch before enabling output. Every cleanup attempts low
   * and hold, even if setup fails; only failed configuration needs a retry. */
  const esp_err_t level = gpio_set_level(CONFIG_CURA_SX1262_RF_ENABLE_GPIO, 0U);
  esp_err_t mode = ESP_OK;
  if (!s_rf_path_configured) {
    const gpio_config_t configuration = {
        .pin_bit_mask = UINT64_C(1) << CONFIG_CURA_SX1262_RF_ENABLE_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    mode = gpio_config(&configuration);
    s_rf_path_configured = mode == ESP_OK;
  }
  const esp_err_t held = gpio_hold_en(CONFIG_CURA_SX1262_RF_ENABLE_GPIO);
  gpio_deep_sleep_hold_en();
  return level != ESP_OK ? level : mode != ESP_OK ? mode : held;
}
