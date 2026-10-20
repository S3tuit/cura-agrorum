#include <stdint.h>
#include <stdio.h>
#include "fake_esp_gpio.h"
#include "sx1262_radio_rf_path.h"

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "line %d: %s\n", __LINE__, #x); return 1; } } while (0)

int main(void) {
  /* Each failed cleanup step must still attempt the remaining safety steps. */
  for (size_t failure = 0; failure < 4; ++failure) {
    fake_esp_gpio_reset();
    if (failure < 3) fake_esp_gpio_fail_call(failure, ESP_FAIL);
    CHECK(sx1262_radio_rf_path_off() == (failure < 3 ? ESP_FAIL : ESP_OK));
    CHECK(fake_esp_gpio_call_count() == 4);
    const fake_esp_gpio_call_t *call = fake_esp_gpio_call_at(0);
    CHECK(call->operation == FAKE_ESP_GPIO_SET_LEVEL && call->gpio_num == 38 && call->level == 0);
    call = fake_esp_gpio_call_at(1);
    CHECK(call->operation == FAKE_ESP_GPIO_CONFIG);
    CHECK(call->configuration.pin_bit_mask == (UINT64_C(1) << 38));
    CHECK(call->configuration.mode == GPIO_MODE_OUTPUT);
    CHECK(call->configuration.pull_up_en == GPIO_PULLUP_DISABLE);
    CHECK(call->configuration.pull_down_en == GPIO_PULLDOWN_DISABLE);
    CHECK(fake_esp_gpio_call_at(2)->operation == FAKE_ESP_GPIO_HOLD_EN);
    CHECK(fake_esp_gpio_call_at(2)->gpio_num == 38);
    CHECK(fake_esp_gpio_call_at(3)->operation == FAKE_ESP_GPIO_SLEEP_HOLD);
  }
  for (size_t failure = 0; failure < 3; ++failure) {
    fake_esp_gpio_reset();
    if (failure < 2) fake_esp_gpio_fail_call(failure, ESP_FAIL);
    CHECK(sx1262_radio_rf_path_on() == (failure < 2 ? ESP_FAIL : ESP_OK));
    CHECK(fake_esp_gpio_call_count() == (failure == 0 ? 1 : 2));
    CHECK(fake_esp_gpio_call_at(0)->operation == FAKE_ESP_GPIO_HOLD_DIS);
    CHECK(fake_esp_gpio_call_at(0)->gpio_num == 38);
    if (failure != 0) {
      CHECK(fake_esp_gpio_call_at(1)->operation == FAKE_ESP_GPIO_SET_LEVEL);
      CHECK(fake_esp_gpio_call_at(1)->gpio_num == 38 && fake_esp_gpio_call_at(1)->level == 1);
    }
  }
  return 0;
}
