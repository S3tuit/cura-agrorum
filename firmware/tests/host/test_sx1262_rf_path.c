#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "fake_esp_gpio.h"
#include "sx1262_radio_rf_path.h"

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "line %d: %s\n", __LINE__, #x); return 1; } } while (0)

static int check_off_calls(bool configured) {
  CHECK(fake_esp_gpio_call_count() == (configured ? 4U : 3U));
  const fake_esp_gpio_call_t *call = fake_esp_gpio_call_at(0);
  CHECK(call->operation == FAKE_ESP_GPIO_SET_LEVEL && call->gpio_num == 38 && call->level == 0);
  if (configured) {
    call = fake_esp_gpio_call_at(1);
    CHECK(call->operation == FAKE_ESP_GPIO_CONFIG);
    CHECK(call->configuration.pin_bit_mask == (UINT64_C(1) << 38));
    CHECK(call->configuration.mode == GPIO_MODE_OUTPUT);
    CHECK(call->configuration.pull_up_en == GPIO_PULLUP_DISABLE);
    CHECK(call->configuration.pull_down_en == GPIO_PULLDOWN_DISABLE);
  }
  const size_t hold_index = configured ? 2U : 1U;
  CHECK(fake_esp_gpio_call_at(hold_index)->operation == FAKE_ESP_GPIO_HOLD_EN);
  CHECK(fake_esp_gpio_call_at(hold_index)->gpio_num == 38);
  CHECK(fake_esp_gpio_call_at(hold_index + 1U)->operation == FAKE_ESP_GPIO_SLEEP_HOLD);
  return 0;
}

static int test_initial_off(size_t failure) {
  /* Failure must not skip later safety steps. Only failed configuration needs
   * another gpio_config(): failure to lower or hold must not reclaim the pin. */
  fake_esp_gpio_reset();
  if (failure < 3U) fake_esp_gpio_fail_call(failure, ESP_FAIL);
  CHECK(sx1262_radio_rf_path_off() == (failure < 3U ? ESP_FAIL : ESP_OK));
  CHECK(check_off_calls(true) == 0);

  fake_esp_gpio_reset();
  CHECK(sx1262_radio_rf_path_off() == ESP_OK);
  CHECK(check_off_calls(failure == 1U) == 0);

  fake_esp_gpio_reset();
  CHECK(sx1262_radio_rf_path_on() == ESP_OK);
  CHECK(fake_esp_gpio_call_count() == 2U);
  CHECK(fake_esp_gpio_call_at(0)->operation == FAKE_ESP_GPIO_HOLD_DIS);
  CHECK(fake_esp_gpio_call_at(0)->gpio_num == 38);
  CHECK(fake_esp_gpio_call_at(1)->operation == FAKE_ESP_GPIO_SET_LEVEL);
  CHECK(fake_esp_gpio_call_at(1)->gpio_num == 38 && fake_esp_gpio_call_at(1)->level == 1);

  fake_esp_gpio_reset();
  CHECK(sx1262_radio_rf_path_off() == ESP_OK);
  CHECK(check_off_calls(false) == 0);
  return 0;
}

static int test_on(size_t failure) {
  fake_esp_gpio_reset();
  if (failure < 2U) fake_esp_gpio_fail_call(failure, ESP_FAIL);
  CHECK(sx1262_radio_rf_path_on() == (failure < 2U ? ESP_FAIL : ESP_OK));
  CHECK(fake_esp_gpio_call_count() == (failure == 0U ? 1U : 2U));
  CHECK(fake_esp_gpio_call_at(0)->operation == FAKE_ESP_GPIO_HOLD_DIS);
  CHECK(fake_esp_gpio_call_at(0)->gpio_num == 38);
  if (failure != 0U) {
    CHECK(fake_esp_gpio_call_at(1)->operation == FAKE_ESP_GPIO_SET_LEVEL);
    CHECK(fake_esp_gpio_call_at(1)->gpio_num == 38 && fake_esp_gpio_call_at(1)->level == 1);
  }
  return 0;
}

static int test_configured_off_failures(void) {
  CHECK(sx1262_radio_rf_path_off() == ESP_OK);
  CHECK(check_off_calls(true) == 0);
  for (size_t failure = 0; failure < 2U; ++failure) {
    fake_esp_gpio_reset();
    fake_esp_gpio_fail_call(failure, ESP_FAIL);
    CHECK(sx1262_radio_rf_path_off() == ESP_FAIL);
    CHECK(check_off_calls(false) == 0);

    fake_esp_gpio_reset();
    CHECK(sx1262_radio_rf_path_off() == ESP_OK);
    CHECK(check_off_calls(false) == 0);
  }
  return 0;
}

int main(int argc, char **argv) {
  CHECK(argc == 2);
  const char *const scenario = argv[1];
  if (strcmp(scenario, "initial_off") == 0) return test_initial_off(3U);
  if (strcmp(scenario, "initial_level_failure") == 0) return test_initial_off(0U);
  if (strcmp(scenario, "initial_config_failure") == 0) return test_initial_off(1U);
  if (strcmp(scenario, "initial_hold_failure") == 0) return test_initial_off(2U);
  if (strcmp(scenario, "on") == 0) return test_on(2U);
  if (strcmp(scenario, "on_release_failure") == 0) return test_on(0U);
  if (strcmp(scenario, "on_level_failure") == 0) return test_on(1U);
  if (strcmp(scenario, "configured_off_failures") == 0) return test_configured_off_failures();
  fprintf(stderr, "unknown scenario: %s\n", scenario);
  return 1;
}
