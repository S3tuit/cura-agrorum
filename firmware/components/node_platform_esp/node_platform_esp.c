#include "node_platform_esp.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define NODE_PLATFORM_ESP_FATAL_RESTART_DELAY_MS UINT32_C(60000)

static const char *const TAG = "node_platform";

static uint64_t monotonic_us(void *context) {
  (void)context;
  const int64_t now = esp_timer_get_time();
  return now < 0 ? UINT64_C(0) : (uint64_t)now;
}

static uint32_t uniform_u32_inclusive(void *context, uint32_t minimum,
                                      uint32_t maximum) {
  (void)context;

  if (minimum == maximum) {
    return minimum;
  }

  const uint64_t interval_width =
      (uint64_t)maximum - (uint64_t)minimum + UINT64_C(1);
  const uint64_t source_width = UINT64_C(1) << 32U;
  const uint64_t acceptance_limit =
      source_width - (source_width % interval_width);

  uint32_t sample = 0U;
  do {
    sample = esp_random();
  } while ((uint64_t)sample >= acceptance_limit);

  const uint64_t offset = (uint64_t)sample % interval_width;
  return (uint32_t)((uint64_t)minimum + offset);
}

static uint8_t get_reset_reason(void *context) {
  (void)context;
  const int reason = (int)esp_reset_reason();
  if (reason < 0 || reason > (int)UINT8_MAX) {
    return (uint8_t)ESP_RST_UNKNOWN;
  }
  return (uint8_t)reason;
}

static void enter_deep_sleep_for(void *context, uint64_t duration_us) {
  (void)context;
  const esp_err_t status = esp_sleep_enable_timer_wakeup(duration_us);
  if (status == ESP_OK) {
    esp_deep_sleep_start();
    abort();
  }

  ESP_LOGE(TAG, "timer wakeup configuration failed: %s (%d)",
           esp_err_to_name(status), (int)status);
  vTaskDelay(pdMS_TO_TICKS(NODE_PLATFORM_ESP_FATAL_RESTART_DELAY_MS));
  esp_restart();
  abort();
}

static const node_platform_ports_t PORTS = {
    .clock =
        {
            .context = NULL,
            .monotonic_us = monotonic_us,
        },
    .randomness =
        {
            .context = NULL,
            .uniform_u32_inclusive = uniform_u32_inclusive,
        },
    .system =
        {
            .context = NULL,
            .get_reset_reason = get_reset_reason,
            .enter_deep_sleep_for = enter_deep_sleep_for,
        },
};

const node_platform_ports_t *node_platform_esp_ports(void) { return &PORTS; }
