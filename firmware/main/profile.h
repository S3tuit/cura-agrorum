#pragma once

/* Basic helpers to time ESP32 firmware execution while profiling is enabled. */
#include <inttypes.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#if CONFIG_CURA_PROFILE_ENABLED
__attribute__((unused)) static int64_t profile_t0_us;

#define DEBUG_LOGI(tag, ...)                                                   \
  do {                                                                         \
    if (0)                                                                     \
      ESP_LOGI(tag, __VA_ARGS__);                                              \
  } while (0)

#define PROFILE_START()                                                        \
  do {                                                                         \
    profile_t0_us = esp_timer_get_time();                                      \
  } while (0)

#define PROFILE_MARK(label)                                                    \
  do {                                                                         \
    const int64_t profile_now_us = esp_timer_get_time();                       \
    const int64_t profile_elapsed_us = profile_now_us - profile_t0_us;         \
    profile_t0_us = profile_now_us;                                            \
    ESP_LOGE(TAG, "profile %s=%" PRId64 "us", (label), profile_elapsed_us);    \
  } while (0)
#else
#define DEBUG_LOGI(tag, ...) ESP_LOGI(tag, __VA_ARGS__)
#define PROFILE_START()                                                        \
  do {                                                                         \
  } while (0)
#define PROFILE_MARK(label)                                                    \
  do {                                                                         \
  } while (0)
#endif
