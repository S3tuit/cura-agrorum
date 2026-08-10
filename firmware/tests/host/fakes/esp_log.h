#pragma once

void fake_esp_log_error(const char *tag, const char *format, ...);

#define ESP_LOGE(tag, format, ...)                                             \
  fake_esp_log_error((tag), (format), __VA_ARGS__)
