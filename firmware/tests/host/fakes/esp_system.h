#pragma once

typedef enum {
  ESP_RST_UNKNOWN = 96,
  ESP_RST_POWERON = 97,
  ESP_RST_EXT = 98,
  ESP_RST_SW = 99,
  ESP_RST_PANIC = 100,
  ESP_RST_INT_WDT = 101,
  ESP_RST_TASK_WDT = 102,
  ESP_RST_WDT = 103,
  ESP_RST_DEEPSLEEP = 104,
  ESP_RST_BROWNOUT = 105,
  ESP_RST_SDIO = 106,
  ESP_RST_USB = 107,
  ESP_RST_JTAG = 108,
  ESP_RST_EFUSE = 109,
  ESP_RST_PWR_GLITCH = 110,
  ESP_RST_CPU_LOCKUP = 111,
} esp_reset_reason_t;

esp_reset_reason_t esp_reset_reason(void);
_Noreturn void esp_restart(void);
