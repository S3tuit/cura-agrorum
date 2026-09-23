#include "esp_err.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "erase_storage";

#define LITTLEFS_BASE_PATH "/storage"
#define LITTLEFS_PARTITION_LABEL "storage"
#define ERASE_WARNING_DELAY_MS 5000

static esp_err_t format_and_mount_littlefs(void) {
  ESP_LOGI(TAG, "formatting LittleFS partition '%s'", LITTLEFS_PARTITION_LABEL);

  esp_err_t ret = esp_littlefs_format(LITTLEFS_PARTITION_LABEL);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "LittleFS format failed: %s", esp_err_to_name(ret));
    return ret;
  }

  const esp_vfs_littlefs_conf_t conf = {
      .base_path = LITTLEFS_BASE_PATH,
      .partition_label = LITTLEFS_PARTITION_LABEL,
      .format_if_mount_failed = false,
      .dont_mount = false,
  };
  ret = esp_vfs_littlefs_register(&conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "LittleFS mount after format failed: %s",
             esp_err_to_name(ret));
    return ret;
  }

  size_t total_bytes = 0;
  size_t used_bytes = 0;
  ret = esp_littlefs_info(LITTLEFS_PARTITION_LABEL, &total_bytes, &used_bytes);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "LittleFS info after mount failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ESP_LOGI(TAG, "LittleFS formatted and mounted: total=%u used=%u",
           (unsigned)total_bytes, (unsigned)used_bytes);
  return ESP_OK;
}

static void sleep_forever(void) {
  ESP_LOGI(TAG,
           "entering deep sleep; reset or reflash to run another firmware");
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_deep_sleep_start();
}

void app_main(void) {
  ESP_LOGE(TAG, "DESTRUCTIVE MAINTENANCE FIRMWARE");
  ESP_LOGE(TAG, "LittleFS storage will be erased in 5 seconds");
  ESP_LOGW(TAG, "NVS identity-lifetime counters will be preserved");
  vTaskDelay(pdMS_TO_TICKS(ERASE_WARNING_DELAY_MS));

  esp_err_t littlefs_ret = format_and_mount_littlefs();

  if (littlefs_ret == ESP_OK) {
    ESP_LOGI(TAG, "LittleFS format completed successfully; NVS was preserved");
  } else {
    ESP_LOGE(TAG, "LittleFS format failed: %s", esp_err_to_name(littlefs_ret));
  }

  sleep_forever();
}
