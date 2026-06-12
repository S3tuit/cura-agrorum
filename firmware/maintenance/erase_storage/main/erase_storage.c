#include "esp_err.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

static const char *TAG = "erase_storage";

#define LITTLEFS_BASE_PATH "/storage"
#define LITTLEFS_PARTITION_LABEL "storage"
#define ERASE_WARNING_DELAY_MS 5000

static esp_err_t erase_and_init_nvs(void) {
  ESP_LOGI(TAG, "erasing default NVS partition");

  esp_err_t ret = nvs_flash_erase();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "NVS erase failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = nvs_flash_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "NVS init after erase failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = nvs_flash_deinit();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "NVS deinit after erase failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ESP_LOGI(TAG, "NVS erased and initialized successfully");
  return ESP_OK;
}

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
    ESP_LOGE(TAG, "LittleFS mount after format failed: %s", esp_err_to_name(ret));
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
  ESP_LOGI(TAG, "entering deep sleep; reset or reflash to run another firmware");
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_deep_sleep_start();
}

void app_main(void) {
  ESP_LOGE(TAG, "DESTRUCTIVE MAINTENANCE FIRMWARE");
  ESP_LOGE(TAG, "NVS and LittleFS storage will be erased in 5 seconds");
  vTaskDelay(pdMS_TO_TICKS(ERASE_WARNING_DELAY_MS));

  esp_err_t nvs_ret = erase_and_init_nvs();
  esp_err_t littlefs_ret = format_and_mount_littlefs();

  if (nvs_ret == ESP_OK && littlefs_ret == ESP_OK) {
    ESP_LOGI(TAG, "storage erase/format completed successfully");
  } else {
    ESP_LOGE(TAG, "storage erase/format completed with failures: nvs=%s littlefs=%s",
             esp_err_to_name(nvs_ret), esp_err_to_name(littlefs_ret));
  }

  sleep_forever();
}
