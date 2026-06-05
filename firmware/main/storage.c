#include "storage.h"

#include <stdbool.h>

#include "esp_log.h"
#include "fault_cntl.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "storage";

#define CURA_NVS_NAMESPACE "cura"
#define CURA_NVS_SAMPLE_ID_KEY "sample_id"

static bool s_storage_initialized;

esp_err_t cura_storage_init(void) {
  if (s_storage_initialized) {
    return ESP_OK;
  }

  esp_err_t ret = nvs_flash_init();
  if (ret != ESP_OK) {
    /* This field deployment preserves a failed NVS partition for later
     * forensic inspection instead of applying ESP-IDF's erase recovery. */
    fault_record(CURA_FAULT_NVS_INIT, ret, 0);
    ESP_LOGE(TAG, "NVS init failed without erase recovery: %s",
             esp_err_to_name(ret));
    return ret;
  }

  s_storage_initialized = true;
  return ESP_OK;
}

esp_err_t cura_storage_next_sample_id(uint32_t *sample_id) {
  if (sample_id == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t ret = cura_storage_init();
  if (ret != ESP_OK) {
    return ret;
  }

  nvs_handle_t handle = 0;
  ret = nvs_open(CURA_NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(ret));
    return ret;
  }

  uint32_t next_sample_id = 0;
  ret = nvs_get_u32(handle, CURA_NVS_SAMPLE_ID_KEY, &next_sample_id);
  if (ret == ESP_ERR_NVS_NOT_FOUND) {
    next_sample_id = 0;
    ret = ESP_OK;
  }
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "NVS sample_id read failed: %s", esp_err_to_name(ret));
    nvs_close(handle);
    return ret;
  }

  // It's near impossible that we'll send so many readings to exhaust an uint32.
  if (next_sample_id == UINT32_MAX) {
    ESP_LOGE(TAG, "sample_id counter exhausted");
    nvs_close(handle);
    return ESP_ERR_INVALID_STATE;
  }

  ret = nvs_set_u32(handle, CURA_NVS_SAMPLE_ID_KEY, next_sample_id + 1);
  if (ret == ESP_OK) {
    ret = nvs_commit(handle);
  }
  nvs_close(handle);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "NVS sample_id commit failed: %s", esp_err_to_name(ret));
    return ret;
  }

  *sample_id = next_sample_id;
  return ESP_OK;
}
