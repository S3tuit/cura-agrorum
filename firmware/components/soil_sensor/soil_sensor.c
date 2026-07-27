#include "soil_sensor.h"

#include <stdint.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "soil_sensor";

#define SOIL_ADC_SAMPLE_COUNT 16
#define SOIL_ADC_GPIO CONFIG_CURA_SOIL_ADC_GPIO

#if CONFIG_CURA_SOIL_ADC_ATTEN_DB_0
#define SOIL_ADC_ATTEN ADC_ATTEN_DB_0
#elif CONFIG_CURA_SOIL_ADC_ATTEN_DB_2_5
#define SOIL_ADC_ATTEN ADC_ATTEN_DB_2_5
#elif CONFIG_CURA_SOIL_ADC_ATTEN_DB_6
#define SOIL_ADC_ATTEN ADC_ATTEN_DB_6
#elif CONFIG_CURA_SOIL_ADC_ATTEN_DB_12
#define SOIL_ADC_ATTEN ADC_ATTEN_DB_12
#endif

typedef enum {
  ADC_CALI_NONE,
  ADC_CALI_CURVE_FITTING,
  ADC_CALI_LINE_FITTING,
} adc_cali_scheme_used_t;

static uint16_t clamp_u16(int value) {
  if (value < 0) {
    return 0;
  }
  if (value > UINT16_MAX) {
    return UINT16_MAX;
  }
  return (uint16_t)value;
}

/* Returns the calibration mode used to convert raw ADC readings to mV. */
static adc_cali_scheme_used_t get_adc_calibration(adc_unit_t unit,
                                                  adc_channel_t channel,
                                                  adc_atten_t atten,
                                                  adc_cali_handle_t *handle) {
  *handle = NULL;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  adc_cali_curve_fitting_config_t curve_config = {
      .unit_id = unit,
      .chan = channel,
      .atten = atten,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  esp_err_t ret = adc_cali_create_scheme_curve_fitting(&curve_config, handle);
  if (ret == ESP_OK) {
    return ADC_CALI_CURVE_FITTING;
  }
  if (ret != ESP_ERR_NOT_SUPPORTED && ret != ESP_ERR_INVALID_STATE) {
    ESP_LOGW(TAG, "ADC curve calibration unavailable: %s",
             esp_err_to_name(ret));
  }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  adc_cali_line_fitting_config_t line_config = {
      .unit_id = unit,
      .atten = atten,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  esp_err_t ret = adc_cali_create_scheme_line_fitting(&line_config, handle);
  if (ret == ESP_OK) {
    return ADC_CALI_LINE_FITTING;
  }
  if (ret != ESP_ERR_NOT_SUPPORTED && ret != ESP_ERR_INVALID_STATE) {
    ESP_LOGW(TAG, "ADC line calibration unavailable: %s", esp_err_to_name(ret));
  }
#endif

  return ADC_CALI_NONE;
}

static void delete_adc_calibration(adc_cali_scheme_used_t scheme,
                                   adc_cali_handle_t handle) {
  if (handle == NULL) {
    return;
  }

  switch (scheme) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  case ADC_CALI_CURVE_FITTING:
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));
    break;
#endif
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  case ADC_CALI_LINE_FITTING:
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
    break;
#endif
  default:
    break;
  }
}

esp_err_t soil_sensor_read_mv(uint16_t *soil_mv) {
  if (soil_mv == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  adc_unit_t unit;
  adc_channel_t channel;
  esp_err_t ret = adc_oneshot_io_to_channel(SOIL_ADC_GPIO, &unit, &channel);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "GPIO %d is not an ADC-capable pin: %s", SOIL_ADC_GPIO,
             esp_err_to_name(ret));
    return ret;
  }

  // On the classic ESP32, ADC2 conflicts with WiFi, so the soil input uses ADC1.
  if (unit != ADC_UNIT_1) {
    ESP_LOGE(TAG, "GPIO %d maps to ADC unit %d, expected ADC1", SOIL_ADC_GPIO,
             unit);
    return ESP_ERR_INVALID_ARG;
  }

  adc_oneshot_unit_handle_t adc_handle;
  adc_oneshot_unit_init_cfg_t init_config = {
      .unit_id = unit,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  ret = adc_oneshot_new_unit(&init_config, &adc_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ADC init failed: %s", esp_err_to_name(ret));
    return ret;
  }

  adc_oneshot_chan_cfg_t channel_config = {
      .atten = SOIL_ADC_ATTEN,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  ret = adc_oneshot_config_channel(adc_handle, channel, &channel_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ADC channel config failed: %s", esp_err_to_name(ret));
    adc_oneshot_del_unit(adc_handle);
    return ret;
  }

  adc_cali_handle_t cali_handle = NULL;
  adc_cali_scheme_used_t cali_scheme =
      get_adc_calibration(unit, channel, SOIL_ADC_ATTEN, &cali_handle);

  int raw_sum = 0;
  int sample_count = 0;
  for (int i = 0; i < SOIL_ADC_SAMPLE_COUNT; ++i) {
    int raw = 0;
    ret = adc_oneshot_read(adc_handle, channel, &raw);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "ADC read failed: %s", esp_err_to_name(ret));
      break;
    }
    raw_sum += raw;
    sample_count++;
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  if (sample_count > 0) {
    const int raw_avg = (raw_sum + (sample_count / 2)) / sample_count;
    if (cali_handle != NULL) {
      int voltage_mv = 0;
      ret = adc_cali_raw_to_voltage(cali_handle, raw_avg, &voltage_mv);
      if (ret == ESP_OK) {
        *soil_mv = clamp_u16(voltage_mv);
      } else {
        ESP_LOGW(TAG, "ADC voltage calibration failed: %s",
                 esp_err_to_name(ret));
      }
    } else {
      ESP_LOGW(TAG, "ADC calibration unavailable; soil mV reading unavailable");
      ret = ESP_ERR_NOT_SUPPORTED;
    }
  }

  delete_adc_calibration(cali_scheme, cali_handle);
  adc_oneshot_del_unit(adc_handle);

  return (sample_count > 0 && ret == ESP_OK) ? ESP_OK : ret;
}
