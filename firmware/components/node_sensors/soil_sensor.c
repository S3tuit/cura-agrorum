#include "soil_sensor.h"

#include <stdint.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "soil_sensor";

#define SOIL_ADC_SAMPLE_COUNT 16
#define SOIL_ADC_SAMPLE_INTERVAL_US 2000
#define SOIL_ADC_ATTEN ADC_ATTEN_DB_12

typedef enum {
  ADC_CALI_NONE,
  ADC_CALI_CURVE_FITTING,
  ADC_CALI_LINE_FITTING,
} adc_cali_scheme_used_t;

static uint16_t clamp_u16(int value) {
  if (value < 0) {
    return 0U;
  }
  if (value > UINT16_MAX) {
    return UINT16_MAX;
  }
  return (uint16_t)value;
}

static adc_cali_scheme_used_t get_adc_calibration(adc_unit_t unit,
                                                  adc_channel_t channel,
                                                  adc_atten_t atten,
                                                  adc_cali_handle_t *handle) {
  *handle = NULL;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  const adc_cali_curve_fitting_config_t curve_config = {
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
  const adc_cali_line_fitting_config_t line_config = {
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

static esp_err_t delete_adc_calibration(adc_cali_scheme_used_t scheme,
                                        adc_cali_handle_t handle) {
  if (handle == NULL) {
    return ESP_OK;
  }
  switch (scheme) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  case ADC_CALI_CURVE_FITTING:
    return adc_cali_delete_scheme_curve_fitting(handle);
#endif
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  case ADC_CALI_LINE_FITTING:
    return adc_cali_delete_scheme_line_fitting(handle);
#endif
  default:
    return ESP_ERR_INVALID_STATE;
  }
}

static void retain_first_error(const char *operation, esp_err_t cleanup_result,
                               esp_err_t *result) {
  if (cleanup_result == ESP_OK) {
    return;
  }
  ESP_LOGE(TAG, "%s failed: %s", operation, esp_err_to_name(cleanup_result));
  if (*result == ESP_OK) {
    *result = cleanup_result;
  }
}

esp_err_t soil_sensor_read_mv(int gpio_num, uint16_t *soil_mv) {
  if (soil_mv == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  *soil_mv = 0U;

  adc_unit_t unit;
  adc_channel_t channel;
  esp_err_t ret = adc_oneshot_io_to_channel(gpio_num, &unit, &channel);
  if (ret != ESP_OK) {
    return ret;
  }

  adc_oneshot_unit_handle_t adc_handle = NULL;
  const adc_oneshot_unit_init_cfg_t init_config = {
      .unit_id = unit,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  ret = adc_oneshot_new_unit(&init_config, &adc_handle);
  if (ret != ESP_OK) {
    return ret;
  }

  const adc_oneshot_chan_cfg_t channel_config = {
      .atten = SOIL_ADC_ATTEN,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  ret = adc_oneshot_config_channel(adc_handle, channel, &channel_config);
  if (ret != ESP_OK) {
    retain_first_error("ADC unit cleanup", adc_oneshot_del_unit(adc_handle),
                       &ret);
    return ret;
  }

  adc_cali_handle_t calibration_handle = NULL;
  const adc_cali_scheme_used_t calibration_scheme =
      get_adc_calibration(unit, channel, SOIL_ADC_ATTEN, &calibration_handle);

  int raw_sum = 0;
  for (int sample = 0; sample < SOIL_ADC_SAMPLE_COUNT; ++sample) {
    int raw = 0;
    ret = adc_oneshot_read(adc_handle, channel, &raw);
    if (ret != ESP_OK) {
      goto cleanup;
    }
    raw_sum += raw;
    if (sample + 1 < SOIL_ADC_SAMPLE_COUNT) {
      esp_rom_delay_us(SOIL_ADC_SAMPLE_INTERVAL_US);
    }
  }

  if (calibration_handle == NULL) {
    ret = ESP_ERR_NOT_SUPPORTED;
    goto cleanup;
  }

  const int raw_average =
      (raw_sum + SOIL_ADC_SAMPLE_COUNT / 2) / SOIL_ADC_SAMPLE_COUNT;
  int voltage_mv = 0;
  ret = adc_cali_raw_to_voltage(calibration_handle, raw_average, &voltage_mv);
  if (ret == ESP_OK) {
    *soil_mv = clamp_u16(voltage_mv);
  }

cleanup:
  retain_first_error(
      "ADC calibration cleanup",
      delete_adc_calibration(calibration_scheme, calibration_handle), &ret);
  retain_first_error("ADC unit cleanup", adc_oneshot_del_unit(adc_handle),
                     &ret);
  if (ret != ESP_OK) {
    *soil_mv = 0U;
  }
  return ret;
}
