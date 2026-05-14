/*
 * SPDX-FileCopyrightText: 2023 Brian Pugh
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "bmp280.h"
#include "driver/gpio.h"
#include "ds18b20.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2cdev.h"
#include "onewire_bus.h"
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "device_config.inc"

static const char *TAG = "cura-agrorum";

#define FILE_SCHEMA_VERSION 2
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define SLEEP_DURATION_US (15ULL * 1000ULL * 1000ULL)
#define SOIL_ADC_SAMPLE_COUNT 16
#define WAKE_CAUSE_BIT(cause) (1UL << (cause))

#define HEADER_MAGIC 0x4641524D // 'FARM' in hex.

/* This is the header of a file that represents an array of readings all with
 * the same structure. */
typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint16_t schema_version;
  uint16_t header_size;
  uint16_t record_size;         // Size, in bytes, of each reading.
  uint64_t sleep_duration_us;   // Requested sleep duration.
  uint16_t soil_gpio;           // GPIO that reads soil sensor voltage.
  uint16_t soil_dry_mv;         // Soil sensor mV when reading air.
  uint16_t soil_wet_mv;         // Soil sensor mV when reading water.
  uint16_t ds18b20_gpio;        // GPIO used for the 1-Wire bus.
  uint8_t ds18b20_resolution;   // ds18b20_resolution_t value.
  uint8_t env280_i2c_addr;      // BME/BMP280 I2C address.
  uint16_t env280_sda_gpio;     // GPIO used for I2C SDA.
  uint16_t env280_scl_gpio;     // GPIO used for I2C SCL.
  uint16_t env280_i2c_freq_khz; // I2C frequency in kHz.
  uint8_t env280_i2c_port;      // i2c_port_t value.
  uint8_t reserved[1];
} file_header_t;

// bitmask of which reading the chip read and store correctly for this reading_t
#define READING_SOIL_RAW_OK (1u << 0)
#define READING_SOIL_MV_OK (1u << 1)
#define READING_DS18B20_TEMP_OK (1u << 2)
#define READING_ENV280_TEMP_OK (1u << 3)
#define READING_ENV280_PRESSURE_OK (1u << 4)
#define READING_ENV280_HUMIDITY_OK (1u << 5)
#define READING_ENV280_CHIP_ID_OK (1u << 6)

typedef struct __attribute__((packed)) {
  uint32_t bootno; // Monotonic across deep-sleep wakes, reset on cold boot.

  /* wake_cause and run_ms should be kept just for dev versions since they're
   * usefull for debugging. */
  uint32_t wake_causes; // Bitmask returned by esp_sleep_get_wakeup_causes().
  uint16_t run_ms;      // How long this wake cycle took before sleep.

  uint16_t soil_raw;           // Raw ADC reading of soil sensor.
  uint16_t soil_mv;            // ADC reading converted in mV.
  int16_t ds18b20_centi_c;     // DS18B20 temperature in 0.01 degree C.
  int16_t env280_centi_c;      // BME/BMP280 temperature in 0.01 degree C.
  uint32_t env280_pressure_pa; // BME/BMP280 pressure in Pa.
  uint16_t env280_humidity_centi_pct; // BME280 relative humidity in 0.01%.
  uint8_t env280_chip_id;             // BMP280_CHIP_ID or BME280_CHIP_ID.
  uint8_t flags;                      // Bitmask of valid fields.
  uint8_t reserved[2];
} reading_t;

_Static_assert(sizeof(file_header_t) == 36, "unexpected file header size");
_Static_assert(sizeof(reading_t) == 28, "unexpected reading size");

#define BASE_PATH "/readings"
#define OUTPUT_FILE_PATH BASE_PATH "/readings-v" STR(FILE_SCHEMA_VERSION) ".bin"

/* Tracks how many times app_main run, i.e., how many times the esp boot.
 * This plus knowing the time when the app start is our only way to get back
 * the time of each reading. */
RTC_DATA_ATTR int32_t bootno = -1;

static uint16_t clamp_u16(int value) {
  if (value < 0) {
    return 0;
  }
  if (value > UINT16_MAX) {
    return UINT16_MAX;
  }
  return (uint16_t)value;
}

static int16_t clamp_i16(int value) {
  if (value < INT16_MIN) {
    return INT16_MIN;
  }
  if (value > INT16_MAX) {
    return INT16_MAX;
  }
  return (int16_t)value;
}

static int16_t celsius_to_centi_c(float temperature_c) {
  const float scaled = temperature_c * 100.0f;
  const int rounded = (int)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
  return clamp_i16(rounded);
}

static uint32_t pascal_to_u32(float pressure_pa) {
  if (pressure_pa < 0.0f) {
    return 0;
  }
  if (pressure_pa > (float)UINT32_MAX) {
    return UINT32_MAX;
  }
  return (uint32_t)(pressure_pa + 0.5f);
}

static uint16_t relative_humidity_to_centi_pct(float humidity_pct) {
  const float scaled = humidity_pct * 100.0f;
  const int rounded = (int)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
  return clamp_u16(rounded);
}

static void log_temperature_centi_c(const char *prefix,
                                    int16_t temperature_centi_c) {
  const int temp = temperature_centi_c;
  const int abs_temp = temp < 0 ? -temp : temp;
  ESP_LOGI(TAG, "%s%s%d.%02dC", prefix, temp < 0 ? "-" : "", abs_temp / 100,
           abs_temp % 100);
}

static void log_humidity_centi_pct(const char *prefix,
                                   uint16_t humidity_centi_pct) {
  ESP_LOGI(TAG, "%s%" PRIu16 ".%02" PRIu16 "%%", prefix,
           humidity_centi_pct / 100, humidity_centi_pct % 100);
}

typedef enum {
  ADC_CALI_NONE,
  ADC_CALI_CURVE_FITTING,
  ADC_CALI_LINE_FITTING,
} adc_cali_scheme_used_t;

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

/* Modifies 'reading' by writing the sampled mV data from the soil sensor and
 * the corresponding flag. Returns ESP_OK when everything went ok, else the
 * error and does not modify 'reading'. */
static esp_err_t read_soil(reading_t *reading) {
  adc_unit_t unit;
  adc_channel_t channel;
  // Find the analog-to-digital converter for the choosed GPIO.
  esp_err_t ret = adc_oneshot_io_to_channel(SOIL_ADC_GPIO, &unit, &channel);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "GPIO %d is not an ADC-capable pin: %s", SOIL_ADC_GPIO,
             esp_err_to_name(ret));
    return ret;
  }

  // On the classic ESP32, ADC2 conflicts with Wi-Fi when Wi-Fi is enabled.
  // So we use ADC1.
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

  // ESP32's ADC natively measure 0 to ~1.1V, so we need 12 dB af attenuation
  // to read up to ~4.4V.
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
    int raw_avg = (raw_sum + (sample_count / 2)) / sample_count;
    reading->soil_raw = clamp_u16(raw_avg);
    reading->flags |= READING_SOIL_RAW_OK;

    if (cali_handle != NULL) {
      int voltage_mv = 0;
      ret = adc_cali_raw_to_voltage(cali_handle, raw_avg, &voltage_mv);
      if (ret == ESP_OK) {
        reading->soil_mv = clamp_u16(voltage_mv);
        reading->flags |= READING_SOIL_MV_OK;
      } else {
        ESP_LOGW(TAG, "ADC voltage calibration failed: %s",
                 esp_err_to_name(ret));
      }
    } else {
      ESP_LOGW(TAG, "ADC calibration unavailable; storing raw reading only");
    }
  }

  delete_adc_calibration(cali_scheme, cali_handle);
  adc_oneshot_del_unit(adc_handle);

  return sample_count > 0 ? ESP_OK : ret;
}

/* Modifies 'reading' by writing the sampled Celsius data from the ds18b20 and
 * the corresponding flag. Returns ESP_OK when everything went ok, else the
 * error and does not modify 'reading'. */
static esp_err_t read_ds18b20(reading_t *reading) {
  esp_err_t ret = gpio_pulldown_dis((gpio_num_t)DS18B20_GPIO);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "DS18B20 GPIO %d pulldown disable failed: %s", DS18B20_GPIO,
             esp_err_to_name(ret));
    return ret;
  }

  // We don't use the internal ESP32 pull-up, we use an external 4.7k resistor.
#if !DS18B20_ENABLE_INTERNAL_PULLUP
  ret = gpio_pullup_dis((gpio_num_t)DS18B20_GPIO);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "DS18B20 GPIO %d pullup disable failed: %s", DS18B20_GPIO,
             esp_err_to_name(ret));
    return ret;
  }
#endif

  onewire_bus_handle_t bus = NULL;
  onewire_bus_config_t bus_config = {
      .bus_gpio_num = DS18B20_GPIO,
      .flags =
          {
              .en_pull_up = DS18B20_ENABLE_INTERNAL_PULLUP,
          },
  };
  onewire_bus_rmt_config_t rmt_config = {
      .max_rx_bytes =
          10, // 1byte ROM command + 8byte ROM number + 1byte device command
  };

  ret = onewire_new_bus_rmt(&bus_config, &rmt_config, &bus);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "1-Wire bus init on GPIO %d failed: %s", DS18B20_GPIO,
             esp_err_to_name(ret));
    return ret;
  }

  ds18b20_device_handle_t sensor = NULL;
  ds18b20_config_t ds18b20_config = {};
  ret = ds18b20_new_device_from_bus(bus, &ds18b20_config, &sensor);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "DS18B20 device init failed: %s", esp_err_to_name(ret));
    goto cleanup;
  }

  ret = ds18b20_set_resolution(sensor, DS18B20_RESOLUTION);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "DS18B20 resolution config failed: %s", esp_err_to_name(ret));
    goto cleanup;
  }

  // This is the blocking call that waits for the conversion to finish.
  // Right now we use the highest resolutions even if it takes around 750ms,
  // worth choosing a slower resolution for the sake of power consumption
  // later on.
  ret = ds18b20_trigger_temperature_conversion(sensor);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "DS18B20 temperature conversion failed: %s",
             esp_err_to_name(ret));
    goto cleanup;
  }

  float temperature_c = 0.0f;
  ret = ds18b20_get_temperature(sensor, &temperature_c);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "DS18B20 temperature read failed: %s", esp_err_to_name(ret));
    goto cleanup;
  }

  reading->ds18b20_centi_c = celsius_to_centi_c(temperature_c);
  reading->flags |= READING_DS18B20_TEMP_OK;

cleanup:
  if (sensor != NULL) {
    esp_err_t del_ret = ds18b20_del_device(sensor);
    if (del_ret != ESP_OK) {
      ESP_LOGW(TAG, "DS18B20 device cleanup failed: %s",
               esp_err_to_name(del_ret));
    }
  }
  esp_err_t bus_del_ret = onewire_bus_del(bus);
  if (bus_del_ret != ESP_OK) {
    ESP_LOGW(TAG, "1-Wire bus cleanup failed: %s",
             esp_err_to_name(bus_del_ret));
  }
  return ret;
}

static esp_err_t read_env280(reading_t *reading) {
  esp_err_t ret = i2cdev_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(ret));
    return ret;
  }

  bmp280_t sensor;
  memset(&sensor, 0, sizeof(sensor));
  bool desc_ready = false;

  ret = bmp280_init_desc(&sensor, ENV280_I2C_ADDR, ENV280_I2C_PORT,
                         (gpio_num_t)ENV280_I2C_SDA_GPIO,
                         (gpio_num_t)ENV280_I2C_SCL_GPIO);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "BME/BMP280 descriptor init failed: %s",
             esp_err_to_name(ret));
    goto cleanup;
  }
  desc_ready = true;
  sensor.i2c_dev.cfg.master.clk_speed = ENV280_I2C_FREQ_HZ;
  sensor.i2c_dev.cfg.sda_pullup_en = ENV280_ENABLE_INTERNAL_PULLUPS;
  sensor.i2c_dev.cfg.scl_pullup_en = ENV280_ENABLE_INTERNAL_PULLUPS;

  bmp280_params_t params;
  ret = bmp280_init_default_params(&params);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "BME/BMP280 default params init failed: %s",
             esp_err_to_name(ret));
    goto cleanup;
  }
  params.mode = BMP280_MODE_FORCED;
  params.filter = BMP280_FILTER_OFF;
  params.oversampling_pressure = BMP280_STANDARD;
  params.oversampling_temperature = BMP280_STANDARD;
  params.oversampling_humidity = BMP280_STANDARD;

  ret = bmp280_init(&sensor, &params);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "BME/BMP280 init failed: %s", esp_err_to_name(ret));
    goto cleanup;
  }

  reading->env280_chip_id = sensor.id;
  reading->flags |= READING_ENV280_CHIP_ID_OK;

  // We use forced mode;
  // It is a power-saving operational state for the BMP280 sensor where it
  // performs a single measurement upon request and then automatically returns
  // to Sleep mode
  ret = bmp280_force_measurement(&sensor);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "BME/BMP280 forced measurement failed: %s",
             esp_err_to_name(ret));
    goto cleanup;
  }

  bool busy = true;
  for (int i = 0; i < 25 && busy; ++i) {
    vTaskDelay(pdMS_TO_TICKS(10));
    ret = bmp280_is_measuring(&sensor, &busy);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "BME/BMP280 measurement status failed: %s",
               esp_err_to_name(ret));
      goto cleanup;
    }
  }
  if (busy) {
    ESP_LOGE(TAG, "BME/BMP280 measurement timed out");
    ret = ESP_ERR_TIMEOUT;
    goto cleanup;
  }

  float temperature_c = 0.0f;
  float pressure_pa = 0.0f;
  float humidity_pct = 0.0f;
  float *humidity_ptr = sensor.id == BME280_CHIP_ID ? &humidity_pct : NULL;
  ret = bmp280_read_float(&sensor, &temperature_c, &pressure_pa, humidity_ptr);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "BME/BMP280 read failed: %s", esp_err_to_name(ret));
    goto cleanup;
  }

  reading->env280_centi_c = celsius_to_centi_c(temperature_c);
  reading->env280_pressure_pa = pascal_to_u32(pressure_pa);
  reading->flags |= READING_ENV280_TEMP_OK | READING_ENV280_PRESSURE_OK;
  if (humidity_ptr != NULL) {
    reading->env280_humidity_centi_pct =
        relative_humidity_to_centi_pct(humidity_pct);
    reading->flags |= READING_ENV280_HUMIDITY_OK;
  }

cleanup:
  if (desc_ready) {
    esp_err_t desc_ret = bmp280_free_desc(&sensor);
    if (desc_ret != ESP_OK) {
      ESP_LOGW(TAG, "BME/BMP280 descriptor cleanup failed: %s",
               esp_err_to_name(desc_ret));
    }
  }
  esp_err_t i2c_ret = i2cdev_done();
  if (i2c_ret != ESP_OK) {
    ESP_LOGW(TAG, "I2C cleanup failed: %s", esp_err_to_name(i2c_ret));
  }
  return ret;
}

static file_header_t expected_file_header(void) {
  return (file_header_t){
      .magic = HEADER_MAGIC,
      .schema_version = FILE_SCHEMA_VERSION,
      .header_size = sizeof(file_header_t),
      .record_size = sizeof(reading_t),
      .sleep_duration_us = SLEEP_DURATION_US,
      .soil_gpio = SOIL_ADC_GPIO,
      .soil_dry_mv = SOIL_DRY_MV,
      .soil_wet_mv = SOIL_WET_MV,
      .ds18b20_gpio = DS18B20_GPIO,
      .ds18b20_resolution = DS18B20_RESOLUTION,
      .env280_i2c_addr = ENV280_I2C_ADDR,
      .env280_sda_gpio = ENV280_I2C_SDA_GPIO,
      .env280_scl_gpio = ENV280_I2C_SCL_GPIO,
      .env280_i2c_freq_khz = ENV280_I2C_FREQ_HZ / 1000,
      .env280_i2c_port = ENV280_I2C_PORT,
      .reserved = {0},
  };
}

static esp_err_t ensure_file_header(int fd) {
  struct stat st;
  if (fstat(fd, &st) < 0) {
    ESP_LOGE(TAG, "fstat: %s", strerror(errno));
    return ESP_FAIL;
  }

  const file_header_t expected = expected_file_header();
  if (st.st_size == 0) {
    if (write(fd, &expected, sizeof(expected)) != sizeof(expected)) {
      ESP_LOGE(TAG, "write header: %s", strerror(errno));
      return ESP_FAIL;
    }
    return ESP_OK;
  }

  // TODO: maybe validating the header each time is overkill, maybe do this each
  // 10/100 boots.

  if (st.st_size < (off_t)sizeof(file_header_t)) {
    ESP_LOGE(TAG, "Reading file is smaller than its header");
    return ESP_ERR_INVALID_SIZE;
  }

  if (lseek(fd, 0, SEEK_SET) < 0) {
    ESP_LOGE(TAG, "seek header: %s", strerror(errno));
    return ESP_FAIL;
  }

  file_header_t actual;
  if (read(fd, &actual, sizeof(actual)) != sizeof(actual)) {
    ESP_LOGE(TAG, "read header: %s", strerror(errno));
    return ESP_FAIL;
  }

  if (actual.magic != expected.magic ||
      actual.schema_version != expected.schema_version ||
      actual.header_size != expected.header_size ||
      actual.record_size != expected.record_size) {
    ESP_LOGE(TAG, "Reading file header does not match this firmware schema");
    return ESP_ERR_INVALID_VERSION;
  }

  if (actual.sleep_duration_us != expected.sleep_duration_us) {
    ESP_LOGE(TAG,
             "Reading file sleep interval differs from current config: "
             "file=%" PRIu64 "us firmware=%" PRIu64 "us",
             actual.sleep_duration_us, expected.sleep_duration_us);
    return ESP_ERR_INVALID_STATE;
  }

  if (actual.soil_gpio != expected.soil_gpio ||
      actual.soil_dry_mv != expected.soil_dry_mv ||
      actual.soil_wet_mv != expected.soil_wet_mv) {
    ESP_LOGE(TAG,
             "Reading file calibration differs from current config: "
             "file_gpio=%" PRIu16 " firmware_gpio=%" PRIu16 " file_dry=%" PRIu16
             "mV firmware_dry=%" PRIu16 "mV file_wet=%" PRIu16
             "mV firmware_wet=%" PRIu16 "mV",
             actual.soil_gpio, expected.soil_gpio, actual.soil_dry_mv,
             expected.soil_dry_mv, actual.soil_wet_mv, expected.soil_wet_mv);
    return ESP_ERR_INVALID_STATE;
  }

  if (actual.ds18b20_gpio != expected.ds18b20_gpio ||
      actual.ds18b20_resolution != expected.ds18b20_resolution) {
    ESP_LOGE(TAG,
             "Reading file DS18B20 config differs from current config: "
             "file_gpio=%" PRIu16 " firmware_gpio=%" PRIu16
             " file_resolution=%u firmware_resolution=%u",
             actual.ds18b20_gpio, expected.ds18b20_gpio,
             (unsigned)actual.ds18b20_resolution,
             (unsigned)expected.ds18b20_resolution);
    return ESP_ERR_INVALID_STATE;
  }

  if (actual.env280_i2c_addr != expected.env280_i2c_addr ||
      actual.env280_sda_gpio != expected.env280_sda_gpio ||
      actual.env280_scl_gpio != expected.env280_scl_gpio ||
      actual.env280_i2c_freq_khz != expected.env280_i2c_freq_khz ||
      actual.env280_i2c_port != expected.env280_i2c_port) {
    ESP_LOGE(TAG,
             "Reading file BME/BMP280 config differs from current config: "
             "file_addr=0x%02x firmware_addr=0x%02x "
             "file_sda=%" PRIu16 " firmware_sda=%" PRIu16 " file_scl=%" PRIu16
             " firmware_scl=%" PRIu16 " file_freq=%" PRIu16
             "kHz firmware_freq=%" PRIu16 "kHz file_port=%u firmware_port=%u",
             (unsigned)actual.env280_i2c_addr,
             (unsigned)expected.env280_i2c_addr, actual.env280_sda_gpio,
             expected.env280_sda_gpio, actual.env280_scl_gpio,
             expected.env280_scl_gpio, actual.env280_i2c_freq_khz,
             expected.env280_i2c_freq_khz, (unsigned)actual.env280_i2c_port,
             (unsigned)expected.env280_i2c_port);
    return ESP_ERR_INVALID_STATE;
  }

  const off_t data_size = st.st_size - (off_t)sizeof(file_header_t);
  // TODO: add a recovery mechanism to ignore the partial record and keep
  // appending as if there was no partial record
  if ((data_size % sizeof(reading_t)) != 0) {
    ESP_LOGE(TAG, "Reading file has a partial trailing record");
    return ESP_ERR_INVALID_SIZE;
  }

  return ESP_OK;
}

static esp_err_t append_reading(const reading_t *reading) {
  esp_err_t ret;

  esp_vfs_littlefs_conf_t conf = {
      .base_path = BASE_PATH,
      .partition_label = "storage",
      .format_if_mount_failed = true,
      .dont_mount = false,
  };

  ret = esp_vfs_littlefs_register(&conf);
  if (ret != ESP_OK) {
    if (ret == ESP_FAIL) {
      ESP_LOGE(TAG, "Failed to mount filesystem");
    } else if (ret == ESP_ERR_NOT_FOUND) {
      ESP_LOGE(TAG, "Failed to find LittleFS partition");
    } else {
      ESP_LOGE(TAG, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
    }
    return ret;
  }

  int fd = open(OUTPUT_FILE_PATH, O_RDWR | O_CREAT, 0644);
  if (fd < 0) {
    ESP_LOGE(TAG, "Open for append failed: %s", strerror(errno));
    ret = ESP_FAIL;
    goto unmount;
  }

  ret = ensure_file_header(fd);
  if (ret != ESP_OK) {
    goto unmount;
  }

  if (lseek(fd, 0, SEEK_END) < 0) {
    ESP_LOGE(TAG, "seek end: %s", strerror(errno));
    ret = ESP_FAIL;
    goto unmount;
  }

  if (write(fd, reading, sizeof(*reading)) != sizeof(*reading)) {
    ESP_LOGE(TAG, "write reading: %s", strerror(errno));
    ret = ESP_FAIL;
    goto unmount;
  }
  if (fsync(fd) < 0) {
    ESP_LOGE(TAG, "fsync: %s", strerror(errno));
    ret = ESP_FAIL;
    goto unmount;
  }

  ret = ESP_OK;

unmount:
  if (fd >= 0 && close(fd) < 0) {
    ESP_LOGE(TAG, "close: %s", strerror(errno));
    if (ret == ESP_OK) {
      ret = ESP_FAIL;
    }
  }
  esp_vfs_littlefs_unregister(conf.partition_label);
  ESP_LOGI(TAG, "LittleFS unmounted");
  return ret;
}

static void enter_deep_sleep(void) {
  ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US));
  ESP_LOGI(TAG, "Sleeping for %" PRIu64 " us", SLEEP_DURATION_US);
  esp_deep_sleep_start();
}

void app_main(void) {
  const int64_t start_us = esp_timer_get_time();
  const uint32_t wake_causes = esp_sleep_get_wakeup_causes();

  if ((wake_causes & WAKE_CAUSE_BIT(ESP_SLEEP_WAKEUP_TIMER)) && bootno >= 0) {
    bootno++;
  } else {
    bootno = 0;
  }

  reading_t reading = {
      .bootno = (uint32_t)bootno,
      .wake_causes = wake_causes,
      .run_ms = 0,
      .soil_raw = 0,
      .soil_mv = 0,
      .ds18b20_centi_c = 0,
      .env280_centi_c = 0,
      .env280_pressure_pa = 0,
      .env280_humidity_centi_pct = 0,
      .env280_chip_id = 0,
      .flags = 0,
      .reserved = {0},
  };

  ESP_LOGI(TAG,
           "boot=%" PRIu32 " wake_causes=0x%08" PRIx32
           " soil_sensor=%s soil_adc_gpio=%d dry=%dmV wet=%dmV "
           "temp_sensor=%s ds18b20_gpio=%d ds18b20_pullup=%d "
           "env_sensor=%s i2c_sda=%d i2c_scl=%d i2c_addr=0x%02x",
           reading.bootno, wake_causes, SOIL_SENSOR_ID, SOIL_ADC_GPIO,
           SOIL_DRY_MV, SOIL_WET_MV, DS18B20_SENSOR_ID, DS18B20_GPIO,
           DS18B20_ENABLE_INTERNAL_PULLUP, ENV280_SENSOR_ID,
           ENV280_I2C_SDA_GPIO, ENV280_I2C_SCL_GPIO, ENV280_I2C_ADDR);

  esp_err_t soil_ret = read_soil(&reading);
  esp_err_t ds18b20_ret = read_ds18b20(&reading);
  esp_err_t env280_ret = read_env280(&reading);
  const int64_t run_us = esp_timer_get_time() - start_us;
  reading.run_ms = clamp_u16((int)((run_us + 999) / 1000));

  if (soil_ret == ESP_OK && (reading.flags & READING_SOIL_MV_OK)) {
    ESP_LOGI(TAG, "soil raw=%" PRIu16 " mv=%" PRIu16, reading.soil_raw,
             reading.soil_mv);
  } else if (soil_ret == ESP_OK) {
    ESP_LOGI(TAG, "soil raw=%" PRIu16, reading.soil_raw);
  } else {
    ESP_LOGW(TAG, "soil read failed: %s", esp_err_to_name(soil_ret));
  }

  if (ds18b20_ret == ESP_OK && (reading.flags & READING_DS18B20_TEMP_OK)) {
    log_temperature_centi_c("ds18b20 temp=", reading.ds18b20_centi_c);
  } else {
    ESP_LOGW(TAG, "DS18B20 read failed: %s", esp_err_to_name(ds18b20_ret));
  }

  if (env280_ret == ESP_OK) {
    ESP_LOGI(TAG, "env280 chip_id=0x%02" PRIx8, reading.env280_chip_id);
    if (reading.flags & READING_ENV280_TEMP_OK) {
      log_temperature_centi_c("env280 temp=", reading.env280_centi_c);
    }
    if (reading.flags & READING_ENV280_PRESSURE_OK) {
      ESP_LOGI(TAG, "env280 pressure=%" PRIu32 "Pa",
               reading.env280_pressure_pa);
    }
    if (reading.flags & READING_ENV280_HUMIDITY_OK) {
      log_humidity_centi_pct("env280 humidity=",
                             reading.env280_humidity_centi_pct);
    }
  } else {
    ESP_LOGW(TAG, "BME/BMP280 read failed: %s", esp_err_to_name(env280_ret));
  }
  ESP_LOGI(TAG, "run=%" PRIu16 "ms", reading.run_ms);

  esp_err_t append_ret = append_reading(&reading);
  if (append_ret != ESP_OK) {
    ESP_LOGW(TAG, "reading not persisted: %s", esp_err_to_name(append_ret));
  }

  // TODO: Add a debug dump path for the LittleFS binary log.
  enter_deep_sleep();
}
