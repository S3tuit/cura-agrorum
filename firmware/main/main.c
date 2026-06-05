#include "bmp280.h"
#include "driver/gpio.h"
#include "ds18b20.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2cdev.h"
#include "onewire_bus.h"
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "ack.h"
#include "device_config.h"
#include "event_queue.h"
#include "fault_cntl.h"
#include "node_identity.h"
#include "profile.h"
#include "reading.h"
#include "storage.h"
#include "wifi.h"
#include "wire.h"

static const char *TAG = "cura-agrorum";

#define SLEEP_DURATION_US (15ULL * 1000ULL * 1000ULL)
#define SOIL_ADC_SAMPLE_COUNT 16
#define WAKE_CAUSE_BIT(cause) (1UL << (cause))

/* Tracks how many times app_main run, i.e., how many times the esp boot.
 * This plus knowing the time when the app start is our only way to get back
 * the time of each reading. */
RTC_DATA_ATTR int32_t bootno = -1;

/* One explicit wire frame builder is reused for reading send attempts. The
 * reading is sampled before WiFi, so sending it later copies one small
 * reading_t into the builder instead of keeping a second frame buffer. */
static wire_builder_t wire_builder;

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
  DEBUG_LOGI(TAG, "%s%s%d.%02dC", prefix, temp < 0 ? "-" : "", abs_temp / 100,
             abs_temp % 100);
}

static void log_humidity_centi_pct(const char *prefix,
                                   uint16_t humidity_centi_pct) {
  DEBUG_LOGI(TAG, "%s%" PRIu16 ".%02" PRIu16 "%%", prefix,
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
      ESP_LOGW(TAG, "ADC calibration unavailable; soil mV reading unavailable");
      ret = ESP_ERR_NOT_SUPPORTED;
    }
  }

  delete_adc_calibration(cali_scheme, cali_handle);
  adc_oneshot_del_unit(adc_handle);

  return (sample_count > 0 && (reading->flags & READING_SOIL_MV_OK)) ? ESP_OK
                                                                     : ret;
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

  ret = gpio_pullup_en((gpio_num_t)DS18B20_GPIO);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "DS18B20 GPIO %d pullup enable failed: %s", DS18B20_GPIO,
             esp_err_to_name(ret));
    return ret;
  }

  onewire_bus_handle_t bus = NULL;
  onewire_bus_config_t bus_config = {
      .bus_gpio_num = DS18B20_GPIO,
      .flags =
          {
              .en_pull_up = true,
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
  // TODO: Right now we use the highest resolutions even if it takes around
  // 750ms, worth choosing a slower resolution for the sake of power consumption
  // later on.
  // Also, we may split the DS18B20 trigger from its read so between them we
  // read soild and bme
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

/* Modifies 'reading' by writing the sampled temperature, pressure, and humidity
 * from the bme280 and the corresponding flags. Returns ESP_OK when everything
 * went ok, else the error and does not modify 'reading'. */
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
    ESP_LOGE(TAG, "BME280 descriptor init failed: %s", esp_err_to_name(ret));
    goto cleanup;
  }
  desc_ready = true;
  sensor.i2c_dev.cfg.master.clk_speed = ENV280_I2C_FREQ_HZ;
  sensor.i2c_dev.cfg.sda_pullup_en = true;
  sensor.i2c_dev.cfg.scl_pullup_en = true;

  bmp280_params_t params;
  ret = bmp280_init_default_params(&params);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "BME280 default params init failed: %s",
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
    ESP_LOGE(TAG, "BME280 init failed: %s", esp_err_to_name(ret));
    goto cleanup;
  }

  // Make sure we're not using BMP which has no humidity sensor.
  if (sensor.id != BME280_CHIP_ID) {
    ESP_LOGE(TAG, "Expected chip id BME280 (%d), got : %d", BME280_CHIP_ID,
             sensor.id);
    ret = ESP_ERR_NOT_SUPPORTED;
    goto cleanup;
  }

  // We use forced mode;
  // It is a power-saving operational state for the BME280 sensor where it
  // performs a single measurement upon request and then automatically returns
  // to Sleep mode
  ret = bmp280_force_measurement(&sensor);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "BME280 forced measurement failed: %s", esp_err_to_name(ret));
    goto cleanup;
  }

  bool busy = true;
  for (int i = 0; i < 25 && busy; ++i) {
    vTaskDelay(pdMS_TO_TICKS(10));
    ret = bmp280_is_measuring(&sensor, &busy);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "BME280 measurement status failed: %s",
               esp_err_to_name(ret));
      goto cleanup;
    }
  }
  if (busy) {
    ESP_LOGE(TAG, "BME280 measurement timed out");
    ret = ESP_ERR_TIMEOUT;
    goto cleanup;
  }

  float temperature_c = 0.0f;
  float pressure_pa = 0.0f;
  float humidity_pct = 0.0f;
  ret = bmp280_read_float(&sensor, &temperature_c, &pressure_pa, &humidity_pct);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "BME280 read failed: %s", esp_err_to_name(ret));
    goto cleanup;
  }

  reading->env280_centi_c = celsius_to_centi_c(temperature_c);
  reading->env280_pressure_pa = pascal_to_u32(pressure_pa);
  reading->env280_humidity_centi_pct =
      relative_humidity_to_centi_pct(humidity_pct);
  reading->flags |= READING_ENV280_TEMP_OK | READING_ENV280_PRESSURE_OK |
                    READING_ENV280_HUMIDITY_OK;

cleanup:
  if (desc_ready) {
    esp_err_t desc_ret = bmp280_free_desc(&sensor);
    if (desc_ret != ESP_OK) {
      ESP_LOGW(TAG, "BME280 descriptor cleanup failed: %s",
               esp_err_to_name(desc_ret));
    }
  }
  esp_err_t i2c_ret = i2cdev_done();
  if (i2c_ret != ESP_OK) {
    ESP_LOGW(TAG, "I2C cleanup failed: %s", esp_err_to_name(i2c_ret));
  }
  return ret;
}

static void enter_deep_sleep(void) {
  ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US));
  DEBUG_LOGI(TAG, "Sleeping for %" PRIu64 " us", SLEEP_DURATION_US);
  esp_deep_sleep_start();
}

/* Opens one gateway connection. */
static int connect_gateway(void) {
  esp_ip4_addr_t gateway_ip = {0};
  uint16_t gateway_port = 0;

  esp_err_t ret = wifi_get_gateway_endpoint(&gateway_ip, &gateway_port);
  if (ret != ESP_OK) {
    return -1;
  }

  return wire_connect(&gateway_ip, gateway_port);
}

/* Sends one frame and waits for the server to acknowledge durable persistence.
 */
static esp_err_t send_batch(int fd, wire_builder_t *builder) {
  if (fd < 0 || builder == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t ret = wire_builder_send(fd, builder);
  if (ret != ESP_OK) {
    return ret;
  }

  ack_t ack = {0};
  const wire_expected_event_t expected_ack = {
      .record_type = ACK_RECORD_TYPE,
      .schema_version = ACK_SCHEMA_VERSION,
      .payload = &ack,
      .payload_size = sizeof(ack),
  };
  ret = wire_read_single_event(fd, &expected_ack);
  if (ret != ESP_OK) {
    return ret;
  }
  if (ack.status != 0) {
    ESP_LOGW(TAG, "server rejected event batch: status=%" PRIu32, ack.status);
    return ESP_FAIL;
  }

  DEBUG_LOGI(TAG, "server persisted event batch");
  return ESP_OK;
}

static void disconnect_gateway(int fd) {
  if (fd < 0) {
    return;
  }

  esp_err_t ret = wire_disconnect(fd);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "gateway disconnect failed: %s", esp_err_to_name(ret));
  }
}

static bool should_try_wifi_sync(void) {
  return bootno >= 0 && (bootno % CONFIG_CURA_SYNC_EVERY_N_BOOTS) == 0;
}

static void buffer_current_reading(const reading_t *reading) {
  esp_err_t ret =
      event_queue_buffer_unsent_event(&wire_builder, (const uint8_t *)reading);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "reading queue buffer failed: %s", esp_err_to_name(ret));
  }
}

/* Tries to send the current reading, a pending fault, and buffered events.
 *
 * current_reading may be NULL when storage failed before a reading could be
 * created. The pending fault is appended to the initial frame and cleared only
 * after that frame receives a successful persistence ACK.
 */
static void sync_events(const reading_t *current_reading) {
  bool current_reading_pending = current_reading != NULL;

  if (current_reading == NULL) {
    wire_builder_init(&wire_builder);
  }

  bool fault_pending_in_builder = false;
  esp_err_t ret =
      fault_append_to_builder(&wire_builder, &fault_pending_in_builder);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "fault event reservation failed: %s", esp_err_to_name(ret));
    if (current_reading_pending) {
      buffer_current_reading(current_reading);
    }
    return;
  }

  const int fd = connect_gateway();
  if (fd < 0) {
    ESP_LOGW(TAG, "gateway connection failed");
    if (current_reading_pending) {
      buffer_current_reading(current_reading);
    }
    return;
  }

  while (1) {
    event_queue_bookmark_t queue_bookmark = {0};

    const esp_err_t prepare_ret =
        event_queue_prepare_send(&wire_builder, &queue_bookmark);
    if (prepare_ret != ESP_OK) {
      ESP_LOGW(TAG, "queue replay prepare failed: %s",
               esp_err_to_name(prepare_ret));
    }

    if (wire_builder.event_count == 0) {
      DEBUG_LOGI(TAG, "queue replay complete");
      break;
    }

    ret = send_batch(fd, &wire_builder);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "reading batch not sent: %s", esp_err_to_name(ret));
      if (current_reading_pending) {
        buffer_current_reading(current_reading);
      }
      break;
    }
    current_reading_pending = false;

    if (fault_pending_in_builder) {
      fault_clear();
      fault_pending_in_builder = false;
      DEBUG_LOGI(TAG, "pending fault persisted");
    }

    if (prepare_ret != ESP_OK) {
      break;
    }

    ret = event_queue_commit_sent(&queue_bookmark);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "queue commit failed after send: %s", esp_err_to_name(ret));
      break;
    }

    if (!queue_bookmark.valid) {
      break;
    }

    wire_builder_init(&wire_builder);
  }

  disconnect_gateway(fd);
}

static uint32_t update_boot_state(void) {
  const uint32_t wake_causes = esp_sleep_get_wakeup_causes();

  if ((wake_causes & WAKE_CAUSE_BIT(ESP_SLEEP_WAKEUP_TIMER)) && bootno >= 0) {
    bootno++;
  } else {
    /* A non-timer wake is a fresh runtime from the protocol point of view. */
    bootno = 0;
  }

  return wake_causes;
}

/* Modifies 'reading' in place with a persisted sample id and all sensor
 * readings. Safe to call with uninitialized 'reading'. */
static void read_all_sensors(reading_t *reading, uint32_t sample_id,
                             uint32_t wake_causes) {
  if (!reading)
    return;

  const int64_t start_us = esp_timer_get_time();

  const reading_t defaults = {
      .node_uuid = CURA_NODE_UUID_BYTES,
      .sample_id = sample_id,
      .bootno = (uint32_t)bootno,
      .wake_causes = wake_causes,
      .run_ms = 0,
      .soil_mv = 0,
      .ds18b20_centi_c = 0,
      .env280_centi_c = 0,
      .env280_pressure_pa = 0,
      .env280_humidity_centi_pct = 0,
      .flags = 0,
      .padding = {0},
  };
  *reading = defaults;

  DEBUG_LOGI(TAG,
             "sample_id=%" PRIu32 " boot=%" PRIu32 " wake_causes=0x%08" PRIx32,
             reading->sample_id, reading->bootno, reading->wake_causes);

  esp_err_t soil_ret = read_soil(reading);
  esp_err_t ds18b20_ret = read_ds18b20(reading);
  esp_err_t env280_ret = read_env280(reading);
  const int64_t run_us = esp_timer_get_time() - start_us;
  reading->run_ms = clamp_u16((int)((run_us + 999) / 1000));

  if (soil_ret == ESP_OK && (reading->flags & READING_SOIL_MV_OK)) {
    DEBUG_LOGI(TAG, "soil mv=%" PRIu16, reading->soil_mv);
  } else {
    ESP_LOGW(TAG, "soil read failed: %s", esp_err_to_name(soil_ret));
  }

  if (ds18b20_ret == ESP_OK && (reading->flags & READING_DS18B20_TEMP_OK)) {
    log_temperature_centi_c("ds18b20 temp=", reading->ds18b20_centi_c);
  } else {
    ESP_LOGW(TAG, "DS18B20 read failed: %s", esp_err_to_name(ds18b20_ret));
  }

  if (env280_ret == ESP_OK) {
    if (reading->flags & READING_ENV280_TEMP_OK) {
      log_temperature_centi_c("env280 temp=", reading->env280_centi_c);
    }
    if (reading->flags & READING_ENV280_PRESSURE_OK) {
      DEBUG_LOGI(TAG, "env280 pressure=%" PRIu32 "Pa",
                 reading->env280_pressure_pa);
    }
    if (reading->flags & READING_ENV280_HUMIDITY_OK) {
      log_humidity_centi_pct("env280 humidity=",
                             reading->env280_humidity_centi_pct);
    }
  } else {
    ESP_LOGW(TAG, "BME280 read failed: %s", esp_err_to_name(env280_ret));
  }
  DEBUG_LOGI(TAG, "run=%" PRIu16 "ms", reading->run_ms);
}

void app_main(void) {
  PROFILE_START();
  const uint32_t wake_causes = update_boot_state();
  fault_init((uint32_t)bootno);

  uint32_t sample_id = 0;
  esp_err_t ret = cura_storage_next_sample_id(&sample_id);

  // We will write the reading directly inside the builder.
  uint8_t *payload = NULL;
  if (ret == ESP_OK) {
    ret = wire_builder_reserve_event(&wire_builder, sizeof(reading_t),
                                     FILE_SCHEMA_VERSION, CURA_RECORD_TYPE,
                                     &payload);
  }

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "sample_id allocation failed; reading not created: %s",
             esp_err_to_name(ret));

    if (should_try_wifi_sync()) {
      ret = cura_wifi_connect();
      if (ret == ESP_OK) {
        sync_events(NULL);
      } else {
        ESP_LOGW(TAG, "WiFi unavailable; pending fault retained: %s",
                 esp_err_to_name(ret));
      }
    }
    enter_deep_sleep();
  }
  fault_set_sample_id(sample_id);

  reading_t reading;
  read_all_sensors(&reading, sample_id, wake_causes);

  if (!should_try_wifi_sync()) {
    DEBUG_LOGI(TAG, "boot=%" PRId32 " skipped WiFi sync; sync every %d boots",
               bootno, CONFIG_CURA_SYNC_EVERY_N_BOOTS);
    PROFILE_MARK("Full run with NO WiFi");
    buffer_current_reading(&reading);
    enter_deep_sleep();
  }

  ret = cura_wifi_connect();
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "WiFi unavailable; reading not sent: %s",
             esp_err_to_name(ret));
    buffer_current_reading(&reading);
    PROFILE_MARK("Full run with WiFi connection but NO send");
    enter_deep_sleep();
  }

  sync_events(&reading);
  PROFILE_MARK("Full run with WiFi send");
  enter_deep_sleep();
}
