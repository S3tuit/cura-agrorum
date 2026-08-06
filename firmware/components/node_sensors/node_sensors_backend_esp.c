#include "node_sensors_backend.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "bme280.h"
#include "driver/gpio.h"
#include "ds18b20.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "node_sensors.h"
#include "node_sensors_ds18b20_identity.h"
#include "node_sensors_power_gate.h"
#include "onewire_bus.h"
#include "onewire_device.h"
#include "sdkconfig.h"
#include "soil_sensor.h"

#define BME280_I2C_PORT I2C_NUM_0
#define BME280_I2C_FREQUENCY_HZ 100000U
#define DS18B20_CHANNEL_COUNT 2U

typedef struct {
  i2c_bus_handle_t bus;
  bme280_handle_t sensor;
  bool initialized;
  esp_err_t initialization_result;
} bme280_backend_state_t;

static bme280_backend_state_t s_bme280;

static node_sensors_backend_result_t result_none(void) {
  return (node_sensors_backend_result_t){
      .kind = NODE_SENSOR_BACKEND_STATUS_NONE,
      .status = 0,
      .operation = CURAG_OP_NONE,
  };
}

static node_sensors_backend_result_t result_esp(curag_operation_t operation,
                                                esp_err_t status) {
  if (status == ESP_OK) {
    return result_none();
  }
  return (node_sensors_backend_result_t){
      .kind = NODE_SENSOR_BACKEND_STATUS_ESP_ERR,
      .status = status,
      .operation = operation,
  };
}

static node_sensors_backend_result_t result_driver(curag_operation_t operation,
                                                   esp_err_t status) {
  if (status == ESP_OK) {
    return result_none();
  }
  return (node_sensors_backend_result_t){
      .kind = NODE_SENSOR_BACKEND_STATUS_DRIVER,
      .status = status,
      .operation = operation,
  };
}

node_sensors_backend_result_t node_sensors_backend_power_on(void) {
  return result_esp(CURAG_OP_POWER_ON, node_sensors_power_gate_on());
}

node_sensors_backend_result_t node_sensors_backend_power_off(void) {
  return result_esp(CURAG_OP_POWER_OFF, node_sensors_power_gate_off());
}

void node_sensors_backend_delay_ms(uint32_t duration_ms) {
  const TickType_t ticks = pdMS_TO_TICKS(duration_ms);
  vTaskDelay(ticks == 0U ? 1U : ticks);
}

node_sensors_backend_result_t node_sensors_backend_read_soil(size_t channel,
                                                             uint16_t *out_mv) {
  if (out_mv == NULL || channel >= 2U) {
    return result_esp(CURAG_OP_VALIDATE, ESP_ERR_INVALID_ARG);
  }
  const int gpios[2] = {CONFIG_CURA_SOIL_0_GPIO, CONFIG_CURA_SOIL_1_GPIO};
  return result_esp(CURAG_OP_READ, soil_sensor_read_mv(gpios[channel], out_mv));
}

static bool convert_centi(float value, int16_t *out_value) {
  if (!isfinite(value)) {
    return false;
  }
  const float scaled = value * 100.0F;
  if (scaled < (float)INT16_MIN || scaled > (float)INT16_MAX) {
    return false;
  }
  *out_value = (int16_t)(scaled >= 0.0F ? scaled + 0.5F : scaled - 0.5F);
  return true;
}

static void retain_cleanup_error(esp_err_t candidate, esp_err_t *result) {
  if (*result == ESP_OK && candidate != ESP_OK) {
    *result = candidate;
  }
}

void node_sensors_backend_sample_ds18b20(
    node_sensors_ds18b20_result_t *out_result) {
  if (out_result == NULL) {
    return;
  }
  memset(out_result, 0, sizeof(*out_result));

  uint64_t configured_roms[DS18B20_CHANNEL_COUNT] = {0U, 0U};
  const char *const configured_text[DS18B20_CHANNEL_COUNT] = {
      CONFIG_CURA_DS18B20_0_ROM,
      CONFIG_CURA_DS18B20_1_ROM,
  };
  if (!node_sensors_ds18b20_resolve_identities(configured_text, configured_roms,
                                               out_result->channel)) {
    return;
  }

  onewire_bus_handle_t bus = NULL;
  onewire_device_iter_handle_t iterator = NULL;
  ds18b20_device_handle_t devices[DS18B20_CHANNEL_COUNT] = {NULL, NULL};
  const onewire_bus_config_t bus_configuration = {
      .bus_gpio_num = CONFIG_CURA_DS18B20_GPIO,
      .flags = {.en_pull_up = false},
  };
  const onewire_bus_rmt_config_t rmt_configuration = {
      .max_rx_bytes = 10U,
  };
  esp_err_t backend_status =
      onewire_new_bus_rmt(&bus_configuration, &rmt_configuration, &bus);
  if (backend_status != ESP_OK) {
    out_result->shared_acquisition =
        result_esp(CURAG_OP_INITIALIZE, backend_status);
    esp_err_t cleanup_result = ESP_OK;
    if (bus != NULL) {
      retain_cleanup_error(onewire_bus_del(bus), &cleanup_result);
    }
    retain_cleanup_error(gpio_reset_pin((gpio_num_t)CONFIG_CURA_DS18B20_GPIO),
                         &cleanup_result);
    if (cleanup_result != ESP_OK) {
      out_result->cleanup = result_esp(CURAG_OP_CLEANUP, cleanup_result);
    }
    return;
  }

  backend_status = onewire_new_device_iter(bus, &iterator);
  if (backend_status != ESP_OK) {
    out_result->shared_acquisition =
        result_esp(CURAG_OP_INITIALIZE, backend_status);
  } else {
    while (true) {
      onewire_device_t discovered = {0};
      const esp_err_t next =
          onewire_device_iter_get_next(iterator, &discovered);
      if (next == ESP_ERR_NOT_FOUND) {
        break;
      }
      if (next != ESP_OK) {
        out_result->shared_acquisition = result_esp(CURAG_OP_INITIALIZE, next);
        break;
      }
      for (size_t channel = 0U; channel < DS18B20_CHANNEL_COUNT; ++channel) {
        if (configured_roms[channel] == 0U || devices[channel] != NULL ||
            discovered.address != configured_roms[channel]) {
          continue;
        }
        const ds18b20_config_t device_configuration = {};
        const esp_err_t create_result = ds18b20_new_device_from_enumeration(
            &discovered, &device_configuration, &devices[channel]);
        if (create_result != ESP_OK) {
          out_result->channel[channel] =
              result_driver(CURAG_OP_INITIALIZE, create_result);
        }
      }
    }
  }

  if (out_result->shared_acquisition.kind == NODE_SENSOR_BACKEND_STATUS_NONE) {
    for (size_t channel = 0U; channel < DS18B20_CHANNEL_COUNT; ++channel) {
      if (configured_roms[channel] != 0U && devices[channel] == NULL &&
          out_result->channel[channel].kind ==
              NODE_SENSOR_BACKEND_STATUS_NONE) {
        out_result->channel[channel] =
            result_driver(CURAG_OP_INITIALIZE, ESP_ERR_NOT_FOUND);
      }
      if (devices[channel] != NULL) {
        const esp_err_t resolution_result =
            ds18b20_set_resolution(devices[channel], DS18B20_RESOLUTION_12B);
        if (resolution_result != ESP_OK) {
          out_result->channel[channel] =
              result_driver(CURAG_OP_INITIALIZE, resolution_result);
        }
      }
    }

    bool has_readable_device = false;
    for (size_t channel = 0U; channel < DS18B20_CHANNEL_COUNT; ++channel) {
      has_readable_device =
          has_readable_device ||
          (devices[channel] != NULL && out_result->channel[channel].kind ==
                                           NODE_SENSOR_BACKEND_STATUS_NONE);
    }
    if (has_readable_device) {
      backend_status = ds18b20_trigger_temperature_conversion_for_all(bus);
      if (backend_status != ESP_OK) {
        out_result->shared_acquisition =
            result_driver(CURAG_OP_READ, backend_status);
      }
    }
  }

  if (out_result->shared_acquisition.kind == NODE_SENSOR_BACKEND_STATUS_NONE) {
    for (size_t channel = 0U; channel < DS18B20_CHANNEL_COUNT; ++channel) {
      if (devices[channel] == NULL || out_result->channel[channel].kind !=
                                          NODE_SENSOR_BACKEND_STATUS_NONE) {
        continue;
      }
      float temperature = 0.0F;
      esp_err_t result =
          ds18b20_get_temperature(devices[channel], &temperature);
      if (result == ESP_OK &&
          !convert_centi(temperature, &out_result->centi_c[channel])) {
        result = ESP_ERR_INVALID_RESPONSE;
      }
      out_result->channel[channel] = result_driver(CURAG_OP_READ, result);
    }
  }

  esp_err_t cleanup_result = ESP_OK;
  for (size_t channel = 0U; channel < DS18B20_CHANNEL_COUNT; ++channel) {
    if (devices[channel] != NULL) {
      retain_cleanup_error(ds18b20_del_device(devices[channel]),
                           &cleanup_result);
    }
  }
  if (iterator != NULL) {
    retain_cleanup_error(onewire_del_device_iter(iterator), &cleanup_result);
  }
  retain_cleanup_error(onewire_bus_del(bus), &cleanup_result);
  retain_cleanup_error(gpio_reset_pin((gpio_num_t)CONFIG_CURA_DS18B20_GPIO),
                       &cleanup_result);

  if (cleanup_result != ESP_OK) {
    out_result->cleanup = result_esp(CURAG_OP_CLEANUP, cleanup_result);
  }
}

static esp_err_t initialize_bme280(void) {
  if (s_bme280.initialized) {
    return s_bme280.initialization_result;
  }
  s_bme280.initialized = true;

  const i2c_config_t bus_configuration = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = CONFIG_CURA_I2C_SDA_GPIO,
      .scl_io_num = CONFIG_CURA_I2C_SCL_GPIO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = BME280_I2C_FREQUENCY_HZ,
      .clk_flags = 0,
  };
  s_bme280.bus = i2c_bus_create(BME280_I2C_PORT, &bus_configuration);
  if (s_bme280.bus == NULL) {
    s_bme280.initialization_result = ESP_FAIL;
    return s_bme280.initialization_result;
  }

  s_bme280.sensor = bme280_create(s_bme280.bus, UINT8_C(0x76));
  if (s_bme280.sensor == NULL) {
    s_bme280.initialization_result = ESP_ERR_NO_MEM;
    return s_bme280.initialization_result;
  }

  s_bme280.initialization_result = bme280_default_init(s_bme280.sensor);
  if (s_bme280.initialization_result == ESP_OK) {
    s_bme280.initialization_result = bme280_set_sampling(
        s_bme280.sensor, BME280_MODE_FORCED, BME280_SAMPLING_X1,
        BME280_SAMPLING_X1, BME280_SAMPLING_X1, BME280_FILTER_OFF,
        BME280_STANDBY_MS_0_5);
  }
  return s_bme280.initialization_result;
}

node_sensors_backend_result_t node_sensors_backend_sample_bme280(
    node_sensors_backend_enclosure_t *out_enclosure) {
  if (out_enclosure == NULL) {
    return result_esp(CURAG_OP_VALIDATE, ESP_ERR_INVALID_ARG);
  }
  memset(out_enclosure, 0, sizeof(*out_enclosure));

  esp_err_t result = initialize_bme280();
  if (result != ESP_OK) {
    return result_driver(CURAG_OP_INITIALIZE, result);
  }
  result = bme280_take_forced_measurement(s_bme280.sensor);

  float temperature = 0.0F;
  float pressure_hpa = 0.0F;
  float humidity_pct = 0.0F;
  if (result == ESP_OK) {
    result = bme280_read_temperature(s_bme280.sensor, &temperature);
  }
  if (result == ESP_OK) {
    result = bme280_read_pressure(s_bme280.sensor, &pressure_hpa);
  }
  if (result == ESP_OK) {
    result = bme280_read_humidity(s_bme280.sensor, &humidity_pct);
  }

  const float pressure_pa = pressure_hpa * 100.0F;
  const float humidity_centi_pct = humidity_pct * 100.0F;
  if (result == ESP_OK &&
      (!convert_centi(temperature, &out_enclosure->temperature_centi_c) ||
       !isfinite(pressure_pa) || pressure_pa < 0.0F ||
       pressure_pa > (float)UINT32_MAX || !isfinite(humidity_centi_pct) ||
       humidity_centi_pct < 0.0F || humidity_centi_pct > (float)UINT16_MAX)) {
    result = ESP_ERR_INVALID_RESPONSE;
  }
  if (result == ESP_OK) {
    out_enclosure->pressure_pa = (uint32_t)(pressure_pa + 0.5F);
    out_enclosure->humidity_centi_pct = (uint16_t)(humidity_centi_pct + 0.5F);
  } else {
    memset(out_enclosure, 0, sizeof(*out_enclosure));
  }
  return result_driver(CURAG_OP_READ, result);
}
