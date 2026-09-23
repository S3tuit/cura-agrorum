#include "node_sensors_backend.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "bme280.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "node_sensors.h"
#include "sdkconfig.h"

#ifndef BME280_DOUBLE_ENABLE
#error "node_sensors requires the reviewed Bosch double compensation path"
#endif

/* INTERFACE.md: BME280 backend contract. These are admission budgets, not
 * preemption of an in-flight ESP-IDF operation. See TESTING.md's wait ledger.
 */
#define TRANSFER_MS 20
#define INITIALIZE_US INT64_C(500000)
#define MEASURE_US INT64_C(50000)
#define RECOVER_US INT64_C(50000)
#define CONVERSION_US UINT32_C(9300)
#define POLL_US UINT32_C(2000)
#define POLL_COUNT 6U
#define CTRL_SLEEP UINT8_C(0x24) /* temperature x1, pressure x1, sleep */
#define CTRL_FORCED UINT8_C(0x25)

static struct {
  struct bme280_dev sensor;
  i2c_master_bus_handle_t bus;
  i2c_master_dev_handle_t device;
  bool initialized;
  bool identified;
  bool failed;
  int64_t deadline_us;
  esp_err_t phase_error;
  node_sensors_backend_result_t failure;
} state;

static node_sensors_backend_result_t error(uint32_t kind, int32_t status,
                                           curag_operation_t operation) {
  return (node_sensors_backend_result_t){kind, status, operation};
}

static void retain_error(esp_err_t status) {
  if (state.phase_error == ESP_OK) {
    state.phase_error = status;
  }
}

static void begin_phase(int64_t budget_us) {
  state.phase_error = ESP_OK;
  state.deadline_us = esp_timer_get_time() + budget_us;
}

static bool admit(int64_t requested_us) {
  const int64_t remaining = state.deadline_us - esp_timer_get_time();
  if (state.phase_error == ESP_OK &&
      (remaining <= 0 || requested_us > remaining)) {
    retain_error(ESP_ERR_TIMEOUT);
  }
  return state.phase_error == ESP_OK;
}

static BME280_INTF_RET_TYPE read_register(uint8_t reg, uint8_t *data,
                                          uint32_t length, void *context) {
  (void)context;
  if (!data || length == 0 || length > 26U) {
    retain_error(ESP_ERR_INVALID_ARG);
  }
  if (!admit(TRANSFER_MS * INT64_C(1000))) {
    return -1;
  }
  retain_error(i2c_master_transmit_receive(state.device, &reg, 1, data, length,
                                           TRANSFER_MS));
  if (!admit(0)) {
    return -1;
  }
  if (reg == BME280_REG_CHIP_ID && length == 1 && data[0] == BME280_CHIP_ID) {
    state.identified = true;
  }
  return BME280_INTF_RET_SUCCESS;
}

static BME280_INTF_RET_TYPE write_register(uint8_t reg, const uint8_t *data,
                                           uint32_t length, void *context) {
  (void)context;
  /* Bosch interleaves at most ten register/value pairs: one initial register
   * plus at most 19 payload bytes. No per-transfer heap allocation. */
  uint8_t bytes[20];
  if (!data || length == 0 || length >= sizeof(bytes)) {
    retain_error(ESP_ERR_INVALID_ARG);
  }
  if (!admit(TRANSFER_MS * INT64_C(1000))) {
    return -1;
  }
  bytes[0] = reg;
  memcpy(bytes + 1, data, length);
  retain_error(
      i2c_master_transmit(state.device, bytes, length + 1U, TRANSFER_MS));
  return admit(0) ? BME280_INTF_RET_SUCCESS : -1;
}

static void delay_us(uint32_t duration_us, void *context) {
  (void)context;
  const uint64_t tick_us = UINT64_C(1000000) / configTICK_RATE_HZ;
  /* One extra tick covers an arbitrary entry point within the current tick.
   * Reserve the rounded delay as well, then check actual monotonic elapsed. */
  const TickType_t ticks =
      (TickType_t)((duration_us + tick_us - 1U) / tick_us + 1U);
  const int64_t reservation = (int64_t)((uint64_t)ticks * tick_us);
  if (!admit(reservation)) {
    return;
  }
  const int64_t start = esp_timer_get_time();
  vTaskDelay(ticks);
  if (esp_timer_get_time() - start < (int64_t)duration_us) {
    retain_error(ESP_ERR_TIMEOUT);
  }
  (void)admit(0);
}

static node_sensors_backend_result_t
driver_result(int8_t status, curag_operation_t operation) {
  if (state.phase_error != ESP_OK) {
    return error(NODE_SENSOR_BACKEND_STATUS_ESP_ERR, state.phase_error,
                 operation);
  }
  if (status != BME280_OK) {
    return error(NODE_SENSOR_BACKEND_STATUS_DRIVER, status, operation);
  }
  return (node_sensors_backend_result_t){0};
}

static int8_t observe_sleep(bool require_channels, bool *asleep) {
  uint8_t registers[3] = {0};
  *asleep = false;
  const int8_t result =
      bme280_get_regs(BME280_REG_CTRL_HUM, registers, 3, &state.sensor);
  if (result == BME280_OK) {
    *asleep = (registers[1] & 0x09U) == 0 && (registers[2] & 0x03U) == 0;
    /* Validate actual x1 settings when accepting sleep. A skipped output code
     * alone does not distinguish a disabled channel from an enabled reading.
     * Preconfiguration and recovery may legitimately have different settings.
     */
    if (*asleep && require_channels &&
        ((registers[0] & 0x07U) != BME280_OVERSAMPLING_1X ||
         (registers[2] & 0xfcU) != CTRL_SLEEP)) {
      retain_error(ESP_ERR_INVALID_RESPONSE);
      return BME280_E_COMM_FAIL;
    }
  }
  return result;
}

static int8_t require_sleep(bool require_channels) {
  bool asleep;
  const int8_t result = observe_sleep(require_channels, &asleep);
  return result != BME280_OK ? result
         : asleep            ? BME280_OK
                             : BME280_E_SLEEP_MODE_FAIL;
}

static int8_t poll_sleep(bool require_channels) {
  for (unsigned attempt = 0; attempt < POLL_COUNT; ++attempt) {
    if (attempt != 0) {
      delay_us(POLL_US, NULL);
    }
    bool asleep;
    const int8_t result = observe_sleep(require_channels, &asleep);
    if (result != BME280_OK || asleep) {
      return result;
    }
  }
  return BME280_E_SLEEP_MODE_FAIL;
}

static int8_t write_control(uint8_t control) {
  uint8_t reg = BME280_REG_CTRL_MEAS;
  return bme280_set_regs(&reg, &control, 1, &state.sensor);
}

static void recover_sleep(void) {
  if (!state.identified) {
    return;
  }
  begin_phase(RECOVER_US);
  /* A mode setter could reset/reload the sensor; one direct sleep request is
   * sufficient. Never retry or claim physical sleep after failed observation.
   */
  if (write_control(CTRL_SLEEP) == BME280_OK) {
    (void)poll_sleep(false);
  }
}

static node_sensors_backend_result_t
fail(node_sensors_backend_result_t result) {
  state.failure =
      result; /* Preserve this before recovery changes phase_error. */
  state.failed = true;
  recover_sleep();
  return result;
}

static node_sensors_backend_result_t initialize(void) {
  begin_phase(INITIALIZE_US);
  state.sensor = (struct bme280_dev){.intf = BME280_I2C_INTF,
                                     .read = read_register,
                                     .write = write_register,
                                     .delay_us = delay_us};
  const i2c_master_bus_config_t bus_config = {
      .i2c_port = I2C_NUM_0,
      .sda_io_num = CONFIG_CURA_I2C_SDA_GPIO,
      .scl_io_num = CONFIG_CURA_I2C_SCL_GPIO,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags = {.enable_internal_pullup = true},
  };
  retain_error(i2c_new_master_bus(&bus_config, &state.bus));
  if (!admit(0)) {
    return driver_result(BME280_E_COMM_FAIL, CURAG_OP_INITIALIZE);
  }
  const i2c_device_config_t device_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = 0x76,
      .scl_speed_hz = 100000,
  };
  retain_error(
      i2c_master_bus_add_device(state.bus, &device_config, &state.device));
  if (!admit(0)) {
    if (!state.device) {
      /* A failed release cannot replace the original creation failure. */
      if (i2c_del_master_bus(state.bus) == ESP_OK) {
        state.bus = NULL;
      }
    }
    return driver_result(BME280_E_COMM_FAIL, CURAG_OP_INITIALIZE);
  }
  int8_t status = bme280_init(&state.sensor);
  if (status == BME280_OK) {
    status = require_sleep(false);
  }
  const struct bme280_settings settings = {
      .osr_t = BME280_OVERSAMPLING_1X,
      .osr_p = BME280_OVERSAMPLING_1X,
      .osr_h = BME280_OVERSAMPLING_1X,
      .filter = BME280_FILTER_COEFF_OFF,
      .standby_time = BME280_STANDBY_TIME_0_5_MS,
  };
  if (status == BME280_OK) {
    status = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings,
                                        &state.sensor);
  }
  if (status == BME280_OK) {
    status = require_sleep(true);
  }
  (void)admit(0);
  const node_sensors_backend_result_t result =
      driver_result(status, CURAG_OP_INITIALIZE);
  state.initialized = result.status == 0;
  return result;
}

static bool convert(double value, double scale, double minimum, double maximum,
                    double *rounded) {
  const double scaled = value * scale;
  if (!isfinite(scaled) || scaled < minimum || scaled > maximum) {
    return false;
  }
  *rounded = round(scaled);
  return *rounded >= minimum && *rounded <= maximum;
}

node_sensors_backend_result_t node_sensors_backend_sample_bme280(
    node_sensors_backend_enclosure_t *out_enclosure) {
  if (!out_enclosure) {
    return error(NODE_SENSOR_BACKEND_STATUS_ESP_ERR, ESP_ERR_INVALID_ARG,
                 CURAG_OP_VALIDATE);
  }
  memset(out_enclosure, 0, sizeof(*out_enclosure));
  if (state.failed) {
    return state.failure;
  }
  if (!state.initialized) {
    const node_sensors_backend_result_t result = initialize();
    if (result.status != 0) {
      return fail(result);
    }
  }
  begin_phase(MEASURE_US);
  int8_t status = require_sleep(true);
  if (status == BME280_OK) {
    status = write_control(CTRL_FORCED);
  }
  struct bme280_data data = {0};
  if (status == BME280_OK) {
    begin_phase(MEASURE_US);
    delay_us(CONVERSION_US, NULL);
    status = poll_sleep(true);
  }
  if (status == BME280_OK) {
    status = bme280_get_sensor_data(BME280_ALL, &data, &state.sensor);
  }
  (void)admit(0);
  node_sensors_backend_result_t result = driver_result(status, CURAG_OP_READ);
  double temperature, pressure, humidity;
  if (result.status == 0 &&
      (!convert(data.temperature, 100, INT16_MIN, INT16_MAX, &temperature) ||
       !convert(data.pressure, 1, 0, UINT32_MAX, &pressure) ||
       !convert(data.humidity, 100, 0, UINT16_MAX, &humidity))) {
    result = error(NODE_SENSOR_BACKEND_STATUS_ESP_ERR, ESP_ERR_INVALID_RESPONSE,
                   CURAG_OP_READ);
  }
  if (result.status == 0 && !admit(0)) {
    result = driver_result(BME280_E_COMM_FAIL, CURAG_OP_READ);
  }
  if (result.status != 0) {
    return fail(result);
  }
  out_enclosure->temperature_centi_c = (int16_t)temperature;
  out_enclosure->pressure_pa = (uint32_t)pressure;
  out_enclosure->humidity_centi_pct = (uint16_t)humidity;
  return result;
}
