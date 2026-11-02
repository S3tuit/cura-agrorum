#include "node_sensors_backend.h"
#include "node_sensors.h"
#include "node_sensors_sen0600.h"
#include "sen0600_modbus.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define SENSOR_UART UART_NUM_1
#define SENSOR_TX GPIO_NUM_1
#define SENSOR_RX GPIO_NUM_2
#define SENSOR_DE GPIO_NUM_4
#define SENSOR_RE GPIO_NUM_43
static bool installed;

static void first_error(esp_err_t next, esp_err_t *status) {
  if (*status == ESP_OK) *status = next;
}

static node_sensors_backend_result_t result(esp_err_t status, curag_operation_t operation) {
  return (node_sensors_backend_result_t){
      .kind = status == ESP_OK ? NODE_SENSOR_BACKEND_STATUS_NONE : NODE_SENSOR_BACKEND_STATUS_ESP_ERR,
      .status = status, .operation = status == ESP_OK ? CURAG_OP_NONE : operation,
  };
}

esp_err_t node_sensors_sen0600_start(void) {
  /* Called only after rail enable: driving a UART idle-high into an unpowered
   * Click would bypass the supply switch through its logic inputs. */
  esp_err_t status = gpio_set_level(SENSOR_DE, 0);
  first_error(gpio_set_level(SENSOR_RE, 1), &status);
  const gpio_config_t control = {
      .pin_bit_mask = (1ULL << SENSOR_DE) | (1ULL << SENSOR_RE),
      .mode = GPIO_MODE_OUTPUT, .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
  };
  first_error(gpio_config(&control), &status);
  if (status != ESP_OK) return status;
  const uart_config_t config = {
      .baud_rate = 9600, .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
  };
  status = uart_param_config(SENSOR_UART, &config);
  if (status != ESP_OK) return status;
  status = uart_set_pin(SENSOR_UART, SENSOR_TX, SENSOR_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (status != ESP_OK) return status;
  status = uart_driver_install(SENSOR_UART, 256, 0, 0, NULL, 0);
  installed = status == ESP_OK;
  return status;
}

esp_err_t node_sensors_sen0600_release(void) {
  esp_err_t status = ESP_OK;
  if (installed) {
    first_error(uart_wait_tx_done(SENSOR_UART, pdMS_TO_TICKS(100)), &status);
    first_error(gpio_set_level(SENSOR_DE, 0), &status);
    const esp_err_t deleted = uart_driver_delete(SENSOR_UART);
    first_error(deleted, &status);
    if (deleted == ESP_OK) installed = false;
  }
  const gpio_config_t released = {
      .pin_bit_mask = (1ULL << SENSOR_TX) | (1ULL << SENSOR_RX) |
                      (1ULL << SENSOR_DE) | (1ULL << SENSOR_RE),
      .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
  };
  /* gpio_reset_pin briefly enables a pull-up, including when force-off runs
   * before initialization. Configure floating inputs directly instead. */
  first_error(gpio_config(&released), &status);
  const gpio_num_t pins[] = {SENSOR_TX, SENSOR_RX, SENSOR_DE, SENSOR_RE};
  for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
    first_error(gpio_set_direction(pins[i], GPIO_MODE_INPUT), &status);
    first_error(gpio_set_pull_mode(pins[i], GPIO_FLOATING), &status);
  }
  return status;
}

static node_sensors_backend_result_t read_slot(uint8_t address, sen0600_reading_t *reading) {
  uint8_t request[8], reply[32];
  sen0600_request(address, request);
  esp_err_t status = uart_flush_input(SENSOR_UART);
  first_error(gpio_set_level(SENSOR_RE, 1), &status);
  first_error(gpio_set_level(SENSOR_DE, 1), &status);
  if (status == ESP_OK && uart_write_bytes(SENSOR_UART, request, sizeof(request)) != sizeof(request))
    status = ESP_FAIL;
  if (status == ESP_OK) status = uart_wait_tx_done(SENSOR_UART, pdMS_TO_TICKS(100));
  first_error(gpio_set_level(SENSOR_DE, 0), &status);
  first_error(gpio_set_level(SENSOR_RE, 0), &status);
  if (status != ESP_OK) return result(status, CURAG_OP_READ);
  const int64_t deadline = esp_timer_get_time() + 1000000;
  sen0600_stream_t stream;
  sen0600_stream_init(&stream, address);
  while (esp_timer_get_time() < deadline) {
    const int count = uart_read_bytes(SENSOR_UART, reply, sizeof(reply), pdMS_TO_TICKS(10));
    if (count < 0) {
      status = ESP_FAIL;
      break;
    }
    if (esp_timer_get_time() > deadline) break;
    if (sen0600_stream_feed(&stream, reply, (size_t)count)) break;
  }
  const esp_err_t cleanup = gpio_set_level(SENSOR_RE, 1);
  if (cleanup != ESP_OK) return result(cleanup, CURAG_OP_CLEANUP);
  if (status != ESP_OK) return result(status, CURAG_OP_READ);
  if (!stream.complete) return result(ESP_ERR_TIMEOUT, CURAG_OP_READ);
  *reading = stream.reading;
  if (stream.status != SEN0600_OK) return (node_sensors_backend_result_t){
      .kind = NODE_SENSOR_BACKEND_STATUS_DRIVER,
      .status = (int32_t)stream.status | ((int32_t)reading->exception << 8),
      .operation = CURAG_OP_READ,
  };
  return result(ESP_OK, CURAG_OP_NONE);
}

void node_sensors_backend_sample_soil(node_sensors_soil_result_t *out) {
  memset(out, 0, sizeof(*out));
  if (!installed) {
    out->shared_acquisition = result(ESP_ERR_INVALID_STATE, CURAG_OP_INITIALIZE);
    return;
  }
  const uint8_t addresses[2] = {CONFIG_CURA_SEN0600_0, CONFIG_CURA_SEN0600_1};
  for (unsigned slot = 0; slot < 2; ++slot) {
    if (!addresses[slot]) continue;
    /* At 9600 8N1, 3.5 character times are below 4 ms. Round up to a tick. */
    node_sensors_backend_delay_ms(5);
    sen0600_reading_t reading = {0};
    out->channel[slot] = read_slot(addresses[slot], &reading);
    out->temperature[slot] = out->channel[slot];
    if (out->channel[slot].kind != NODE_SENSOR_BACKEND_STATUS_NONE) continue;
    out->soil[slot] = reading.moisture_deci_pct;
    if (reading.temperature_valid) out->centi_c[slot] = reading.temperature_centi_c;
    else out->temperature[slot] = result(ESP_ERR_INVALID_RESPONSE, CURAG_OP_VALIDATE);
  }
}
