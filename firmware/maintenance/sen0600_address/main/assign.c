#include "commission.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>

enum { GATE = 6, TX = 1, RX = 2, DE = 4, RE_N = 43 };
static const uart_port_t port = UART_NUM_1;
static bool installed;
static const char *tag = "sen0600_address";

static void first_error(esp_err_t next, esp_err_t *status) {
  if (*status == ESP_OK) *status = next;
}

static esp_err_t release(void) {
  esp_err_t status = ESP_OK;
  if (installed) {
    first_error(uart_wait_tx_done(port, pdMS_TO_TICKS(100)), &status);
    first_error(gpio_set_level(DE, 0), &status);
    first_error(uart_driver_delete(port), &status);
    installed = false;
  }
  /* Release the Click inputs before removing its supply, avoiding logic
   * backfeed. The external NPN pulldown establishes off after pad release. */
  const gpio_config_t inputs = {
      .pin_bit_mask = (1ULL << TX) | (1ULL << RX) | (1ULL << DE) | (1ULL << RE_N),
      .mode = GPIO_MODE_INPUT,
  };
  first_error(gpio_config(&inputs), &status);
  first_error(gpio_set_level(GATE, 0), &status);
  first_error(gpio_set_direction(GATE, GPIO_MODE_INPUT), &status);
  first_error(gpio_set_pull_mode(GATE, GPIO_FLOATING), &status);
  return status;
}

static esp_err_t start(void) {
  esp_err_t status = release();
  const gpio_config_t gate = {.pin_bit_mask = 1ULL << GATE, .mode = GPIO_MODE_OUTPUT};
  first_error(gpio_set_level(GATE, 0), &status);
  first_error(gpio_config(&gate), &status);
  if (status != ESP_OK) return status;
  status = gpio_set_level(GATE, 1);
  if (status != ESP_OK) return status;
  vTaskDelay(pdMS_TO_TICKS(450) + 1);

  first_error(gpio_set_level(DE, 0), &status);
  first_error(gpio_set_level(RE_N, 1), &status);
  const gpio_config_t control = {
      .pin_bit_mask = (1ULL << DE) | (1ULL << RE_N), .mode = GPIO_MODE_OUTPUT,
  };
  first_error(gpio_config(&control), &status);
  if (status != ESP_OK) return status;
  const uart_config_t config = {
      .baud_rate = 9600, .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
  };
  status = uart_param_config(port, &config);
  if (status != ESP_OK) return status;
  status = uart_set_pin(port, TX, RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (status != ESP_OK) return status;
  status = uart_driver_install(port, 256, 0, 0, NULL, 0);
  installed = status == ESP_OK;
  return status;
}

static int exchange(void *context, const uint8_t request[8],
                    uint8_t *reply, size_t capacity) {
  (void)context;
  vTaskDelay(pdMS_TO_TICKS(20));
  esp_err_t status = uart_flush_input(port);
  first_error(gpio_set_level(RE_N, 1), &status);
  first_error(gpio_set_level(DE, 1), &status);
  ESP_LOG_BUFFER_HEX_LEVEL(tag, request, 8, ESP_LOG_INFO);
  if (status == ESP_OK && uart_write_bytes(port, request, 8) != 8) status = ESP_FAIL;
  if (status == ESP_OK) status = uart_wait_tx_done(port, pdMS_TO_TICKS(100));
  first_error(gpio_set_level(DE, 0), &status);
  first_error(gpio_set_level(RE_N, 0), &status);
  if (status != ESP_OK) return -1;
  const int64_t deadline = esp_timer_get_time() + 1000000;
  size_t used = 0;
  while (esp_timer_get_time() < deadline) {
    int n = uart_read_bytes(port, reply + used, capacity - used, pdMS_TO_TICKS(20));
    if (n < 0) { status = ESP_FAIL; break; }
    if (!n && used) break;
    used += (size_t)n;
    if (used == capacity) { status = ESP_ERR_INVALID_SIZE; break; }
  }
  first_error(gpio_set_level(RE_N, 1), &status);
  if (used) ESP_LOG_BUFFER_HEX_LEVEL(tag, reply, used, ESP_LOG_INFO);
  else ESP_LOGI(tag, "no reply");
  /* Allow nonvolatile address storage to settle before verification. This
   * interval is an engineering allowance, not a documented persistence bound. */
  if (request[1] == 6) vTaskDelay(pdMS_TO_TICKS(1000));
  return status == ESP_OK ? (int)used : -1;
}

void app_main(void) {
  ESP_LOGI(tag, "One isolated SEN0600 only; GPIO6 active-high NPN fixture");
  const esp_err_t status = start();
  if (status == ESP_OK) {
    const sen0600_commission_result_t result = sen0600_assign_two(exchange, NULL);
    const char *names[] = {"ASSIGNED_2: power-cycle and verify again",
        "ALREADY_2: register verified; no write", "ABSENT: no write",
        "CONFLICT: both addresses respond; no write", "BAD_REPLY: no write",
        "WRITE_UNCERTAIN: no retry; inspect at addresses 1 and 2",
        "VERIFY_FAILED: no retry; inspect at addresses 1 and 2"};
    ESP_LOGI(tag, "RESULT %s", names[result]);
  } else {
    ESP_LOGE(tag, "SETUP_FAILED: %s", esp_err_to_name(status));
  }
  const esp_err_t cleanup = release();
  ESP_LOGI(tag, "CLEANUP %s; finished, no automatic restart", esp_err_to_name(cleanup));
}
