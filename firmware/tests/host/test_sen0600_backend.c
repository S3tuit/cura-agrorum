#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "node_sensors.h"
#include "node_sensors_backend.h"
#include "sen0600_modbus.h"

static int64_t now;
static gpio_mode_t modes[48];
static uint32_t levels[48];
static bool rail, uart, install_failure, first_timeout;
static uint8_t address;
static unsigned bad_response;
static unsigned requests[3], releases;
static const uint8_t foreign_reply[] = {1, 3, 4, 1, 0xe6, 0xff, 0x9f, 0x1b, 0xa0};
static const uint8_t current_reply[] = {2, 3, 4, 2, 0x8a, 0, 0xd7, 0xa9, 0x3f};
static const uint8_t foreign_exception[] = {1, 0x83, 2, 0xc0, 0xf1};
static const uint8_t current_exception[] = {2, 0x83, 2, 0x30, 0xf1};
static bool stream_case, foreign_flood, read_error;
static uint8_t incoming[32];
static size_t incoming_length, incoming_at, chunk_size;
static unsigned second_reads;
static int64_t second_start, final_byte_at;

int64_t esp_timer_get_time(void) { return now; }
void vTaskDelay(TickType_t ticks) { now += (int64_t)ticks * 1000; }
esp_err_t gpio_set_level(gpio_num_t pin, uint32_t level) {
  if (pin == 44) {
    if (rail && level == 0) {
      assert(!uart);
      assert(modes[1] == GPIO_MODE_INPUT && modes[2] == GPIO_MODE_INPUT);
      assert(modes[4] == GPIO_MODE_INPUT && modes[43] == GPIO_MODE_INPUT);
      ++releases;
    }
    rail = level != 0;
  }
  levels[pin] = level;
  return ESP_OK;
}
esp_err_t gpio_config(const gpio_config_t *config) {
  assert(config->pull_up_en == GPIO_PULLUP_DISABLE && config->pull_down_en == GPIO_PULLDOWN_DISABLE);
  for (unsigned pin = 0; pin < 48; ++pin)
    if (config->pin_bit_mask & (UINT64_C(1) << pin)) modes[pin] = config->mode;
  return ESP_OK;
}
esp_err_t gpio_reset_pin(gpio_num_t pin) { modes[pin] = GPIO_MODE_INPUT; return ESP_OK; }
esp_err_t gpio_set_direction(gpio_num_t pin, gpio_mode_t mode) { modes[pin] = mode; return ESP_OK; }
esp_err_t gpio_set_pull_mode(gpio_num_t pin, gpio_pull_mode_t pull) {
  (void)pin; assert(pull == GPIO_FLOATING); return ESP_OK;
}
esp_err_t uart_param_config(int port, const uart_config_t *config) {
  assert(port == 1 && rail && config->baud_rate == 9600 && config->data_bits == 8);
  return ESP_OK;
}
esp_err_t uart_set_pin(int port, int tx, int rx, int rts, int cts) {
  assert(port == 1 && rail && tx == 1 && rx == 2 && rts == -1 && cts == -1);
  modes[tx] = GPIO_MODE_OUTPUT; modes[rx] = GPIO_MODE_INPUT; return ESP_OK;
}
esp_err_t uart_driver_install(int port, int rx, int tx, int qs, void *queue, int flags) {
  (void)port; (void)rx; (void)tx; (void)qs; (void)queue; (void)flags;
  if (install_failure) return ESP_FAIL;
  uart = true; return ESP_OK;
}
esp_err_t uart_driver_delete(int port) {
  (void)port;
  if (stream_case) assert(levels[43] == 1);
  uart = false;
  return ESP_OK;
}
esp_err_t uart_wait_tx_done(int port, TickType_t timeout) {
  (void)port; assert(uart && timeout > 0); return ESP_OK;
}
esp_err_t uart_flush_input(int port) { (void)port; assert(uart); return ESP_OK; }
int uart_write_bytes(int port, const void *bytes, size_t length) {
  (void)port; assert(uart && rail && levels[4] == 1 && levels[43] == 1 && length == 8);
  address = ((const uint8_t *)bytes)[0]; assert(address == 1 || address == 2);
  if (address == 2) second_start = now;
  ++requests[address]; return (int)length;
}
int uart_read_bytes(int port, void *bytes, size_t capacity, TickType_t timeout) {
  (void)port; assert(uart && rail && levels[4] == 0 && levels[43] == 0 && capacity >= 9);
  now += (int64_t)timeout * 1000;
  if (stream_case) {
    if (address == 1) return 0;
    ++second_reads;
    if (read_error) return -1;
    if (foreign_flood) {
      memcpy(bytes, foreign_reply, sizeof(foreign_reply));
      return (int)sizeof(foreign_reply);
    }
    size_t count = incoming_length - incoming_at;
    if (count > chunk_size) count = chunk_size;
    if (count > capacity) count = capacity;
    if (count && final_byte_at && incoming_at + count == incoming_length)
      now = second_start + final_byte_at;
    memcpy(bytes, incoming + incoming_at, count);
    incoming_at += count;
    return (int)count;
  }
  if (address == 1 && first_timeout) return 0;
  uint8_t response[] = {1, 3, 4, 1, 0xe6, 0xff, 0x9f, 0, 0};
  response[0] = address;
  uint16_t crc = 0xffff;
  for (unsigned i = 0; i < 7; ++i) {
    crc ^= response[i];
    for (unsigned j = 0; j < 8; ++j)
      crc = (crc & 1U) ? (uint16_t)((crc >> 1) ^ 0xa001U) : (uint16_t)(crc >> 1);
  }
  if (address == 1 && bad_response == 2) now += 1000000;
  response[7] = (uint8_t)crc; response[8] = (uint8_t)(crc >> 8);
  if (address == 1 && bad_response == 1) response[8] ^= 1;
  memcpy(bytes, response, sizeof(response)); return 9;
}
node_sensors_backend_result_t node_sensors_backend_sample_bme280(node_sensors_backend_enclosure_t *out) {
  assert(!rail && !uart); *out = (node_sensors_backend_enclosure_t){2000, 100000, 5000};
  return (node_sensors_backend_result_t){0};
}

static void test_stream_transaction(const char *name) {
  assert(CONFIG_CURA_SEN0600_1 == 2);
  stream_case = true;
  chunk_size = 9;
  memcpy(incoming, foreign_reply, 9);
  memcpy(incoming + 9, current_reply, 9);
  incoming_length = 18;
  bool success = true;
  uint32_t expected_kind = NODE_SENSOR_BACKEND_STATUS_NONE;
  uint32_t expected_status = 0;
  unsigned expected_reads = 2;
  int64_t expected_elapsed = 20000;
  if (strcmp(name, "late_split") == 0) {
    /* One late foreign frame, then the healthy reply in a later UART read. */
  } else if (strcmp(name, "late_combined") == 0) {
    chunk_size = 32; expected_reads = 1; expected_elapsed = 10000;
  } else if (strcmp(name, "late_fragmented") == 0) {
    chunk_size = 4; expected_reads = 5; expected_elapsed = 50000;
  } else if (strcmp(name, "late_exception") == 0) {
    memcpy(incoming, foreign_exception, 5);
    memcpy(incoming + 5, current_reply, 9);
    incoming_length = 14;
    chunk_size = 1; expected_reads = 14; expected_elapsed = 140000;
  } else if (strcmp(name, "deadline_equal") == 0) {
    final_byte_at = 1000000; expected_elapsed = final_byte_at;
  } else if (strcmp(name, "deadline_late") == 0) {
    final_byte_at = 1000001; expected_elapsed = final_byte_at;
    success = false;
    expected_kind = NODE_SENSOR_BACKEND_STATUS_ESP_ERR;
    expected_status = ESP_ERR_TIMEOUT;
  } else if (strcmp(name, "foreign_flood") == 0 || strcmp(name, "incomplete") == 0) {
    foreign_flood = strcmp(name, "foreign_flood") == 0;
    incoming_length = 17;
    success = false; expected_reads = 100; expected_elapsed = 1000000;
    expected_kind = NODE_SENSOR_BACKEND_STATUS_ESP_ERR;
    expected_status = ESP_ERR_TIMEOUT;
  } else if (strcmp(name, "current_exception") == 0) {
    memcpy(incoming + 9, current_exception, 5);
    incoming_length = 14;
    success = false;
    expected_kind = NODE_SENSOR_BACKEND_STATUS_DRIVER;
    expected_status = SEN0600_EXCEPTION | (2U << 8);
  } else if (strcmp(name, "current_bad_crc") == 0) {
    incoming[17] ^= 1;
    success = false;
    expected_kind = NODE_SENSOR_BACKEND_STATUS_DRIVER;
    expected_status = SEN0600_BAD_CRC;
  } else if (strcmp(name, "read_error") == 0) {
    read_error = true; success = false; expected_reads = 1; expected_elapsed = 10000;
    expected_kind = NODE_SENSOR_BACKEND_STATUS_ESP_ERR;
    expected_status = (uint32_t)ESP_FAIL;
  } else {
    assert(!"unknown scenario");
  }

  node_sensor_sample_t sample;
  diagn_context_t diagnostic;
  assert(curag_error_code(node_sensors_sample_all(&sample, &diagnostic)) == CURAG_ESENSORS_EPARTIAL_SAMPLE);
  assert(sample.sensor_selection == 12 && sample.validity == (success ? 0x58 : 0x10));
  assert(sample.soil_0_moisture_deci_pct == 0 && sample.soil_temp_0_centi_c == 0);
  assert(sample.soil_1_moisture_deci_pct == (success ? 650 : 0));
  assert(sample.soil_temp_1_centi_c == (success ? 2150 : 0));
  assert(sample.enclosure_centi_c == 2000 && sample.enclosure_pressure_pa == 100000);
  assert(requests[1] == 1 && requests[2] == 1 && second_reads == expected_reads);
  assert(now - second_start == expected_elapsed);
  assert(!rail && !uart && releases == 1);
  const unsigned pairs[] = {NODE_SENSOR_CONTEXT_SOIL_1, NODE_SENSOR_CONTEXT_SOIL_TEMP_1};
  for (size_t i = 0; i < sizeof(pairs) / sizeof(pairs[0]); ++i) {
    const uint8_t *pair = diagnostic.context + pairs[i] * 8;
    for (unsigned byte = 0; byte < 4; ++byte) {
      assert(pair[byte] == (uint8_t)(expected_kind >> (byte * 8)));
      assert(pair[4 + byte] == (uint8_t)(expected_status >> (byte * 8)));
    }
  }
}

int main(int argc, char **argv) {
  if (argc == 2) {
    test_stream_transaction(argv[1]);
    return 0;
  }
  assert(argc == 1);
  const bool dual = CONFIG_CURA_SEN0600_1 != 0;
  const uint8_t selection = dual ? 12 : 4;
  const uint8_t complete = dual ? 0x7c : 0x34;
  const uint8_t partial = dual ? 0x58 : 0x10;
  diagn_context_t diagnostic;
  node_sensor_sample_t sample;
  assert(node_sensors_force_power_off(&diagnostic) == CURAG_OK);
  assert(now == 0); /* Unknown prior off-time incurs no wait. */
  assert(node_sensors_sample_all(&sample, &diagnostic) == CURAG_OK);
  assert(sample.sensor_selection == selection && sample.validity == complete);
  assert(sample.soil_0_moisture_deci_pct == 486 && sample.soil_1_moisture_deci_pct == (dual ? 486 : 0));
  assert(sample.soil_temp_0_centi_c == -970 && sample.soil_temp_1_centi_c == (dual ? -970 : 0));
  assert(sample.soil_0_mv == 0 && sample.soil_1_mv == 0);
  assert(now >= 450000 && now < 1000000 && releases == 1);
  const int64_t off = now;
  now += 2000000;
  assert(node_sensors_force_power_off(&diagnostic) == CURAG_OK);
  first_timeout = true;
  assert(curag_error_code(node_sensors_sample_all(&sample, &diagnostic)) == CURAG_ESENSORS_EPARTIAL_SAMPLE);
  assert(now - off >= 11450000); /* Force-off did not restart the known interval. */
  assert(now - off < 12000000);
  assert(sample.sensor_selection == selection && sample.validity == partial);
  assert(sample.soil_0_moisture_deci_pct == 0 && sample.soil_temp_0_centi_c == 0);
  assert(sample.soil_1_moisture_deci_pct == (dual ? 486 : 0) && requests[1] == 2 && requests[2] == (dual ? 2U : 0U));
  first_timeout = false;
  for (bad_response = 1; bad_response <= 2; ++bad_response) {
    assert(curag_error_code(node_sensors_sample_all(&sample, &diagnostic)) == CURAG_ESENSORS_EPARTIAL_SAMPLE);
    assert(sample.sensor_selection == selection && sample.validity == partial);
    assert(sample.soil_0_moisture_deci_pct == 0 && sample.soil_temp_0_centi_c == 0);
    assert(sample.soil_1_moisture_deci_pct == (dual ? 486 : 0));
  }
  install_failure = true;
  assert(curag_error_code(node_sensors_sample_all(&sample, &diagnostic)) == CURAG_ESENSORS_EPOWER_CONTROL);
  assert(sample.sensor_selection == selection && sample.validity == NODE_SENSOR_ENCLOSURE_ENV_VALID);
  assert(!rail && !uart && releases == 5);
  return 0;
}
