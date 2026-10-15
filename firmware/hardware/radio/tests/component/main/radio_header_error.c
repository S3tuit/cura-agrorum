/* Local timing and fixed storage; no sensor sampling or production injection API. */
#include "radio_header_error.h"
#include "sx1262_radio_backend.h"
#include "sx126x_hal.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "unity.h"
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>

static struct {
  uint64_t tx_before, tx_after, abort_before, abort_after;
  uint16_t irq, errors;
} timing;
static bool timing_active;

sx126x_hal_status_t __real_sx126x_hal_write(const void *, const uint8_t *, uint16_t,
                                         const uint8_t *, uint16_t);
sx126x_hal_status_t __wrap_sx126x_hal_write(const void *context, const uint8_t *command,
    uint16_t command_length, const uint8_t *data, uint16_t data_length) {
  uint64_t before = esp_timer_get_time();
  sx126x_hal_status_t result = __real_sx126x_hal_write(context, command, command_length, data, data_length);
  uint64_t after = esp_timer_get_time();
  if (timing_active && command_length && command[0] == 0x83) {
    timing.tx_before = before; timing.tx_after = after;
  }
  if (timing_active && command_length && command[0] == 0x80) {
    timing.abort_before = before; timing.abort_after = after;
  }
  return result;
}

void radio_header_wait_since(uint64_t previous_start) {
  while ((uint64_t)esp_timer_get_time() < previous_start + UINT64_C(12200000)) vTaskDelay(1);
}

uint64_t radio_header_abort(const uint8_t *payload, size_t length) {
  sx1262_radio_backend_error_t error;
  TEST_ASSERT_EQUAL_UINT(54, length);
  TEST_ASSERT_EQUAL(SX1262_COMMAND_CONFIRMED, sx1262_radio_backend_set_standby(&error));
  TEST_ASSERT_EQUAL(SX1262_COMMAND_CONFIRMED, sx1262_radio_backend_set_packet_params(length, false, &error));
  TEST_ASSERT_EQUAL(SX1262_COMMAND_CONFIRMED, sx1262_radio_backend_configure_irq(
      SX1262_RADIO_IRQ_TX_DONE | SX1262_RADIO_IRQ_TIMEOUT, &error));
  TEST_ASSERT_EQUAL(SX1262_COMMAND_CONFIRMED, sx1262_radio_backend_clear_irq(SX1262_RADIO_IRQ_ALL, &error));
  TEST_ASSERT_EQUAL(SX1262_COMMAND_CONFIRMED, sx1262_radio_backend_write_payload(payload, length, &error));
  timing_active = true;
  sx1262_radio_backend_call_result_t result = sx1262_radio_backend_start_tx(9600, &error);
  if (result != SX1262_COMMAND_CONFIRMED) timing_active = false;
  TEST_ASSERT_EQUAL(SX1262_COMMAND_CONFIRMED, result);
  TEST_ASSERT_NOT_EQUAL_UINT64(0, timing.tx_before);
  while ((uint64_t)esp_timer_get_time() < timing.tx_before + 18000) esp_rom_delay_us(5);
  result = sx1262_radio_backend_set_standby(&error);
  timing_active = false;
  TEST_ASSERT_EQUAL(SX1262_COMMAND_CONFIRMED, result);
  TEST_ASSERT_TRUE(timing.abort_before - timing.tx_before >= 18000 &&
                   timing.abort_before - timing.tx_before <= 18500);
  TEST_ASSERT_EQUAL(SX1262_COMMAND_CONFIRMED, sx1262_radio_backend_get_irq(&timing.irq, &error));
  TEST_ASSERT_EQUAL(SX1262_COMMAND_CONFIRMED, sx1262_radio_backend_get_device_errors(&timing.errors, &error));
  TEST_ASSERT_EQUAL_UINT16(0, timing.irq);
  TEST_ASSERT_EQUAL_UINT16(0, timing.errors);
  TEST_ASSERT_EQUAL(SX1262_COMMAND_CONFIRMED, sx1262_radio_backend_clear_irq(SX1262_RADIO_IRQ_ALL, &error));
  return timing.tx_before;
}

void radio_header_dump(void) {
  if (!timing.tx_before) return;
  printf("RF_TRACE {\"operation\":\"header_abort\",\"before\":%" PRIu64
         ",\"after\":%" PRIu64 ",\"argument\":18000,\"result\":2,"
         "\"tx_hal_after\":%" PRIu64 ",\"abort_hal_before\":%" PRIu64
         ",\"irq\":%u,\"device_errors\":%u}\n",
         timing.tx_before, timing.abort_after, timing.tx_after, timing.abort_before, timing.irq, timing.errors);
}
