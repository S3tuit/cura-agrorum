/* Passive, bounded link wrappers for the production-node RF test build.
 * No injection, extra radio commands, or UART work in the receive path. */
#include "esp_timer.h"
#include "sx1262_radio_backend.h"
#include "sx126x_hal.h"
#include <inttypes.h>
#include <stdio.h>

static struct {
  uint64_t before_us, after_us, irq_at_us;
  uint16_t value;
  uint8_t opcode, result;
} events[128];
static unsigned count;
static bool overflow;
static uint64_t last_irq_at_us;

static void observe(uint8_t opcode, uint16_t value, uint64_t before,
                    sx126x_hal_status_t result) {
  if (count == sizeof(events) / sizeof(events[0])) {
    overflow = true;
    return;
  }
  events[count].before_us = before;
  events[count].after_us = (uint64_t)esp_timer_get_time();
  events[count].irq_at_us = opcode == 0x12U ? last_irq_at_us : 0U;
  events[count].value = value;
  events[count].opcode = opcode;
  events[count].result = (uint8_t)result;
  count++;
}

bool __real_sx1262_radio_backend_wait_dio1(uint64_t, bool *, uint64_t *,
                                           sx1262_radio_backend_error_t *);
bool __wrap_sx1262_radio_backend_wait_dio1(
    uint64_t deadline, bool *observed, uint64_t *at,
    sx1262_radio_backend_error_t *error) {
  const bool result =
      __real_sx1262_radio_backend_wait_dio1(deadline, observed, at, error);
  last_irq_at_us = result && *observed ? *at : 0U;
  return result;
}

sx126x_hal_status_t __real_sx126x_hal_read(const void *, const uint8_t *,
                                           uint16_t, uint8_t *, uint16_t);
sx126x_hal_status_t __wrap_sx126x_hal_read(const void *context,
                                           const uint8_t *command,
                                           uint16_t command_length,
                                           uint8_t *data,
                                           uint16_t data_length) {
  const uint64_t before = (uint64_t)esp_timer_get_time();
  const sx126x_hal_status_t result = __real_sx126x_hal_read(
      context, command, command_length, data, data_length);
  if (command_length && command[0] == 0x12U && data_length == 2U) {
    observe(0x12U,
            result == SX126X_HAL_STATUS_OK
                ? (uint16_t)((uint16_t)data[0] << 8U | data[1])
                : 0U,
            before, result);
  }
  return result;
}

sx126x_hal_status_t __real_sx126x_hal_write(const void *, const uint8_t *,
                                            uint16_t, const uint8_t *,
                                            uint16_t);
sx126x_hal_status_t __wrap_sx126x_hal_write(const void *context,
                                            const uint8_t *command,
                                            uint16_t command_length,
                                            const uint8_t *data,
                                            uint16_t data_length) {
  const uint64_t before = (uint64_t)esp_timer_get_time();
  const sx126x_hal_status_t result = __real_sx126x_hal_write(
      context, command, command_length, data, data_length);
  if (command_length && (command[0] == 0x82U || command[0] == 0x83U)) {
    observe(command[0], 0U, before, result);
  }
  return result;
}

/* Called after production cycle finalization, immediately before sleep marker.
 */
void node_rf_observe_dump(void) {
  for (unsigned i = 0U; i < count; ++i) {
    printf("RF_NODE_PHY {\"index\":%u,\"opcode\":%u,\"value\":%u,"
           "\"before_us\":%" PRIu64 ",\"after_us\":%" PRIu64
           ",\"irq_at_us\":%" PRIu64 ",\"result\":%u}\n",
           i, events[i].opcode, events[i].value, events[i].before_us,
           events[i].after_us, events[i].irq_at_us, events[i].result);
  }
  printf("RF_NODE_PHY_END count=%u overflow=%u\n", count, overflow ? 1U : 0U);
}
