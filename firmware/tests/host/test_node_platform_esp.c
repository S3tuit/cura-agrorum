#include <setjmp.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_err.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "node_platform_esp.h"

#define TEST_ASSERT(expression)                                                \
  do {                                                                         \
    if (!(expression)) {                                                       \
      fprintf(stderr, "%s:%d: assertion failed: %s\n", __FILE__, __LINE__,     \
              #expression);                                                    \
      return false;                                                            \
    }                                                                          \
  } while (0)

#define RANDOM_SCRIPT_CAPACITY 8U

typedef enum {
  TERMINAL_NONE = 0,
  TERMINAL_DEEP_SLEEP,
  TERMINAL_RESTART,
} terminal_operation_t;

static int64_t g_timer_value;
static int g_reset_reason;
static uint32_t g_random_script[RANDOM_SCRIPT_CAPACITY];
static size_t g_random_script_length;
static size_t g_random_script_index;
static esp_err_t g_timer_wakeup_result;
static uint64_t g_timer_wakeup_duration_us;
static size_t g_timer_wakeup_calls;
static size_t g_deep_sleep_calls;
static TickType_t g_delay_ticks;
static size_t g_delay_calls;
static size_t g_restart_calls;
static size_t g_error_log_calls;
static terminal_operation_t g_terminal_operation;
static jmp_buf g_terminal_jump;

static void fake_reset(void) {
  g_timer_value = 0;
  g_reset_reason = ESP_RST_UNKNOWN;
  g_random_script_length = 0U;
  g_random_script_index = 0U;
  g_timer_wakeup_result = ESP_OK;
  g_timer_wakeup_duration_us = 0U;
  g_timer_wakeup_calls = 0U;
  g_deep_sleep_calls = 0U;
  g_delay_ticks = 0U;
  g_delay_calls = 0U;
  g_restart_calls = 0U;
  g_error_log_calls = 0U;
  g_terminal_operation = TERMINAL_NONE;
}

static void script_random(uint32_t value) {
  if (g_random_script_length < RANDOM_SCRIPT_CAPACITY) {
    g_random_script[g_random_script_length] = value;
    ++g_random_script_length;
  }
}

int64_t esp_timer_get_time(void) { return g_timer_value; }

uint32_t esp_random(void) {
  if (g_random_script_index >= g_random_script_length) {
    return 0U;
  }
  const uint32_t value = g_random_script[g_random_script_index];
  ++g_random_script_index;
  return value;
}

esp_reset_reason_t esp_reset_reason(void) {
  return (esp_reset_reason_t)g_reset_reason;
}

esp_err_t esp_sleep_enable_timer_wakeup(uint64_t time_in_us) {
  ++g_timer_wakeup_calls;
  g_timer_wakeup_duration_us = time_in_us;
  return g_timer_wakeup_result;
}

_Noreturn void esp_deep_sleep_start(void) {
  ++g_deep_sleep_calls;
  g_terminal_operation = TERMINAL_DEEP_SLEEP;
  longjmp(g_terminal_jump, 1);
}

void vTaskDelay(TickType_t ticks_to_delay) {
  ++g_delay_calls;
  g_delay_ticks = ticks_to_delay;
}

_Noreturn void esp_restart(void) {
  ++g_restart_calls;
  g_terminal_operation = TERMINAL_RESTART;
  longjmp(g_terminal_jump, 1);
}

const char *esp_err_to_name(esp_err_t code) {
  (void)code;
  return "fake error";
}

void fake_esp_log_error(const char *tag, const char *format, ...) {
  (void)tag;
  (void)format;
  ++g_error_log_calls;
  va_list arguments;
  va_start(arguments, format);
  va_end(arguments);
}

static bool test_port_table_is_immutable_and_complete(void) {
  const node_platform_ports_t *const first = node_platform_esp_ports();
  const node_platform_ports_t *const second = node_platform_esp_ports();
  TEST_ASSERT(first != NULL);
  TEST_ASSERT(first == second);
  TEST_ASSERT(first->clock.context == NULL);
  TEST_ASSERT(first->clock.monotonic_us != NULL);
  TEST_ASSERT(first->randomness.context == NULL);
  TEST_ASSERT(first->randomness.uniform_u32_inclusive != NULL);
  TEST_ASSERT(first->system.context == NULL);
  TEST_ASSERT(first->system.get_reset_reason != NULL);
  TEST_ASSERT(first->system.enter_deep_sleep_for != NULL);
  return true;
}

static bool test_clock_normalizes_negative_values(void) {
  fake_reset();
  const node_platform_ports_t *const ports = node_platform_esp_ports();
  g_timer_value = -1;
  TEST_ASSERT(ports->clock.monotonic_us(ports->clock.context) == 0U);
  g_timer_value = INT64_MAX;
  TEST_ASSERT(ports->clock.monotonic_us(ports->clock.context) ==
              (uint64_t)INT64_MAX);
  return true;
}

static bool test_reset_reason_normalization(void) {
  fake_reset();
  const node_platform_ports_t *const ports = node_platform_esp_ports();
  g_reset_reason = ESP_RST_DEEPSLEEP;
  TEST_ASSERT(ports->system.get_reset_reason(ports->system.context) == 8U);
  g_reset_reason = 255;
  TEST_ASSERT(ports->system.get_reset_reason(ports->system.context) == 255U);
  g_reset_reason = -1;
  TEST_ASSERT(ports->system.get_reset_reason(ports->system.context) == 0U);
  g_reset_reason = 256;
  TEST_ASSERT(ports->system.get_reset_reason(ports->system.context) == 0U);
  return true;
}

static bool test_randomness_boundaries_and_rejection(void) {
  fake_reset();
  const node_platform_ports_t *const ports = node_platform_esp_ports();

  TEST_ASSERT(ports->randomness.uniform_u32_inclusive(ports->randomness.context,
                                                      42U, 42U) == 42U);
  TEST_ASSERT(g_random_script_index == 0U);

  script_random(UINT32_C(0xdeadbeef));
  TEST_ASSERT(ports->randomness.uniform_u32_inclusive(ports->randomness.context,
                                                      0U, UINT32_MAX) ==
              UINT32_C(0xdeadbeef));
  TEST_ASSERT(g_random_script_index == 1U);

  script_random(UINT32_MAX);
  script_random(5U);
  TEST_ASSERT(ports->randomness.uniform_u32_inclusive(ports->randomness.context,
                                                      10U, 12U) == 12U);
  TEST_ASSERT(g_random_script_index == 3U);

  script_random(UINT32_MAX);
  TEST_ASSERT(ports->randomness.uniform_u32_inclusive(
                  ports->randomness.context, UINT32_MAX - 1U, UINT32_MAX) ==
              UINT32_MAX);
  TEST_ASSERT(g_random_script_index == 4U);
  return true;
}

static bool test_successful_deep_sleep_is_terminal(void) {
  fake_reset();
  const node_platform_ports_t *const ports = node_platform_esp_ports();
  if (setjmp(g_terminal_jump) == 0) {
    ports->system.enter_deep_sleep_for(ports->system.context,
                                       UINT64_C(900000000));
    TEST_ASSERT(false);
  }
  TEST_ASSERT(g_terminal_operation == TERMINAL_DEEP_SLEEP);
  TEST_ASSERT(g_timer_wakeup_calls == 1U);
  TEST_ASSERT(g_timer_wakeup_duration_us == UINT64_C(900000000));
  TEST_ASSERT(g_deep_sleep_calls == 1U);
  TEST_ASSERT(g_error_log_calls == 0U);
  TEST_ASSERT(g_delay_calls == 0U);
  TEST_ASSERT(g_restart_calls == 0U);
  return true;
}

static bool test_failed_wakeup_configuration_delays_and_restarts(void) {
  fake_reset();
  g_timer_wakeup_result = ESP_FAIL;
  const node_platform_ports_t *const ports = node_platform_esp_ports();
  if (setjmp(g_terminal_jump) == 0) {
    ports->system.enter_deep_sleep_for(ports->system.context, UINT64_C(123));
    TEST_ASSERT(false);
  }
  TEST_ASSERT(g_terminal_operation == TERMINAL_RESTART);
  TEST_ASSERT(g_timer_wakeup_calls == 1U);
  TEST_ASSERT(g_timer_wakeup_duration_us == UINT64_C(123));
  TEST_ASSERT(g_deep_sleep_calls == 0U);
  TEST_ASSERT(g_error_log_calls == 1U);
  TEST_ASSERT(g_delay_calls == 1U);
  TEST_ASSERT(g_delay_ticks == pdMS_TO_TICKS(UINT32_C(60000)));
  TEST_ASSERT(g_restart_calls == 1U);
  return true;
}

int main(void) {
  if (!test_port_table_is_immutable_and_complete() ||
      !test_clock_normalizes_negative_values() ||
      !test_reset_reason_normalization() ||
      !test_randomness_boundaries_and_rejection() ||
      !test_successful_deep_sleep_is_terminal() ||
      !test_failed_wakeup_configuration_delays_and_restarts()) {
    return 1;
  }
  puts("PASS node_platform_esp");
  return 0;
}
