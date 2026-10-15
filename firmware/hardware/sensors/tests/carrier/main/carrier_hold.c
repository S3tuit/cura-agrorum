#include "carrier_hold.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "esp_rom_serial_output.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Use the same nonblocking UART primitive as ESP-IDF Unity. Unlike unity_gets,
 * this reader has a deadline and does not discard an already-arrived command. */
static bool read_command(char *text, size_t capacity, int64_t deadline) {
  size_t length = 0;
  while (deadline == 0 || esp_timer_get_time() < deadline) {
    uint8_t byte;
    if (esp_rom_output_rx_one_char(&byte) != 0) {
      vTaskDelay(1);
      continue;
    }
    if (byte == '\r') continue;
    if (byte == '\n') {
      if (length == 0) continue;
      text[length] = '\0';
      return deadline == 0 || esp_timer_get_time() < deadline;
    }
    if (byte == 0 || length + 1 >= capacity) return false;
    text[length++] = (char)byte;
  }
  return false;
}

int carrier_hold_select(void) {
  char mode[16];
  puts("CARRIER_HOLD_MODE");
  fflush(stdout);
  if (!read_command(mode, sizeof(mode), esp_timer_get_time() + INT64_C(30000000))) return -1;
  if (strcmp(mode, "auto") == 0) return 0;
  if (strcmp(mode, "guided") == 0) return 1;
  if (strcmp(mode, "exploration") == 0) return CARRIER_HOLD_EXPLORATION;
  return -1;
}

bool carrier_hold_wait(const char *name, int mode) {
  if (mode < CARRIER_HOLD_AUTO || mode > CARRIER_HOLD_EXPLORATION) return false;
  const bool exploration = mode == CARRIER_HOLD_EXPLORATION;
  const int64_t started = esp_timer_get_time();
  printf("CARRIER_HOLD_READY %s seconds=%s\n", name,
         exploration ? "unlimited" : mode == CARRIER_HOLD_GUIDED ? "180" : "60");
  fflush(stdout);
  bool complete = true;
  if (mode != CARRIER_HOLD_AUTO) {
    char received[80], expected[80];
    snprintf(expected, sizeof(expected), "CARRIER_HOLD_ACK %s", name);
    complete = read_command(received, sizeof(received), exploration ? 0 : started + INT64_C(180000000)) &&
               strcmp(received, expected) == 0;
    /* A host acknowledgement cannot bypass the electrical settling interval. */
    const bool on = strcmp(name, "gate-on") == 0 || strcmp(name, "transition-on") == 0;
    const int64_t minimum_us = on ? INT64_C(5000000) : INT64_C(10000000);
    const int64_t elapsed = esp_timer_get_time() - started;
    complete = complete && (exploration || (elapsed >= minimum_us && elapsed < INT64_C(180000000)));
    printf("CARRIER_HOLD_%s %s elapsed_us=%" PRId64 "\n",
           complete ? "ACKED" : "INCOMPLETE", name, elapsed);
  } else {
    vTaskDelay(pdMS_TO_TICKS(60000U) + 1U);
  }
  printf("CARRIER_HOLD_END %s\n", name);
  fflush(stdout);
  return complete;
}
