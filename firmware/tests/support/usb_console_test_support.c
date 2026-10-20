#include "usb_console_test_support.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_random.h"
#include "esp_rom_serial_output.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static bool wait_for_receipt(const char *purpose) {
  /* A connected USB controller does not prove the host reopened its tty.
   * Repeat a nonce-bound greeting before emitting identity/menu records. */
  const uint32_t nonce = esp_random();
  char expected[40], received[40];
  snprintf(expected, sizeof(expected), "CURAG_USB_%sACK %08" PRIx32, purpose, nonce);
  size_t used = 0;
  const int64_t deadline = esp_timer_get_time() + INT64_C(30000000);
  int64_t next_greeting = 0;
  while (esp_timer_get_time() < deadline) {
    const int64_t now = esp_timer_get_time();
    if (now >= next_greeting) {
      printf("CURAG_USB_%sREADY %08" PRIx32 "\n", purpose, nonce);
      fflush(stdout);
      next_greeting = now + INT64_C(250000);
    }
    uint8_t byte;
    if (esp_rom_output_rx_one_char(&byte) == 0) {
      if (byte == '\r') continue;
      if (byte == '\n') {
        received[used] = '\0';
        if (strcmp(received, expected) == 0) return true;
        used = 0;
      } else if (used + 1 < sizeof(received)) {
        received[used++] = (char)byte;
      } else {
        return false;
      }
    } else {
      vTaskDelay(1);
    }
  }
  return false;
}

bool hwtest_usb_console_ready(void) { return wait_for_receipt(""); }

/* Receipt proves the reader retained preceding records before USB powers off.
 * A distinct greeting keeps this barrier separate from unexpected boot events. */
bool hwtest_usb_console_prepare_sleep(void) { return wait_for_receipt("SLEEP_"); }
