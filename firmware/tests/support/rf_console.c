#include "rf_console.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "esp_random.h"
#include "esp_rom_serial_output.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#ifdef CURA_RF_UNITY_OUTPUT
#include "unity_config.h"
#endif

static bool output_failed;
static bool line_start = true;
static unsigned burst;

int rf_console_printf(const char *format, ...) {
  char text[512];
  va_list args;
  va_start(args, format);
  const int length = vsnprintf(text, sizeof(text), format, args);
  va_end(args);
  if (length < 0 || (size_t)length >= sizeof(text)) {
    output_failed = true;
    return -1;
  }
  /* The polled USB VFS may silently discard a prefix after Unity fills the
   * FIFO. Use one ROM path and pace bursts before lines, never after prompts. */
  for (int index = 0; index < length; ++index) {
    const uint8_t byte = (uint8_t)text[index];
    if (line_start && byte != '\n' && byte != '\r' && burst >= 256U) {
      const TickType_t ticks = pdMS_TO_TICKS(10);
      vTaskDelay(ticks == 0 ? 1 : ticks);
      burst = 0;
    }
#ifdef CURA_RF_UNITY_OUTPUT
    /* Share Unity's pacing state across its summary and our first record. */
    unity_putc(byte);
#else
    if (esp_rom_output_tx_one_char(byte) != 0) {
      output_failed = true;
      return -1;
    }
#endif
    line_start = byte == '\n';
    ++burst;
  }
  return length;
}

int rf_console_puts(const char *text) { return rf_console_printf("%s\n", text); }

static void park(uint32_t nonce, bool acknowledged, bool observe_node) {
  /* No restart/sleep fallback: either would reopen an autonomous TX budget.
   * Repeating this status is safe; only the host's current STOP can park with
   * acknowledged=true. A timeout must never resemble confirmed completion. */
  for (;;) {
    rf_console_printf("%sRF_%s %08" PRIx32 "\n%s",
                      observe_node ? "\x1e" : "",
                      acknowledged ? "PARKED" : "FINALIZE_FAILED", nonce,
                      observe_node ? "\x1f\n" : "");
    const TickType_t ticks = pdMS_TO_TICKS(250);
    vTaskDelay(ticks == 0 ? 1 : ticks);
  }
}

static void prepare_sleep(bool observe_node, uint64_t duration_us, uint8_t reset_reason) {
  /* Ordinary RAM is recreated on wake; a second finalization within one boot
   * retains this identity and cannot masquerade as another wake. Repeat the
   * record with greetings because native USB can reconnect after ROM output. */
  static bool boot_identified;
  static uint64_t boot_id;
  if (observe_node && !boot_identified) {
    boot_id = ((uint64_t)esp_random() << 32) | esp_random();
    boot_identified = true;
  }
  const uint32_t nonce = esp_random();
  char sleep[48], stop[48], received[48];
  snprintf(sleep, sizeof(sleep), "RF_FINALIZE_ACK %08" PRIx32 " SLEEP", nonce);
  snprintf(stop, sizeof(stop), "RF_FINALIZE_ACK %08" PRIx32 " STOP", nonce);
  size_t used = 0;
  bool oversized = false;
  const int64_t deadline = esp_timer_get_time() + INT64_C(30000000);
  int64_t next_greeting = 0;
  while (!output_failed && esp_timer_get_time() < deadline) {
    const int64_t now = esp_timer_get_time();
    if (now >= next_greeting) {
      if (observe_node) {
        /* RS/US bound the whole greeting. USB may discard a suffix while no
         * host is attached; the next RS then replaces an incomplete frame. */
        rf_console_printf("\x1eRF_NODE_SLEEP boot=%016" PRIx64 " reset_reason=%u duration_us=%" PRIu64
                          " finalize=%08" PRIx32 "\nRF_FINALIZE %08" PRIx32 "\n\x1f\n",
                          boot_id, (unsigned)reset_reason, duration_us, nonce, nonce);
      } else {
        rf_console_printf("RF_FINALIZE %08" PRIx32 "\n", nonce);
      }
      next_greeting = now + INT64_C(250000);
    }
    uint8_t byte;
    if (esp_rom_output_rx_one_char(&byte) != 0) {
      vTaskDelay(1);
      continue;
    }
    if (byte == '\n') {
      if (used != 0 && received[used - 1] == '\r') --used;
      received[used] = '\0';
      if (!oversized && !output_failed) {
        if (strcmp(received, sleep) == 0) return;
        if (strcmp(received, stop) == 0) park(nonce, true, observe_node);
      }
      used = 0;
      oversized = false;
    } else if (byte == 0 || (byte < 0x20 && byte != '\r') || byte > 0x7e) {
      oversized = true;
    } else if (used + 1 < sizeof(received)) {
      received[used++] = (char)byte;
    } else {
      oversized = true;
    }
  }
  park(nonce, false, observe_node);
}

void rf_console_prepare_sleep(void) { prepare_sleep(false, 0, 0); }

void rf_console_prepare_node_sleep(uint64_t duration_us, uint8_t reset_reason) {
  prepare_sleep(true, duration_us, reset_reason);
}
