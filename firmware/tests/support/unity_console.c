#include <stdbool.h>
#include <stddef.h>
#include "esp_rom_serial_output.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Keep ESP-IDF's actual parser, echo and editing while allowing IDLE to run
 * during input waits. Other console callers retain their own wait policy. */
static bool s_unity_wait;
static bool s_line_start = true;
static size_t s_output_bytes;
void __real_unity_gets(char *destination, size_t size);
void __real_unity_putc(int byte);
int __real_esp_rom_output_rx_one_char(uint8_t *byte);

void __wrap_unity_putc(int byte) {
  /* ROM USB output can drop a menu burst once the host tty buffers fill.
   * Pace bursts before a line starts: sleeping after a prompt would let a
   * fast host send its selector before unity_gets flushes pending input. */
  if (s_line_start && byte != '\n' && byte != '\r' && s_output_bytes >= 256) {
    const TickType_t ticks = pdMS_TO_TICKS(10);
    vTaskDelay(ticks == 0 ? 1 : ticks);
    s_output_bytes = 0;
  }
  __real_unity_putc(byte);
  s_line_start = byte == '\n';
  ++s_output_bytes;
}

void __wrap_unity_gets(char *destination, size_t size) {
  s_output_bytes = 0;
  s_unity_wait = true;
  __real_unity_gets(destination, size);
  s_unity_wait = false;
}

int __wrap_esp_rom_output_rx_one_char(uint8_t *byte) {
  const int result = __real_esp_rom_output_rx_one_char(byte);
  if (s_unity_wait && result != 0) {
    vTaskDelay(1);
  }
  return result;
}
