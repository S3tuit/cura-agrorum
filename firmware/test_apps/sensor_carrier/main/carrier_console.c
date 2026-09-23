#include <stdbool.h>
#include <stddef.h>
#include "esp_rom_serial_output.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Keep ESP-IDF's actual Unity parser, including its menu, echo and editing.
 * Only its empty UART poll needs to block so the single C6 core can run IDLE.
 * Other UART callers (including the bounded observation hold) are unchanged. */
static bool s_unity_wait;
void __real_unity_gets(char *destination, size_t size);
int __real_esp_rom_output_rx_one_char(uint8_t *byte);

void __wrap_unity_gets(char *destination, size_t size) {
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
