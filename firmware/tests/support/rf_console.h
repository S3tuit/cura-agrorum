#pragma once
#include <stdint.h>

int rf_console_printf(const char *format, ...) __attribute__((format(printf, 1, 2)));
int rf_console_puts(const char *text);

/* Called only after radio cleanup and cycle finalization. Returns only for a
 * current SLEEP receipt; STOP, output failure and timeout park until reset. */
void rf_console_prepare_sleep(void);
void rf_console_prepare_node_sleep(uint64_t duration_us, uint8_t reset_reason);
