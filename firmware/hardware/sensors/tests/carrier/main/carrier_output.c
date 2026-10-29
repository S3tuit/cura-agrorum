#include "carrier_output.h"

#include <stdarg.h>
#include <stdio.h>
#include "unity_config.h"

/* Unity uses the ROM console, whereas polled USB stdio can silently drop a
 * prefix after Unity fills the FIFO. All carrier records must share Unity's
 * paced output path. Emit raw characters: UnityPrint escapes embedded newlines.
 * Reject oversized chunks before emitting anything, so truncation cannot turn
 * one protocol record into another. Long hex records are emitted in chunks. */
int carrier_output_printf(const char *format, ...) {
  char text[512];
  va_list args;
  va_start(args, format);
  const int length = vsnprintf(text, sizeof(text), format, args);
  va_end(args);
  if (length < 0 || (size_t)length >= sizeof(text)) return -1;
  for (int index = 0; index < length; ++index) unity_putc((unsigned char)text[index]);
  return length;
}

int carrier_output_puts(const char *text) {
  return carrier_output_printf("%s\n", text);
}
