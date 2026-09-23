#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* App-local UART framing; no radio or platform dependencies. */
typedef enum {
  RF_LINE_PENDING, RF_LINE_COMPLETE, RF_LINE_INVALID, RF_LINE_OVERSIZED,
  RF_LINE_EXPIRED
} rf_line_status_t;

typedef struct {
  char text[160];
  size_t length;
  uint64_t first_byte_us;
  bool started;
  rf_line_status_t status;
} rf_line_t;

/* byte=-1 polls the deadline without supplying input. Terminal states latch. */
rf_line_status_t rf_line_feed(rf_line_t *line, int byte, uint64_t now_us);

typedef struct {
  char run[33], selection[32];
  uint32_t boot;
  unsigned phase;
} rf_command_t;

/* Exact canonical ASCII syntax, independent of libc scanf range extensions. */
bool rf_command_parse(const char *text, rf_command_t *command);
