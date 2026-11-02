#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
  SEN0600_OK = 0,
  SEN0600_BAD_LENGTH,
  SEN0600_BAD_CRC,
  SEN0600_BAD_ADDRESS,
  SEN0600_BAD_FUNCTION,
  SEN0600_EXCEPTION,
} sen0600_status_t;

typedef struct {
  uint16_t moisture_deci_pct;
  int16_t temperature_centi_c;
  bool temperature_valid;
  uint8_t exception;
} sen0600_reading_t;

void sen0600_request(uint8_t address, uint8_t request[8]);
sen0600_status_t sen0600_parse(uint8_t address, const uint8_t *reply,
                              size_t length, sen0600_reading_t *reading);

/* One addressed transaction, independent of UART chunk boundaries. Start at a
 * frame boundary after draining stale input. Only complete, validated foreign
 * replies are skipped; malformed input terminates without guessing a new frame
 * boundary. The caller owns the deadline, which foreign traffic must not extend.
 * A true feed result latches status/reading and ignores any subsequent bytes. */
typedef struct {
  uint8_t address;
  uint8_t frame[9];
  size_t used;
  bool complete;
  sen0600_status_t status;
  sen0600_reading_t reading;
} sen0600_stream_t;

void sen0600_stream_init(sen0600_stream_t *stream, uint8_t address);
bool sen0600_stream_feed(sen0600_stream_t *stream, const uint8_t *bytes, size_t length);
