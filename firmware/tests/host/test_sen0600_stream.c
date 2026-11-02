#include "sen0600_modbus.h"
#include <assert.h>
#include <string.h>

static const uint8_t foreign[] = {1, 3, 4, 1, 0xe6, 0xff, 0x9f, 0x1b, 0xa0};
static const uint8_t current[] = {2, 3, 4, 2, 0x8a, 0, 0xd7, 0xa9, 0x3f};
static const uint8_t foreign_exception[] = {1, 0x83, 2, 0xc0, 0xf1};
static const uint8_t current_exception[] = {2, 0x83, 2, 0x30, 0xf1};

static void check_result(const sen0600_stream_t *stream, sen0600_status_t status) {
  assert(stream->complete && stream->status == status);
  if (status == SEN0600_OK) {
    assert(stream->reading.moisture_deci_pct == 650);
    assert(stream->reading.temperature_valid && stream->reading.temperature_centi_c == 2150);
    assert(stream->reading.exception == 0);
  } else {
    assert(stream->reading.moisture_deci_pct == 0);
    assert(!stream->reading.temperature_valid && stream->reading.temperature_centi_c == 0);
    assert(stream->reading.exception == (status == SEN0600_EXCEPTION ? 2 : 0));
  }
}

/* Exercise every pair of chunk boundaries, including empty feeds and chunks
 * containing the end of a foreign reply plus the start of the addressed reply.
 * Fixed CRC vectors and different measurements expose accidental stale reuse. */
static void check_partitions(const uint8_t *wire, size_t length, size_t terminal_at,
                             sen0600_status_t status) {
  for (size_t first = 0; first <= length; ++first) {
    for (size_t second = first; second <= length; ++second) {
      sen0600_stream_t stream;
      sen0600_stream_init(&stream, 2);
      assert(sen0600_stream_feed(&stream, wire, first) == (first >= terminal_at));
      assert(sen0600_stream_feed(&stream, wire + first, second - first) == (second >= terminal_at));
      assert(sen0600_stream_feed(&stream, wire + second, length - second));
      check_result(&stream, status);
      /* A later valid frame cannot overwrite a latched error or measurement. */
      assert(sen0600_stream_feed(&stream, current, sizeof(current)));
      check_result(&stream, status);
    }
  }
  sen0600_stream_t stream;
  sen0600_stream_init(&stream, 2);
  for (size_t i = 0; i < length; ++i)
    assert(sen0600_stream_feed(&stream, wire + i, 1) == (i + 1 >= terminal_at));
  check_result(&stream, status);
}

int main(void) {
  check_partitions(current, sizeof(current), sizeof(current), SEN0600_OK);
  check_partitions(current_exception, sizeof(current_exception), sizeof(current_exception), SEN0600_EXCEPTION);

  uint8_t wire[32];
  memcpy(wire, foreign, sizeof(foreign));
  memcpy(wire + 9, current, sizeof(current));
  check_partitions(wire, 18, 18, SEN0600_OK);
  memcpy(wire, foreign_exception, sizeof(foreign_exception));
  memcpy(wire + 5, current, sizeof(current));
  check_partitions(wire, 14, 14, SEN0600_OK);
  memcpy(wire, foreign, sizeof(foreign));
  memcpy(wire + 9, foreign_exception, sizeof(foreign_exception));
  memcpy(wire + 14, current, sizeof(current));
  memcpy(wire + 23, foreign, sizeof(foreign));
  check_partitions(wire, 32, 23, SEN0600_OK);
  memcpy(wire + 14, current_exception, sizeof(current_exception));
  check_partitions(wire, 19, 19, SEN0600_EXCEPTION);

  /* Foreign traffic can exceed any UART buffer while decoder storage remains
   * bounded. It never completes a transaction or publishes stale measurements. */
  sen0600_stream_t stream;
  sen0600_stream_init(&stream, 2);
  assert(!sen0600_stream_feed(&stream, NULL, 0));
  for (unsigned i = 0; i < 10000; ++i) {
    assert(!sen0600_stream_feed(&stream, foreign, sizeof(foreign)));
    assert(!sen0600_stream_feed(&stream, foreign_exception, sizeof(foreign_exception)));
    assert(stream.used == 0 && stream.reading.moisture_deci_pct == 0);
  }
  assert(sen0600_stream_feed(&stream, current, sizeof(current)));
  check_result(&stream, SEN0600_OK);

  for (size_t prefix = 0; prefix < sizeof(current); ++prefix) {
    sen0600_stream_init(&stream, 2);
    assert(!sen0600_stream_feed(&stream, current, prefix));
    assert(stream.reading.moisture_deci_pct == 0 && !stream.reading.temperature_valid);
  }

  /* Corrupt or unsupported frames are not silently skipped, even if followed
   * by a good reply. CRC-valid foreign addresses alone do not certify framing. */
  const uint8_t bad_function[] = {1, 4, 4, 1, 0xe6, 0xff, 0x9f, 0x1a, 0x17};
  const uint8_t bad_length[] = {1, 3, 3, 1, 0xe6, 0xff, 0x9f, 0xae, 0x60};
  memcpy(wire, bad_function, 9);
  memcpy(wire + 9, current, 9);
  check_partitions(wire, 18, 9, SEN0600_BAD_ADDRESS);
  memcpy(wire, bad_length, 9);
  check_partitions(wire, 18, 9, SEN0600_BAD_ADDRESS);
  memcpy(wire, foreign, 9);
  wire[8] ^= 1;
  check_partitions(wire, 18, 9, SEN0600_BAD_CRC);
  memcpy(wire, current, 9);
  wire[8] ^= 1;
  check_partitions(wire, 18, 9, SEN0600_BAD_CRC);
  return 0;
}
