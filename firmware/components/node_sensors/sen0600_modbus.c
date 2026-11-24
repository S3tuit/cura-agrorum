#include "sen0600_modbus.h"
#include <limits.h>

static uint16_t crc16(const uint8_t *bytes, size_t length) {
  uint16_t crc = 0xffff;
  for (size_t i = 0; i < length; ++i) {
    crc ^= bytes[i];
    for (unsigned bit = 0; bit < 8; ++bit)
      crc = (crc & 1U) ? (uint16_t)((crc >> 1) ^ 0xa001U) : (uint16_t)(crc >> 1);
  }
  return crc;
}

void sen0600_request(uint8_t address, uint8_t request[8]) {
  const uint8_t command[6] = {address, 3, 0, 0, 0, 2};
  for (size_t i = 0; i < 6; ++i) request[i] = command[i];
  const uint16_t crc = crc16(command, 6);
  request[6] = (uint8_t)crc;
  request[7] = (uint8_t)(crc >> 8);
}

sen0600_status_t sen0600_parse(uint8_t address, const uint8_t *reply,
                              size_t length, sen0600_reading_t *reading) {
  if (!reading) return SEN0600_BAD_LENGTH;
  *reading = (sen0600_reading_t){0};
  if (!reply || (length != 5 && length != 9)) return SEN0600_BAD_LENGTH;
  const uint16_t received = (uint16_t)((uint16_t)reply[length - 2] |
                                     (uint16_t)((uint16_t)reply[length - 1] << 8));
  if (crc16(reply, length - 2) != received) return SEN0600_BAD_CRC;
  if (reply[0] != address) return SEN0600_BAD_ADDRESS;
  if (reply[1] == 0x83 && length == 5) {
    reading->exception = reply[2];
    return SEN0600_EXCEPTION;
  }
  if (reply[1] != 3) return SEN0600_BAD_FUNCTION;
  if (length != 9 || reply[2] != 4) return SEN0600_BAD_LENGTH;
  reading->moisture_deci_pct = (uint16_t)((uint16_t)reply[3] << 8 | reply[4]);
  const uint16_t encoded = (uint16_t)((uint16_t)reply[5] << 8 | reply[6]);
  const int32_t tenths = encoded >= 0x8000U ? (int32_t)encoded - 65536 : encoded;
  const int32_t centi = tenths * 10;
  /* Successful acquisition does not certify physical plausibility. Preserve
   * moisture even if conversion of the separate temperature cannot fit. */
  reading->temperature_valid = centi >= INT16_MIN && centi <= INT16_MAX;
  if (reading->temperature_valid) reading->temperature_centi_c = (int16_t)centi;
  return SEN0600_OK;
}

void sen0600_stream_init(sen0600_stream_t *stream, uint8_t address) {
  *stream = (sen0600_stream_t){.address = address};
}

bool sen0600_stream_feed(sen0600_stream_t *stream, const uint8_t *bytes, size_t length) {
  for (size_t i = 0; i < length && !stream->complete; ++i) {
    stream->frame[stream->used++] = bytes[i];
    if (stream->used < 2) continue;
    const size_t frame_length = stream->frame[1] == 0x83 ? 5U : sizeof(stream->frame);
    if (stream->used < frame_length) continue;

    stream->status = sen0600_parse(stream->address, stream->frame, frame_length,
                                   &stream->reading);
    if (stream->status == SEN0600_BAD_ADDRESS) {
      /* BAD_ADDRESS certifies CRC but precedes function/length validation.
       * Only a known complete response gives us a safe boundary to skip. */
      sen0600_reading_t foreign;
      const sen0600_status_t checked = sen0600_parse(stream->frame[0], stream->frame,
                                                    frame_length, &foreign);
      if (checked == SEN0600_OK || checked == SEN0600_EXCEPTION) {
        stream->used = 0;
        continue;
      }
    }
    stream->complete = true;
  }
  return stream->complete;
}
