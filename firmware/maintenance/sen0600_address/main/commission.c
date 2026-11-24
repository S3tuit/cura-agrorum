#include "commission.h"
#include <stdbool.h>
#include <string.h>

static uint16_t crc16(const uint8_t *data, size_t n) {
  uint16_t crc = 0xffff;
  for (size_t i = 0; i < n; ++i) {
    crc ^= data[i];
    for (unsigned b = 0; b < 8; ++b)
      crc = (uint16_t)((crc >> 1) ^ ((crc & 1) ? 0xa001 : 0));
  }
  return crc;
}

static bool valid_crc(const uint8_t *data, size_t n) {
  return n >= 2 && crc16(data, n - 2) ==
      (uint16_t)(data[n - 2] | (uint16_t)data[n - 1] << 8);
}

static int transaction(sen0600_exchange_t exchange, void *context,
                       uint8_t address, uint8_t function, uint16_t value,
                       uint8_t reply[16]) {
  /* DFRobot's address register is 0x07d0; calibration and baud are untouched. */
  uint8_t request[8] = {address, function, 0x07, 0xd0,
                       (uint8_t)(value >> 8), (uint8_t)value, 0, 0};
  const uint16_t crc = crc16(request, 6);
  request[6] = (uint8_t)crc;
  request[7] = (uint8_t)(crc >> 8);
  return exchange(context, request, reply, 16);
}

/* 0 means silence, 1 means the addressed register read agrees, -1 means
 * traffic/error that must not be mistaken for an unused bus address. */
static int probe(sen0600_exchange_t exchange, void *context, uint8_t address) {
  uint8_t reply[16];
  const int n = transaction(exchange, context, address, 3, 1, reply);
  if (!n) return 0;
  if (n != 7 || !valid_crc(reply, 7) || reply[0] != address ||
      reply[1] != 3 || reply[2] != 2 || reply[3] != 0 || reply[4] != address)
    return -1;
  return 1;
}

sen0600_commission_result_t sen0600_assign_two(sen0600_exchange_t exchange,
                                             void *context) {
  const int two = probe(exchange, context, 2);
  const int one = probe(exchange, context, 1);
  if (one < 0 || two < 0) return SEN0600_BAD_REPLY;
  if (one && two) return SEN0600_CONFLICT;
  if (two) return SEN0600_ALREADY_TWO;
  if (!one) return SEN0600_ABSENT;

  uint8_t reply[16];
  const int n = transaction(exchange, context, 1, 6, 2, reply);
  /* Address changes may acknowledge under the old or new address. Silence
   * leaves the write uncertain; never retry it. Readback resolves that case. */
  if (n && (n != 8 || !valid_crc(reply, 8) ||
      (reply[0] != 1 && reply[0] != 2) || reply[1] != 6 ||
      memcmp(reply + 2, "\x07\xd0\x00\x02", 4)))
    return SEN0600_WRITE_UNCERTAIN;
  const int after_two = probe(exchange, context, 2);
  const int after_one = probe(exchange, context, 1);
  if (after_two != 1 || after_one != 0) return SEN0600_VERIFY_FAILED;
  return SEN0600_ASSIGNED;
}
