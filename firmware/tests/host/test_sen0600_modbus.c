#include "sen0600_modbus.h"
#include <assert.h>
#include <string.h>

int main(void) {
  const uint8_t expected[8] = {1,3,0,0,0,2,0xc4,0x0b};
  uint8_t request[8];
  sen0600_request(1, request);
  assert(memcmp(request, expected, 8) == 0);
  /* Fixed wire vectors include physical outliers, zero and the representable
   * temperature boundaries. Expected values do not come from the parser. */
  const struct { uint8_t wire[9]; uint16_t moisture; int16_t temp; bool valid; } cases[] = {
    {{0x01, 0x03, 0x04, 0x01, 0xe6, 0xff, 0x9f, 0x1b, 0xa0}, 486, -970, true},
    {{0x01, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00, 0xfa, 0x33}, 0, 0, true},
    {{0x01, 0x03, 0x04, 0xff, 0xff, 0x0c, 0xcc, 0xff, 0x42}, 65535, 32760, true},
    {{0x01, 0x03, 0x04, 0x03, 0xe9, 0xf3, 0x34, 0x6e, 0xa4}, 1001, -32760, true},
    {{0x01, 0x03, 0x04, 0x01, 0xf4, 0x0c, 0xcd, 0x7e, 0xa8}, 500, 0, false},
    {{0x01, 0x03, 0x04, 0x01, 0xf4, 0xf3, 0x33, 0xbe, 0xd8}, 500, 0, false},
  };
  for (size_t i = 0; i < sizeof(cases)/sizeof(cases[0]); ++i) {
    sen0600_reading_t reading;
    assert(sen0600_parse(1, cases[i].wire, 9, &reading) == SEN0600_OK);
    assert(reading.moisture_deci_pct == cases[i].moisture);
    assert(reading.temperature_valid == cases[i].valid);
    assert(reading.temperature_centi_c == cases[i].temp);
    assert(sen0600_parse(2, cases[i].wire, 9, &reading) == SEN0600_BAD_ADDRESS);
    for (size_t j = 0; j < 9; ++j) {
      uint8_t broken[9]; memcpy(broken, cases[i].wire, 9); broken[j] ^= 1;
      assert(sen0600_parse(1, broken, 9, &reading) == SEN0600_BAD_CRC);
    }
  }
  const uint8_t exception[5] = {1,0x83,2,0xc0,0xf1};
  sen0600_reading_t reading;
  assert(sen0600_parse(1, exception, 5, &reading) == SEN0600_EXCEPTION);
  assert(reading.exception == 2);
  assert(sen0600_parse(1, NULL, 9, &reading) == SEN0600_BAD_LENGTH);
  assert(sen0600_parse(1, expected, 8, &reading) == SEN0600_BAD_LENGTH);
  return 0;
}
