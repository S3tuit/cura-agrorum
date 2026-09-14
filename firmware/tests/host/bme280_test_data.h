#pragma once
#include <stdint.h>
#include <string.h>

/* Synthetic independent vectors with closed-form datasheet 4.2/Appendix A
 * results: T1=T3=0,T2=16384 => t_fine=raw_T, T=raw_T/5120 C.
 * P1=40960, other P coefficients=0 => P=(1048576-raw_P)*6250/40960 Pa.
 * H2=16384, other H coefficients=0 => H=raw_H/4 percent, saturated 0..100.
 * raw_T=81920, raw_P=393216, raw_H=200 => exactly 16 C, 100000 Pa, 50%.
 * No expected value is sampled from the driver implementation's output.
 */
static inline void bme_test_u16(uint8_t *bytes, uint16_t value) {
  bytes[0] = (uint8_t)value;
  bytes[1] = (uint8_t)(value >> 8);
}
static inline void bme_test_raw20(uint8_t *bytes, uint32_t value) {
  bytes[0] = (uint8_t)(value >> 12);
  bytes[1] = (uint8_t)(value >> 4);
  bytes[2] = (uint8_t)(value << 4);
}
static inline void bme_test_registers(uint8_t registers[256]) {
  memset(registers, 0, 256);
  registers[0xd0] = 0x60;
  bme_test_u16(registers + 0x8a, 16384);
  bme_test_u16(registers + 0x8e, 40960);
  bme_test_u16(registers + 0xe1, 16384);
  bme_test_raw20(registers + 0xf7, 393216);
  bme_test_raw20(registers + 0xfa, 81920);
  registers[0xfd] = 0;
  registers[0xfe] = 200;
}
