#include "bme280.h"
#include "bme280_test_data.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint8_t registers[256];
static unsigned polls, fail_poll, busy_polls, reads, writes, data_reads, delays;
static BME280_INTF_RET_TYPE read_regs(uint8_t reg, uint8_t *data, uint32_t len,
                                      void *ctx) {
  (void)ctx;
  assert(data && len && (unsigned)reg + len <= 256);
  ++reads;
  if (reg == 0xf3) {
    assert(len == 1);
    ++polls;
    if (polls == fail_poll)
      return -27;
    registers[reg] = polls <= busy_polls ? 1 : 0;
  }
  if (reg == 0xf7) {
    assert(len == 8);
    ++data_reads;
  }
  memcpy(data, registers + reg, len);
  return 0;
}
static BME280_INTF_RET_TYPE write_regs(uint8_t reg, const uint8_t *data,
                                       uint32_t len, void *ctx) {
  (void)ctx;
  assert(data && len);
  ++writes;
  for (uint32_t i = 0; i < len; i += 2) {
    registers[reg] = data[i];
    if (i + 1 < len)
      reg = data[i + 1];
  }
  return 0;
}
static void delay(uint32_t us, void *ctx) {
  (void)ctx;
  assert(us == 2000);
  ++delays;
}
int main(int argc, char **argv) {
  assert(argc == 2);
  bme_test_registers(registers);
  struct bme280_dev dev = {.intf = BME280_I2C_INTF,
                           .read = read_regs,
                           .write = write_regs,
                           .delay_us = delay};
  const char *test = argv[1];
  if (!strcmp(test, "nvm_first_error") || !strcmp(test, "nvm_later_error")) {
    fail_poll = !strcmp(test, "nvm_first_error") ? 1U : 3U;
    busy_polls = 6;
    assert(bme280_init(&dev) == BME280_E_COMM_FAIL);
    assert(dev.intf_rslt == -27 && polls == fail_poll && delays == polls);
  } else if (!strcmp(test, "nvm_bound")) {
    busy_polls = 100;
    assert(bme280_init(&dev) == BME280_E_NVM_COPY_FAILED);
    assert(polls == 6 && delays == 6);
  } else if (!strcmp(test, "wrong_id")) {
    registers[0xd0] = 0x58;
    assert(bme280_init(&dev) == BME280_E_DEV_NOT_FOUND);
    assert(reads == 1 && writes == 0 && delays == 0);
  } else if (!strcmp(test, "signed_calibration")) {
    registers[0xe4] = 0xff;
    registers[0xe5] = 0x1e;
    registers[0xe6] = 0x80;
    registers[0xe7] = 0xff;
    assert(bme280_init(&dev) == 0);
    assert(dev.calib_data.dig_h4 == -2 && dev.calib_data.dig_h5 == -2047 &&
           dev.calib_data.dig_h6 == -1);
  } else {
    assert(bme280_init(&dev) == 0);
    assert(polls == 1 && delays == 1);
    if (!strcmp(test, "temperature_outlier"))
      bme_test_raw20(registers + 0xfa, 640000);
    if (!strcmp(test, "pressure_outlier"))
      bme_test_raw20(registers + 0xf7, 0);
    if (!strcmp(test, "invalid_pressure"))
      dev.calib_data.dig_p1 = 0;
    if (!strcmp(test, "humidity_saturation"))
      registers[0xfd] = 2;
    struct bme280_data data;
    assert(bme280_get_sensor_data(BME280_ALL, &data, &dev) == 0);
    assert(data_reads == 1);
    assert(data.temperature ==
           (!strcmp(test, "temperature_outlier") ? 125.0 : 16.0));
    if (!strcmp(test, "invalid_pressure"))
      assert(isnan(data.pressure));
    else
      assert(data.pressure ==
             (!strcmp(test, "pressure_outlier") ? 160000.0 : 100000.0));
    assert(data.humidity ==
           (!strcmp(test, "humidity_saturation") ? 100.0 : 50.0));
  }
  puts(test);
  return 0;
}
