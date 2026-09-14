#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef struct fake_i2c_bus *i2c_master_bus_handle_t;
typedef struct fake_i2c_device *i2c_master_dev_handle_t;
#define I2C_NUM_0 0
#define I2C_CLK_SRC_DEFAULT 0
#define I2C_ADDR_BIT_LEN_7 0
typedef struct {
  int i2c_port, sda_io_num, scl_io_num, clk_source;
  uint8_t glitch_ignore_cnt;
  struct {
    bool enable_internal_pullup;
  } flags;
} i2c_master_bus_config_t;
typedef struct {
  int dev_addr_length;
  uint16_t device_address;
  uint32_t scl_speed_hz;
} i2c_device_config_t;
esp_err_t i2c_new_master_bus(const i2c_master_bus_config_t *,
                             i2c_master_bus_handle_t *);
esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t,
                                    const i2c_device_config_t *,
                                    i2c_master_dev_handle_t *);
esp_err_t i2c_del_master_bus(i2c_master_bus_handle_t);
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t, const uint8_t *, size_t,
                              int);
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t, const uint8_t *,
                                      size_t, uint8_t *, size_t, int);
esp_err_t i2c_master_probe(i2c_master_bus_handle_t, uint16_t, int);
esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t);
