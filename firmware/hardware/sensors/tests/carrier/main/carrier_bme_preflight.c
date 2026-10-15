#include "carrier_bme_preflight.h"
#include "driver/i2c_master.h"
#include "sdkconfig.h"
#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>

esp_err_t carrier_bme_preflight(bool missing) {
  /* Identification only: no reset, driver initialization, or conversion.
   * Bosch BME280 datasheet section 5.4.1 specifies D0 -> 60. */
  const i2c_master_bus_config_t configuration = {
      .i2c_port = I2C_NUM_0,
      .sda_io_num = CONFIG_CURA_I2C_SDA_GPIO,
      .scl_io_num = CONFIG_CURA_I2C_SCL_GPIO,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags = {.enable_internal_pullup = true},
  };
  const i2c_device_config_t device_configuration = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = 0x76,
      .scl_speed_hz = 100000,
  };
  i2c_master_bus_handle_t bus = NULL;
  i2c_master_dev_handle_t device = NULL;
  uint8_t id = 0;
  esp_err_t result = i2c_new_master_bus(&configuration, &bus);
  if (result == ESP_OK) {
    if (missing) {
      const esp_err_t probe = i2c_master_probe(bus, 0x76, 20);
      printf("CARRIER_BME_ABSENT address=76 probe=%" PRId32 "\n",
             (int32_t)probe);
      /* Only a NACK-specific probe result establishes this fixture. */
      result = probe == ESP_ERR_NOT_FOUND ? ESP_OK
               : probe == ESP_OK          ? ESP_ERR_INVALID_RESPONSE
                                          : probe;
    } else {
      result = i2c_master_bus_add_device(bus, &device_configuration, &device);
      if (result == ESP_OK) {
        const uint8_t reg = 0xd0;
        result = i2c_master_transmit_receive(device, &reg, 1, &id, 1, 20);
        if (result == ESP_OK && id != 0x60)
          result = ESP_ERR_INVALID_RESPONSE;
      }
    }
  }
  if (!missing)
    printf("CARRIER_BME address=76 register=D0 id=%02X status=%" PRId32 "\n", id,
           (int32_t)result);
  esp_err_t cleanup = ESP_OK;
  if (device != NULL)
    cleanup = i2c_master_bus_rm_device(device);
  if (bus != NULL) {
    const esp_err_t release = i2c_del_master_bus(bus);
    if (cleanup == ESP_OK)
      cleanup = release;
  }
  printf("CARRIER_BME_CLEANUP status=%" PRId32 "\n", (int32_t)cleanup);
  if (result == ESP_OK)
    result = cleanup;
  return result;
}
