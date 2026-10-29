#include "carrier_output.h"
#include "usb_console_test_support.h"
#include <stdio.h>

#include "esp_app_desc.h"
#include "esp_err.h"
#include "esp_mac.h"
#include "sdkconfig.h"
#include "unity_test_runner.h"

/* Fail the build, but leave menuconfig usable to repair a stale sdkconfig. */
#if !CONFIG_IDF_TARGET_ESP32S3 || CONFIG_CURA_SOIL_0_GPIO != 1 ||              \
    CONFIG_CURA_SOIL_1_GPIO != 2 || CONFIG_CURA_SENSOR_POWER_GATE_GPIO != 44 || \
    CONFIG_CURA_DS18B20_GPIO != 4 || CONFIG_CURA_I2C_SDA_GPIO != 5 ||         \
    CONFIG_CURA_I2C_SCL_GPIO != 6 ||                                          \
    CONFIG_CURA_SENSOR_POWER_STABILIZATION_MS != 200
#error                                                                         \
    "Sensor carrier sdkconfig mismatch: use menuconfig to restore GPIOs 1/2/44/4/5/6 and stabilization 200 ms"
#endif

#if !defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
#error "This S3 assembly requires the native USB Serial/JTAG console"
#endif

void app_main(void) {
  if (!hwtest_usb_console_ready()) return;
  uint8_t mac[6];
  char elf_sha256[65];
  /* Use the six-byte factory MAC to bind USB boot records to this DUT. */
  ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_EFUSE_FACTORY));
  /* The convenience string API truncates to CONFIG_APP_RETRIEVE_LEN_ELF_SHA.
   * Identity matching needs every hash byte, independent of log settings. */
  const esp_app_desc_t *description = esp_app_get_description();
  for (size_t index = 0; index < sizeof(description->app_elf_sha256); ++index) {
    snprintf(&elf_sha256[index * 2U], 3U, "%02x",
             description->app_elf_sha256[index]);
  }
  carrier_output_printf("CARRIER_BOOT dut=%02x%02x%02x%02x%02x%02x elf=%s "
         "soil0=%d soil1=%d gate=%d onewire=%d sda=%d scl=%d "
         "stabilization_ms=%d rom0=%s rom1=%s\n",
         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], elf_sha256,
         CONFIG_CURA_SOIL_0_GPIO, CONFIG_CURA_SOIL_1_GPIO,
         CONFIG_CURA_SENSOR_POWER_GATE_GPIO, CONFIG_CURA_DS18B20_GPIO,
         CONFIG_CURA_I2C_SDA_GPIO, CONFIG_CURA_I2C_SCL_GPIO,
         CONFIG_CURA_SENSOR_POWER_STABILIZATION_MS, CONFIG_CURA_DS18B20_0_ROM,
         CONFIG_CURA_DS18B20_1_ROM);
  unity_run_menu();
}
