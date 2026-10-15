#include <stdio.h>

#include "esp_app_desc.h"
#include "esp_err.h"
#include "esp_mac.h"
#include "sdkconfig.h"
#include "unity_test_runner.h"

/* Fail the build, but leave menuconfig usable to repair a stale sdkconfig. */
#if !CONFIG_IDF_TARGET_ESP32C6 || CONFIG_CURA_SOIL_0_GPIO != 0 ||              \
    CONFIG_CURA_SOIL_1_GPIO != 1 || CONFIG_CURA_SENSOR_POWER_GATE_GPIO != 2 || \
    CONFIG_CURA_DS18B20_GPIO != 3 || CONFIG_CURA_I2C_SDA_GPIO != 21 ||         \
    CONFIG_CURA_I2C_SCL_GPIO != 22 ||                                          \
    CONFIG_CURA_SENSOR_POWER_STABILIZATION_MS != 200
#error                                                                         \
    "Sensor carrier sdkconfig mismatch: use menuconfig to restore GPIOs 0/1/2/3/21/22 and stabilization 200 ms"
#endif

#if !defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) ||                               \
    CONFIG_ESP_CONSOLE_UART_NUM != 0 ||                                        \
    CONFIG_ESP_CONSOLE_UART_BAUDRATE != 115200
#error "Sensor carrier requires the default UART0 console at 115200 baud"
#endif

void app_main(void) {
  uint8_t mac[6];
  char elf_sha256[65];
  /* The default efuse API returns eight EUI-64 bytes on the C6. Request the
   * explicitly six-byte factory MAC used to identify this UART DUT. */
  ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_EFUSE_FACTORY));
  /* The convenience string API truncates to CONFIG_APP_RETRIEVE_LEN_ELF_SHA.
   * Identity matching needs every hash byte, independent of log settings. */
  const esp_app_desc_t *description = esp_app_get_description();
  for (size_t index = 0; index < sizeof(description->app_elf_sha256); ++index) {
    snprintf(&elf_sha256[index * 2U], 3U, "%02x",
             description->app_elf_sha256[index]);
  }
  printf("CARRIER_BOOT dut=%02x%02x%02x%02x%02x%02x elf=%s "
         "soil0=%d soil1=%d gate=%d onewire=%d sda=%d scl=%d "
         "stabilization_ms=%d rom0=%s rom1=%s\n",
         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], elf_sha256,
         CONFIG_CURA_SOIL_0_GPIO, CONFIG_CURA_SOIL_1_GPIO,
         CONFIG_CURA_SENSOR_POWER_GATE_GPIO, CONFIG_CURA_DS18B20_GPIO,
         CONFIG_CURA_I2C_SDA_GPIO, CONFIG_CURA_I2C_SCL_GPIO,
         CONFIG_CURA_SENSOR_POWER_STABILIZATION_MS, CONFIG_CURA_DS18B20_0_ROM,
         CONFIG_CURA_DS18B20_1_ROM);
  fflush(stdout);
  unity_run_menu();
}
