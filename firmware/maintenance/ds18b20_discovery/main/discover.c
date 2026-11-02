#include <inttypes.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "onewire_bus.h"
#include "onewire_device.h"

static bool valid_rom(uint64_t rom) {
  uint8_t crc = 0;
  for (unsigned byte = 0; byte < 8; ++byte) {
    uint8_t value = (uint8_t)(rom >> (8 * byte));
    for (unsigned bit = 0; bit < 8; ++bit) {
      const bool mix = ((crc ^ value) & 1U) != 0;
      crc >>= 1;
      if (mix) crc ^= 0x8c;
      value >>= 1;
    }
  }
  return (uint8_t)rom == 0x28 && crc == 0;
}

static void first_error(esp_err_t next, esp_err_t *status) {
  if (*status == ESP_OK) *status = next;
}

void app_main(void) {
  onewire_bus_handle_t bus = NULL;
  onewire_device_iter_handle_t iterator = NULL;
  unsigned count = 0;
  esp_err_t status = gpio_set_level(GPIO_NUM_44, 1);
  const gpio_config_t gate = {
      .pin_bit_mask = 1ULL << 44, .mode = GPIO_MODE_OUTPUT_OD,
      .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE,
  };
  first_error(gpio_config(&gate), &status);
  if (status != ESP_OK) goto cleanup;
  status = gpio_set_level(GPIO_NUM_44, 0);
  if (status != ESP_OK) goto cleanup;
  vTaskDelay(pdMS_TO_TICKS(200));
  const onewire_bus_config_t config = {
      .bus_gpio_num = 4, .flags = {.en_pull_up = false},
  };
  const onewire_bus_rmt_config_t rmt = {.max_rx_bytes = 10};
  status = onewire_new_bus_rmt(&config, &rmt, &bus);
  if (status != ESP_OK) goto cleanup;
  status = onewire_new_device_iter(bus, &iterator);
  if (status != ESP_OK) goto cleanup;
  for (;;) {
    onewire_device_t device = {0};
    status = onewire_device_iter_get_next(iterator, &device);
    if (status == ESP_ERR_NOT_FOUND) {
      status = count > 0 ? ESP_OK : ESP_ERR_NOT_FOUND;
      break;
    }
    if (status != ESP_OK) break;
    if (!valid_rom(device.address) || count == 2) {
      status = ESP_ERR_INVALID_RESPONSE;
      break;
    }
    ++count;
    printf("DS18B20_ROM=%016" PRIX64 "\n", device.address);
  }
cleanup:
  if (iterator) first_error(onewire_del_device_iter(iterator), &status);
  if (bus) first_error(onewire_bus_del(bus), &status);
  first_error(gpio_set_direction(GPIO_NUM_4, GPIO_MODE_INPUT), &status);
  first_error(gpio_set_pull_mode(GPIO_NUM_4, GPIO_FLOATING), &status);
  first_error(gpio_set_level(GPIO_NUM_44, 1), &status);
  first_error(gpio_set_direction(GPIO_NUM_44, GPIO_MODE_INPUT), &status);
  first_error(gpio_set_pull_mode(GPIO_NUM_44, GPIO_FLOATING), &status);
  printf("DS18B20_DISCOVERY count=%u status=%s\n", count, esp_err_to_name(status));
}
