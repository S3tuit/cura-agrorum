#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "node_sensors.h"
#include "node_sensors_ds18b20_gpio.h"
#include "node_sensors_ds18b20_identity.h"
#include "node_sensors_power_gate.h"
#include "onewire_bus.h"
#include "onewire_device.h"
#include "sdkconfig.h"
#include "unity.h"

#define DISCOVERY_CAPACITY 8U

static bool s_hardware_used;

typedef struct {
  uint64_t roms[DISCOVERY_CAPACITY];
  size_t count;
  size_t ds18b20_count;
  esp_err_t result;
  esp_err_t cleanup;
} discovery_t;

static void delay_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms) + 1U); }

static void retain_error(esp_err_t candidate, esp_err_t *first,
                         const char *step) {
  if (candidate != ESP_OK) {
    printf("CARRIER_ERROR step=%s status=%" PRId32 " name=%s\n", step,
           (int32_t)candidate, esp_err_to_name(candidate));
    if (*first == ESP_OK) {
      *first = candidate;
    }
  }
}

static discovery_t discover_roms(void) {
  discovery_t found = {0};
  onewire_bus_handle_t bus = NULL;
  onewire_device_iter_handle_t iterator = NULL;
  s_hardware_used = true;
  found.result = node_sensors_power_gate_on();
  if (found.result != ESP_OK) {
    goto cleanup;
  }
  delay_ms(CONFIG_CURA_SENSOR_POWER_STABILIZATION_MS);
  const onewire_bus_config_t config = {
      .bus_gpio_num = CONFIG_CURA_DS18B20_GPIO,
      .flags = {.en_pull_up = false},
  };
  const onewire_bus_rmt_config_t rmt = {.max_rx_bytes = 10U};
  found.result = onewire_new_bus_rmt(&config, &rmt, &bus);
  if (found.result != ESP_OK) {
    goto cleanup;
  }
  found.result = onewire_new_device_iter(bus, &iterator);
  if (found.result != ESP_OK) {
    goto cleanup;
  }
  /* One extra search detects overflow; neither a truncated nor a repeating
   * enumeration can be reported as a complete inventory. Each driver search
   * uses bounded RMT operations. The host deadline also reports incompletion.
   */
  for (size_t attempt = 0U; attempt <= DISCOVERY_CAPACITY; ++attempt) {
    onewire_device_t device = {0};
    const esp_err_t next = onewire_device_iter_get_next(iterator, &device);
    if (next == ESP_ERR_NOT_FOUND) {
      break;
    }
    if (next != ESP_OK) {
      found.result = next;
      break;
    }
    if (found.count == DISCOVERY_CAPACITY) {
      puts("CARRIER_DISCOVERY overflow");
      found.result = ESP_ERR_INVALID_SIZE;
      break;
    }
    for (size_t previous = 0; previous < found.count; ++previous) {
      if (found.roms[previous] == device.address) {
        puts("CARRIER_DISCOVERY repeated_address");
        found.result = ESP_ERR_INVALID_RESPONSE;
        goto cleanup;
      }
    }
    char canonical[17];
    snprintf(canonical, sizeof(canonical), "%016" PRIX64, device.address);
    const char *const text[2] = {canonical, "0000000000000000"};
    uint64_t parsed[2];
    node_sensors_backend_result_t channels[2];
    if (!node_sensors_ds18b20_resolve_identities(text, parsed, channels) ||
        parsed[0] != device.address || channels[0].status != 0) {
      found.result = ESP_ERR_INVALID_RESPONSE;
      goto cleanup;
    }
    const uint8_t family = (uint8_t)(device.address & UINT64_C(0xff));
    printf("CARRIER_ROM value=%s family=%02x type=%s\n", canonical, family,
           family == 0x28U ? "DS18B20" : "unknown");
    found.roms[found.count++] = device.address;
    if (family == 0x28U) {
      ++found.ds18b20_count;
    }
  }

cleanup:
  if (iterator != NULL) {
    retain_error(onewire_del_device_iter(iterator), &found.cleanup, "iterator");
  }
  if (bus != NULL) {
    retain_error(onewire_bus_del(bus), &found.cleanup, "onewire_bus");
  }
  retain_error(node_sensors_ds18b20_release_gpio(), &found.cleanup,
               "onewire_release");
  retain_error(node_sensors_power_gate_off(), &found.cleanup, "gate_off");
  printf("CARRIER_DISCOVERY count=%u ds18b20=%u status=%" PRId32
         " cleanup=%" PRId32 "\n",
         (unsigned)found.count, (unsigned)found.ds18b20_count,
         (int32_t)found.result, (int32_t)found.cleanup);
  fflush(stdout);
  return found;
}

static esp_err_t identify_bme280(void) {
  s_hardware_used = true;
  /* Identification only: no reset, driver initialization, or conversion.
   * Bosch BME280 datasheet section 5.4.1 specifies D0 -> 60. */
  const i2c_config_t configuration = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = CONFIG_CURA_I2C_SDA_GPIO,
      .scl_io_num = CONFIG_CURA_I2C_SCL_GPIO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = 100000U,
      .clk_flags = 0,
  };
  i2c_bus_handle_t bus = i2c_bus_create(I2C_NUM_0, &configuration);
  i2c_bus_device_handle_t device = NULL;
  uint8_t id = 0;
  esp_err_t result = ESP_FAIL;
  if (bus != NULL) {
    device = i2c_bus_device_create(bus, 0x76U, 100000U);
    if (device != NULL) {
      result = i2c_bus_read_byte(device, 0xd0U, &id);
      if (result == ESP_OK && id != 0x60U) {
        result = ESP_ERR_INVALID_RESPONSE;
      }
    } else {
      result = ESP_ERR_NO_MEM;
    }
  }
  printf("CARRIER_BME address=76 register=D0 id=%02X status=%" PRId32 "\n", id,
         (int32_t)result);
  if (device != NULL) {
    retain_error(i2c_bus_device_delete(&device), &result, "i2c_device");
  }
  if (bus != NULL) {
    retain_error(i2c_bus_delete(&bus), &result, "i2c_bus");
  }
  return result;
}

static void require_configured_roms(uint64_t roms[2]) {
  const char *const text[2] = {CONFIG_CURA_DS18B20_0_ROM,
                               CONFIG_CURA_DS18B20_1_ROM};
  node_sensors_backend_result_t channels[2];
  const bool needs_bus =
      node_sensors_ds18b20_resolve_identities(text, roms, channels);
  TEST_ASSERT_TRUE_MESSAGE(needs_bus && channels[0].status == 0 &&
                               channels[1].status == 0 && roms[0] != 0 &&
                               roms[1] != 0 && roms[0] != roms[1],
                           "configure both distinct physical DS18B20 ROMs");
}

TEST_CASE("carrier setup discovery", "[sensor_carrier]") {
  puts("CARRIER_SETUP discovery_only_no_nominal_acceptance");
  const discovery_t found = discover_roms();
  const esp_err_t bme = identify_bme280();
  TEST_ASSERT_EQUAL(ESP_OK, found.result);
  TEST_ASSERT_EQUAL(ESP_OK, found.cleanup);
  TEST_ASSERT_GREATER_THAN_MESSAGE(0, found.ds18b20_count,
                                   "no DS18B20 found; discovery incomplete");
  TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, bme,
                            "BME280 identification/cleanup failed");
}

TEST_CASE("carrier nominal preflight", "[sensor_carrier]") {
  uint64_t configured[2];
  require_configured_roms(configured);
  const discovery_t found = discover_roms();
  const esp_err_t bme = identify_bme280();
  TEST_ASSERT_EQUAL(ESP_OK, found.result);
  TEST_ASSERT_EQUAL(ESP_OK, found.cleanup);
  TEST_ASSERT_EQUAL_MESSAGE(2, found.count,
                            "nominal requires exactly two ROMs");
  TEST_ASSERT_EQUAL(2, found.ds18b20_count);
  for (size_t channel = 0; channel < 2; ++channel) {
    TEST_ASSERT_TRUE_MESSAGE(configured[channel] == found.roms[0] ||
                                 configured[channel] == found.roms[1],
                             "configured DS18B20 absent or replaced");
  }
  TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, bme,
                            "BME280 identification/cleanup failed");
  puts("CARRIER_PREFLIGHT nominal_complete_resources_released_reset_required");
}

static void observation_hold(const char *name) {
  printf("CARRIER_HOLD_READY %s seconds=60\n", name);
  fflush(stdout);
  delay_ms(60000U);
  printf("CARRIER_HOLD_END %s\n", name);
  fflush(stdout);
}

TEST_CASE("carrier production gate-on hold", "[sensor_carrier]") {
  s_hardware_used = true;
  const esp_err_t on = node_sensors_power_gate_on();
  if (on == ESP_OK) {
    delay_ms(CONFIG_CURA_SENSOR_POWER_STABILIZATION_MS);
    observation_hold("gate-on");
  }
  const esp_err_t off = node_sensors_power_gate_off();
  TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, on, "production gate-on failed");
  TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, off, "production gate-off cleanup failed");
}

TEST_CASE("carrier production gate-off hold", "[sensor_carrier]") {
  s_hardware_used = true;
  const esp_err_t on = node_sensors_power_gate_on();
  if (on == ESP_OK) {
    delay_ms(CONFIG_CURA_SENSOR_POWER_STABILIZATION_MS);
  }
  const esp_err_t off = node_sensors_power_gate_off();
  TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, on, "production gate-on setup failed");
  TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, off, "production gate-off failed");
  observation_hold("gate-off");
}

TEST_CASE("carrier nominal acquisition and sample-return hold",
          "[sensor_carrier]") {
  TEST_ASSERT_FALSE_MESSAGE(
      s_hardware_used, "fresh boot required; no preinitialization allowed");
  uint64_t configured[2];
  require_configured_roms(configured); /* Pure parsing, no hardware access. */
  s_hardware_used = true;
  node_sensor_sample_t sample;
  diagn_context_t diagnostic;
  memset(&diagnostic, 0xa5, sizeof(diagnostic));
  const int64_t start_us = esp_timer_get_time();
  const err_curag_t result = node_sensors_sample_all(&sample, &diagnostic);
  const int64_t elapsed_us = esp_timer_get_time() - start_us;
  printf("CARRIER_SAMPLE result=%08" PRIx32 " duration_us=%" PRId64
         " validity=%02x soil0_mv=%u soil1_mv=%u temp0_centi_c=%d "
         "temp1_centi_c=%d enclosure_centi_c=%d pressure_pa=%" PRIu32
         " humidity_centi_pct=%u\n",
         (uint32_t)result, elapsed_us, sample.validity, sample.soil_0_mv,
         sample.soil_1_mv, sample.soil_temp_0_centi_c,
         sample.soil_temp_1_centi_c, sample.enclosure_centi_c,
         sample.enclosure_pressure_pa, sample.enclosure_humidity_centi_pct);
  printf("CARRIER_DIAGNOSTIC operation=%u schema=%u length=%u context=",
         (unsigned)diagnostic.operation, diagnostic.context_schema,
         diagnostic.context_length);
  for (size_t index = 0; index < sizeof(diagnostic.context); ++index) {
    printf("%02x", diagnostic.context[index]);
  }
  putchar('\n');

  /* No sensor or gate operation follows sampling, including teardown. Keep
   * the observation available even on a failed/partial returned sample. */
  observation_hold("sample-return");
  TEST_ASSERT_EQUAL_HEX32(CURAG_OK, result);
  TEST_ASSERT_EQUAL_HEX8(NODE_SENSOR_VALIDITY_ALL, sample.validity);
  TEST_ASSERT_TRUE_MESSAGE(sample.soil_0_mv >= 2000U &&
                               sample.soil_0_mv <= 2700U,
                           "soil0 outside nominal air-probe range");
  TEST_ASSERT_TRUE_MESSAGE(sample.soil_1_mv >= 2000U &&
                               sample.soil_1_mv <= 2700U,
                           "soil1 outside nominal air-probe range");
  TEST_ASSERT_EQUAL(CURAG_OP_NONE, diagnostic.operation);
  TEST_ASSERT_EQUAL(0, diagnostic.context_schema);
  TEST_ASSERT_EQUAL(0, diagnostic.context_length);
  for (size_t index = 0; index < sizeof(diagnostic.context); ++index) {
    TEST_ASSERT_EQUAL_HEX8(0, diagnostic.context[index]);
  }
}
