#include <inttypes.h>
#include <errno.h>
#include <stdlib.h>
#include "carrier_observer.h"
#include "carrier_hold.h"
#include "node_platform_esp.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_sleep.h"
#include "esp_system.h"
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
  TEST_ASSERT_TRUE(carrier_hold_wait(name, false));
}

TEST_CASE("carrier production gate-on hold", "[sensor_carrier]") {
  const int mode = carrier_hold_select();
  TEST_ASSERT_TRUE_MESSAGE(mode >= 0, "missing/invalid hold mode");
  bool held = false;
  s_hardware_used = true;
  const esp_err_t on = node_sensors_power_gate_on();
  if (on == ESP_OK) {
    delay_ms(CONFIG_CURA_SENSOR_POWER_STABILIZATION_MS);
    held = carrier_hold_wait("gate-on", mode);
  }
  const esp_err_t off = node_sensors_power_gate_off();
  TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, on, "production gate-on failed");
  TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, off, "production gate-off cleanup failed");
  TEST_ASSERT_TRUE_MESSAGE(held, "guided hold incomplete: valid acknowledgement required");
}

TEST_CASE("carrier production gate-off hold", "[sensor_carrier]") {
  const int mode = carrier_hold_select();
  TEST_ASSERT_TRUE_MESSAGE(mode >= 0, "missing/invalid hold mode");
  s_hardware_used = true;
  const esp_err_t on = node_sensors_power_gate_on();
  if (on == ESP_OK) {
    delay_ms(CONFIG_CURA_SENSOR_POWER_STABILIZATION_MS);
  }
  const esp_err_t off = node_sensors_power_gate_off();
  TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, on, "production gate-on setup failed");
  TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, off, "production gate-off failed");
  TEST_ASSERT_TRUE_MESSAGE(carrier_hold_wait("gate-off", mode),
                           "guided hold incomplete: valid acknowledgement required");
}

static void print_sample(node_sensor_sample_t sample, diagn_context_t diagnostic,
                         err_curag_t result, int64_t elapsed_us) {
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

}

static void assert_sample(const node_sensor_sample_t *sample,
                          const diagn_context_t *diagnostic, err_curag_t result,
                          bool nominal) {
  TEST_ASSERT_EQUAL_HEX32(CURAG_OK, result);
  TEST_ASSERT_EQUAL_HEX8(NODE_SENSOR_VALIDITY_ALL, sample->validity);
  if (nominal) {
  TEST_ASSERT_TRUE_MESSAGE(sample->soil_0_mv >= 2000U &&
                               sample->soil_0_mv <= 2700U,
                           "soil0 outside nominal air-probe range");
  TEST_ASSERT_TRUE_MESSAGE(sample->soil_1_mv >= 2000U &&
                               sample->soil_1_mv <= 2700U,
                           "soil1 outside nominal air-probe range");
  }
  TEST_ASSERT_EQUAL(CURAG_OP_NONE, diagnostic->operation);
  TEST_ASSERT_EQUAL(0, diagnostic->context_schema);
  TEST_ASSERT_EQUAL(0, diagnostic->context_length);
  for (size_t index = 0; index < sizeof(diagnostic->context); ++index) {
    TEST_ASSERT_EQUAL_HEX8(0, diagnostic->context[index]);
  }
}

static void fresh_acquisition(uint64_t configured[2]) {
  TEST_ASSERT_FALSE_MESSAGE(s_hardware_used,
                           "fresh boot required; no preinitialization allowed");
  require_configured_roms(configured); /* Pure parsing; no hardware access. */
  s_hardware_used = true;
}

static err_curag_t acquire_sample(node_sensor_sample_t *sample,
                                 diagn_context_t *diagnostic) {
  memset(diagnostic, 0xa5, sizeof(*diagnostic));
  const int64_t start_us = esp_timer_get_time();
  const err_curag_t result = node_sensors_sample_all(sample, diagnostic);
  print_sample(*sample, *diagnostic, result, esp_timer_get_time() - start_us);
  return result;
}

TEST_CASE("carrier nominal acquisition and sample-return hold", "[sensor_carrier]") {
  uint64_t configured[2];
  fresh_acquisition(configured);
  const int mode = carrier_hold_select();
  TEST_ASSERT_TRUE_MESSAGE(mode >= 0, "missing/invalid hold mode");
  node_sensor_sample_t sample;
  diagn_context_t diagnostic;
  const err_curag_t result = acquire_sample(&sample, &diagnostic);
  /* No sensor/gate operation after return, including on a failed sample. */
  TEST_ASSERT_TRUE_MESSAGE(carrier_hold_wait("sample-return", mode),
                           "guided hold incomplete: valid acknowledgement required");
  assert_sample(&sample, &diagnostic, result, true);
}

TEST_CASE("carrier repeated nominal acquisition", "[sensor_carrier]") {
  uint64_t configured[2];
  fresh_acquisition(configured);
  puts("CARRIER_REPEAT_COUNT");
  fflush(stdout);
  char text[24];
  unity_gets(text, sizeof(text));
  char *end = NULL;
  errno = 0;
  unsigned long long requested = strtoull(text, &end, 10);
  TEST_ASSERT_TRUE_MESSAGE(errno == 0 && end != text && *end == '\0' &&
                               text[0] != '-' && requested >= 100 && requested <= UINT32_MAX,
                           "complete requested repetition count must be >=100");
  for (uint64_t index = 1; index <= requested; ++index) {
    node_sensor_sample_t sample;
    diagn_context_t diagnostic;
    carrier_observer_begin(configured[0], configured[1]);
    const err_curag_t result = acquire_sample(&sample, &diagnostic);
    assert_sample(&sample, &diagnostic, result, true);
    const carrier_observation_t seen = carrier_observer_snapshot();
    printf("CARRIER_OBSERVER conversions=%u reads=%u wait_us=%" PRId64
           " adc=%u/%u onewire=%u/%u ds=%u/%u bme=%u invalid=%u\n",
           seen.conversions, seen.reads, seen.minimum_wait_us,
           seen.adc_new, seen.adc_del, seen.bus_new, seen.bus_del,
           seen.ds_new, seen.ds_del, seen.bme_forced, seen.invalid_sequence);
    TEST_ASSERT_TRUE_MESSAGE(carrier_observer_sample_valid(),
                             "fresh conversion/resource observer failed");
    printf("CARRIER_ITERATION index=%" PRIu64 " total=%llu\n", index, requested);
    fflush(stdout);
  }
  printf("CARRIER_REPEAT_DONE total=%llu\n", requested);
  fflush(stdout);
}

TEST_CASE("carrier final cleanup hold", "[sensor_carrier]") {
  uint64_t configured[2];
  fresh_acquisition(configured);
  const int mode = carrier_hold_select();
  TEST_ASSERT_TRUE_MESSAGE(mode >= 0, "missing/invalid hold mode");
  node_sensor_sample_t sample, retained;
  diagn_context_t diagnostic, cleanup[2];
  const err_curag_t result = acquire_sample(&sample, &diagnostic);
  assert_sample(&sample, &diagnostic, result, true);
  memcpy(&retained, &sample, sizeof(sample));
  carrier_observer_begin(configured[0], configured[1]);
  err_curag_t results[2];
  bool unchanged[2];
  for (size_t i = 0; i < 2; ++i) {
    memset(&cleanup[i], 0xa5, sizeof(cleanup[i]));
    results[i] = node_sensors_force_power_off(&cleanup[i]);
    unchanged[i] = memcmp(&sample, &retained, sizeof(sample)) == 0;
    printf("CARRIER_CLEANUP index=%u result=%08" PRIx32 " unchanged=%u\n",
           (unsigned)i + 1, (uint32_t)results[i], unchanged[i]);
  }
  const bool observed = carrier_observer_cleanup_valid(2);
  printf("CARRIER_CLEANUP_OBSERVER valid=%u\n", observed);
  /* No further sensor/gate calls; this is independent of sample-return. */
  TEST_ASSERT_TRUE_MESSAGE(carrier_hold_wait("final-cleanup", mode),
                           "guided hold incomplete: valid acknowledgement required");
  for (size_t i = 0; i < 2; ++i) {
    TEST_ASSERT_TRUE(unchanged[i]);
    assert_sample(&sample, &cleanup[i], results[i], true);
  }
  TEST_ASSERT_TRUE_MESSAGE(observed, "cleanup initialized a bus or enabled the rail");
}

TEST_CASE("carrier reference acquisition and sample-return hold", "[sensor_carrier]") {
  uint64_t configured[2];
  fresh_acquisition(configured);
  node_sensor_sample_t sample;
  diagn_context_t diagnostic;
  const err_curag_t result = acquire_sample(&sample, &diagnostic);
  /* External references bypass the air-probe range only. Production acquisition
   * still performs its real gate stabilization and calibrated ADC averaging. */
  observation_hold("sample-return");
  assert_sample(&sample, &diagnostic, result, false);
}

static int transition_mode(void) {
  const int mode = carrier_hold_select();
  TEST_ASSERT_TRUE_MESSAGE(mode == CARRIER_HOLD_GUIDED || mode == CARRIER_HOLD_EXPLORATION,
                           "transition requires guided or exploration mode");
  return mode;
}

static void enabled_transition(const char *name, int mode) {
  TEST_ASSERT_FALSE_MESSAGE(s_hardware_used, "fresh transition boot required");
  s_hardware_used = true;
  TEST_ASSERT_EQUAL(ESP_OK, node_sensors_power_gate_on());
  delay_ms(CONFIG_CURA_SENSOR_POWER_STABILIZATION_MS);
  const bool held = carrier_hold_wait("transition-on", mode);
  if (!held) {
    /* No reset/sleep transition follows a failed hold. Release the enabled
     * rail on this failure path only; successful transitions keep it on. */
    TEST_ASSERT_EQUAL(ESP_OK, node_sensors_power_gate_off());
  }
  TEST_ASSERT_TRUE_MESSAGE(held, "transition hold requires valid acknowledgement");
  printf("CARRIER_TRANSITION %s\n", name);
  fflush(stdout);
}
static bool s_observe_restart;
static unsigned s_restart_off_calls;
err_curag_t __real_node_sensors_force_power_off(diagn_context_t *diagnostic);
err_curag_t __wrap_node_sensors_force_power_off(diagn_context_t *diagnostic) {
  const err_curag_t result = __real_node_sensors_force_power_off(diagnostic);
  if (s_observe_restart) {
    ++s_restart_off_calls;
    bool empty = diagnostic && diagnostic->operation == CURAG_OP_NONE &&
                 diagnostic->context_schema == 0 && diagnostic->context_length == 0;
    if (diagnostic) {
      for (size_t i = 0; i < sizeof(diagnostic->context); ++i) empty &= diagnostic->context[i] == 0;
    }
    printf("CARRIER_RESTART_CLEANUP calls=%u result=%08" PRIx32 " diagnostic_empty=%u observer_valid=%u\n",
           s_restart_off_calls, (uint32_t)result, empty, carrier_observer_cleanup_valid(1));
    fflush(stdout);
  }
  return result;
}

static void reset_stage_1(void) {
  enabled_transition("reset", transition_mode());
  carrier_observer_begin(0, 0);
  s_observe_restart = true;
  node_platform_esp_restart();
  TEST_FAIL_MESSAGE("restart returned");
}
static void reset_stage_2(void) {
  TEST_ASSERT_FALSE(s_hardware_used);
  TEST_ASSERT_EQUAL(ESP_RST_SW, esp_reset_reason());
  TEST_ASSERT_TRUE_MESSAGE(carrier_hold_wait("reset-off", transition_mode()),
                           "reset observation requires valid acknowledgement");
}
static void held_stage_1(void) {
  enabled_transition("held-reset", transition_mode());
  while (true) delay_ms(1000); /* Operator asserts EN; no software cleanup. */
}
static void held_stage_2(void) {
  TEST_ASSERT_FALSE(s_hardware_used);
  TEST_ASSERT_EQUAL(ESP_RST_POWERON, esp_reset_reason()); /* C6 EN reset. */
}
static void sleep_stage_1(void) {
  const int mode = transition_mode();
  /* Fresh boot: no wake sources. Guided measurements/YES precede EN/reset;
   * exploration ends with /done and EN/reset. Neither needs a timer dwell. */
  enabled_transition(mode == CARRIER_HOLD_EXPLORATION ?
                     "deep-sleep seconds=unlimited" : "deep-sleep end=operator-reset", mode);
  esp_deep_sleep_start();
  TEST_FAIL_MESSAGE("deep sleep returned");
}
static void sleep_stage_2(void) {
  TEST_ASSERT_FALSE(s_hardware_used);
  (void)transition_mode();
  TEST_ASSERT_EQUAL(ESP_RST_POWERON, esp_reset_reason()); /* C6 EN reset. */
}
TEST_CASE_MULTIPLE_STAGES("carrier production restart cleanup", "[sensor_carrier][reset=SW_CPU_RESET]",
                         reset_stage_1, reset_stage_2);
TEST_CASE_MULTIPLE_STAGES("carrier enabled rail held reset", "[sensor_carrier][reset=POWERON_RESET]",
                         held_stage_1, held_stage_2);
TEST_CASE_MULTIPLE_STAGES("carrier enabled rail deep sleep", "[sensor_carrier][reset=POWERON_RESET]",
                         sleep_stage_1, sleep_stage_2);
