#include <inttypes.h>
#include <errno.h>
#include <stdlib.h>
#include "carrier_observer.h"
#include "carrier_checks.h"
#include "carrier_core.h"
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
#include "carrier_bme_preflight.h"
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
  const esp_err_t bme = carrier_bme_preflight(false);
  TEST_ASSERT_EQUAL(ESP_OK, found.result);
  TEST_ASSERT_EQUAL(ESP_OK, found.cleanup);
  TEST_ASSERT_GREATER_THAN_MESSAGE(0, found.ds18b20_count,
                                   "no DS18B20 found; discovery incomplete");
  TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, bme,
                            "BME280 identification/cleanup failed");
}

static void fixture_preflight(const char *fixture, unsigned expected_ds_mask, bool bme_present) {
  uint64_t configured[2];
  require_configured_roms(configured);
  const discovery_t found = discover_roms();
  const esp_err_t bme = carrier_bme_preflight(!bme_present);
  TEST_ASSERT_EQUAL(ESP_OK, found.result);
  TEST_ASSERT_EQUAL(ESP_OK, found.cleanup);
  carrier_assert_inventory(configured, found.roms, found.count,
                           found.ds18b20_count, expected_ds_mask);
  TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, bme,
                            "BME280 identification/cleanup failed");
  printf("CARRIER_PREFLIGHT %s_complete_resources_released_reset_required\n", fixture);
}

TEST_CASE("carrier nominal preflight", "[sensor_carrier]") {
  fixture_preflight("nominal", 3, true);
}

TEST_CASE("carrier missing_ds0 preflight", "[sensor_carrier]") {
  fixture_preflight("missing_ds0", 2, true);
}

TEST_CASE("carrier missing_ds1 preflight", "[sensor_carrier]") {
  fixture_preflight("missing_ds1", 1, true);
}

TEST_CASE("carrier missing_bme280 preflight", "[sensor_carrier]") {
  fixture_preflight("missing_bme280", 3, false);
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
  carrier_assert_sample(sample, diagnostic, result, 3, nominal, true);
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
  const int64_t duration_us = esp_timer_get_time() - start_us;
  print_sample(*sample, *diagnostic, result, duration_us);
  TEST_ASSERT_TRUE_MESSAGE(duration_us >= 0 && duration_us <= 30000000,
                           "production sample did not return within 30 seconds");
  return result;
}

static void print_observation(void) {
  const carrier_observation_t seen = carrier_observer_snapshot();
  printf("CARRIER_OBSERVER conversions=%u reads=%u wait_us=%" PRId64
         " adc=%u/%u onewire=%u/%u ds=%u/%u bme=%u invalid=%u\n",
         seen.conversions, seen.reads, seen.minimum_wait_us,
         seen.adc_new, seen.adc_del, seen.bus_new, seen.bus_del,
         seen.ds_new, seen.ds_del, seen.bme_forced, seen.invalid_sequence);
  printf("CARRIER_SOIL_OBSERVER stabilization_calls=%u stabilization_ms=%u"
         " reads0=%u reads1=%u delays0=%u delays1=%u cali0=%u cali1=%u"
         " mv0=%d mv1=%d power_released=%u\n",
         seen.stabilization_calls, seen.stabilization_ms,
         seen.soil_reads[0], seen.soil_reads[1], seen.soil_delays[0], seen.soil_delays[1],
         seen.soil_calibrations[0], seen.soil_calibrations[1],
         seen.soil_mv[0], seen.soil_mv[1], seen.power_released);
  if (seen.bme_cal_read) {
    printf("CARRIER_BME_TCAL raw=");
    for (unsigned i = 0; i < sizeof(seen.bme_cal_t); ++i) printf("%02X", seen.bme_cal_t[i]);
    puts("");
  }
  printf("CARRIER_BME_DATA reads=%u read_error=%" PRId32 " error_register=%02X raw=",
         seen.bme_data_reads, (int32_t)seen.bme_read_error, seen.bme_error_register);
  for (unsigned i = 0; i < sizeof(seen.bme_raw); ++i) printf("%02X", seen.bme_raw[i]);
  printf(" driver_result=%d temperature=%.9g pressure=%.9g humidity=%.9g\n",
         seen.bme_data_result, seen.bme_temperature, seen.bme_pressure, seen.bme_humidity);
}

static void assert_observation(const node_sensor_sample_t *sample, unsigned expected_ds_mask, bool bme_present) {
  TEST_ASSERT_TRUE_MESSAGE(carrier_observer_sample_valid(expected_ds_mask, bme_present),
                           "production acquisition conditions/resource observer failed");
  const carrier_observation_t seen = carrier_observer_snapshot();
  TEST_ASSERT_EQUAL_INT(seen.soil_mv[0], sample->soil_0_mv);
  TEST_ASSERT_EQUAL_INT(seen.soil_mv[1], sample->soil_1_mv);
}

static void fixture_acquisition(unsigned expected_ds_mask, bool bme_present) {
  uint64_t configured[2];
  fresh_acquisition(configured);
  const int mode = carrier_hold_select();
  TEST_ASSERT_TRUE_MESSAGE(mode >= 0, "missing/invalid hold mode");
  node_sensor_sample_t sample;
  diagn_context_t diagnostic;
  carrier_observer_begin(configured[0], configured[1]);
  const err_curag_t result = acquire_sample(&sample, &diagnostic);
  print_observation();
  /* No sensor/gate operation after return, including on a failed sample. */
  TEST_ASSERT_TRUE_MESSAGE(carrier_hold_wait("sample-return", mode),
                           "guided hold incomplete: valid acknowledgement required");
  carrier_assert_sample(&sample, &diagnostic, result, expected_ds_mask, true, bme_present);
  assert_observation(&sample, expected_ds_mask, bme_present);
}

TEST_CASE("carrier nominal acquisition and sample-return hold", "[sensor_carrier]") {
  fixture_acquisition(3, true);
}

TEST_CASE("carrier missing_ds0 acquisition and sample-return hold", "[sensor_carrier]") {
  fixture_acquisition(2, true);
}

TEST_CASE("carrier missing_ds1 acquisition and sample-return hold", "[sensor_carrier]") {
  fixture_acquisition(1, true);
}

TEST_CASE("carrier missing_bme280 acquisition and sample-return hold", "[sensor_carrier]") {
  fixture_acquisition(3, false);
}

static void fixture_reading(unsigned ds_mask, bool bme_present, bool air_soil,
                            uint16_t expected_flags) {
  uint64_t configured[2];
  fresh_acquisition(configured);
  carrier_core_observation_t seen;
  carrier_core_run(configured, ds_mask, bme_present, &seen);
  print_sample(seen.sample, seen.diagnostic, seen.result, seen.duration_us);
  TEST_ASSERT_TRUE_MESSAGE(seen.duration_us >= 0 && seen.duration_us <= 30000000,
                           "production sample did not return within 30 seconds");
  carrier_assert_sample(&seen.sample, &seen.diagnostic, seen.result,
                        ds_mask, air_soil, bme_present);
  TEST_ASSERT_TRUE_MESSAGE(seen.acquisition_valid,
                           "real core acquisition conditions/resource observer failed");
  TEST_ASSERT_EQUAL_INT(seen.acquisition.soil_mv[0], seen.sample.soil_0_mv);
  TEST_ASSERT_EQUAL_INT(seen.acquisition.soil_mv[1], seen.sample.soil_1_mv);
  const carrier_observation_t final = carrier_observer_snapshot();
  TEST_ASSERT_EQUAL_UINT(2, final.gate_off); /* sample-owned plus core final cleanup */
  TEST_ASSERT_EQUAL_UINT(1, final.gate_on);
  TEST_ASSERT_TRUE(final.power_released);
  TEST_ASSERT_FALSE(final.invalid_sequence);

  /* Expectations are the INTERFACE/protocol mapping, applied to the exact
   * returned acquisition. Never synthesize a second reading or sample again. */
  const cura_lora_v2_reading_t *const reading = &seen.reading;
  TEST_ASSERT_EQUAL_HEX16(expected_flags, reading->flags & UINT16_C(0x00fe));
  TEST_ASSERT_EQUAL_UINT16(seen.sample.soil_0_mv, reading->soil_0_mv);
  TEST_ASSERT_EQUAL_UINT16(seen.sample.soil_1_mv, reading->soil_1_mv);
  TEST_ASSERT_EQUAL_INT16(seen.sample.soil_temp_0_centi_c, reading->soil_temp_0_centi_c);
  TEST_ASSERT_EQUAL_INT16(seen.sample.soil_temp_1_centi_c, reading->soil_temp_1_centi_c);
  TEST_ASSERT_EQUAL_INT16(seen.sample.enclosure_centi_c, reading->enclosure_centi_c);
  TEST_ASSERT_EQUAL_UINT32(seen.sample.enclosure_pressure_pa, reading->enclosure_pressure_pa);
  TEST_ASSERT_EQUAL_UINT16(seen.sample.enclosure_humidity_centi_pct, reading->enclosure_humidity_centi_pct);
  TEST_ASSERT_EQUAL_HEX16(bme_present ? 0x00e0 : 0, reading->flags & 0x00e0);
}

TEST_CASE("carrier nominal core reading", "[sensor_carrier]") {
  fixture_reading(3, true, true, 0x00fe);
}

TEST_CASE("carrier missing_ds0 core reading", "[sensor_carrier]") {
  fixture_reading(2, true, true, 0x00f6);
}

TEST_CASE("carrier missing_ds1 core reading", "[sensor_carrier]") {
  fixture_reading(1, true, true, 0x00ee);
}

TEST_CASE("carrier missing_bme280 core reading", "[sensor_carrier]") {
  fixture_reading(3, false, true, 0x001e);
}

TEST_CASE("carrier adc_reference core reading", "[sensor_carrier]") {
  fixture_reading(3, true, false, 0x00fe);
}

static void observe_bme_sleep(const node_sensor_sample_t *sample) {
  node_sensor_sample_t retained;
  memcpy(&retained, sample, sizeof(retained));
  uint8_t registers[2] = {0};
  const esp_err_t result = carrier_observer_bme_registers(registers);
  printf("CARRIER_BME_SLEEP status=%02X control=%02X mode=%u result=%" PRId32 "\n",
         registers[0], registers[1], registers[1] & 3, (int32_t)result);
  TEST_ASSERT_EQUAL_MEMORY(&retained, sample, sizeof(retained));
  TEST_ASSERT_EQUAL(ESP_OK, result);
  carrier_assert_bme_sleep(registers[0], registers[1]);
}

TEST_CASE("carrier BME280 sleep observation", "[sensor_carrier]") {
  uint64_t configured[2];
  fresh_acquisition(configured);
  node_sensor_sample_t sample;
  diagn_context_t diagnostic;
  carrier_observer_begin(configured[0], configured[1]);
  const err_curag_t result = acquire_sample(&sample, &diagnostic);
  print_observation();
  assert_sample(&sample, &diagnostic, result, true);
  assert_observation(&sample, 3, true);
  observe_bme_sleep(&sample);
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
    print_observation();
    assert_sample(&sample, &diagnostic, result, true);
    assert_observation(&sample, 3, true);
    observe_bme_sleep(&sample);
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
