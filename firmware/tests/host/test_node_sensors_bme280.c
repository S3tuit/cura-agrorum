#include "bme280_test_data.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "node_sensors.h"
#include "node_sensors_backend.h"
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct fake_i2c_bus {
  int id;
};
struct fake_i2c_device {
  int id;
};
static struct fake_i2c_bus bus;
static struct fake_i2c_device device;
static uint8_t registers[256];
static int64_t now, triggered_at;
static unsigned calls, bus_new, device_new, bus_del, triggers, sleep_writes;
static unsigned nvm_reads, observations, data_reads, reset_writes;
static unsigned fail_nvm, nvm_busy, fail_observation, busy_observations;
static esp_err_t new_error, add_error, delete_error, transport_error;
static bool early_tick, short_delay, late_init, late_data, stuck, fail_data;
static bool ignore_sleep, fail_sleep, false_idle, fail_trigger, bad_init_mode;
static unsigned initialization_observations, fail_init_observation;
static int64_t data_elapsed, bus_elapsed;
static bool late_publish;
static unsigned publication_checks;
static uint8_t channel_register, channel_mask, channel_bits;
static unsigned corrupt_init_observation, corrupt_measure_observation;

static void corrupt_channel(void) {
  registers[channel_register] =
      (uint8_t)((registers[channel_register] & (uint8_t)~channel_mask) |
                channel_bits);
}

int64_t esp_timer_get_time(void) {
  if (late_publish && data_reads && ++publication_checks == 3)
    now = triggered_at + 50000;
  return now;
}
void vTaskDelay(TickType_t ticks) {
  assert(ticks > 0);
  ++calls;
  if (!short_delay)
    now += ((int64_t)ticks - (early_tick ? 1 : 0)) * 10000;
}
esp_err_t i2c_new_master_bus(const i2c_master_bus_config_t *cfg,
                             i2c_master_bus_handle_t *out) {
  ++calls;
  ++bus_new;
  assert(cfg->i2c_port == 0 && cfg->sda_io_num == 21 && cfg->scl_io_num == 22);
  if (new_error)
    return new_error;
  now += bus_elapsed;
  *out = &bus;
  return 0;
}
esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t handle,
                                    const i2c_device_config_t *cfg,
                                    i2c_master_dev_handle_t *out) {
  ++calls;
  ++device_new;
  assert(handle == &bus);
  assert(cfg->device_address == 0x76 && cfg->scl_speed_hz == 100000 &&
         cfg->dev_addr_length == 0);
  if (add_error)
    return add_error;
  *out = &device;
  return 0;
}
esp_err_t i2c_del_master_bus(i2c_master_bus_handle_t handle) {
  ++calls;
  ++bus_del;
  assert(handle == &bus);
  return delete_error;
}
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t handle,
                              const uint8_t *data, size_t length,
                              int timeout_ms) {
  ++calls;
  now += 100;
  assert(handle == &device && data && length >= 2 && length <= 20 &&
         timeout_ms == 20);
  const uint8_t reg = data[0];
  if (reg == 0xe0) {
    assert(data[1] == 0xb6);
    ++reset_writes;
    registers[0xf2] = registers[0xf4] = registers[0xf5] = 0;
  }
  if (reg == 0xf4) {
    assert((data[1] & 3) != 3); /* no normal-mode intermediate */
    if ((data[1] & 3) == 1) {
      assert(data[1] == 0x25 && registers[0xf2] == 1 && registers[0xf5] == 0);
      if (fail_trigger)
        return 0x456789ab;
      ++triggers;
      triggered_at = now;
      observations = 0;
    } else if (triggers || fail_nvm || nvm_busy || late_init || short_delay ||
               (corrupt_init_observation &&
                initialization_observations >= corrupt_init_observation)) {
      ++sleep_writes;
      if (fail_sleep)
        return ESP_ERR_INVALID_STATE;
      if (ignore_sleep)
        return 0;
    }
  }
  uint8_t address = reg;
  for (size_t i = 1; i < length; i += 2) {
    registers[address] = data[i];
    if (i + 1 < length)
      address = data[i + 1];
  }
  return 0;
}
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t handle,
                                      const uint8_t *command,
                                      size_t command_size, uint8_t *data,
                                      size_t length, int timeout_ms) {
  ++calls;
  now += 100;
  assert(handle == &device && command_size == 1 && data && length &&
         length <= 26 && timeout_ms == 20);
  const uint8_t reg = *command;
  if (reg == 0xd0 && transport_error)
    return transport_error;
  if (reg == 0x88 && late_init)
    now += 500000;
  if (reg == 0xf3 && length == 1) {
    ++nvm_reads;
    if (nvm_reads == fail_nvm)
      return 0x12345678;
    registers[reg] = nvm_reads <= nvm_busy ? 1 : 0;
  }
  const bool observation = (reg == 0xf2 && length == 3) ||
                           (reg == 0xf3 && length == 2);
  if (observation && !triggers) {
    ++initialization_observations;
    if (initialization_observations == fail_init_observation)
      return 0x56789abc;
    if (bad_init_mode)
      registers[0xf4] = 3;
    if (initialization_observations == corrupt_init_observation)
      corrupt_channel();
  }
  if (observation && triggers) {
    ++observations;
    if (observations == fail_observation)
      return 0x23456789;
    if (false_idle || (!stuck && observations > busy_observations &&
                       now - triggered_at >= 9300)) {
      registers[0xf3] = 0;
      registers[0xf4] &= 0xfc;
    } else {
      registers[0xf3] = 8;
    }
    if (observations == corrupt_measure_observation)
      corrupt_channel();
  }
  if (reg == 0xf7) {
    ++data_reads;
    assert(length == 8 && triggers && now - triggered_at >= 9300);
    if (late_data)
      now += 50000;
    now += data_elapsed;
    if (fail_data)
      return 0x3456789a;
  }
  memcpy(data, registers + reg, length);
  return 0;
}

static node_sensors_backend_enclosure_t sample;
static node_sensors_backend_result_t acquire(void) {
  memset(&sample, 0xa5, sizeof(sample));
  return node_sensors_backend_sample_bme280(&sample);
}
static void failure(uint32_t kind, int32_t status,
                    curag_operation_t operation) {
  node_sensors_backend_result_t result = acquire();
  assert(result.kind == kind && result.status == status &&
         result.operation == operation);
  assert(sample.temperature_centi_c == 0 && sample.pressure_pa == 0 &&
         sample.humidity_centi_pct == 0);
  unsigned before = calls;
  result = acquire();
  assert(result.kind == kind && result.status == status &&
         result.operation == operation && calls == before);
  assert(sample.temperature_centi_c == 0 && sample.pressure_pa == 0 &&
         sample.humidity_centi_pct == 0);
}
int main(int argc, char **argv) {
  assert(argc == 2);
  bme_test_registers(registers);
  const char *test = argv[1];
  if (!strcmp(test, "null")) {
    node_sensors_backend_result_t r = node_sensors_backend_sample_bme280(NULL);
    assert(r.kind == 1 && r.status == ESP_ERR_INVALID_ARG &&
           r.operation == CURAG_OP_VALIDATE && calls == 0);
  } else if (!strcmp(test, "bus_allocation")) {
    new_error = ESP_ERR_NO_MEM;
    failure(1, ESP_ERR_NO_MEM, CURAG_OP_INITIALIZE);
    assert(bus_new == 1 && device_new == 0 && bus_del == 0 &&
           reset_writes == 0);
  } else if (!strcmp(test, "device_allocation") ||
             !strcmp(test, "allocation_cleanup_error")) {
    add_error = ESP_ERR_NO_MEM;
    delete_error = !strcmp(test, "allocation_cleanup_error") ? ESP_FAIL : 0;
    failure(1, ESP_ERR_NO_MEM, CURAG_OP_INITIALIZE);
    assert(bus_new == 1 && device_new == 1 && bus_del == 1 &&
           reset_writes == 0);
  } else if (!strcmp(test, "missing")) {
    transport_error = ESP_ERR_INVALID_RESPONSE;
    failure(1, 264, CURAG_OP_INITIALIZE);
    assert(reset_writes == 0 && sleep_writes == 0);
  } else if (!strcmp(test, "transport_status")) {
    transport_error = 0x12345678;
    failure(1, 0x12345678, CURAG_OP_INITIALIZE);
  } else if (!strcmp(test, "wrong_id")) {
    registers[0xd0] = 0x58;
    failure(2, -4, CURAG_OP_INITIALIZE);
    assert(reset_writes == 0);
  } else if (!strcmp(test, "nvm_first_error") ||
             !strcmp(test, "nvm_later_error")) {
    fail_nvm = !strcmp(test, "nvm_first_error") ? 1U : 3U;
    nvm_busy = 6;
    failure(1, 0x12345678, CURAG_OP_INITIALIZE);
    assert(nvm_reads == fail_nvm && sleep_writes == 1 && triggers == 0);
  } else if (!strcmp(test, "nvm_bound")) {
    nvm_busy = 100;
    failure(2, -6, CURAG_OP_INITIALIZE);
    assert(nvm_reads == 6 && sleep_writes == 1);
  } else if (!strcmp(test, "initialization_late")) {
    late_init = true;
    failure(1, 263, CURAG_OP_INITIALIZE);
    assert(triggers == 0 && sleep_writes == 1);
  } else if (!strcmp(test, "bus_late")) {
    bus_elapsed = 500000;
    failure(1, 263, CURAG_OP_INITIALIZE);
    assert(device_new == 0 && reset_writes == 0);
  } else if (!strcmp(test, "init_mode_error")) {
    fail_init_observation = 1;
    failure(1, 0x56789abc, CURAG_OP_INITIALIZE);
    assert(triggers == 0);
  } else if (!strcmp(test, "configuration_mode_error")) {
    fail_init_observation = 2;
    failure(1, 0x56789abc, CURAG_OP_INITIALIZE);
    assert(triggers == 0);
  } else if (!strcmp(test, "init_not_sleeping")) {
    bad_init_mode = true;
    failure(2, -5, CURAG_OP_INITIALIZE);
    assert(triggers == 0 && initialization_observations <= 7);
  } else if (!strcmp(test, "trigger_error")) {
    fail_trigger = true;
    failure(1, 0x456789ab, CURAG_OP_READ);
    assert(triggers == 0 && data_reads == 0);
  } else if (!strcmp(test, "pretrigger_not_sleeping")) {
    assert(acquire().status == 0);
    stuck = true;
    registers[0xf4] = 3;
    failure(2, -5, CURAG_OP_READ);
    assert(triggers == 1 && data_reads == 1 && sleep_writes == 1);
  } else if (!strcmp(test, "deadline_equal")) {
    /* Trigger-relative elapsed = 20000 delay + two 100 us transfers. */
    data_elapsed = 29800;
    failure(1, 263, CURAG_OP_READ);
    assert(triggers == 1 && data_reads == 1 && sleep_writes == 1);
  } else if (!strcmp(test, "publication_late")) {
    late_publish = true;
    failure(1, 263, CURAG_OP_READ);
    assert(data_reads == 1 && sleep_writes == 1);
  } else if (!strcmp(test, "temperature_above_scaled_max") ||
             !strcmp(test, "temperature_below_scaled_min")) {
    bme_test_u16(registers + 0x8a, !strcmp(test, "temperature_above_scaled_max")
                                       ? 32767
                                       : (uint16_t)-32768);
    bme_test_raw20(registers + 0xfa, 838861);
    failure(1, 264, CURAG_OP_READ);
  } else if (!strcmp(test, "pressure_above_max")) {
    bme_test_u16(registers + 0x8e, 1);
    bme_test_raw20(registers + 0xf7, 361381);
    failure(1, 264, CURAG_OP_READ);
  } else if (!strcmp(test, "delay_short")) {
    short_delay = true;
    failure(1, 263, CURAG_OP_INITIALIZE);
    assert(nvm_reads == 0 && triggers == 0);
  } else if (!strcmp(test, "poll_first_error") ||
             !strcmp(test, "poll_later_error")) {
    early_tick = true;
    fail_observation = !strcmp(test, "poll_first_error") ? 1U : 2U;
    busy_observations = fail_observation - 1;
    failure(1, 0x23456789, CURAG_OP_READ);
    assert(triggers == 1 && sleep_writes == 1 && data_reads == 0);
  } else if (!strcmp(test, "measurement_bound") ||
             !strcmp(test, "recovery_failure")) {
    stuck = true;
    ignore_sleep = true;
    fail_sleep = !strcmp(test, "recovery_failure");
    failure(1, 263, CURAG_OP_READ);
    assert(triggers == 1 && sleep_writes == 1 && observations <= 12 &&
           data_reads == 0 && now < 500000);
  } else if (!strcmp(test, "data_error") ||
             !strcmp(test, "first_error_preserved")) {
    fail_data = true;
    fail_sleep = !strcmp(test, "first_error_preserved");
    failure(1, 0x3456789a, CURAG_OP_READ);
    assert(data_reads == 1 && sleep_writes == 1);
  } else if (!strcmp(test, "data_late")) {
    late_data = true;
    failure(1, 263, CURAG_OP_READ);
    assert(sleep_writes == 1);
  } else if (!strncmp(test, "skipped_", 8) ||
             !strncmp(test, "misconfigured_", 14)) {
    /* Fault the actual register state while returning successful I2C reads.
     * Leave data numerically ordinary: settings must detect the failure. */
    const bool humidity = strstr(test, "humidity") != NULL;
    const bool temperature = strstr(test, "temperature") != NULL;
    const bool initializing = strstr(test, "initialize") != NULL;
    const bool pretrigger = strstr(test, "pretrigger") != NULL;
    const unsigned shift = humidity ? 0U : temperature ? 5U : 2U;
    channel_register = humidity ? 0xf2 : 0xf4;
    channel_mask = (uint8_t)(7U << shift);
    channel_bits = (uint8_t)((!strncmp(test, "skipped_", 8) ? 0U : 2U) << shift);
    if (initializing) {
      corrupt_init_observation = 2;
    } else if (pretrigger) {
      assert(acquire().status == 0);
      corrupt_channel();
    } else {
      corrupt_measure_observation = 1;
    }
    failure(1, 264, initializing ? CURAG_OP_INITIALIZE : CURAG_OP_READ);
    assert(data_reads == (pretrigger ? 1U : 0U));
    assert(triggers == (initializing ? 0U : 1U) && sleep_writes == 1);
    /* Recovery must still observe sleep when humidity remains disabled/x2. */
    assert((registers[0xf4] & 3) == 0);
  } else if (!strcmp(test, "invalid_compensation")) {
    bme_test_u16(registers + 0x8e, 0);
    failure(1, 264, CURAG_OP_READ);
  } else if (!strcmp(test, "pressure_unrepresentable")) {
    bme_test_u16(registers + 0x8e, 1);
    bme_test_raw20(registers + 0xf7, 0);
    failure(1, 264, CURAG_OP_READ);
  } else if (!strcmp(test, "temperature_unrepresentable")) {
    bme_test_u16(registers + 0x8a, 32767);
    bme_test_raw20(registers + 0xfa, 0xfffff);
    failure(1, 264, CURAG_OP_READ);
  } else {
    int16_t expected_t = 1600;
    uint32_t expected_p = 100000;
    uint16_t expected_h = 5000;
    false_idle = !strcmp(test, "initially_idle");
    early_tick = !strcmp(test, "tick_edge");
    /* Closed-form coefficients in bme280_test_data.h independently yield
     * midpoint T=102.4 C and P=80000 Pa. For humidity use H2=100 and other
     * H coefficients zero: H=raw_H*100/65536 => exactly 50 percent. */
    if (!strcmp(test, "enabled_midpoint_temperature") ||
        !strcmp(test, "enabled_midpoints")) {
      bme_test_raw20(registers + 0xfa, 0x80000);
      expected_t = 10240;
    }
    if (!strcmp(test, "enabled_midpoint_pressure") ||
        !strcmp(test, "enabled_midpoints")) {
      bme_test_raw20(registers + 0xf7, 0x80000);
      expected_p = 80000;
    }
    if (!strcmp(test, "enabled_midpoint_humidity") ||
        !strcmp(test, "enabled_midpoints")) {
      registers[0xfd] = 0x80;
      registers[0xfe] = 0;
      bme_test_u16(registers + 0xe1, 100);
      expected_h = 5000;
    }
    if (!strcmp(test, "enabled_midpoint_bench_temperature")) {
      /* DUT factory bytes 7C6C3A673200, independently evaluated using the
       * datasheet double formula: 25.185268207732587 C => 2519 centi-C.
       * Synthetic P/H coefficients remain independent of temperature. */
      bme_test_u16(registers + 0x88, 27772);
      bme_test_u16(registers + 0x8a, 26426);
      bme_test_u16(registers + 0x8c, 50);
      bme_test_raw20(registers + 0xfa, 0x80000);
      expected_t = 2519;
    }
    if (!strcmp(test, "outliers")) {
      bme_test_raw20(registers + 0xfa, 640000);
      bme_test_raw20(registers + 0xf7, 0);
      expected_t = 12500;
      expected_p = 160000;
    }
    if (!strcmp(test, "round_positive") || !strcmp(test, "round_negative")) {
      bme_test_raw20(registers + 0xfa, 128);
      expected_t = 3;
      if (!strcmp(test, "round_negative")) {
        bme_test_u16(registers + 0x8a, (uint16_t)-16384);
        expected_t = -3;
      }
    }
    /* Adjacent raw counts straddle the destination boundary. Closed-form
     * temperature centi-C = raw*T2*5/4194304; pressure = (1048576-raw)*6250/P1.
     */
    if (!strcmp(test, "temperature_max") || !strcmp(test, "temperature_min")) {
      const bool positive = !strcmp(test, "temperature_max");
      bme_test_u16(registers + 0x8a, positive ? 32767 : (uint16_t)-32768);
      bme_test_raw20(registers + 0xfa, 838860);
      expected_t = positive ? INT16_MAX : INT16_MIN;
    }
    if (!strcmp(test, "pressure_near_max")) {
      bme_test_u16(registers + 0x8e, 1);
      bme_test_raw20(registers + 0xf7, 361382);
      expected_p = UINT32_C(4294962500);
    }
    if (!strcmp(test, "valid_zero")) {
      bme_test_raw20(registers + 0xfa, 0);
      registers[0xfe] = 0;
      expected_t = 0;
      expected_h = 0;
    }
    node_sensors_backend_result_t result = acquire();
    assert(result.status == 0 && result.kind == 0 && result.operation == 0);
    assert(sample.temperature_centi_c == expected_t &&
           sample.pressure_pa == expected_p &&
           sample.humidity_centi_pct == expected_h);
    assert(registers[0xf4] == 0x24 && triggers == 1 && data_reads == 1 &&
           sleep_writes == 0);
    assert(bus_new == 1 && device_new == 1 && reset_writes == 1);
    assert(acquire().status == 0 && triggers == 2 && data_reads == 2 &&
           bus_new == 1 && device_new == 1 && reset_writes == 1);
    assert(sample.temperature_centi_c == expected_t &&
           sample.pressure_pa == expected_p &&
           sample.humidity_centi_pct == expected_h);
  }
  puts(test);
  return 0;
}
