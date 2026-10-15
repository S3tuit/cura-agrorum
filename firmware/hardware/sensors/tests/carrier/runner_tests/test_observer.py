"""Compile the actual forwarding observers; stubs exist only at their IDF boundary."""
import os
from pathlib import Path
import subprocess
import pytest
from carrier_runner import APP


@pytest.fixture(scope='module')
def observer_binary(tmp_path_factory):
    root = tmp_path_factory.mktemp('observer')
    types = '''#pragma once
#include <stdint.h>
#include <stddef.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_INVALID_STATE 259
#define CONFIG_CURA_SENSOR_POWER_GATE_GPIO 2
typedef int gpio_num_t;
typedef int i2c_port_t;
struct bme280_dev { int unused; };
struct bme280_data { double temperature, pressure, humidity; };
#define BME280_OK 0
typedef void *i2c_master_bus_handle_t;
typedef void *i2c_master_dev_handle_t;
typedef struct { int unused; } i2c_master_bus_config_t;
typedef struct { int unused; } i2c_device_config_t;
typedef struct { int status; } node_sensors_backend_result_t;
typedef struct { int temperature; } node_sensors_backend_enclosure_t;
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t, const uint8_t *, size_t, uint8_t *, size_t, int);
typedef void *onewire_bus_handle_t;
typedef void *onewire_device_iter_handle_t;
typedef void *ds18b20_device_handle_t;
typedef void *adc_oneshot_unit_handle_t;
typedef struct { int unit_id; } adc_oneshot_unit_init_cfg_t;
typedef struct { int atten, bitwidth; } adc_oneshot_chan_cfg_t;
typedef int adc_channel_t;
typedef void *adc_cali_handle_t;
#define ADC_UNIT_1 0
#define ADC_CHANNEL_0 0
#define ADC_CHANNEL_1 1
#define ADC_ATTEN_DB_12 12
#define ADC_BITWIDTH_DEFAULT 0
typedef struct { int unused; } onewire_bus_config_t;
typedef struct { int unused; } onewire_bus_rmt_config_t;
typedef struct { int unused; } ds18b20_config_t;
typedef struct { int unused; } onewire_device_t;
typedef struct { int unused; } i2c_config_t;
int64_t esp_timer_get_time(void);
'''
    (root / 'types.h').write_text(types)
    for name in ['bme280.h', 'driver/gpio.h', 'ds18b20.h', 'esp_adc/adc_oneshot.h',
                 'esp_adc/adc_cali.h', 'esp_timer.h', 'driver/i2c_master.h', 'node_sensors_backend.h', 'esp_err.h', 'node_sensors_power_gate.h',
                 'onewire_bus.h', 'onewire_device.h', 'sdkconfig.h']:
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('#include "types.h"\n')
    # Include the actual implementation so typed __real/__wrap declarations are
    # checked together. These are dependency stubs, not simulated sensors.
    harness = root / 'harness.c'
    harness.write_text('''#include <assert.h>
#include <stdlib.h>
#include "carrier_observer.c"
static int calls, status, next_raw = 100;
static int64_t clock_us;
int64_t esp_timer_get_time(void) { return clock_us; }
#define REAL(name, params) esp_err_t __real_##name params { ++calls; return status; }
REAL(node_sensors_power_gate_on, (void))
REAL(node_sensors_power_gate_off, (void))
REAL(adc_oneshot_new_unit, (const adc_oneshot_unit_init_cfg_t *a, adc_oneshot_unit_handle_t *b))
REAL(adc_oneshot_del_unit, (adc_oneshot_unit_handle_t a))
REAL(adc_oneshot_config_channel, (adc_oneshot_unit_handle_t a, adc_channel_t b, const adc_oneshot_chan_cfg_t *c))
esp_err_t __real_adc_oneshot_read(adc_oneshot_unit_handle_t a, adc_channel_t b, int *raw) {
  ++calls; *raw = next_raw++; return status;
}
esp_err_t __real_adc_cali_raw_to_voltage(adc_cali_handle_t a, int raw, int *mv) {
  ++calls; *mv = raw * 2; return status;
}
void __real_esp_rom_delay_us(uint32_t us) { ++calls; clock_us += us; }
void __real_node_sensors_backend_delay_ms(uint32_t ms) { ++calls; clock_us += ms * 1000; }
REAL(onewire_new_bus_rmt, (const onewire_bus_config_t *a, const onewire_bus_rmt_config_t *b, onewire_bus_handle_t *c))
REAL(onewire_bus_del, (onewire_bus_handle_t a))
REAL(onewire_new_device_iter, (onewire_bus_handle_t a, onewire_device_iter_handle_t *b))
REAL(onewire_del_device_iter, (onewire_device_iter_handle_t a))
REAL(ds18b20_new_device_from_enumeration, (const onewire_device_t *a, const ds18b20_config_t *b, ds18b20_device_handle_t *c))
REAL(ds18b20_del_device, (ds18b20_device_handle_t a))
int8_t __real_bme280_init(struct bme280_dev *a) { ++calls; return status; }
node_sensors_backend_result_t __real_node_sensors_backend_sample_bme280(node_sensors_backend_enclosure_t *out) {
  ++calls; if (out) out->temperature = 123;
  return (node_sensors_backend_result_t){status};
}
REAL(gpio_set_level, (gpio_num_t a, uint32_t b))
REAL(i2c_new_master_bus, (const i2c_master_bus_config_t *a, i2c_master_bus_handle_t *b))
esp_err_t __real_i2c_master_bus_add_device(i2c_master_bus_handle_t a, const i2c_device_config_t *b, i2c_master_dev_handle_t *out) {
  ++calls; *out = (void *)2; return status;
}
REAL(i2c_master_transmit, (i2c_master_dev_handle_t a, const uint8_t *b, size_t c, int d))
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t a, const uint8_t *reg, size_t n, uint8_t *data, size_t len, int timeout) {
  assert(a == (void *)2 && n == 1 && *reg == 0xf3 && len == 2 && timeout == 20);
  ++calls; data[0] = 0x08; data[1] = 0x27; return status;
}
esp_err_t __real_onewire_bus_write_bytes(onewire_bus_handle_t bus, const uint8_t *data, uint8_t size) {
  assert(bus == (void *)3 && data && size); ++calls; return status;
}
esp_err_t __real_onewire_bus_read_bytes(onewire_bus_handle_t bus, uint8_t *data, uint8_t size) {
  assert(bus == (void *)3 && size == 9); ++calls; data[4] = 0x7f; return status;
}
esp_err_t __real_i2c_master_transmit_receive(i2c_master_dev_handle_t a, const uint8_t *reg, size_t n, uint8_t *data, size_t len, int timeout) {
  assert(n == 1 && ((*reg == 0xf7 && len == 8) || (*reg == 0x88 && len == 26)) && timeout == 20);
  ++calls; for (unsigned i = 0; i < 8; ++i) data[i] = i + 0x80;
  return status;
}
int8_t __real_bme280_get_sensor_data(uint8_t select, struct bme280_data *data, struct bme280_dev *sensor) {
  ++calls; data->temperature = 25.14; data->pressure = 96770; data->humidity = 62.90;
  return status;
}
static void read_rom(uint64_t rom) {
  uint8_t command[10] = {0x55};
  for (unsigned i = 0; i < 8; ++i) command[i+1] = rom >> (8*i);
  command[9] = 0xbe;
  __wrap_onewire_bus_write_bytes((void *)3, command, 10);
  uint8_t data[9] = {0};
  __wrap_onewire_bus_read_bytes((void *)3, data, 9);
  assert(data[4] == 0x7f);
}
int main(int argc, char **argv) {
  assert(argc == 2);
  int scenario = atoi(argv[1]);
  carrier_observer_begin(0x1128, 0x2228);
  if (scenario == 24) {
    observed.bme_samples = 1;
    const uint8_t reg = 0xf7;
    uint8_t raw[8];
    struct bme280_data data;
    assert(__wrap_i2c_master_transmit_receive(NULL, &reg, 1, raw, 8, 20) == 0);
    assert(__wrap_bme280_get_sensor_data(7, &data, NULL) == 0);
    assert(calls == 2 && observed.bme_data_reads == 1 && observed.bme_raw[0] == 0x80 && observed.bme_raw[7] == 0x87);
    assert(data.temperature == 25.14 && observed.bme_temperature == data.temperature && observed.bme_pressure == data.pressure && observed.bme_humidity == data.humidity);
    status = 264;
    assert(__wrap_i2c_master_transmit_receive(NULL, &reg, 1, raw, 8, 20) == 264);
    assert(observed.bme_read_error == 264 && observed.bme_error_register == 0xf7);
    status = 263;
    __wrap_i2c_master_transmit_receive(NULL, &reg, 1, raw, 8, 20);
    assert(calls == 4 && observed.bme_read_error == 264);
    uint8_t calibration[26]; const uint8_t cal_reg = 0x88; status = 0;
    __wrap_i2c_master_transmit_receive(NULL, &cal_reg, 1, calibration, 26, 20);
    assert(calls == 5 && observed.bme_cal_read && observed.bme_cal_t[0] == 0x80 && observed.bme_cal_t[5] == 0x85);
    return 0;
  }
  if (scenario == 22) {
    uint8_t registers[2];
    assert(carrier_observer_bme_registers(registers) == ESP_ERR_INVALID_STATE && calls == 0);
    i2c_master_dev_handle_t handle;
    __wrap_i2c_master_bus_add_device(NULL, NULL, &handle);
    carrier_observer_begin(0x1128, 0x2228);
    const int before = calls;
    assert(carrier_observer_bme_registers(registers) == 0 && calls == before + 1);
    assert(registers[0] == 8 && registers[1] == 0x27); /* no mode modification */
    status = 0x12345678;
    assert(carrier_observer_bme_registers(registers) == status && calls == before + 2);
    return 0;
  }
  if (scenario >= 6 && scenario <= 9) {
    if (scenario == 9) {
      __wrap_node_sensors_power_gate_off();
      assert(carrier_observer_cleanup_valid(1));
      assert(!carrier_observer_cleanup_valid(2));
      return 0;
    }
    __wrap_node_sensors_power_gate_off(); __wrap_node_sensors_power_gate_off();
    if (scenario == 7) __wrap_onewire_new_bus_rmt(NULL, NULL, NULL);
    if (scenario == 8) __wrap_gpio_set_level(2, 0);
    assert(carrier_observer_cleanup_valid(2) == (scenario == 6));
    return 0;
  }
  __wrap_node_sensors_power_gate_on(); __wrap_gpio_set_level(2, 0);
  __wrap_node_sensors_backend_delay_ms(scenario == 10 ? 201 : 200);
  const adc_oneshot_unit_init_cfg_t adc_config = {.unit_id = ADC_UNIT_1};
  const adc_oneshot_chan_cfg_t channel_config = {
    .atten = scenario == 11 ? 6 : ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
  for (int i = 0; i < 2; ++i) {
    __wrap_adc_oneshot_new_unit(&adc_config, NULL);
    __wrap_adc_oneshot_config_channel((void *)5, i, &channel_config);
    for (int n = 0; n < 16; ++n) {
      int raw;
      status = scenario == 12 && n == 0 ? 17 : 0;
      __wrap_adc_oneshot_read((void *)5, i, &raw);
      assert(raw == 100 + i * 16 + n);
      if (n < 15 && !(scenario == 13 && n == 0))
        __wrap_esp_rom_delay_us(scenario == 14 ? 2001 : 2000);
    }
    status = 0;
    int mv;
    __wrap_adc_cali_raw_to_voltage((void *)6, 108 + i * 16 + (scenario == 15), &mv);
    assert(mv == 2 * (108 + i * 16 + (scenario == 15)));
    __wrap_adc_oneshot_del_unit((void *)5);
  }
  unsigned mask = scenario == 20 ? 2 : scenario == 21 ? 1 : 3;
  for (int i = 0; i < (mask == 3 ? 2 : 1); ++i) {
    __wrap_ds18b20_new_device_from_enumeration(NULL, NULL, NULL);
    __wrap_ds18b20_del_device(NULL);
  }
  __wrap_onewire_new_bus_rmt(NULL, NULL, NULL); __wrap_onewire_bus_del(NULL);
  __wrap_onewire_new_device_iter(NULL, NULL); __wrap_onewire_del_device_iter(NULL);
  if (scenario == 16) __wrap_node_sensors_backend_sample_bme280(NULL);
  status = scenario == 17 ? 17 : 0;
  __wrap_node_sensors_power_gate_off();
  status = 0;
  node_sensors_backend_enclosure_t enclosure = {0};
  assert(__wrap_node_sensors_backend_sample_bme280(&enclosure).status == 0);
  assert(enclosure.temperature == 123);
  const uint8_t forced[2] = {0xf4, 0x25};
  if (scenario != 23) __wrap_i2c_master_transmit(NULL, forced, 2, 20);
  const uint8_t convert[2] = {0xcc, 0x44};
  clock_us = 1000;
  if (scenario == 4) status = 17;
  int before = calls;
  assert(__wrap_onewire_bus_write_bytes((void *)3, convert, 2) == status);
  assert(calls == before + 1);
  status = 0;
  if (scenario == 5) carrier_observer_begin(0x1128, 0x2228);
  clock_us += scenario == 1 ? 749999 : 750000;
  if (mask & 1) read_rom(scenario == 2 ? 0x3328 : 0x1128);
  if (scenario != 3 && (mask & 2)) read_rom(0x2228);
  assert(carrier_observer_sample_valid(mask, scenario != 23) == (scenario == 0 || scenario == 20 || scenario == 21 || scenario == 23));
  if (scenario == 20 || scenario == 21) {
    assert(!carrier_observer_sample_valid(3, true));
    assert(!carrier_observer_sample_valid(3 ^ mask, true));
    const carrier_observation_t seen = carrier_observer_snapshot();
    assert(seen.soil_mv[0] == 216 && seen.soil_mv[1] == 248);
  }
  return 0;
}
''')
    binary = root / 'observer'
    subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-Wno-unused-parameter', '-fsanitize=address,undefined',
                    '-fno-omit-frame-pointer', '-g', '-I', str(root), '-I', str(APP / 'main'),
                    str(harness), '-o', str(binary)], check=True, capture_output=True, text=True)
    return binary


@pytest.mark.parametrize('scenario', [*range(18), 20, 21, 22, 23, 24], ids=[
    'fresh', 'too_early', 'wrong_rom', 'missing_read', 'write_failure',
    'previous_epoch', 'cleanup', 'cleanup_initializes', 'cleanup_enables', 'restart_cleanup',
    'wrong_stabilization', 'wrong_attenuation', 'adc_error', 'missing_gap', 'wrong_gap',
    'wrong_average', 'bme_before_off', 'off_failed', 'missing_ds0', 'missing_ds1', 'read_only_bme', 'missing_bme', 'bme_raw_forwarding'])
def test_observer_evidence_and_forwarding(observer_binary, scenario):
    subprocess.run([str(observer_binary), str(scenario)], check=True, capture_output=True, text=True)
