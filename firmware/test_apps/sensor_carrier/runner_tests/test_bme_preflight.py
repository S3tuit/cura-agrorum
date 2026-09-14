"""Actual app fixture transport: fake only the ESP-IDF I2C boundary."""
import os
import subprocess
import pytest
from carrier_runner import APP


@pytest.fixture(scope='module')
def probe_binary(tmp_path_factory):
    root = tmp_path_factory.mktemp('bme-preflight')
    host = APP.parents[1] / 'tests/host'
    harness = root / 'probe.c'
    harness.write_text('''#include <assert.h>
#include <stdlib.h>
#include "driver/i2c_master.h"
#include "carrier_bme_preflight.h"
struct fake_i2c_bus { int unused; }; struct fake_i2c_device { int unused; };
static struct fake_i2c_bus bus; static struct fake_i2c_device dev;
static int scenario, reads, probes, adds, removes, deletes;
esp_err_t i2c_new_master_bus(const i2c_master_bus_config_t *cfg, i2c_master_bus_handle_t *out) {
  assert(cfg->sda_io_num == 21 && cfg->scl_io_num == 22 && cfg->i2c_port == 0);
  if (scenario == 6) return ESP_ERR_NO_MEM;
  *out = &bus; return 0;
}
esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t b, const i2c_device_config_t *cfg, i2c_master_dev_handle_t *out) {
  assert(b == &bus && cfg->device_address == 0x76 && cfg->scl_speed_hz == 100000);
  ++adds; if (scenario == 7) return ESP_ERR_NO_MEM;
  *out = &dev; return 0;
}
esp_err_t i2c_master_probe(i2c_master_bus_handle_t b, uint16_t addr, int timeout) {
  assert(b == &bus && addr == 0x76 && timeout == 20); ++probes;
  return scenario == 2 ? 0 : scenario == 3 ? ESP_ERR_TIMEOUT :
         scenario == 4 ? ESP_FAIL : ESP_ERR_NOT_FOUND;
}
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t d, const uint8_t *reg, size_t n, uint8_t *data, size_t len, int timeout) {
  assert(d == &dev && n == 1 && *reg == 0xd0 && len == 1 && timeout == 20); ++reads;
  *data = scenario == 8 ? 0x58 : 0x60;
  return scenario == 9 ? ESP_ERR_TIMEOUT : 0;
}
esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t d) {
  assert(d == &dev); ++removes; return scenario == 10 ? ESP_FAIL : 0;
}
esp_err_t i2c_del_master_bus(i2c_master_bus_handle_t b) {
  assert(b == &bus); ++deletes; return scenario == 5 || scenario == 11 ? ESP_FAIL : 0;
}
int main(int argc, char **argv) {
  assert(argc == 2); scenario = atoi(argv[1]);
  const bool missing = scenario >= 1 && scenario <= 6;
  const int expected[] = {0, 0, 264, 263, -1, -1, 257, 257, 264, 263, -1, -1};
  assert(carrier_bme_preflight(missing) == expected[scenario]);
  assert(deletes == (scenario != 6));
  if (missing) assert(adds == 0 && reads == 0 && removes == 0 && probes == (scenario != 6));
  else assert(probes == 0 && adds == 1 && reads == (scenario != 7) && removes == (scenario != 7));
}
''')
    binary = root / 'probe'
    subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-g',
                    '-DCONFIG_CURA_I2C_SDA_GPIO=21', '-DCONFIG_CURA_I2C_SCL_GPIO=22',
                    '-I', str(host/'bme280_fakes'), '-I', str(host/'fakes'), '-I', str(APP/'main'),
                    str(harness), str(APP/'main/carrier_bme_preflight.c'), '-o', str(binary)],
                   check=True, capture_output=True, text=True)
    return binary


@pytest.mark.parametrize('scenario', range(12), ids=[
    'nominal', 'absent', 'responding_is_not_absent', 'timeout_is_not_absent',
    'generic_error_is_not_absent', 'absent_cleanup_failure', 'bus_allocation',
    'device_allocation', 'wrong_id', 'read_error', 'device_cleanup', 'bus_cleanup'])
def test_fixture_preflight(probe_binary, scenario):
    subprocess.run([str(probe_binary), str(scenario)], check=True, capture_output=True)
