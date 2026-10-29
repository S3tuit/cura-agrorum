"""Bind the host reference protocol to the actual target case and hold output."""
import os
import subprocess
from types import SimpleNamespace

import pytest

import carrier_runner as runner
from test_runner import build_dir, scripted_dut


@pytest.fixture(scope='module')
def reference_binary(tmp_path_factory):
    root = tmp_path_factory.mktemp('reference-hold')
    (root / 'freertos').mkdir()
    (root / 'esp_rom_serial_output.h').write_text(
        '#include <stdint.h>\nint esp_rom_output_rx_one_char(uint8_t *byte);\n')
    (root / 'esp_timer.h').write_text(
        '#include <stdint.h>\nint64_t esp_timer_get_time(void);\n')
    (root / 'freertos/FreeRTOS.h').write_text('#define pdMS_TO_TICKS(ms) ((ms) / 10U)\n')
    (root / 'freertos/task.h').write_text('void vTaskDelay(unsigned ticks);\n')
    source = (runner.APP / 'main/test_sensor_carrier.c').read_text()
    hold = source[source.index('static void observation_hold('):
                  source.index('TEST_CASE("carrier production gate-on hold')]
    reference = source[source.index('TEST_CASE("carrier reference acquisition'):
                       source.index('static int transition_mode(')]
    harness = root / 'reference.c'
    harness.write_text(r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "carrier_hold.h"
#include "node_sensors.h"

static int64_t now;
static unsigned samples, checks, delays;
#define TEST_CASE(name, tags) static void reference_case(void)
#define TEST_ASSERT_TRUE(value) assert(value)
int64_t esp_timer_get_time(void) { return now; }
int esp_rom_output_rx_one_char(uint8_t *byte) {
  (void)byte;
  assert(!"reference case must not wait for a host mode/ack command");
  return -1;
}
void vTaskDelay(unsigned ticks) {
  assert(samples == 1 && checks == 0 && ++delays == 1);
  assert(ticks == 6001);
  now += ticks * INT64_C(10000);
}
static void fresh_acquisition(uint64_t configured[2]) {
  assert(samples == 0);
  configured[0] = 0x1128; configured[1] = 0x2228;
}
static err_curag_t acquire_sample(node_sensor_sample_t *sample,
                                  diagn_context_t *diagnostic,
                                  int64_t *duration_us) {
  assert(++samples == 1 && delays == 0);
  *sample = (node_sensor_sample_t){.soil_0_mv = 1155, .soil_1_mv = 1622};
  *diagnostic = (diagn_context_t){0};
  *duration_us = 1351652;
  puts("CARRIER_SAMPLE result=00000000 duration_us=1351652 soil0_mv=1155 soil1_mv=1622");
  return 0;
}
static void assert_acquisition_duration(int64_t duration_us) {
  assert(delays == 1 && duration_us == 1351652);
}
static void assert_sample(const node_sensor_sample_t *sample,
                          const diagn_context_t *diagnostic,
                          err_curag_t result, bool nominal) {
  assert(delays == 1 && ++checks == 1 && result == 0 && !nominal);
  assert(sample->soil_0_mv == 1155 && sample->soil_1_mv == 1622);
  assert(diagnostic->operation == 0);
}
''' + hold + reference + r'''
int main(void) {
  reference_case();
  assert(samples == 1 && checks == 1 && delays == 1 && now == 60010000);
}
''')
    components = runner.APP.parents[3] / 'components'
    binary = root / 'reference'
    subprocess.run([
        os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra', '-Werror',
        '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-g',
        '-I', str(root), '-I', str(runner.APP / 'main'),
        '-I', str(components / 'node_sensors/include'),
        '-I', str(components / 'node_common/include'),
        '-I', str(runner.APP / 'runner_tests/native'), str(harness),
        str(runner.APP / 'main/carrier_hold.c'),
        str(runner.APP / 'main/carrier_output.c'),
        str(runner.APP / 'runner_tests/native/unity_output.c'), '-o', str(binary),
    ], check=True, capture_output=True, text=True)
    return binary


@pytest.mark.parametrize('outcome', ['PASS', 'FAIL', 'IGNORE', 'ZERO', 'missing_end'])
def test_actual_reference_case_needs_no_mode_command(reference_binary, scripted_dut, outcome):
    output = subprocess.run([str(reference_binary)], check=True, capture_output=True, text=True).stdout
    assert 'CARRIER_HOLD_MODE' not in output
    assert output.index('CARRIER_SAMPLE') < output.index('CARRIER_HOLD_READY sample-return seconds=60')
    assert output.endswith('CARRIER_HOLD_END sample-return\n')
    dut, _, state = scripted_dut
    dut.expect_exact('Press ENTER to see the list of tests.')
    case = SimpleNamespace(index=5, name=runner.CASES['adc-reference'])
    commands, observed = [], []

    def write(command):
        commands.append(command)
        assert commands == ['5']
        state.feed(output.replace('CARRIER_HOLD_END sample-return\n', '')
                   if outcome == 'missing_end' else output)
        if outcome != 'missing_end':
            state.result(case.name, outcome)

    dut.write = write
    if outcome == 'PASS':
        runner.execute_case(dut, case, 'adc-reference', on_hold=lambda *args: observed.append(args))
    else:
        with pytest.raises((AssertionError, RuntimeError, ValueError)):
            runner.execute_case(dut, case, 'adc-reference', on_hold=lambda *args: observed.append(args))
    assert commands == ['5']
    assert len(observed) == 1
    assert observed[0][0] == 'sample-return' and b'soil0_mv=1155 soil1_mv=1622' in observed[0][1]
