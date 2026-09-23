"""Execute the target's acquisition/hold ordering with dependencies outside it."""
import os
import subprocess

import pytest

from carrier_runner import APP


@pytest.fixture(scope='module')
def acquisition_binary(tmp_path_factory):
    root = tmp_path_factory.mktemp('carrier-acquisition-hold')
    source = (APP/'main/test_sensor_carrier.c').read_text()
    # Compile the actual functions verbatim. ESP-IDF acquisition, observations
    # and the Unity failure sink are the dependency boundary, not the subject.
    acquisition = source[source.index('static err_curag_t acquire_sample('):
                         source.index('static void print_observation(')]
    fixture = source[source.index('static void fixture_acquisition('):
                     source.index('TEST_CASE("carrier nominal acquisition')]
    harness = root/'acquisition.c'
    harness.write_text(r'''
#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "node_sensors.h"

static unsigned holds, timer_calls, sample_calls, checked_mask;
static int mode;
static bool held, unexpected, checked_bme;
static int64_t duration;
static node_sensor_sample_t returned_sample;
static diagn_context_t returned_diagnostic;
static err_curag_t returned_result;

#define TEST_ASSERT_TRUE_MESSAGE(value, message) do { \
  if (!(value)) { printf("FAIL holds=%u %s\n", holds, message); exit(42); } \
} while (0)

static void fresh_acquisition(uint64_t configured[2]) {
  configured[0] = 0x1128; configured[1] = 0x2228;
}
static int carrier_hold_select(void) { return mode; }
static int64_t esp_timer_get_time(void) {
  ++timer_calls;
  assert(timer_calls <= 2); /* Holding must not change acquisition duration. */
  return timer_calls == 1 ? 100 : 100 + duration;
}
err_curag_t node_sensors_sample_all(node_sensor_sample_t *sample,
                                  diagn_context_t *diagnostic) {
  assert(holds == 0 && ++sample_calls == 1);
  *sample = returned_sample;
  *diagnostic = returned_diagnostic;
  return returned_result;
}
static void print_sample(node_sensor_sample_t sample, diagn_context_t diagnostic,
                         err_curag_t result, int64_t elapsed) {
  assert(memcmp(&sample, &returned_sample, sizeof(sample)) == 0);
  assert(memcmp(&diagnostic, &returned_diagnostic, sizeof(diagnostic)) == 0);
  assert(result == returned_result && elapsed == duration);
  printf("returned=%" PRId64 "\n", elapsed);
}
static void carrier_observer_begin(uint64_t first, uint64_t second) {
  assert(first == 0x1128 && second == 0x2228 && sample_calls == 0);
}
static void print_observation(void) { assert(sample_calls == 1); }
static bool carrier_hold_wait(const char *name, int selected) {
  assert(strcmp(name, "sample-return") == 0 && selected == mode);
  assert(sample_calls == 1 && timer_calls == 2 && ++holds == 1);
  puts("HOLD");
  return held;
}
static void carrier_assert_sample(const node_sensor_sample_t *sample,
                                  const diagn_context_t *diagnostic,
                                  err_curag_t result, unsigned ds_mask,
                                  bool air_soil, bool bme_present) {
  assert(holds == 1 && held && ds_mask == checked_mask && air_soil);
  assert(bme_present == checked_bme && result == returned_result);
  assert(memcmp(sample, &returned_sample, sizeof(*sample)) == 0);
  assert(memcmp(diagnostic, &returned_diagnostic, sizeof(*diagnostic)) == 0);
  TEST_ASSERT_TRUE_MESSAGE(!unexpected, "unexpected sample");
}
static void assert_observation(const node_sensor_sample_t *sample,
                               unsigned ds_mask, bool bme_present) {
  assert(sample->soil_0_mv == 2300 && ds_mask == checked_mask);
  assert(bme_present == checked_bme && holds == 1);
}
''' + acquisition + fixture + r'''
int main(int argc, char **argv) {
  assert(argc == 7);
  duration = strtoll(argv[1], NULL, 10);
  checked_mask = (unsigned)atoi(argv[2]); checked_bme = atoi(argv[3]);
  mode = atoi(argv[4]); held = atoi(argv[5]); unexpected = atoi(argv[6]);
  returned_sample.soil_0_mv = 2300;
  returned_sample.validity = checked_bme ? (checked_mask == 3 ? 0x1f :
                                           checked_mask == 2 ? 0x1b : 0x17) : 0x0f;
  returned_result = returned_sample.validity == 0x1f ? 0 : 0x00030002;
  returned_diagnostic.operation = returned_result ? 1 : 0;
  fixture_acquisition(checked_mask, checked_bme);
  assert(holds == 1 && sample_calls == 1 && timer_calls == 2);
  puts("PASS");
}
''')
    components = APP.parents[1]/'components'
    binary = root/'acquisition'
    build = subprocess.run([
        os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra', '-Werror',
        '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-g',
        '-I', str(components/'node_sensors/include'),
        '-I', str(components/'node_common/include'),
        str(harness), '-o', str(binary),
    ], capture_output=True, text=True)
    assert build.returncode == 0, build.stderr
    return binary


@pytest.mark.parametrize('ds_mask,bme', [(3, True), (2, True), (1, True), (3, False)])
@pytest.mark.parametrize('mode', [0, 1])
@pytest.mark.parametrize('duration', [0, 30_000_000, 30_000_001, -1])
def test_every_return_reaches_hold_before_duration_assertion(
        acquisition_binary, ds_mask, bme, mode, duration):
    run = subprocess.run([str(acquisition_binary), str(duration), str(ds_mask),
                          str(int(bme)), str(mode), '1', '0'], capture_output=True, text=True)
    assert run.stdout.count('HOLD\n') == 1, run.stdout + run.stderr
    assert f'returned={duration}\n' in run.stdout
    assert run.returncode == (0 if 0 <= duration <= 30_000_000 else 42), run.stderr
    if run.returncode:
        assert 'FAIL holds=1 production sample did not return within 30 seconds' in run.stdout


@pytest.mark.parametrize('ds_mask,bme', [(3, True), (2, True), (1, True), (3, False)])
@pytest.mark.parametrize('held,unexpected', [(False, False), (True, True)])
def test_failed_hold_or_unexpected_sample_still_fails(
        acquisition_binary, ds_mask, bme, held, unexpected):
    run = subprocess.run([str(acquisition_binary), '1234567', str(ds_mask),
                          str(int(bme)), '1', str(int(held)), str(int(unexpected))],
                         capture_output=True, text=True)
    assert run.returncode == 42, run.stdout + run.stderr
    assert run.stdout.count('HOLD\n') == 1
    assert ('guided hold incomplete' if not held else 'unexpected sample') in run.stdout
