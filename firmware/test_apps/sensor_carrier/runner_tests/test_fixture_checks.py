"""Test the app's actual acceptance assertions, not simulated DUT acquisitions."""
import os
import subprocess

import pytest

from carrier_runner import APP


@pytest.fixture(scope='module')
def checks_binary(tmp_path_factory):
    root = tmp_path_factory.mktemp('carrier-checks')
    # Only the Unity assertion sink is substituted. Production value layouts
    # and the actual app-local assertions are compiled unchanged.
    (root/'unity.h').write_text('''#include <stdlib.h>
#define CHECK(x) do { if (!(x)) exit(42); } while (0)
#define TEST_ASSERT_TRUE(x) CHECK(x)
#define TEST_ASSERT_TRUE_MESSAGE(x,m) CHECK(x)
#define TEST_ASSERT_EQUAL(a,b) CHECK((a)==(b))
#define TEST_ASSERT_EQUAL_MESSAGE(a,b,m) CHECK((a)==(b))
#define TEST_ASSERT_EQUAL_HEX8(a,b) CHECK((a)==(b))
#define TEST_ASSERT_EQUAL_HEX32(a,b) CHECK((a)==(b))
#define TEST_ASSERT_EQUAL_INT16(a,b) CHECK((a)==(b))
#define TEST_ASSERT_EQUAL_UINT32(a,b) CHECK((a)==(b))
''')
    (root/'esp_err.h').write_text('#define ESP_ERR_NOT_FOUND 0x105\n')
    harness = root/'checks.c'
    harness.write_text('''#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include "carrier_checks.h"
int main(int argc, char **argv) {
  assert(argc >= 4);
  unsigned mask = atoi(argv[2]);
  if (strcmp(argv[1], "inventory") == 0) {
    const uint64_t configured[2] = {0x1128, 0x2228};
    uint64_t roms[8];
    assert(argc <= 12);
    for (int i = 4; i < argc; ++i) roms[i-4] = strtoull(argv[i], NULL, 16);
    carrier_assert_inventory(configured, roms, argc-4, atoi(argv[3]), mask);
  } else {
    const char *mutation = argv[3];
    node_sensor_sample_t sample = {.soil_0_mv=2000, .soil_1_mv=2700,
      .validity=mask==1 ? 0x17 : mask==2 ? 0x1b : 0x1f};
    /* Zero is allowed for valid DS/enclosure values: no plausibility oracle. */
    diagn_context_t diag = {0};
    err_curag_t result = mask==3 ? 0 : 0x00030002;
    if (mask != 3) {
      diag.operation=1; diag.context_schema=1; diag.context_length=48;
      unsigned offset = mask==1 ? 32 : 24;
      diag.context[offset]=2; diag.context[offset+4]=5; diag.context[offset+5]=1;
    }
    if (strcmp(mutation, "result")==0) result=0;
    if (strcmp(mutation, "operation")==0) diag.operation=3;
    if (strcmp(mutation, "schema")==0) diag.context_schema=2;
    if (strcmp(mutation, "length")==0) diag.context_length=47;
    if (strcmp(mutation, "validity")==0) sample.validity=0x1f;
    if (strcmp(mutation, "survivor")==0) sample.validity ^= mask==1 ? 4 : 8;
    if (strcmp(mutation, "soil0_invalid")==0) sample.validity ^= 1;
    if (strcmp(mutation, "soil1_invalid")==0) sample.validity ^= 2;
    if (strcmp(mutation, "enclosure_invalid")==0) sample.validity ^= 16;
    if (strcmp(mutation, "absent_value")==0) {
      if (mask==1) sample.soil_temp_1_centi_c=1; else sample.soil_temp_0_centi_c=1;
    }
    if (strcmp(mutation, "soil0_low")==0) sample.soil_0_mv=1999;
    if (strcmp(mutation, "soil0_high")==0) sample.soil_0_mv=2701;
    if (strcmp(mutation, "soil1_low")==0) sample.soil_1_mv=1999;
    if (strcmp(mutation, "soil1_high")==0) sample.soil_1_mv=2701;
    if (strcmp(mutation, "byte")==0) {
      assert(argc==5); unsigned byte=atoi(argv[4]); assert(byte<sizeof(diag.context));
      diag.context[byte] ^= 1;
    }
    carrier_assert_sample(&sample, &diag, result, mask, true);
  }
}
''')
    binary = root/'checks'
    components = APP.parents[1]/'components'
    result = subprocess.run([
        os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra', '-Werror',
        '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-g',
        '-I', str(root), '-I', str(APP/'main'),
        '-I', str(components/'node_sensors/include'),
        '-I', str(components/'node_common/include'),
        str(harness), str(APP/'main/carrier_checks.c'), '-o', str(binary),
    ], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    return binary


@pytest.mark.parametrize('mask', [1, 2, 3])
@pytest.mark.parametrize('roms', [[], ['1128'], ['2228'], ['3328'], ['1128', '2228'],
                                ['2228', '1128'], ['1128', '1128'], ['2228', '2228'],
                                ['1128', '3328'], ['1128', '2228', '3328']])
def test_exact_declared_rom_inventory(checks_binary, mask, roms):
    expected = {1: {'1128'}, 2: {'2228'}, 3: {'1128', '2228'}}[mask]
    passes = set(roms) == expected and len(roms) == len(expected)
    run = subprocess.run([str(checks_binary), 'inventory', str(mask), str(len(roms)), *roms],
                         capture_output=True, text=True)
    assert run.returncode == (0 if passes else 42), run.stderr


@pytest.mark.parametrize('mask,roms', [(1, ['1128']), (2, ['2228']), (3, ['1128', '2228'])])
def test_non_ds_inventory_cannot_pass(checks_binary, mask, roms):
    run = subprocess.run([str(checks_binary), 'inventory', str(mask), '0', *roms],
                         capture_output=True, text=True)
    assert run.returncode == 42, run.stderr


@pytest.mark.parametrize('mask', [1, 2, 3])
def test_contract_values_and_valid_zero_groups(checks_binary, mask):
    subprocess.run([str(checks_binary), 'sample', str(mask), 'none'], check=True, capture_output=True)


@pytest.mark.parametrize('mask', [1, 2])
@pytest.mark.parametrize('mutation', ['result', 'operation', 'schema', 'length', 'validity',
                                    'survivor', 'soil0_invalid', 'soil1_invalid', 'enclosure_invalid',
                                    'absent_value', 'soil0_low', 'soil0_high', 'soil1_low', 'soil1_high'])
def test_wrong_missing_sample_fails(checks_binary, mask, mutation):
    run = subprocess.run([str(checks_binary), 'sample', str(mask), mutation], capture_output=True, text=True)
    assert run.returncode == 42, run.stderr


@pytest.mark.parametrize('mask', [1, 2])
@pytest.mark.parametrize('byte', [*range(48), 48, 251])
def test_every_diagnostic_pair_byte_and_unused_tail(checks_binary, mask, byte):
    run = subprocess.run([str(checks_binary), 'sample', str(mask), 'byte', str(byte)],
                         capture_output=True, text=True)
    assert run.returncode == 42, run.stderr
