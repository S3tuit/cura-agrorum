"""Serial/evidence boundary regressions, not sensor or electrical validation."""
import json
from types import SimpleNamespace

import pytest

import carrier_evidence
import carrier_runner as runner
from carrier_reading import validate_reading


from reading_fixture import BODY, serial_reading


@pytest.mark.parametrize('fixture', ['nominal', 'missing_ds0', 'missing_ds1', 'missing_bme280', 'adc_reference'])
def test_exact_fixture_values_and_body(fixture):
    observed = validate_reading(serial_reading(fixture), fixture)
    assert observed['reading']['run_ms'] == 1345
    assert observed['body'] == observed['pending_body']
    assert observed['reading']['soil0_mv'] == observed['sample']['soil0_mv']


@pytest.mark.parametrize('bit', range(1, 8))
def test_each_sensor_protocol_flag_is_required(bit):
    changed = bytearray(BODY)
    changed[30] ^= 1 << bit
    serial = serial_reading().replace(BODY.hex().encode(), changed.hex().encode())
    with pytest.raises(AssertionError, match='flags/metadata'):
        validate_reading(serial, 'nominal')


@pytest.mark.parametrize('offset', [6, 8, 10, 12, 14, 16, 20])
def test_each_same_acquisition_value_must_reach_body(offset):
    changed = bytearray(BODY)
    changed[offset] ^= 1
    serial = serial_reading().replace(BODY.hex().encode(), changed.hex().encode())
    with pytest.raises(AssertionError, match='same core acquisition'):
        validate_reading(serial, 'nominal')


@pytest.mark.parametrize('mutation', ['persisted', 'short_body', 'duplicate_sample', 'duplicate_body',
                                     'no_body', 'no_sample', 'no_diagnostic', 'wrong_fixture',
                                     'sample_twice', 'rx', 'duration', 'extra_flags', 'air_range',
                                     'wrong_diagnostic', 'duplicate_field'])
def test_incomplete_or_inconsistent_output_cannot_pass(mutation):
    serial = serial_reading()
    fixture = 'nominal'
    if mutation == 'persisted':
        serial = serial.replace(b'pending_body=00', b'pending_body=01')
    elif mutation == 'short_body':
        serial = serial.replace(BODY.hex().encode(), b'00')
    elif mutation in {'duplicate_sample', 'duplicate_body'}:
        serial += serial.splitlines(keepends=True)[1 if mutation == 'duplicate_sample' else 0]
    elif mutation in {'no_body', 'no_sample', 'no_diagnostic'}:
        marker = {'no_body': b'CARRIER_READING', 'no_sample': b'CARRIER_SAMPLE',
                  'no_diagnostic': b'CARRIER_DIAGNOSTIC'}[mutation]
        serial = b''.join(line for line in serial.splitlines(keepends=True) if not line.startswith(marker))
    elif mutation == 'wrong_fixture':
        fixture = 'missing_ds0'
    elif mutation == 'sample_twice':
        serial = serial.replace(b'samples=1', b'samples=2')
    elif mutation == 'rx':
        serial = serial.replace(b'rx_calls=0', b'rx_calls=1')
    elif mutation == 'duration':
        serial = serial.replace(b'duration_us=1340000', b'duration_us=30000001')
    elif mutation == 'extra_flags':
        serial = serial.replace(BODY.hex().encode(), (BODY[:-1] + b'\x04').hex().encode())
    elif mutation == 'air_range':
        serial = serial_reading('adc_reference')
    elif mutation == 'wrong_diagnostic':
        serial = serial.replace(b'context=00', b'context=01')
    else:
        serial = serial.replace(b'samples=1', b'samples=1 samples=1')
    with pytest.raises((AssertionError, ValueError)):
        validate_reading(serial, fixture)


@pytest.mark.parametrize('fixture,offset,field', [('missing_ds0', 10, 'temp0_centi_c'),
                                               ('missing_ds1', 12, 'temp1_centi_c'),
                                               ('missing_bme280', 14, 'enclosure_centi_c'),
                                               ('missing_bme280', 16, 'pressure_pa'),
                                               ('missing_bme280', 20, 'humidity_centi_pct')])
def test_invalid_fields_cannot_be_nonzero_in_both_sample_and_body(fixture, offset, field):
    serial = serial_reading(fixture)
    old = validate_reading(serial, fixture)['body']
    body = bytearray.fromhex(old)
    body[offset] = 1
    serial = serial.replace(old.encode(), body.hex().encode()).replace(f'{field}=0'.encode(), f'{field}=1'.encode())
    with pytest.raises(AssertionError, match='must be zero'):
        validate_reading(serial, fixture)


@pytest.mark.parametrize('path', ['firmware/components/node_core/node_core.c',
                                 'firmware/components/node_persistence/node_persistence_backend_esp.c',
                                 'firmware/components/protocol_v2_lora/protocol_v2_lora_crypto.c',
                                 'firmware/tests/support/persistence_test_support.c',
                                 'firmware/hardware/sensors/tests/carrier/sdkconfig'])
def test_new_dependency_change_rejects_stale_seal(carrier_source_tree, tmp_path, monkeypatch, path):
    repo, _ = carrier_source_tree
    sources = carrier_evidence.source_manifest()
    assert path in sources
    (tmp_path/'carrier-build.json').write_text(json.dumps(dict(
        sources=sources, bosch_bme280={}, elf_sha256='elf', config_sha256='config')))
    monkeypatch.setattr(carrier_evidence, 'bosch_manifest', lambda _: {})
    build = SimpleNamespace(elf_sha256='elf', config_sha256='config')
    assert carrier_evidence.verify_build(tmp_path, build)['sources'] == sources
    (repo / path).write_text('changed source or configuration')
    with pytest.raises(ValueError, match='source/build manifest mismatch'):
        carrier_evidence.verify_build(tmp_path, build)
