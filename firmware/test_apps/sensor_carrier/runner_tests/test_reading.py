"""Serial/evidence boundary regressions, not sensor or electrical validation."""
import hashlib
import json
import struct
from types import SimpleNamespace

import pytest

import carrier_evidence
import carrier_guided
import carrier_runner as runner
from carrier_evidence import Evidence
from carrier_reading import validate_reading
from pytest_sensor_carrier import test_sensor_carrier as run_hardware
from test_runner import build_dir, scripted_dut


# Reviewed wire-layout example: ID=0, run=1345 ms, soil=2301/2402 mV,
# temperatures=-1234/2345/2500 centi-C, pressure=100123 Pa, humidity=4567,
# POWERON=1, no previous metrics, seven sensor flags. These bytes are local
# parser test input; target expectations never come from this fixture builder.
BODY = bytes.fromhex('00000000 4105 fd08 6209 2efb 2909 c409 1b870100 d711 '
                     '01 00 0000 0000 00 00 fe00')


def serial_reading(fixture='nominal'):
    body = bytearray(BODY)
    validity = {'nominal': 0x1f, 'adc_reference': 0x1f, 'missing_ds0': 0x1b,
                'missing_ds1': 0x17, 'missing_bme280': 0x0f}[fixture]
    flags = {'nominal': 0xfe, 'adc_reference': 0xfe, 'missing_ds0': 0xf6,
             'missing_ds1': 0xee, 'missing_bme280': 0x1e}[fixture]
    sample = dict(result=0 if validity == 0x1f else 0x30002, duration_us=1_340_000,
                  validity=validity, soil0_mv=2301, soil1_mv=2402,
                  temp0_centi_c=-1234, temp1_centi_c=2345,
                  enclosure_centi_c=2500, pressure_pa=100123, humidity_centi_pct=4567)
    if fixture == 'adc_reference':
        sample.update(soil0_mv=1183, soil1_mv=1645)
        body[6:10] = bytes.fromhex('9f04 6d06')
    if fixture == 'missing_ds0':
        sample['temp0_centi_c'] = 0
        body[10:12] = b'\0\0'
    if fixture == 'missing_ds1':
        sample['temp1_centi_c'] = 0
        body[12:14] = b'\0\0'
    if fixture == 'missing_bme280':
        sample.update(enclosure_centi_c=0, pressure_pa=0, humidity_centi_pct=0)
        body[14:22] = b'\0' * 8
    body[30:32] = bytes((flags, 0))
    context = bytearray(252)
    if fixture.startswith('missing'):
        offset = {'missing_ds0': 24, 'missing_ds1': 32, 'missing_bme280': 40}[fixture]
        struct.pack_into('<Ii', context, offset, 1 if fixture == 'missing_bme280' else 2,
                         264 if fixture == 'missing_bme280' else 261)
    sample_line = ' '.join(f'{key}={value:08x}' if key in {'validity', 'result'} else f'{key}={value}'
                           for key, value in sample.items())
    partial = fixture.startswith('missing')
    return (f'CARRIER_READING samples=1 tx_calls=1 rx_calls=0 terminal_calls=1 '
            f'sleep_us=900000000 body={body.hex()} pending_body={body.hex()}\n'
            f'CARRIER_SAMPLE {sample_line}\n'
            f'CARRIER_DIAGNOSTIC operation={int(partial)} schema={int(partial)} '
            f'length={48 if partial else 0} context={context.hex()}\n').encode()


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


@pytest.mark.parametrize('fixture', ['nominal', 'missing_ds0', 'missing_ds1', 'missing_bme280', 'adc_reference'])
@pytest.mark.parametrize('outcome', ['PASS', 'FAIL', 'IGNORE', 'ZERO', 'bad_body', 'wrong_case'])
def test_real_unity_parser_and_fresh_boot_sequence(scripted_dut, tmp_path, fixture, outcome):
    dut, build, state = scripted_dut
    state.fixture = fixture if fixture.startswith('missing') else 'nominal'
    old_write = dut.write
    def write(command):
        if command == '':
            state.commands.append(command)
            state.feed("Here's the test menu, pick your combo:\n"
                       f'(2)\t"carrier {state.fixture} preflight" [sensor_carrier]\n'
                       f'(7)\t"carrier {fixture} core reading" [sensor_carrier]\n'
                       'Enter test for running.\n')
        elif command == '7':
            state.commands.append(command)
            serial = serial_reading(fixture)
            if outcome == 'bad_body':
                serial = serial.replace(b'pending_body=00', b'pending_body=01')
            state.feed(serial.decode())
            state.result('unexpected case' if outcome == 'wrong_case' else f'carrier {fixture} core reading',
                         'PASS' if outcome in {'bad_body', 'wrong_case'} else outcome)
        else:
            old_write(command)
    dut.write = write
    evidence = Evidence(tmp_path/'reading.json', {'fixture': fixture}, {})
    def run():
        runner.run_operation(dut, build, 'reading', lambda *a: None, fixture=fixture, evidence=evidence)
    if outcome == 'PASS':
        run()
        assert len([e for e in evidence.data['events'] if e['kind'] == 'core_reading']) == 1
    else:
        with pytest.raises((AssertionError, ValueError)):
            run()
        assert not any(e['kind'] == 'software_completed' for e in evidence.data['events'])
    assert state.commands == ['', '2', '', '7']
    assert state.resets == 1


def test_reading_adc_requires_actual_pair_and_fresh_measurements(tmp_path):
    metadata = dict(operation='reading', fixture='adc_reference', carrier_revision='R12 fitted',
                    expected_dut='001122334455', elf_sha256='same-image', config_sha256='same-config',
                    rom0='ROM0', rom1='ROM1')
    a = Evidence(tmp_path/'a.json', metadata, {})
    procedure = carrier_guided.Guided(a, 'reading', 'A', None)
    observed = validate_reading(serial_reading('adc_reference'), 'adc_reference')
    procedure.reading_result(observed)
    procedure.measurements = {'TP_3V3': '3.3', 'TP_ADC0': '1.181', 'TP_ADC1': '1.640'}
    a.add('software_completed', operation='reading')
    with pytest.raises(carrier_guided.IncompleteCase, match='requires position B'):
        procedure.complete()
    prior = carrier_guided.prior_position(a.path, metadata, 'B')
    assert prior['sha256'] == hashlib.sha256(a.path.read_bytes()).hexdigest()
    b = Evidence(tmp_path/'b.json', metadata, {})
    second = carrier_guided.Guided(b, 'reading', 'B', prior)
    second.reading_result(observed)
    # Reusing A's measurements/routing cannot accept position B.
    second.measurements = procedure.measurements
    with pytest.raises(AssertionError, match='A/B routing'):
        second.complete()
    second.sample = {**observed['sample'], 'soil0_mv': 1645, 'soil1_mv': 1183}
    second.measurements = {'TP_3V3': '3.3', 'TP_ADC0': '1.640', 'TP_ADC1': '1.181'}
    second.complete()
    assert b.data['status'] == 'accepted'
    with pytest.raises(ValueError, match='operation'):
        carrier_guided.prior_position(a.path, {**metadata, 'operation': 'adc-reference'}, 'B')


@pytest.mark.parametrize('guided,position', [(False, None), (False, 'A'), (True, None)])
def test_adc_options_rejected_before_dut(build_dir, tmp_path, monkeypatch, guided, position):
    monkeypatch.setattr(__import__('pytest_sensor_carrier'), 'verify_build', lambda *a: {})
    options = dict(sensor_operation='reading', sensor_fixture='adc_reference',
                   sensor_guided=guided, sensor_position=position,
                   app_path=str(runner.APP), target='esp32c6', embedded_services='esp,idf',
                   count=1, port='/dev/null', build_dir=str(build_dir), sensor_dut='001122334455',
                   sensor_repeat_count=100, root_logdir=str(tmp_path/'run'))
    request = SimpleNamespace(config=SimpleNamespace(getoption=options.get),
                              getfixturevalue=lambda _: pytest.fail('must not request DUT'))
    with pytest.raises(pytest.UsageError):
        run_hardware(request, lambda *a: None)


@pytest.mark.parametrize('path', ['firmware/components/node_core/node_core.c',
                                 'firmware/components/node_persistence/node_persistence_backend_esp.c',
                                 'firmware/components/protocol_v2_lora/protocol_v2_lora_crypto.c',
                                 'firmware/test_apps/on_device/main/persistence_test_support.c'])
def test_new_dependency_change_rejects_stale_seal(tmp_path, monkeypatch, path):
    sources = carrier_evidence.source_manifest()
    assert path in sources
    (tmp_path/'carrier-build.json').write_text(json.dumps(dict(
        sources=sources, bosch_bme280={}, elf_sha256='elf', config_sha256='config')))
    monkeypatch.setattr(carrier_evidence, 'bosch_manifest', lambda _: {})
    monkeypatch.setattr(carrier_evidence, 'source_manifest', lambda: {**sources, path: 'changed'})
    with pytest.raises(ValueError, match='source/build manifest mismatch'):
        carrier_evidence.verify_build(tmp_path, SimpleNamespace(elf_sha256='elf', config_sha256='config'))
