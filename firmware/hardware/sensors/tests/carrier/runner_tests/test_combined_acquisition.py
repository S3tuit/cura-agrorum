"""Combined-stage ordering and evidence at serial/operator dependency boundaries."""
import hashlib
import json
from types import SimpleNamespace

import pytest

import carrier_evidence
import carrier_guided as guided
import carrier_runner as runner
from carrier_evidence import Evidence
from carrier_reading import validate_reading
from pytest_sensor_carrier import test_sensor_carrier as run_hardware
from reading_fixture import serial_reading
from test_runner import build_dir, scripted_dut


@pytest.mark.parametrize('fixture', ['nominal', 'missing_ds0', 'missing_ds1', 'missing_bme280', 'adc_reference'])
@pytest.mark.parametrize('outcome', ['PASS', 'FAIL', 'IGNORE', 'ZERO', 'bad_body', 'wrong_case'])
def test_both_stages_use_real_unity_parser_and_fresh_boots(scripted_dut, tmp_path, fixture, outcome):
    dut, build, state = scripted_dut
    state.fixture = fixture
    original = dut.write

    def write(command):
        if command != '7':
            return original(command)
        state.commands.append(command)
        output = serial_reading(fixture)
        if outcome == 'bad_body':
            output = output.replace(b'pending_body=00', b'pending_body=01')
        state.feed(output.decode())
        state.result('unexpected case' if outcome == 'wrong_case' else f'carrier {fixture} core reading',
                     'PASS' if outcome in {'bad_body', 'wrong_case'} else outcome)

    dut.write = write
    evidence = Evidence(tmp_path / 'combined.json', {'fixture': fixture}, {})
    operation = 'adc-reference' if fixture == 'adc_reference' else 'acquire'
    # This matrix covers orchestration/Unity failures; actual guided reference
    # input and acceptance are exercised below with the real Guided class.
    procedure = (SimpleNamespace(before_sample=lambda **kwargs: None, hold=lambda *args: None,
                                 acquisition_result=lambda: None, core_result=lambda result: None)
                 if operation == 'adc-reference' else None)

    def run():
        runner.run_operation(dut, build, operation, lambda *args: None,
                             fixture=fixture, evidence=evidence, guided=procedure)

    if outcome == 'PASS':
        run()
        assert [e['stage'] for e in evidence.data['events'] if e['kind'] == 'stage_completed'] == ['acquisition', 'core']
        assert len([e for e in evidence.data['events'] if e['kind'] == 'core_reading']) == 1
    else:
        with pytest.raises((AssertionError, ValueError)):
            run()
        assert [e['stage'] for e in evidence.data['events'] if e['kind'] == 'stage_completed'] == ['acquisition']
        assert not any(e['kind'] == 'software_completed' for e in evidence.data['events'])
    assert state.commands == ['', '2', '', '5', *([] if operation == 'adc-reference' else ['auto']), '', '7']
    assert state.resets == 2


def test_missing_core_case_rejected_before_fixture_preflight(scripted_dut):
    dut, build, state = scripted_dut
    state.omit_core = True
    with pytest.raises(ValueError, match='selected 0 Unity cases'):
        runner.run_operation(dut, build, 'acquire', lambda *args: None)
    assert state.commands == [''] and state.resets == 0


def test_second_boot_revalidates_image_before_core(scripted_dut):
    dut, build, state = scripted_dut
    original = dut.serial.hard_reset

    def reset():
        if state.resets == 1:
            state.elf = '0' * 64
        original()

    dut.serial.hard_reset = reset
    with pytest.raises(ValueError, match='DUT boot elf'):
        runner.run_operation(dut, build, 'acquire', lambda *args: None)
    assert state.commands == ['', '2', '', '5', 'auto']
    assert state.resets == 2


@pytest.mark.parametrize('valid_meter', [False, True])
def test_guided_acquire_preserves_hold_before_core_and_acceptance(
        scripted_dut, tmp_path, monkeypatch, valid_meter):
    dut, build, state = scripted_dut
    evidence = Evidence(tmp_path / 'guided.json', {}, {})
    procedure = guided.Guided(evidence, 'acquire', None, None)
    clock = SimpleNamespace(now=0)
    monkeypatch.setattr(guided, 'time', SimpleNamespace(monotonic=lambda: clock.now))

    def ask(prompt, *, deadline):
        assert state.resets == 1 and '7' not in state.commands
        clock.now = 10
        return f'TP_3V3=3.3 TP_GATE=3.3 TP_SW={"0" if valid_meter else ".2"} TP_DQ=0'

    monkeypatch.setattr(guided, 'ask', ask)

    def run():
        runner.run_operation(dut, build, 'acquire', lambda *args: None,
                             evidence=evidence, guided=procedure)
        procedure.complete()

    if valid_meter:
        run()
        assert evidence.data['status'] == 'accepted'
        assert state.commands == ['', '2', '', '5', 'guided', 'CARRIER_HOLD_ACK sample-return', '', '7']
        assert [e['stage'] for e in evidence.data['events'] if e['kind'] == 'stage_completed'] == ['acquisition', 'core']
    else:
        with pytest.raises(AssertionError, match='meter observations'):
            run()
        assert state.resets == 1 and '7' not in state.commands
        assert not any(e['kind'] == 'stage_completed' for e in evidence.data['events'])


@pytest.mark.parametrize('stages', [[], ['acquisition'], ['core'], ['core', 'acquisition'],
                                    ['acquisition', 'core', 'core'], ['acquisition', 'core']])
def test_meter_hold_alone_cannot_accept_acquire(tmp_path, stages):
    evidence = Evidence(tmp_path / 'incomplete.json', {}, {})
    procedure = guided.Guided(evidence, 'acquire', None, None)
    procedure.holds = ['sample-return']
    for stage in stages:
        evidence.add('stage_completed', stage=stage)
    if stages == ['acquisition', 'core']:
        procedure.complete()
        assert evidence.data['status'] == 'accepted'
    else:
        with pytest.raises(ValueError, match='stages must complete'):
            procedure.complete()
        assert evidence.data['status'] == 'incomplete'


def reference_metadata():
    return dict(operation='adc-reference', fixture='adc_reference',
                fixture_sha256=carrier_evidence.fixture_manifest(), expected_dut='001122334455',
                elf_sha256='same-image', config_sha256='same-config', rom0='ROM0', rom1='ROM1')


def completed_reference_a(path, metadata):
    evidence = Evidence(path, metadata, {})
    procedure = guided.Guided(evidence, 'adc-reference', 'A', None)
    observed = validate_reading(serial_reading('adc_reference'), 'adc_reference')
    for stage in ('acquisition', 'core'):
        procedure.reference_stage = stage
        procedure.measurements = {'TP_3V3': '3.3', 'TP_ADC0': '1.181', 'TP_ADC1': '1.640'}
        if stage == 'acquisition':
            procedure.sample = observed['sample']
            procedure.acquisition_result()
        else:
            procedure.core_result(observed)
        evidence.add('stage_completed', stage=stage)
    evidence.add('software_completed', operation='adc-reference')
    with pytest.raises(guided.IncompleteCase, match='requires position B'):
        procedure.complete()
    return evidence


@pytest.mark.parametrize('position', ['A', 'B'])
@pytest.mark.parametrize('failure', [None, 'missing_core_meter', 'wrong_core_meter', 'wrong_acquisition_meter'])
def test_adc_pair_uses_fresh_inputs_for_each_actual_stage(
        scripted_dut, tmp_path, monkeypatch, position, failure):
    dut, build, state = scripted_dut
    state.fixture = 'adc_reference'
    output = serial_reading('adc_reference', position=position)
    state.sample_serial = b''.join(output.splitlines(keepends=True)[1:]).decode()
    state.core_serial = output
    metadata = reference_metadata()
    prior = None
    if position == 'B':
        a = completed_reference_a(tmp_path / 'a.json', metadata)
        prior = guided.prior_position(a.path, metadata, 'B')
    evidence = Evidence(tmp_path / 'run.json', metadata, {})
    procedure = guided.Guided(evidence, 'adc-reference', position, prior)
    calls, prompts = [], []
    monkeypatch.setattr(guided, 'confirm', prompts.append)

    def ask(prompt, *, deadline):
        calls.append(state.resets)
        if len(calls) == 2 and failure == 'missing_core_meter':
            raise guided.IncompleteCase('missing fresh core reference')
        low, high = ('1.181', '1.640') if len(calls) == 1 else ('1.190', '1.650')
        if ((len(calls) == 1 and failure == 'wrong_acquisition_meter') or
                (len(calls) == 2 and failure == 'wrong_core_meter')):
            low = '.500'
        first, second = (low, high) if position == 'A' else (high, low)
        return f'TP_3V3=3.3 TP_ADC0={first} TP_ADC1={second}'

    monkeypatch.setattr(guided, 'ask', ask)

    def run():
        runner.run_operation(dut, build, 'adc-reference', lambda *args: None,
                             fixture='adc_reference', evidence=evidence, guided=procedure)
        procedure.complete()

    if failure:
        with pytest.raises((guided.IncompleteCase, AssertionError)):
            run()
        assert evidence.data['status'] == 'incomplete'
        assert not any(e['kind'] == 'position_result' for e in evidence.data['events'])
        if failure == 'wrong_acquisition_meter':
            assert calls == [1] and state.resets == 1 and '7' not in state.commands
    else:
        if position == 'A':
            with pytest.raises(guided.IncompleteCase, match='requires position B'):
                run()
            assert evidence.data['status'] == 'position_A_complete_sequence_incomplete'
        else:
            run()
            assert evidence.data['status'] == 'accepted'
            assert prior['sha256'] == hashlib.sha256(a.path.read_bytes()).hexdigest()
        assert calls == [1, 2]
        assert 'fresh acquisition boot' in prompts[0] and 'fresh core boot' in prompts[1]
        saved, = [e for e in evidence.data['events'] if e['kind'] == 'position_result']
        assert set(saved['stages']) == {'acquisition', 'core'}
        assert saved['stages']['acquisition']['volts'] != saved['stages']['core']['volts']


def test_core_cannot_reuse_component_reference_measurements(tmp_path):
    evidence = Evidence(tmp_path / 'no-fresh-core.json', reference_metadata(), {})
    procedure = guided.Guided(evidence, 'adc-reference', 'A', None)
    observed = validate_reading(serial_reading('adc_reference'), 'adc_reference')
    procedure.sample = observed['sample']
    procedure.reference_stage = 'acquisition'
    procedure.measurements = {'TP_3V3': '3.3', 'TP_ADC0': '1.181', 'TP_ADC1': '1.640'}
    procedure.acquisition_result()
    with pytest.raises(guided.IncompleteCase, match='fresh reference measurements required for core'):
        procedure.core_result(observed)


@pytest.mark.parametrize('mutation', ['legacy', 'missing_core_comparison', 'missing_core_completion',
                                     'duplicate_stage', 'wrong_operation'])
def test_adc_b_rejects_single_stage_or_incompatible_a(tmp_path, mutation):
    metadata = reference_metadata()
    a = completed_reference_a(tmp_path / 'a.json', metadata)
    if mutation == 'legacy':
        a.data['events'] = [e for e in a.data['events'] if e['kind'] != 'stage_completed']
    elif mutation == 'missing_core_completion':
        a.data['events'] = [e for e in a.data['events'] if not (e['kind'] == 'stage_completed' and e['stage'] == 'core')]
    elif mutation == 'duplicate_stage':
        a.add('stage_completed', stage='core')
    elif mutation == 'missing_core_comparison':
        result, = [e for e in a.data['events'] if e['kind'] == 'position_result']
        del result['stages']['core']
    else:
        a.data['metadata'] = {**a.data['metadata'], 'operation': 'reading'}
    a.save()
    with pytest.raises(ValueError):
        guided.prior_position(a.path, metadata, 'B')


@pytest.mark.parametrize('operation', ['reading', 'core-reading'])
def test_internal_or_removed_operation_is_not_public(operation):
    assert operation not in runner.OPERATIONS
    with pytest.raises(ValueError):
        runner.validate_selection(operation, 'nominal', True)
    request = SimpleNamespace(config=SimpleNamespace(getoption=lambda name: operation if name == 'sensor_operation' else None),
                              getfixturevalue=lambda name: pytest.fail('DUT must not be opened'))
    with pytest.raises(pytest.UsageError):
        run_hardware(request, lambda *args: None)


@pytest.mark.parametrize('is_guided,position', [(False, None), (False, 'A'), (True, None)])
def test_adc_options_rejected_before_dut(build_dir, tmp_path, monkeypatch, is_guided, position):
    monkeypatch.setattr(__import__('pytest_sensor_carrier'), 'verify_build', lambda *args: {})
    options = dict(sensor_operation='adc-reference', sensor_fixture='adc_reference',
                   sensor_guided=is_guided, sensor_position=position,
                   app_path=str(runner.APP), target='esp32s3', embedded_services='esp,idf',
                   count=1, port='/dev/null', build_dir=str(build_dir), sensor_dut='001122334455',
                   sensor_repeat_count=100, root_logdir=str(tmp_path / 'run'))
    request = SimpleNamespace(config=SimpleNamespace(getoption=options.get),
                              getfixturevalue=lambda name: pytest.fail('must not request DUT'))
    with pytest.raises(pytest.UsageError):
        run_hardware(request, lambda *args: None)


def test_legacy_adc_a_rejected_before_requesting_dut(build_dir, tmp_path, monkeypatch):
    monkeypatch.setattr(__import__('pytest_sensor_carrier'), 'verify_build', lambda *args: {})
    build = runner.load_build(build_dir, 'adc-reference')
    metadata = {**reference_metadata(), 'elf_sha256': build.elf_sha256,
                'config_sha256': build.config_sha256,
                'rom0': build.boot_values['rom0'], 'rom1': build.boot_values['rom1']}
    a = completed_reference_a(tmp_path / 'legacy-a.json', metadata)
    a.data['events'] = [e for e in a.data['events'] if e['kind'] != 'stage_completed']
    a.save()
    options = dict(sensor_operation='adc-reference', sensor_fixture='adc_reference',
                   sensor_guided=True, sensor_position='B', sensor_prior_evidence=str(a.path),
                   app_path=str(runner.APP), target='esp32s3', embedded_services='esp,idf',
                   count=1, port='/dev/null', build_dir=str(build_dir), sensor_dut='001122334455',
                   sensor_repeat_count=100, root_logdir=str(tmp_path / 'run'))
    request = SimpleNamespace(config=SimpleNamespace(getoption=options.get),
                              getfixturevalue=lambda name: pytest.fail('legacy A reached DUT access'))
    with pytest.raises(ValueError, match='stages must complete'):
        run_hardware(request, lambda *args: None)
