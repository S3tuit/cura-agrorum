"""Missing sensor selections at the runner's serial/operator boundary."""
import importlib.util
import json
from types import SimpleNamespace

import pytest

import carrier_guided as guided
import carrier_runner as runner
from carrier_evidence import Evidence
from pytest_sensor_carrier import test_sensor_carrier as run_hardware_test
from test_runner import build_dir, scripted_dut


@pytest.mark.parametrize('fixture', ['missing_ds0', 'missing_ds1', 'missing_bme280'])
@pytest.mark.parametrize('operation', sorted(runner.OPERATIONS))
def test_missing_fixture_only_selects_acquire(fixture, operation):
    if operation == 'acquire':
        runner.validate_selection(operation, fixture, 'R12 fitted', True)
    else:
        with pytest.raises(ValueError):
            runner.validate_selection(operation, fixture, 'R12 fitted', True)


@pytest.mark.parametrize('fixture', ['missing_ds0', 'missing_ds1', 'missing_bme280'])
@pytest.mark.parametrize('phase,outcome', [('preflight', 'PASS'), ('preflight', 'FAIL'),
                                         ('preflight', 'IGNORE'), ('preflight', 'ZERO'),
                                         ('sample', 'FAIL'), ('sample', 'IGNORE'), ('sample', 'ZERO')])
def test_declared_preflight_fresh_boot_and_exact_acquisition(scripted_dut, tmp_path, fixture, phase, outcome):
    dut, build, state = scripted_dut
    state.fixture = fixture
    setattr(state, phase, outcome)
    survivor = '8877665544332228' if fixture == 'missing_ds0' else '1122334455667728'
    state.inventory = f'CARRIER_ROM value={survivor} family=28 type=DS18B20\n'
    if fixture == 'missing_bme280':
        state.inventory += 'CARRIER_ROM value=8877665544332228 family=28 type=DS18B20\nCARRIER_BME_ABSENT address=76 probe=261\n'
    state.sample_serial = 'CARRIER_SAMPLE result=00030002\nCARRIER_DIAGNOSTIC selected_backend_status\n'
    evidence = Evidence(tmp_path/'evidence.json', {'fixture': fixture}, {})
    def run():
        runner.run_operation(dut, build, 'acquire', lambda *a: None, fixture=fixture, evidence=evidence)
    if outcome == 'PASS':
        run()
        assert state.commands == ['', '2', '', '5', 'auto']
        assert [c.name for c in dut.testsuite.testcases] == [
            f'carrier {fixture} preflight', f'carrier {fixture} acquisition and sample-return hold']
        events = json.loads(evidence.path.read_text())['events']
        assert survivor in next(e for e in events if e['kind'] == 'preflight')['serial']
        assert state.sample_serial in next(e for e in events if e['kind'] == 'observation_ready')['serial']
    else:
        with pytest.raises((AssertionError, ValueError)):
            run()
        assert not any(e['kind'] == 'software_completed' for e in evidence.data['events'])
    preflight_failed = phase == 'preflight' and outcome != 'PASS'
    assert state.resets == (0 if preflight_failed else 1)
    assert state.commands.count('5') == (0 if preflight_failed else 1)


@pytest.mark.parametrize('fixture', ['missing_ds0', 'missing_ds1', 'missing_bme280'])
def test_nominal_menu_cannot_substitute_for_requested_missing_case(scripted_dut, fixture):
    dut, build, state = scripted_dut
    with pytest.raises(ValueError, match='selected 0 Unity cases'):
        runner.run_operation(dut, build, 'acquire', lambda *a: None, fixture=fixture)
    assert state.commands == ['']


@pytest.mark.parametrize('fixture', ['missing_ds0', 'missing_ds1', 'missing_bme280'])
def test_missing_fixture_exploration_rejected_before_dut(fixture):
    options = dict(sensor_operation='acquire', sensor_fixture=fixture, exploration=True)
    request = SimpleNamespace(config=SimpleNamespace(getoption=options.get),
                              getfixturevalue=lambda _: pytest.fail('must not open DUT'))
    with pytest.raises(pytest.UsageError, match='--exploration requires'):
        run_hardware_test(request, lambda *a: None)


@pytest.mark.parametrize('fixture,missing', [('missing_ds0', 0), ('missing_ds1', 1)])
def test_guided_wiring_names_both_fixed_roms_and_entire_connector(tmp_path, monkeypatch, fixture, missing):
    evidence = Evidence(tmp_path/'evidence.json', {'fixture': fixture, 'rom0': 'fixed-DS0', 'rom1': 'fixed-DS1'}, {})
    prompts = []
    monkeypatch.setattr(guided, 'confirm', prompts.append)
    procedure = guided.Guided(evidence, 'acquire', None, None)
    procedure.wiring()
    assert len(prompts) == 1
    assert f'logical DS{missing}, ROM fixed-DS{missing}' in prompts[0]
    assert f'Logical DS{1-missing}, ROM fixed-DS{1-missing}, remains connected' in prompts[0]
    assert 'power removed' in prompts[0] and 'power, ground and data connector' in prompts[0]
    with pytest.raises(guided.IncompleteCase, match='not all required electrical'):
        procedure.complete()
    assert evidence.data['status'] != 'accepted'


@pytest.mark.parametrize('fixture', ['missing_ds0', 'missing_ds1', 'missing_bme280'])
@pytest.mark.parametrize('outcome', ['good', 'rail_high', 'missing', 'early', 'late'])
def test_missing_fixture_meter_acceptance_keeps_existing_limits(tmp_path, monkeypatch, fixture, outcome):
    evidence = Evidence(tmp_path/'evidence.json', {'fixture': fixture}, {})
    procedure = guided.Guided(evidence, 'acquire', None, None)
    clock = SimpleNamespace(now=0)
    monkeypatch.setattr(guided, 'time', SimpleNamespace(monotonic=lambda: clock.now))
    def ask(prompt, *, deadline):
        assert deadline == 180
        if outcome == 'late':
            raise guided.IncompleteCase('missing or late operator input')
        clock.now = 9 if outcome == 'early' else 10
        return ('TP_3V3=3.1 TP_GATE=3.0 TP_SW=' + ('.1001' if outcome == 'rail_high' else '.1') +
                ('' if outcome == 'missing' else ' TP_DQ=.1'))
    monkeypatch.setattr(guided, 'ask', ask)
    if outcome == 'good':
        procedure.hold('sample-return', b'', 195)
        procedure.complete()
        assert evidence.data['status'] == 'accepted'
    else:
        with pytest.raises((guided.IncompleteCase, AssertionError)):
            procedure.hold('sample-return', b'', 195)
        assert evidence.data['status'] != 'accepted'
    if outcome == 'rail_high':
        assert evidence.data['events'][-1]['volts']['TP_SW'] == '0.1001'


@pytest.mark.parametrize('fixture', ['missing_ds0', 'missing_ds1', 'missing_bme280'])
def test_requested_operation_with_zero_selected_tests_is_error(fixture):
    spec = importlib.util.spec_from_file_location('carrier_missing_hooks', runner.APP/'conftest.py')
    hooks = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(hooks)
    options = dict(sensor_operation='acquire', sensor_fixture=fixture)
    session = SimpleNamespace(items=[], config=SimpleNamespace(getoption=options.get))
    with pytest.raises(pytest.UsageError, match='selected no hardware test'):
        hooks.pytest_collection_finish(session)


def test_guided_missing_bme_keeps_both_configured_ds(tmp_path, monkeypatch):
    evidence = Evidence(tmp_path/'evidence.json', {'fixture': 'missing_bme280', 'rom0': 'fixed-DS0', 'rom1': 'fixed-DS1'}, {})
    prompts = []
    monkeypatch.setattr(guided, 'confirm', prompts.append)
    procedure = guided.Guided(evidence, 'acquire', None, None)
    procedure.wiring()
    assert 'power removed' in prompts[0] and 'power, ground, SDA and SCL connector' in prompts[0]
    assert 'DS0 ROM fixed-DS0 and DS1 ROM fixed-DS1' in prompts[0]
    with pytest.raises(guided.IncompleteCase):
        procedure.complete()
