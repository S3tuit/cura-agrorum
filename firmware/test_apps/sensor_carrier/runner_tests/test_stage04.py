"""New orchestration failure paths, with serial input outside the runner layer."""
import json
from types import SimpleNamespace
import pytest
from test_runner import build_dir, scripted_dut  # Local fixtures have a second real use.
import carrier_runner as runner
from carrier_evidence import Evidence
from carrier_guided import (IncompleteCase, measured_volts, validate_reference,
                            validate_identity, validate_electrical, prior_position)


@pytest.mark.parametrize('text', ['', 'TP_ADC0=nan TP_ADC1=1 TP_3V3=3.3',
                                  'TP_ADC0=1 TP_ADC0=2 TP_3V3=3.3',
                                  'TP_ADC0=-1 TP_ADC1=1 TP_3V3=3.3'])
def test_missing_invalid_or_duplicate_meter_input(text):
    with pytest.raises((IncompleteCase, ValueError)):
        measured_volts(text, {'TP_3V3', 'TP_ADC0', 'TP_ADC1'})


def test_adc_75mv_is_inclusive_and_swap_uses_fresh_measurements():
    a = dict(TP_3V3='3.298', TP_ADC0='1.180', TP_ADC1='1.641')
    validate_reference(dict(soil0_mv=1255, soil1_mv=1566), a, 'A')
    with pytest.raises(AssertionError):
        validate_reference(dict(soil0_mv=1256, soil1_mv=1566), a, 'A')
    b = dict(TP_3V3='3.300', TP_ADC0='1.648', TP_ADC1='1.183')
    validate_reference(dict(soil0_mv=1648, soil1_mv=1183), b, 'B')
    with pytest.raises(AssertionError):
        validate_reference(dict(soil0_mv=1183, soil1_mv=1648), b, 'B')


@pytest.mark.parametrize('difference,passes', [(199, False), (200, True), (201, True), (-200, False)])
def test_warmed_physical_identity_requires_clear_difference(difference, passes):
    sample = dict(temp0_centi_c=2500+difference, temp1_centi_c=2500)
    if passes:
        validate_identity(sample, 0)
    else:
        with pytest.raises(IncompleteCase):
            validate_identity(sample, 0)


def test_electrical_limits_and_missing_points():
    off = dict(TP_3V3='3.1', TP_GATE='3.0', TP_SW='.1', TP_DQ='.1')
    validate_electrical('deep-sleep', off)
    with pytest.raises(AssertionError):
        validate_electrical('deep-sleep', {**off, 'TP_SW': '.1001'})
    with pytest.raises(IncompleteCase):
        validate_electrical('deep-sleep', {'TP_GATE': '3.3'})
    on = dict(TP_3V3='3.3', TP_GATE='.2', TP_SW='3.2', TP_DQ='3.1', TP_ADC0='2', TP_ADC1='2.7')
    validate_electrical('gate-on', on)
    with pytest.raises(AssertionError):
        validate_electrical('gate-on', {**on, 'TP_ADC0': '1.999'})


def test_pair_cannot_accept_missing_or_wrong_image_a(tmp_path):
    metadata = dict(operation='ds-identity', fixture='nominal', carrier_revision='r1',
                    expected_dut='001122334455', elf_sha256='one', config_sha256='config',
                    rom0='rom0', rom1='rom1')
    evidence = Evidence(tmp_path/'a.json', metadata, {})
    with pytest.raises(ValueError):
        prior_position(evidence.path, metadata, 'B')
    evidence.add('position_result', position='A', warmed_channel=0)
    evidence.add('software_completed')
    evidence.finish('position_A_complete_sequence_incomplete')
    assert prior_position(evidence.path, metadata, 'B')['run_id'] == evidence.data['run_id']
    with pytest.raises(ValueError):
        prior_position(evidence.path, {**metadata, 'elf_sha256': 'different'}, 'B')
    with pytest.raises(IncompleteCase):
        prior_position(None, metadata, 'B')
    with pytest.raises(FileExistsError):
        Evidence(evidence.path, metadata, {})


@pytest.mark.parametrize('failure', [None, 'short', 'gap', 'duplicate', 'extra', 'reset', 'IGNORE', 'FAIL'])
def test_repetition_requires_complete_requested_count(scripted_dut, failure):
    dut, build, state = scripted_dut
    dut.expect_exact('Press ENTER to see the list of tests.')
    case = SimpleNamespace(index=6, name=runner.CASES['repeat'])
    def write(command):
        if command == '6':
            state.feed('CARRIER_REPEAT_COUNT\n')
        else:
            assert command == '100'
            indices = list(range(1, 101))
            if failure == 'short': indices.pop()
            if failure == 'gap': indices.remove(50)
            if failure == 'duplicate': indices[49] = 49
            if failure == 'extra': indices.append(101)
            for i in indices:
                if failure == 'reset' and i == 50:
                    state.feed('CARRIER_BOOT bad\n')
                state.feed(f'CARRIER_ITERATION index={i} total=100\n')
            state.feed('CARRIER_REPEAT_DONE total=100\n')
            state.result(case.name, failure if failure in {'IGNORE', 'FAIL'} else 'PASS')
    dut.write = write
    if failure:
        with pytest.raises((AssertionError, RuntimeError)):
            runner.execute_case(dut, case, 'repeat')
    else:
        runner.execute_case(dut, case, 'repeat')



def test_late_or_missing_meter_input_is_incomplete(monkeypatch):
    import carrier_guided as guided
    monkeypatch.setattr(guided.time, 'monotonic', lambda: 100)
    with pytest.raises(IncompleteCase):
        guided.ask('measurement', deadline=99)


@pytest.mark.parametrize('operation, hold', [
    ('gate-off', 'gate-off'), ('acquire', 'sample-return'),
    ('final-cleanup', 'final-cleanup'), ('reset', 'reset-off'),
    ('held-reset', 'held-reset'), ('deep-sleep', 'deep-sleep'),
])
@pytest.mark.parametrize('legacy_diagnostic', [False, True])
def test_failed_off_measurement_retains_evidence_without_adding_load(
        tmp_path, monkeypatch, capsys, operation, hold, legacy_diagnostic):
    import carrier_guided as guided
    evidence = Evidence(tmp_path/'failed-meter.json', {}, {})
    case = guided.Guided(evidence, operation, None, None,
                         diagnostic_original={'run_id': 'prior'} if legacy_diagnostic else None)
    clock = [100.0]
    monkeypatch.setattr(guided, 'time', SimpleNamespace(monotonic=lambda: clock[0]))

    def answer(prompt, *, deadline):
        assert deadline == 280.0  # Preserve the existing 15-second result margin.
        clock[0] += 10.0
        return 'TP_3V3=3.3 TP_GATE=3.28 TP_SW=0.101 TP_DQ=0.002'

    monkeypatch.setattr(guided, 'ask', answer)
    with pytest.raises(AssertionError):
        case.measure_hold(hold, 295.0)

    saved = json.loads(evidence.path.read_text())
    assert saved['status'] != 'accepted'
    meter, = saved['events']
    assert meter['kind'] == 'meter' and meter['hold'] == hold
    assert meter['volts']['TP_SW'] == '0.101'
    assert meter['settling_seconds'] == 10 and meter['stable_updates'] == 3
    assert meter['loaded'] is legacy_diagnostic
    message = capsys.readouterr().out
    assert 'Keep this failure' in message
    assert 'keep permanent R12' in message
    assert 'Do not add another discharge resistor' in message
    assert '--exploration' in message
    assert 'remove power before any wiring change' in message
    assert 'Open-circuit' not in message
    assert '--sensor-diagnostic-load' not in message


def test_manual_sequence_cannot_complete_without_all_holds(tmp_path):
    from carrier_guided import Guided
    evidence = Evidence(tmp_path/'sleep.json', {}, {})
    case = Guided(evidence, 'deep-sleep', None, None)
    case.holds = ['transition-on']
    with pytest.raises(IncompleteCase):
        case.complete()
    assert evidence.data['status'] == 'incomplete'


# Manual-completion sleep coverage is in test_manual_sleep.py. The former
# full timer-dwell assertions were replaced by the approved electrical procedure.
