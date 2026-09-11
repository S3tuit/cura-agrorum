"""Exploration evidence and real local runner at its stdin/UART boundaries."""
import io
import json
import subprocess
from types import SimpleNamespace

import pytest

import carrier_exploration as exploration
import carrier_runner as runner
from carrier_evidence import Evidence
from carrier_guided import IncompleteCase, prior_position, backpower_original
from pytest_sensor_carrier import test_sensor_carrier as run_hardware_test
from test_runner import build_dir, scripted_dut
from test_hold import hold_binary


@pytest.mark.parametrize('arrival', [0, 601000000, 3600000000])
def test_actual_exploration_hold_has_no_observation_deadline(hold_binary, arrival):
    run = subprocess.run([str(hold_binary), 'exploration', str(arrival),
                          'CARRIER_HOLD_ACK sample-return\n'],
                         check=True, capture_output=True, text=True)
    assert 'seconds=unlimited' in run.stdout
    assert f'HARNESS_RESULT 1 {arrival}\n' in run.stdout


def inputs(monkeypatch, text):
    stream = io.StringIO(text)
    monkeypatch.setattr(exploration.sys, 'stdin', stream)
    monkeypatch.setattr(exploration, 'select', SimpleNamespace(select=lambda *a: ([stream], [], [])))


def test_notes_are_saved_verbatim_immediately_beyond_ten_minutes(scripted_dut, tmp_path, monkeypatch):
    dut, _, _ = scripted_dut
    dut.expect_exact('Press ENTER to see the list of tests.')
    evidence = Evidence(tmp_path/'evidence.json', {'exploration': True}, {})
    case = exploration.Exploration(evidence, 'gate-off')
    case.dut = dut
    notes = [' TP_SW about 150mV, still falling ', 'C3 restored; 100 kΩ absent', '']
    inputs(monkeypatch, '\n'.join(notes + ['/done']) + '\n')
    clock = iter([0, 601, 900, 1200, 1500])
    monkeypatch.setattr(exploration, 'time', SimpleNamespace(monotonic=lambda: next(clock)))
    saved = evidence.save
    observed = []
    def save():
        saved()
        observed.append(json.loads(evidence.path.read_text())['events'][-1])
    monkeypatch.setattr(evidence, 'save', save)
    case.hold('gate-off', b'real UART prefix', None)
    entries = [e for e in observed if e['kind'] == 'exploration_note']
    assert [e['text'] for e in entries] == notes
    assert [e['elapsed_seconds'] for e in entries] == [601, 900, 1200]
    assert all(e['at'] and e['hold'] == 'gate-off' for e in entries)
    with pytest.raises(IncompleteCase, match='not an acceptance'):
        case.complete()
    assert json.loads(evidence.path.read_text())['status'] == 'exploration_complete_not_acceptance'


@pytest.mark.parametrize('marker', ['CARRIER_BOOT wrong\n', 'CARRIER_HOLD_END gate-off\n',
                                   'Guru Meditation Error\n', 'task_wdt: IDLE starved\n'])
def test_state_change_stops_notes_before_old_state_can_be_reused(scripted_dut, tmp_path, monkeypatch, marker):
    dut, _, state = scripted_dut
    dut.expect_exact('Press ENTER to see the list of tests.')
    evidence = Evidence(tmp_path/'evidence.json', {}, {})
    case = exploration.Exploration(evidence, 'gate-off')
    case.dut = dut
    inputs(monkeypatch, 'must not become an old-state observation\n/done\n')
    state.feed(marker)
    with pytest.raises(RuntimeError, match='changed state'):
        case.hold('gate-off', b'', None)
    assert not any(e['kind'] == 'exploration_note' for e in evidence.data['events'])
    assert evidence.data['events'][-1]['kind'] == 'exploration_state_lost'


def test_eof_does_not_finish_observation(scripted_dut, tmp_path, monkeypatch):
    dut, _, _ = scripted_dut
    dut.expect_exact('Press ENTER to see the list of tests.')
    evidence = Evidence(tmp_path/'evidence.json', {}, {})
    case = exploration.Exploration(evidence, 'gate-off')
    case.dut = dut
    inputs(monkeypatch, 'partial note\n')
    with pytest.raises(IncompleteCase, match='before /done'):
        case.hold('gate-off', b'', None)
    assert case.holds == []
    assert evidence.data['status'] == 'incomplete'


@pytest.mark.parametrize('failure', ['removed', 'replaced', 'reader_stopped'])
def test_uart_loss_detected_even_when_log_stays_open(tmp_path, failure):
    port = tmp_path/'uart'
    port.symlink_to('/dev/null')
    evidence = Evidence(tmp_path/'evidence.json', {'port': str(port)}, {})
    reader = SimpleNamespace(is_alive=lambda: True)
    dut = SimpleNamespace(serial=SimpleNamespace(_redirect_thread=reader))
    case = exploration.Exploration(evidence, 'gate-off')
    case.bind(dut)
    if failure == 'reader_stopped':
        reader.is_alive = lambda: False
    else:
        port.unlink()
        if failure == 'replaced':
            port.symlink_to('/dev/zero')
    with pytest.raises(RuntimeError, match='UART disconnected'):
        case._check_state('gate-off')
    assert evidence.data['events'][-1]['kind'] == 'exploration_state_lost'


@pytest.mark.parametrize('operation', ['reset', 'held-reset', 'deep-sleep'])
def test_exploration_transition_keeps_real_menu_reset_checks(scripted_dut, tmp_path, monkeypatch, operation):
    dut, build, state = scripted_dut
    dut.expect_exact('Press ENTER to see the list of tests.')
    unity = SimpleNamespace(index=9, name=runner.CASES[operation], type='multi_stage',
                            is_ignored=False, subcases=[dict(index=1, name='one'), dict(index=2, name='two')])
    selected = [None]
    commands = []
    def write(command):
        commands.append(command)
        if command == '9':
            state.feed('(1) "one"\n(2) "two"\n')
        elif command in {'1', '2'}:
            selected[0] = command
            if command == '2' and operation == 'held-reset':
                state.result(unity.name, 'PASS')
            else:
                state.feed('CARRIER_HOLD_MODE\n')
        elif command == 'exploration':
            if selected[0] == '2' and operation == 'deep-sleep':
                state.result(unity.name, 'PASS')
            else:
                name = 'transition-on' if selected[0] == '1' else 'reset-off'
                state.feed(f'CARRIER_HOLD_READY {name} seconds=unlimited\n')
        elif command == '':
            state.feed("Here's the test menu, pick your combo:\n"
                       f'(9)\t"{unity.name}" [sensor_carrier][multi_stage]\n'
                       '\t(1)\t"one"\n\t(2)\t"two"\nEnter test for running.\n')
        else:
            name = command.removeprefix('CARRIER_HOLD_ACK ')
            state.feed(f'CARRIER_HOLD_ACKED {name} elapsed_us=601000000\nCARRIER_HOLD_END {name}\n')
            if name == 'transition-on':
                suffix = ' seconds=unlimited' if operation == 'deep-sleep' else ''
                state.feed(f'CARRIER_TRANSITION {operation}{suffix}\n')
                if operation == 'reset':
                    state.feed('CARRIER_RESTART_CLEANUP calls=1 result=00000000 diagnostic_empty=1 observer_valid=1\n')
                    state.boot()
            else:
                state.result(unity.name, 'PASS')
    dut.write = write
    evidence = Evidence(tmp_path/'evidence.json', {'exploration': True}, {})
    case = exploration.Exploration(evidence, operation)
    case.dut = dut
    inputs(monkeypatch, 'on note\n/done\noff note\n/done\n')
    confirmations = []
    def confirm(prompt):
        confirmations.append(prompt)
        if prompt.startswith(('Release', 'Press and release')):
            state.boot()
    monkeypatch.setattr(exploration, 'confirm', confirm)
    runner.execute_transition(dut, build, [unity], operation, '001122334455', case, evidence)
    assert len(dut.testsuite.testcases) == 1
    assert dut.testsuite.testcases[0].result == 'PASS'
    with pytest.raises(IncompleteCase, match='not an acceptance'):
        case.complete()
    assert 'auto' not in commands and 'guided' not in commands
    assert bool(confirmations) == (operation != 'reset')


@pytest.mark.parametrize('options', [dict(sensor_operation='repeat'), dict(sensor_guided=True),
                                   dict(sensor_diagnostic_load=True), dict(sensor_position='A'),
                                   dict(sensor_prior_evidence='old.json')])
def test_exploration_invalid_combinations_fail_before_dut(options):
    options = dict({'sensor_operation': 'gate-off', 'exploration': True}, **options)
    request = SimpleNamespace(config=SimpleNamespace(getoption=options.get),
                              getfixturevalue=lambda name: pytest.fail('DUT must not be opened'))
    with pytest.raises(pytest.UsageError, match='--exploration requires'):
        run_hardware_test(request, lambda *a: None)


@pytest.mark.parametrize('reader', [prior_position, backpower_original])
def test_exploration_cannot_supply_acceptance_prerequisite(tmp_path, reader):
    path = tmp_path/'exploration.json'
    path.write_text(json.dumps({'metadata': {'exploration': True}, 'status': 'position_A_complete_sequence_incomplete'}))
    with pytest.raises(ValueError, match='exploration cannot'):
        reader(path, {}, *(['B'] if reader is prior_position else []))
