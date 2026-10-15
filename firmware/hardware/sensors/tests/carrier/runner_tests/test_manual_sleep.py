"""Real guided procedure/menu/evidence at the UART, operator and clock boundaries."""
import json
from types import SimpleNamespace

import pytest

import carrier_guided as guided
import carrier_runner as runner
from carrier_evidence import Evidence
from test_runner import build_dir, scripted_dut


@pytest.mark.parametrize('failure', [
    None, 'missing_meter', 'invalid_meter', 'before_settling', 'missing_yes',
    'declined_yes', 'late_meter', 'late_yes', 'boot_during_meter', 'boot_during_yes',
    'boot_while_waiting', 'watchdog_while_waiting', 'reader_stopped',
    'wrong_elf', 'wrong_dut', 'wrong_reason', 'ignored', 'zero', 'missing_reset', 'usb_replaced_at_reset',
])
def test_manual_sleep_acceptance_and_failures(scripted_dut, tmp_path, monkeypatch, failure):
    dut, build, state = scripted_dut
    dut.expect_exact('Press ENTER to see the list of tests.')
    unity = SimpleNamespace(index=11, name=runner.CASES['deep-sleep'], type='multi_stage',
                            is_ignored=False, subcases=[dict(index=1, name='one'), dict(index=2, name='two')])
    port = tmp_path/'uart'
    port.symlink_to('/dev/null')
    evidence = Evidence(tmp_path/'evidence.json', {'port': str(port)}, {})
    procedure = guided.Guided(evidence, 'deep-sleep', None, None)
    clock = SimpleNamespace(now=0)
    monkeypatch.setattr(runner, 'time', SimpleNamespace(monotonic=lambda: clock.now))
    monkeypatch.setattr(guided, 'time', SimpleNamespace(monotonic=lambda: clock.now))
    commands, prompts = [], []
    selected = [None]

    def write(command):
        commands.append(command)
        if command == '11':
            state.feed('(1) "one"\n(2) "two"\n')
        elif command in {'1', '2'}:
            selected[0] = command
            state.feed('CARRIER_HOLD_MODE\n')
        elif command == 'guided':
            if selected[0] == '1':
                state.feed('CARRIER_HOLD_READY transition-on seconds=180\n')
            else:
                # The target owns reset-reason validation; a wrong reason
                # reaches orchestration as its real Unity FAIL result.
                result = {'wrong_reason': 'FAIL', 'ignored': 'IGNORE', 'zero': 'ZERO'}.get(failure, 'PASS')
                state.result(unity.name, result)
        elif command == 'CARRIER_HOLD_ACK transition-on':
            state.feed('CARRIER_HOLD_ACKED transition-on elapsed_us=11000000\n'
                       'CARRIER_HOLD_END transition-on\nCARRIER_TRANSITION deep-sleep end=operator-reset\n')
        elif command == '':
            state.feed("Here's the test menu, pick your combo:\n"
                       f'(11)\t"{unity.name}" [sensor_carrier][multi_stage]\n'
                       '\t(1)\t"one"\n\t(2)\t"two"\nEnter test for running.\n')
        else:
            pytest.fail(f'unexpected UART command {command}')

    dut.write = write
    input_index = [0]

    def readline():
        input_index[0] += 1
        if input_index[0] == 1:
            clock.now = 11
            return 'TP_3V3=3.3 TP_GATE=0.001 TP_SW=3.3 TP_DQ=3.3\n'
        if input_index[0] == 2:
            clock.now = 20 if failure == 'before_settling' else 611 if failure == 'late_meter' else 22
            if failure == 'boot_during_meter':
                state.feed('ESP-ROM:esp32c6\n')
            if failure == 'missing_meter':
                return 'TP_3V3=3.3\n'
            return ('TP_3V3=3.3 TP_GATE=3.3 TP_DQ=0.001 TP_SW=' +
                    ('0.101' if failure == 'invalid_meter' else '0.001') + '\n')
        assert input_index[0] == 3  # No second YES after EN/reset.
        # Meter values must already be durable even if attestation fails.
        saved = json.loads(evidence.path.read_text())
        assert saved['events'][-1]['kind'] == 'meter'
        assert saved['events'][-1]['hold'] == 'deep-sleep'
        assert saved['status'] == 'incomplete'
        clock.now = 611 if failure == 'late_yes' else 23
        if failure == 'boot_during_yes':
            state.boot()
        return '' if failure == 'missing_yes' else 'NO\n' if failure == 'declined_yes' else 'YES\n'

    stream = SimpleNamespace(readline=readline)
    monkeypatch.setattr(guided.sys, 'stdin', stream)

    def select_input(readers, _w, _e, timeout):
        if input_index[0] == 1 and failure in {'boot_while_waiting', 'watchdog_while_waiting', 'reader_stopped'}:
            assert timeout <= 0.1  # State loss is checked while no input arrives.
            if failure == 'reader_stopped':
                dut.serial._redirect_thread = SimpleNamespace(is_alive=lambda: False)
            else:
                state.feed('CARRIER_BOOT early\n' if failure == 'boot_while_waiting' else 'task_wdt: IDLE\n')
            return [], [], []
        return readers, [], []

    monkeypatch.setattr(guided, 'select', SimpleNamespace(select=select_input))

    def output(prompt, **kwargs):
        prompts.append(prompt)
        assert 'Press and release EN/reset now' in prompt
        saved = json.loads(evidence.path.read_text())
        assert saved['events'][-1]['kind'] == 'deep_sleep_attestation'
        assert saved['events'][-1]['elapsed_seconds'] == 12
        assert saved['status'] == 'incomplete'
        if failure != 'missing_reset':
            if failure == 'usb_replaced_at_reset':
                port.unlink()
                port.symlink_to('/dev/zero')
            if failure == 'wrong_elf':
                state.elf = '0' * 64
            if failure == 'wrong_dut':
                state.feed('CARRIER_BOOT dut=ffeeddccbbaa elf=' + build.elf_sha256 + ' ' +
                           ' '.join(f'{k}={v}' for k, v in build.boot_values.items()) + '\n')
            else:
                state.boot()

    monkeypatch.setattr(runner, 'print', output, raising=False)
    if failure:
        with pytest.raises((RuntimeError, AssertionError, ValueError, OSError)):
            runner.execute_transition(dut, build, [unity], 'deep-sleep', '001122334455', procedure, evidence)
        assert evidence.data['status'] == 'incomplete'
        assert not any(e['kind'] == 'deep_sleep_operator_reset' for e in evidence.data['events'])
        if failure not in {'wrong_elf', 'wrong_dut', 'wrong_reason', 'ignored', 'zero', 'missing_reset', 'usb_replaced_at_reset'}:
            assert not prompts  # No reset permission after a failed observation/YES.
            assert '2' not in commands
    else:
        runner.execute_transition(dut, build, [unity], 'deep-sleep', '001122334455', procedure, evidence)
        procedure.complete()
        assert evidence.data['status'] == 'accepted'
        assert clock.now == 23  # Completes early, without a timer wait.
        assert len(prompts) == 1 and input_index[0] == 3
        assert evidence.data['events'][-1]['kind'] == 'deep_sleep_operator_reset'
        assert commands == ['11', '1', 'guided', 'CARRIER_HOLD_ACK transition-on', '', '11', '2', 'guided']


@pytest.mark.parametrize('attested,reset', [(False, False), (True, False), (False, True)])
def test_meter_holds_alone_cannot_accept_manual_sleep(tmp_path, attested, reset):
    evidence = Evidence(tmp_path/'evidence.json', {}, {})
    procedure = guided.Guided(evidence, 'deep-sleep', None, None)
    procedure.holds = ['transition-on', 'deep-sleep']
    if attested:
        evidence.add('deep_sleep_attestation')
    if reset:
        evidence.add('deep_sleep_operator_reset')
    with pytest.raises(guided.IncompleteCase, match='YES attestation and verified operator reset'):
        procedure.complete()


@pytest.mark.parametrize('attested_at,boot_deadline', [(20, 50), (599, 615)])
def test_manual_reset_retains_active_and_overall_deadlines(monkeypatch, attested_at, boot_deadline):
    clock = SimpleNamespace(now=0)
    monkeypatch.setattr(runner, 'time', SimpleNamespace(monotonic=lambda: clock.now))
    monkeypatch.setattr(runner, 'ACTIVE_SECONDS', 30)
    unity = SimpleNamespace(name=runner.CASES['deep-sleep'])
    dut = SimpleNamespace(testsuite=SimpleNamespace(testcases=[]), expect_exact=lambda *a, **k: None)
    monkeypatch.setattr(runner, 'select_case', lambda *a: unity)
    monkeypatch.setattr(runner, 'select_stage', lambda *a: None)
    monkeypatch.setattr(runner, 'select_transition_mode', lambda *a: None)
    monkeypatch.setattr(runner, 'acknowledge_hold', lambda *a, **kw: b'')
    monkeypatch.setattr(runner, 'required_marker', lambda *a: b'')

    def observe(dut, started, deadline):
        assert started == 0 and deadline == 615
        clock.now = attested_at
        return None, None

    def boot(dut, build, *, expected_dut, deadline):
        assert deadline == boot_deadline
        raise RuntimeError('no operator reset')

    monkeypatch.setattr(runner, 'boot_menu', boot)
    procedure = SimpleNamespace(hold=lambda *a: None, deep_sleep=observe)
    with pytest.raises(RuntimeError, match='no operator reset'):
        runner.execute_transition(dut, None, [], 'deep-sleep', 'identity', procedure)


@pytest.mark.parametrize('failure', ['removed', 'replaced'])
def test_guided_sleep_detects_uart_loss_while_waiting(scripted_dut, tmp_path, monkeypatch, failure):
    dut, _, _ = scripted_dut
    dut.expect_exact('Press ENTER to see the list of tests.')
    port = tmp_path/'uart'
    port.symlink_to('/dev/null')
    evidence = Evidence(tmp_path/'evidence.json', {'port': str(port)}, {})
    procedure = guided.Guided(evidence, 'deep-sleep', None, None)
    monkeypatch.setattr(guided, 'time', SimpleNamespace(monotonic=lambda: 0))

    def no_input(*args):
        port.unlink()
        if failure == 'replaced':
            port.symlink_to('/dev/zero')
        return [], [], []

    monkeypatch.setattr(guided, 'select', SimpleNamespace(select=no_input))
    with pytest.raises(RuntimeError, match='UART disconnected'):
        procedure.deep_sleep(dut, 0, 615)
    assert evidence.data['events'][-1]['kind'] == 'deep_sleep_state_lost'
    assert procedure.holds == []
