"""Exercise the actual bounded UART hold; only UART and clock dependencies are fake."""
import os
import subprocess
from types import SimpleNamespace

import pytest

import carrier_runner as runner
from carrier_guided import IncompleteCase, measured_volts
from test_runner import build_dir, scripted_dut


@pytest.fixture(scope='module')
def hold_binary(tmp_path_factory):
    root = tmp_path_factory.mktemp('hold')
    (root / 'freertos').mkdir()
    (root / 'esp_rom_serial_output.h').write_text(
        '#include <stdint.h>\nint esp_rom_output_rx_one_char(uint8_t *byte);\n')
    (root / 'esp_timer.h').write_text(
        '#include <stdint.h>\nint64_t esp_timer_get_time(void);\n')
    (root / 'freertos/FreeRTOS.h').write_text('#define pdMS_TO_TICKS(ms) ((ms) / 10U)\n')
    (root / 'freertos/task.h').write_text('void vTaskDelay(unsigned ticks);\n')
    harness = root / 'harness.c'
    harness.write_text('''#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "carrier_hold.h"
static int64_t now, arrival;
static const char *input;
int64_t esp_timer_get_time(void) { return now; }
void vTaskDelay(unsigned ticks) { now += ticks * INT64_C(10000); }
int esp_rom_output_rx_one_char(uint8_t *byte) {
  if (now < arrival || !*input) return -1;
  *byte = (uint8_t)*input++;
  return 0;
}
int main(int argc, char **argv) {
  assert(argc == 4);
  arrival = strtoll(argv[2], NULL, 10);
  input = argv[3];
  int mode = strcmp(argv[1], "exploration") == 0 ? CARRIER_HOLD_EXPLORATION :
             strcmp(argv[1], "auto") != 0;
  int result = strcmp(argv[1], "select") == 0 ? carrier_hold_select() :
    carrier_hold_wait(mode == CARRIER_HOLD_GUIDED ? argv[1] : "sample-return", mode);
  printf("HARNESS_RESULT %d %lld\\n", result, (long long)now);
}
''')
    binary = root / 'hold'
    subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-g',
                    '-I', str(root), '-I', str(runner.APP / 'main'), str(harness),
                    str(runner.APP / 'main/carrier_hold.c'), '-o', str(binary)],
                   check=True, capture_output=True, text=True)
    return binary


@pytest.mark.parametrize('mode,arrival,command,result,elapsed', [
    ('auto', 0, 'CARRIER_HOLD_ACK sample-return\n', 1, 60010000),
    ('gate-on', 5000000, 'CARRIER_HOLD_ACK gate-on\n', 1, 5000000),
    ('gate-on', 4990000, 'CARRIER_HOLD_ACK gate-on\n', 0, 4990000),
    ('transition-on', 5000000, 'CARRIER_HOLD_ACK transition-on\n', 1, 5000000),
    ('transition-on', 4990000, 'CARRIER_HOLD_ACK transition-on\n', 0, 4990000),
    ('reset-off', 10000000, 'CARRIER_HOLD_ACK reset-off\n', 1, 10000000),
    ('reset-off', 9990000, 'CARRIER_HOLD_ACK reset-off\n', 0, 9990000),
    ('final-cleanup', 10000000, 'CARRIER_HOLD_ACK final-cleanup\n', 1, 10000000),
    ('final-cleanup', 9990000, 'CARRIER_HOLD_ACK final-cleanup\n', 0, 9990000),
    ('final-cleanup', 180000000, 'CARRIER_HOLD_ACK final-cleanup\n', 0, 180000000),
    ('gate-off', 10000000, 'CARRIER_HOLD_ACK gate-off\r\n', 1, 10000000),
    ('sample-return', 10000000, 'CARRIER_HOLD_ACK sample-return\n', 1, 10000000),
    ('sample-return', 9990000, 'CARRIER_HOLD_ACK sample-return\n', 0, 9990000),
    ('sample-return', 179990000, 'CARRIER_HOLD_ACK sample-return\n', 1, 179990000),
    ('sample-return', 180000000, 'CARRIER_HOLD_ACK sample-return\n', 0, 180000000),
    ('gate-on', 0, '', 0, 180000000),
    ('gate-off', 10000000, 'CARRIER_HOLD_ACK sample-return\n', 0, 10000000),
    ('sample-return', 10000000, 'CARRIER_HOLD_ACK sample-return', 0, 180000000),
    ('sample-return', 10000000, 'x' * 100 + '\n', 0, 10000000),
    ('select', 0, '\r\nauto\r\n', 0, 0),
    ('select', 0, 'guided\n', 1, 0),
    ('select', 0, 'exploration\n', 2, 0),
    ('select', 0, 'unexpected\n', -1, 0),
    ('select', 0, '', -1, 30000000),
])
def test_actual_uart_hold_boundaries(hold_binary, mode, arrival, command, result, elapsed):
    run = subprocess.run([str(hold_binary), mode, str(arrival), command],
                         check=True, capture_output=True, text=True)
    assert f'HARNESS_RESULT {result} {elapsed}\n' in run.stdout
    if mode not in {'auto', 'select'}:
        assert 'seconds=180' in run.stdout
        assert ('CARRIER_HOLD_ACKED' in run.stdout) == bool(result)
        assert ('CARRIER_HOLD_INCOMPLETE' in run.stdout) == (not result)
    elif mode == 'auto':
        assert 'seconds=60' in run.stdout
        assert 'CARRIER_HOLD_ACKED' not in run.stdout


@pytest.mark.parametrize('operation', ['acquire', 'gate-on', 'gate-off', 'final-cleanup', 'ds-identity'])
@pytest.mark.parametrize('outcome', ['valid', 'invalid_input', 'late_input', 'target_incomplete', 'missing_ack'])
def test_runner_requires_valid_input_and_target_ack(scripted_dut, monkeypatch, operation, outcome):
    dut, _, state = scripted_dut
    dut.expect_exact('Press ENTER to see the list of tests.')
    case = SimpleNamespace(index=5, name=runner.CASES[operation])
    hold = 'sample-return' if operation in {'acquire', 'ds-identity'} else operation
    ack_required = operation != 'ds-identity'
    commands = []
    clock = SimpleNamespace(now=0)
    # Isolate the runner's clock without changing pexpect's real timeout clock.
    monkeypatch.setattr(runner, 'time', SimpleNamespace(monotonic=lambda: clock.now))
    monkeypatch.setattr(runner, 'GUIDED_HOLD_SECONDS', 15.1)

    def end():
        state.feed(f'CARRIER_HOLD_END {hold}\n')
        state.result(case.name, 'PASS')

    def write(command):
        commands.append(command)
        if command == '5':
            state.feed('CARRIER_HOLD_MODE\n')
        elif command in {'auto', 'guided'}:
            assert command == ('guided' if ack_required else 'auto')
            state.feed(f'CARRIER_HOLD_READY {hold} seconds={180 if ack_required else 60}\n')
            if not ack_required:
                end()
        else:
            assert command == f'CARRIER_HOLD_ACK {hold}'
            if outcome == 'target_incomplete':
                state.feed(f'CARRIER_HOLD_INCOMPLETE {hold}\n')
            elif outcome != 'missing_ack':
                state.feed(f'CARRIER_HOLD_ACKED {hold} elapsed_us=10000000\n')
            end()

    def on_hold(name, serial, deadline):
        assert name == hold
        assert f'CARRIER_HOLD_ACK {hold}' not in commands
        if outcome == 'invalid_input':
            raise IncompleteCase('missing measurement')
        if outcome == 'late_input':
            clock.now = deadline - 15

    dut.write = write
    fails = outcome == 'invalid_input' or (ack_required and outcome != 'valid')
    if fails:
        with pytest.raises((RuntimeError, AssertionError)):
            runner.execute_case(dut, case, operation, on_hold=on_hold)
    else:
        runner.execute_case(dut, case, operation, on_hold=on_hold)
    assert commands[:2] == ['5', 'guided' if ack_required else 'auto']
    if not ack_required or outcome in {'invalid_input', 'late_input'}:
        assert len(commands) == 2
    else:
        assert commands[2:] == [f'CARRIER_HOLD_ACK {hold}']


@pytest.mark.parametrize('line', ['3.299,0.002', 'TP_3V3=3.299, TP_DQ=0.002',
                                 'TP_3V3=3.299 TP_DQ=0.002,', '3.299 0.002'])
def test_meter_csv_or_unnamed_values_rejected(line):
    with pytest.raises((IncompleteCase, ValueError)):
        measured_volts(line, {'TP_3V3', 'TP_DQ'})


def test_meter_named_volts_allow_arbitrary_order():
    assert measured_volts('TP_DQ=0.002 TP_3V3=3.299', {'TP_3V3', 'TP_DQ'}) == {
        'TP_DQ': '0.002', 'TP_3V3': '3.299'}


@pytest.mark.parametrize('failure', [None, 'transition-on:input', 'reset-off:input',
                                    'transition-on:ack', 'reset-off:ack', 'restart:off_failure'])
def test_reset_requires_both_measurements_and_acks_across_real_boot_parser(scripted_dut, failure):
    dut, build, state = scripted_dut
    dut.expect_exact('Press ENTER to see the list of tests.')
    case = SimpleNamespace(index=9, name=runner.CASES['reset'], type='multi_stage',
                           is_ignored=False, subcases=[dict(index=1, name='one'), dict(index=2, name='two')])
    commands, holds = [], []

    def write(command):
        commands.append(command)
        if command == '9':
            state.feed('(1) "one"\n(2) "two"\n')
        elif command in {'1', '2'}:
            state.feed('CARRIER_HOLD_MODE\n')
        elif command == 'guided':
            hold_name = 'transition-on' if commands[-2] == '1' else 'reset-off'
            state.feed(f'CARRIER_HOLD_READY {hold_name} seconds=180\n')
        elif command == '':
            state.feed("Here's the test menu, pick your combo:\n"
                       f'(9)\t"{case.name}" [sensor_carrier][multi_stage]\n'
                       '\t(1)\t"one"\n\t(2)\t"two"\nEnter test for running.\n')
        else:
            name = command.removeprefix('CARRIER_HOLD_ACK ')
            assert name in {'transition-on', 'reset-off'}
            if failure == name + ':ack':
                state.feed(f'CARRIER_HOLD_INCOMPLETE {name}\n')
                state.result(case.name, 'FAIL')
                return
            state.feed(f'CARRIER_HOLD_ACKED {name} elapsed_us=11000000\n'
                       f'CARRIER_HOLD_END {name}\n')
            if name == 'transition-on':
                status = '00000001' if failure == 'restart:off_failure' else '00000000'
                state.feed(f'CARRIER_TRANSITION reset\nCARRIER_RESTART_CLEANUP calls=1 result={status} diagnostic_empty=1 observer_valid=1\n')
                state.boot()
            else:
                state.result(case.name, 'PASS')

    def hold(name, serial, deadline):
        assert 194 < deadline - runner.time.monotonic() <= 195
        assert f'seconds=180'.encode() in serial
        assert f'CARRIER_HOLD_ACK {name}' not in commands
        holds.append(name)
        if failure == name + ':input':
            raise IncompleteCase('incomplete meter input')

    dut.write = write
    action = lambda: runner.execute_transition(dut, build, [case], 'reset',
                                               '001122334455', SimpleNamespace(hold=hold))
    if failure:
        with pytest.raises((RuntimeError, AssertionError)):
            action()
        if failure.endswith(':input'):
            assert f'CARRIER_HOLD_ACK {failure.split(":")[0]}' not in commands
        if failure.startswith('transition-on'):
            assert '2' not in commands
    else:
        action()
        assert holds == ['transition-on', 'reset-off']
        assert [x.result for x in dut.testsuite.testcases] == ['PASS']
        assert commands == ['9', '1', 'guided', 'CARRIER_HOLD_ACK transition-on', '',
                            '9', '2', 'guided', 'CARRIER_HOLD_ACK reset-off']


@pytest.mark.parametrize('seconds,input_text,passes', [
    (11, 'TP_3V3=3.3 TP_GATE=3.3 TP_SW=0.001 TP_DQ=0.001', True),
    (175, 'TP_3V3=3.3 TP_GATE=3.3 TP_SW=0.001 TP_DQ=0.001', True),
    (9, 'TP_3V3=3.3 TP_GATE=3.3 TP_SW=0.001 TP_DQ=0.001', False),
    (11, 'TP_3V3=3.3 TP_GATE=3.3 TP_SW=0.101 TP_DQ=0.001', False),
    (11, 'TP_3V3=3.3', False),
])
def test_held_reset_release_immediately_after_valid_input(tmp_path, monkeypatch, seconds, input_text, passes):
    import carrier_guided as guided
    from carrier_evidence import Evidence
    clock = SimpleNamespace(now=0)
    monkeypatch.setattr(guided, 'time', SimpleNamespace(monotonic=lambda: clock.now))
    confirmations = []
    monkeypatch.setattr(guided, 'confirm', lambda prompt, **kw: confirmations.append((clock.now, kw)))
    def ask(prompt, *, deadline):
        assert deadline == 180
        clock.now = seconds
        return input_text
    monkeypatch.setattr(guided, 'ask', ask)
    evidence = Evidence(tmp_path/'held.json', {}, {})
    procedure = guided.Guided(evidence, 'held-reset', None, None)
    if passes:
        assert procedure.held_reset(lambda: 'monitor') == ('monitor', seconds)
        assert confirmations == [(0, {}), (seconds, {'deadline': 195})]
        assert evidence.data['events'][-1]['held_seconds'] == seconds
    else:
        with pytest.raises((RuntimeError, AssertionError)):
            procedure.held_reset(lambda: 'monitor')
        assert len(confirmations) == 1  # No release instruction before valid readings.


def test_adc_input_window_is_three_minutes_and_dispatches_on_entry(tmp_path, monkeypatch):
    import carrier_guided as guided
    from carrier_evidence import Evidence
    monkeypatch.setattr(guided, 'time', SimpleNamespace(monotonic=lambda: 50))
    monkeypatch.setattr(guided, 'confirm', lambda *a: None)
    def ask(prompt, *, deadline):
        assert deadline == 230
        return 'TP_3V3=3.3 TP_ADC0=1.2 TP_ADC1=1.6'
    monkeypatch.setattr(guided, 'ask', ask)
    evidence = Evidence(tmp_path/'adc.json', {}, {})
    procedure = guided.Guided(evidence, 'adc-reference', 'A', None)
    procedure.before_sample()
    assert procedure.measurements == dict(TP_3V3='3.3', TP_ADC0='1.2', TP_ADC1='1.6')
