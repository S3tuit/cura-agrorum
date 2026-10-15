"""Local, non-accepting electrical observation log. No voltage interpretation."""
import select
import sys
import time

import pexpect
from pytest_embedded.unity import UNITY_SUMMARY_LINE_REGEX

from carrier_guided import IncompleteCase, confirm
from carrier_runner import check_uart, uart_identity


OPERATIONS = {'gate-on', 'gate-off', 'acquire', 'final-cleanup',
              'reset', 'held-reset', 'deep-sleep'}


class Exploration:
    exploration = True

    def __init__(self, evidence, operation):
        self.evidence = evidence
        self.operation = operation
        self.holds = []
        self.dut = None
        self._uart_identity = None

    def _port_identity(self):
        return uart_identity(self.evidence.data['metadata'].get('port'))

    def bind(self, dut):
        self.dut = dut
        self._uart_identity = self._port_identity()

    def wiring(self):
        print('EXPLORATION ONLY: describe actual wiring, connected/disconnected branches, '
              'C3 and any temporary load. Power must be removed before wiring changes.', flush=True)
        description = sys.stdin.readline()
        if not description.strip():
            raise IncompleteCase('exploration requires an actual fixture description')
        self.evidence.add('exploration_setup', text=description.rstrip('\r\n'))
        if self.operation in {'acquire', 'final-cleanup'}:
            confirm('The complete nominal fixture is restored, including C3, both soil probes, '
                    'both configured DS probes and BME, soil shunts fitted and references disconnected. '
                    'Acquisition preflight remains required.')
        confirm('Wiring changes were made with power removed; source/drain, gate resistors, '
                'connector pin functions and common grounds were checked. The identified DUT '
                'and described circuit are powered and ready.')
        print('Enter any observation text; each line is saved immediately. /done ends the '
              'current observation. Times are entry times, not inferred measurement times. '
              'No electrical acceptance is possible. Change wiring only between powered-down runs.',
              flush=True)

    def before_sample(self):
        pass

    def _check_state(self, name):
        try:
            check_uart(self.dut, self.evidence.data['metadata'].get('port'), self._uart_identity)
        except OSError as exc:
            self.evidence.add('exploration_state_lost', hold=name, reason=str(exc))
            raise RuntimeError('UART disconnected or reader stopped during exploration') from exc
        try:
            match = self.dut.expect(
                [rb'CARRIER_BOOT ', UNITY_SUMMARY_LINE_REGEX, rb'CARRIER_HOLD_INCOMPLETE ',
                 rb'CARRIER_HOLD_END ', rb'Guru Meditation Error', rb'task_wdt:'], timeout=0)
        except pexpect.TIMEOUT:
            return
        except pexpect.EOF as exc:
            self.evidence.add('exploration_state_lost', hold=name, reason='UART disconnected')
            raise RuntimeError('UART disconnected during exploration') from exc
        self.evidence.add('exploration_state_lost', hold=name,
                          marker=match.group(0).decode(errors='replace'))
        raise RuntimeError('DUT changed state or failed during exploration; observation ended')

    def hold(self, name, serial, deadline):
        # deadline is deliberately absent for exploration; firmware work before
        # and after this observation still has bounded host handshakes.
        assert deadline is None
        started = time.monotonic()
        self.evidence.add('exploration_phase_start', hold=name,
                          serial=serial.decode(errors='replace'))
        print(f'{name}: unlimited observation; enter notes, then /done.', flush=True)
        while True:
            self._check_state(name)
            if not select.select([sys.stdin], [], [], 0.1)[0]:
                continue
            line = sys.stdin.readline()
            self._check_state(name)
            if not line:
                raise IncompleteCase('exploration input closed before /done')
            text = line.rstrip('\r\n')
            elapsed = time.monotonic() - started
            if text == '/done':
                self.evidence.add('exploration_phase_end', hold=name, elapsed_seconds=elapsed)
                self.holds.append(name)
                return
            self.evidence.add('exploration_note', hold=name, text=text, elapsed_seconds=elapsed)

    def held_reset(self):
        confirm('Press and KEEP HOLDING DUT EN/reset. Keep carrier USB power connected.')
        self.hold('held-reset', b'', None)
        self.end_reset('held-reset')

    def end_reset(self, name):
        instruction = ('Release EN/reset now.' if name == 'held-reset' else
                       'Press and release EN/reset now to end indefinite deep sleep.')
        confirm(instruction + ' Confirm once done; this is an operator reset, not timer wakeup.')
        self.evidence.add('exploration_operator_reset', hold=name)

    def complete(self):
        expected = {'acquire': ['sample-return'], 'final-cleanup': ['final-cleanup'],
                    'gate-on': ['gate-on'], 'gate-off': ['gate-off'],
                    'reset': ['transition-on', 'reset-off'],
                    'held-reset': ['transition-on', 'held-reset'],
                    'deep-sleep': ['transition-on', 'deep-sleep']}
        if self.holds != expected[self.operation]:
            raise IncompleteCase('exploration did not complete every requested observation phase')
        self.evidence.finish('exploration_complete_not_acceptance')
        raise IncompleteCase('exploration recorded; this is not an acceptance result')
