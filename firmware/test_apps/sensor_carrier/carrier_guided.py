"""Operator procedures for this carrier. All voltages are independent meter inputs."""
from decimal import Decimal, InvalidOperation
import hashlib
import json
from pathlib import Path
import re
import select
import sys
import time

import pexpect
from pytest_embedded.unity import UNITY_SUMMARY_LINE_REGEX

from carrier_runner import GUIDED_HOLD_SECONDS, MISSING_FIXTURES, check_uart, uart_identity


class IncompleteCase(RuntimeError):
    pass


def ask(prompt, *, deadline=None, check_state=None):
    print(prompt, flush=True)
    if check_state:
        check_state()
    if deadline is not None:
        while True:
            seconds = deadline - time.monotonic()
            if seconds <= 0:
                raise IncompleteCase('measurement window ended before operator input')
            if select.select([sys.stdin], [], [], min(seconds, 0.1) if check_state else seconds)[0]:
                break
            if check_state:
                check_state()
            else:
                raise IncompleteCase('measurement window ended before operator input')
    line = sys.stdin.readline()
    if check_state:
        check_state()
    if not line or (deadline is not None and time.monotonic() >= deadline):
        raise IncompleteCase('missing or late operator input')
    return line.strip()


def confirm(prompt, **kwargs):
    if ask(prompt + ' Type YES to attest.', **kwargs) != 'YES':
        raise IncompleteCase('operator confirmation missing')


def measured_volts(text, points):
    fields = text.split()
    if len(fields) != len(points):
        raise IncompleteCase('every requested meter point is required, in volts')
    values = {}
    for field in fields:
        key, separator, raw = field.partition('=')
        if not separator or key in values or key not in points:
            raise ValueError('duplicate or unexpected meter point')
        try:
            value = Decimal(raw)
        except InvalidOperation as exc:
            raise ValueError('meter voltage must be a finite decimal') from exc
        if not value.is_finite() or value < 0:
            raise ValueError('meter voltage must be nonnegative')
        values[key] = str(value)
    if set(values) != set(points):
        raise IncompleteCase('missing meter point')
    return values


def sample_from_serial(serial):
    matches = re.findall(rb'CARRIER_SAMPLE ([^\r\n]+)', serial)
    if len(matches) != 1:
        raise ValueError('expected exactly one sample record')
    fields = dict(part.split('=') for part in matches[0].decode().split())
    needed = {'result', 'duration_us', 'validity', 'soil0_mv', 'soil1_mv',
              'temp0_centi_c', 'temp1_centi_c', 'enclosure_centi_c',
              'pressure_pa', 'humidity_centi_pct'}
    if set(fields) != needed:
        raise ValueError('incomplete sample record')
    sample = {k: int(v, 16 if k in {'result', 'validity'} else 10) for k, v in fields.items()}
    if sample['result'] != 0 or sample['validity'] != 0x1f:
        raise AssertionError('guided acquisition did not return all five valid groups')
    return sample


def validate_identity(sample, warmed):
    difference = sample[f'temp{warmed}_centi_c'] - sample[f'temp{1-warmed}_centi_c']
    if difference < 200:
        raise IncompleteCase('declared warmed ROM must report at least 2 C warmer; preserve and repeat')


def validate_reference(sample, measurements, position):
    volts = {key: Decimal(value) for key, value in measurements.items()}
    if set(volts) != {'TP_3V3', 'TP_ADC0', 'TP_ADC1'} or any(
            not value.is_finite() or value < 0 for value in volts.values()):
        raise ValueError('complete finite ADC reference measurements required')
    for channel in range(2):
        measured = volts[f'TP_ADC{channel}']
        if not 0 < measured < min(Decimal('2.0'), volts['TP_3V3']):
            raise AssertionError('reference must be below the air-probe range and DUT supply')
        error_mv = abs(Decimal(sample[f'soil{channel}_mv']) - 1000 * measured)
        if error_mv > 75:
            raise AssertionError(f'ADC{channel} differs from its fresh measurement by {error_mv} mV (>75)')
    difference = 1000 * (volts['TP_ADC1'] - volts['TP_ADC0'])
    # Non-overlapping +/-75 mV intervals make a channel swap detectable.
    if (difference if position == 'A' else -difference) <= 150:
        raise AssertionError('measured references must follow A/B routing and have disjoint 75 mV intervals')


def validate_electrical(name, readings):
    v = {key: Decimal(value) for key, value in readings.items()}
    required = {'TP_3V3', 'TP_GATE', 'TP_SW', 'TP_DQ'}
    if name == 'gate-on':
        required |= {'TP_ADC0', 'TP_ADC1'}
    if set(v) != required or any(not x.is_finite() or x < 0 for x in v.values()):
        raise IncompleteCase('every electrical test point needs a finite nonnegative voltage')
    if name in {'gate-on', 'transition-on'}:
        valid = (v['TP_GATE'] <= Decimal('.2') and
                 abs(v['TP_SW'] - v['TP_3V3']) <= Decimal('.1') and
                 abs(v['TP_DQ'] - v['TP_SW']) <= Decimal('.1'))
        if name == 'gate-on':
            valid &= all(Decimal('2') <= v[p] <= Decimal('2.7') for p in ('TP_ADC0', 'TP_ADC1'))
    else:
        valid = (v['TP_GATE'] >= 3 and abs(v['TP_GATE'] - v['TP_3V3']) <= Decimal('.1') and
                 v['TP_SW'] <= Decimal('.1') and v['TP_DQ'] <= Decimal('.1'))
    if not valid:
        raise AssertionError(f'{name} meter observations do not meet carrier limits')


def prior_position(path, metadata, position):
    if position == 'A':
        if path:
            raise ValueError('position A must not reuse prior evidence')
        return None
    if position != 'B' or not path:
        raise IncompleteCase('position B requires complete position-A evidence')
    raw = Path(path).read_bytes()
    prior = json.loads(raw)
    if prior.get('metadata', {}).get('exploration'):
        raise ValueError('exploration cannot supply position-A acceptance evidence')
    if prior.get('schema') != 1 or prior.get('status') != 'position_A_complete_sequence_incomplete':
        raise ValueError('prior evidence is not a completed position A')
    for key in ('operation', 'fixture', 'carrier_revision', 'expected_dut',
                'elf_sha256', 'config_sha256', 'rom0', 'rom1'):
        if prior.get('metadata', {}).get(key) != metadata.get(key):
            raise ValueError(f'position A/B mismatch: {key}')
    events = [event for event in prior.get('events', []) if event['kind'] == 'position_result']
    if len(events) != 1 or events[0].get('position') != 'A':
        raise ValueError('prior position results missing/duplicated')
    if not any(e['kind'] == 'software_completed' for e in prior['events']):
        raise ValueError('position A lacks passing software completion')
    return dict(event=events[0], run_id=prior['run_id'], sha256=hashlib.sha256(raw).hexdigest())


class Guided:
    def __init__(self, evidence, operation, position, prior):
        self.evidence = evidence
        self.operation = operation
        self.position = position
        self.prior = prior
        self.warmed = None
        self.measurements = None
        self.sample = None
        self.holds = []

    def wiring(self):
        fixture = self.evidence.data['metadata'].get('fixture')
        if fixture == 'missing_bme280':
            metadata = self.evidence.data['metadata']
            confirm('The missing_bme280 fixture is ready: with power removed, the complete BME280 '
                    'power, ground, SDA and SCL connector was removed. '
                    f'Both configured DS probes, DS0 ROM {metadata["rom0"]} and DS1 ROM {metadata["rom1"]}, '
                    'remain connected, with both air-exposed soil probes and soil shunts. '
                    'Permanent R12 is fitted, reference enable is open and reference leads are removed. '
                    'Wiring preflight was completed after the change; the identified DUT is ready.')
            self.evidence.add('missing_bme_fixture', rom0=metadata['rom0'], rom1=metadata['rom1'])
        elif fixture in MISSING_FIXTURES:
            missing = 0 if fixture == 'missing_ds0' else 1
            metadata = self.evidence.data['metadata']
            confirm(f'The {fixture} fixture is ready: with power removed, the complete power, '
                    f'ground and data connector of logical DS{missing}, ROM {metadata[f"rom{missing}"]}, '
                    f'was removed. Logical DS{1-missing}, ROM {metadata[f"rom{1-missing}"]}, remains connected. '
                    'Both air-exposed soil probes, soil shunts and BME remain connected. '
                    'Permanent R12 is fitted, reference enable is open and reference leads are removed. '
                    'Wiring preflight was completed after the change; the identified DUT is ready.')
            self.evidence.add('missing_probe_fixture', missing_channel=missing,
                              missing_rom=metadata[f'rom{missing}'], surviving_rom=metadata[f'rom{1-missing}'])
        elif self.operation == 'ds-identity':
            arrangement = ('configured DS0 in J_DS0 and DS1 in J_DS1' if self.position == 'A'
                           else 'configured DS0 in J_DS1 and DS1 in J_DS0; same labeled probes')
            confirm('Power was removed before connector changes, wiring preflight was completed, '
                    f'both configured probes and all nominal sensors are present: {arrangement}. '
                    'Reference leads are removed, JP_REF_ENABLE open, soil shunts fitted. DUT is ready.')
        elif self.operation == 'adc-reference':
            routing = ('VREF_A -> J_INJECT0; VREF_B -> J_INJECT1' if self.position == 'A'
                       else 'VREF_B -> J_INJECT0; VREF_A -> J_INJECT1')
            confirm('Power was removed before changing wiring and wiring preflight was repeated. '
                    'Both soil connectors and JP_SOIL0/1 shunts are removed; JP_REF_ENABLE fitted. '
                    'R5/R6=10k/5.6k; R7/R8=2.2k/2.2k; R3/R4=1k. Both DS probes and BME remain. '
                    f'Position {self.position}: {routing}. DUT is powered and ready.')
        else:
            confirm('The nominal fixture is ready: both air-exposed soil probes, configured DS probes, '
                    'BME, soil shunts, reference enable open and reference leads removed. '
                    'Wiring preflight was completed after any wiring change.')
        print('At every hold: measure DC VOLTS against carrier ground. Wait at least 5 s for ON or '
              '10 s for OFF, then require three stable display updates at each point. '
              'Enter all named points on one line, separated by SPACES without commas (TP_3V3=3.299). '
              'Awake electrical holds allow 180 s and finish on valid entry; '
              'Deep sleep allows 600 s for readings and YES, then ends by prompted EN/reset. '
              'No meter transient/current claim.',
              flush=True)
        self.evidence.add('wiring_confirmation', position=self.position)

    def before_sample(self):
        if self.operation == 'ds-identity':
            if self.prior:
                self.warmed = self.prior['event']['warmed_channel']
            else:
                answer = ask('Which labeled physical probe will you keep warmer? Enter logical channel 0 or 1:')
                if answer not in {'0', '1'}:
                    raise IncompleteCase('declare warmed channel 0 or 1')
                self.warmed = int(answer)
            rom = self.evidence.data['metadata'][f'rom{self.warmed}']
            confirm(f'Keep physical probe ROM {rom} warmer by at least 2 C, with both probes present. '
                    'The acquisition is ready; maintain this difference during sampling.')
            self.evidence.add('thermal_setup', warmed_channel=self.warmed, warmed_rom=rom)

        elif self.operation == 'adc-reference':
            confirm('At this fresh acquisition boot, allow at least 5 seconds of reference settling. '
                    'Measure TP_3V3, TP_ADC0 and TP_ADC1 now; each display must be stable for three updates. '
                    'Do not change wiring or power after these measurements.')
            self.measurements = measured_volts(ask(
                'Within 180 seconds, enter fresh VOLTS: TP_3V3=... TP_ADC0=... TP_ADC1=...; '
                'acquisition starts on complete input.', deadline=time.monotonic() + GUIDED_HOLD_SECONDS - 15),
                {'TP_3V3', 'TP_ADC0', 'TP_ADC1'})
            self.evidence.add('reference_measurements', position=self.position, volts=self.measurements,
                              settling_seconds=5, stable_updates=3)

    def hold(self, name, serial, deadline):
        if self.operation in {'ds-identity', 'adc-reference'}:
            self.sample = sample_from_serial(serial)
            self.evidence.add('sample', sample=self.sample, hold=name)
        if self.operation not in {'ds-identity', 'adc-reference'}:
            self.measure_hold(name, deadline)

    def measure_hold(self, name, deadline, *, check_state=None):
        # 15 s is transport/result margin, never extra meter-observation time.
        deadline -= 15
        on = name in {'gate-on', 'transition-on'}
        points = {'TP_3V3', 'TP_GATE', 'TP_SW', 'TP_DQ'}
        if name == 'gate-on':
            points |= {'TP_ADC0', 'TP_ADC1'}
        settling = 5 if on else 10
        started = time.monotonic()
        readings = measured_volts(ask(
            f'{name}: wait >= {settling} seconds; each reading needs 3 stable updates. '
            'Enter named VOLTS separated by spaces, NO COMMAS: ' + ' '.join(p + '=...' for p in sorted(points)),
            deadline=deadline, **({'check_state': check_state} if check_state else {})), points)
        if time.monotonic() - started < settling:
            raise IncompleteCase('readings supplied before required settling')
        # Supplying the readings attests the settling and stable-display procedure.
        self.evidence.add('meter', hold=name, volts=readings, settling_seconds=settling,
                          stable_updates=3)
        self.holds.append(name)
        try:
            validate_electrical(name, readings)
        except AssertionError:
            if not on and Decimal(readings['TP_SW']) > Decimal('.1'):
                print('TP_SW exceeds 0.1 V. Keep this failure. For the approved carrier, keep permanent '
                      'R12 (100 kohm from 3V3_SW to GND) fitted. Do not add another discharge resistor '
                      'in parallel. Investigate in separately declared --exploration runs; '
                      'remove power before any wiring change.', flush=True)
            raise

    def deep_sleep(self, dut, started, deadline):
        port = self.evidence.data['metadata'].get('port')
        identity = uart_identity(port)

        def check_state():
            try:
                check_uart(dut, port, identity)
                match = dut.expect([rb'ESP-ROM:', rb'CARRIER_BOOT ', UNITY_SUMMARY_LINE_REGEX,
                                    rb'Guru Meditation Error', rb'task_wdt:'], timeout=0)
            except pexpect.TIMEOUT:
                return
            except (OSError, pexpect.EOF) as exc:
                reason = f'UART disconnected or reader stopped: {exc}'
            else:
                reason = 'DUT booted or failed before deep-sleep attestation: ' + match.group(0).decode(errors='replace')
            self.evidence.add('deep_sleep_state_lost', reason=reason,
                              elapsed_seconds=time.monotonic() - started)
            raise RuntimeError(reason)

        print('Deep sleep has started without timer wakeup. Within 600 seconds, enter all four '
              'readings and attest YES. Keep USB connected and do not press EN/reset until prompted.', flush=True)
        self.measure_hold('deep-sleep', deadline, check_state=check_state)
        confirm('Readings saved and within limits. Confirm they were measured while the MCU remained '
                'asleep, after 10 seconds settling and three stable display updates at every point. '
                'Keep USB connected; do not reset yet.', deadline=deadline - 15, check_state=check_state)
        check_state()
        attested = time.monotonic()
        if attested >= deadline - 15:
            raise IncompleteCase('deep-sleep observation window ended before attestation completed')
        self.evidence.add('deep_sleep_attestation', elapsed_seconds=attested - started,
                          timer_wakeup=False)
        return port, identity

    def held_reset(self, start_monitor):
        confirm('Press and KEEP HOLDING the DUT EN/reset button now. Keep carrier USB power connected.')
        monitor = start_monitor()
        started = time.monotonic()
        deadline = started + GUIDED_HOLD_SECONDS
        self.measure_hold('held-reset', deadline)
        held_until = time.monotonic()
        confirm('All readings recorded while EN was held. Release the reset button now.', deadline=deadline)
        self.evidence.add('held_reset_release', held_seconds=held_until-started)
        return monitor, held_until

    def complete(self):
        expected = {'acquire': ['sample-return'], 'gate-on': ['gate-on'],
                    'gate-off': ['gate-off'], 'final-cleanup': ['final-cleanup'],
                    'reset': ['transition-on', 'reset-off'],
                    'deep-sleep': ['transition-on', 'deep-sleep'],
                    'held-reset': ['transition-on', 'held-reset']}
        if self.operation in expected and self.holds != expected[self.operation]:
            raise IncompleteCase('not all required electrical stages were measured')
        if self.operation == 'deep-sleep':
            kinds = {event['kind'] for event in self.evidence.data['events']}
            if not {'deep_sleep_attestation', 'deep_sleep_operator_reset'} <= kinds:
                raise IncompleteCase('deep sleep requires YES attestation and verified operator reset')
        if self.operation == 'ds-identity':
            validate_identity(self.sample, self.warmed)
            self.evidence.add('position_result', position=self.position,
                              warmed_channel=self.warmed, sample=self.sample)
        elif self.operation == 'adc-reference':
            validate_reference(self.sample, self.measurements, self.position)
            if self.prior:
                validate_reference(self.prior['event']['sample'], self.prior['event']['volts'], 'A')
            self.evidence.add('position_result', position=self.position,
                              sample=self.sample, volts=self.measurements)
        if self.operation in {'ds-identity', 'adc-reference'}:
            if self.position == 'A':
                self.evidence.finish('position_A_complete_sequence_incomplete')
                raise IncompleteCase('position A recorded successfully; complete case requires position B')
            self.evidence.add('paired_acceptance', prior=self.prior)
            self.evidence.finish('accepted')
        else:
            self.evidence.finish('accepted')
