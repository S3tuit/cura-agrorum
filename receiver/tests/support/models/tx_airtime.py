"""Independent list/rational-time oracle; no production imports or circular array.

Tracks complete durable promises, volatile ownership and observed transmissions.
UTC is evidence at snapshot/load only. Live history uses virtual interval starts.
"""
from copy import deepcopy
from fractions import Fraction
from math import ceil, floor

WINDOW = 3_600_000_000
WIDTH = 60_000_000
TAIL = 250_000
BUDGET = 36_000_000
LIMIT = 8_000_000
ACK = 67_866
FAST = Fraction(10037, 10000)
SLOW = Fraction(9963, 10000)
SPACING = ceil(WIDTH * FAST)


class LedgerModel:
    def __init__(self, charges=()):
        self.now = 100
        self.durable = (1, (0, 0), (0,) * (62 - len(charges)) + tuple(charges))
        self.attempts = []
        self.restart(True, 0)

    def restart(self, enabled, bias):
        self.live = None
        self.grant = self.pending = self.sample = None
        self.frozen = False
        self.set_trust(enabled, bias)
        self.restore()

    def set_trust(self, enabled, bias):
        self.sample = ((self.now, self.now - 100 + bias, abs(bias) + 1,
                        self.now + 10_000_000_000) if enabled else None)

    def evidence(self):
        if self.sample is None:
            return None
        mono, utc, error, horizon = self.sample
        error += ceil(Fraction((self.now - mono) * 37, 9963))
        if self.now >= horizon or error >= 40_000_000:
            return None
        return utc + self.now - mono, error

    def restore(self):
        old, current = self.durable[1], self.evidence()
        elapsed = max(0, current[0] - old[0] - current[1] - old[1]) if old and current else 0
        whole, phase = divmod(elapsed, WIDTH)
        self.origin = self.now - ceil(phase * FAST)
        self.live = []
        for position, charge in enumerate(self.durable[2]):
            d = 61 - position
            left = WINDOW + WIDTH + TAIL - d * WIDTH - elapsed
            if charge and left > 0:
                self.live.append((charge, self.origin - (d + whole) * SPACING,
                                  self.now + ceil(left * FAST)))

    @staticmethod
    def aged(entries, now):
        return [entry for entry in entries if entry[2] > now]

    def advance(self, duration):
        self.now += duration

    def reason(self):
        if self.pending is not None:
            return 'PERSISTENCE_PENDING'
        if self.frozen or self.grant is None:
            return 'GRANT_REQUIRED'
        if self.now >= self.grant['deadline']:
            return 'GRANT_EXPIRED'
        if self.grant['available'] < ACK:
            return 'BUDGET_EXHAUSTED'
        return 'ALLOWED'

    @property
    def available(self):
        return self.grant['available'] if self.reason() == 'ALLOWED' else 0

    @property
    def total(self):
        return None if self.live is None or self.pending else sum(c for c, _, _ in self.live)

    def recover(self):
        self.live = self.aged(self.live, self.now)
        return 'STATE_READY'

    def acquire(self, outcome='committed'):
        if self.pending is not None:
            return self.reconcile()
        if self.reason() == 'ALLOWED':
            return 'ALLOWED'
        if self.grant is not None:
            return self.settle(True, outcome)
        self.recover()
        return self.prepare(True, outcome)

    def spend(self, certainty):
        reason = self.reason()
        if reason != 'ALLOWED':
            return reason
        self.live = self.aged(self.live, self.now)
        self.grant['available'] -= ACK
        if certainty == 'NOT_STARTED':
            self.grant['available'] += ACK
        else:
            self.attempts.append((self.now - 100, ACK))
            observed = sum(charge for at, charge in self.attempts
                           if self.now - 100 - WINDOW <= at <= self.now - 100)
            assert observed <= BUDGET, ('continuous physical window exceeded', observed)
        return reason

    def settle(self, precharge, outcome='committed'):
        if self.pending is not None:
            return self.reconcile()
        if self.grant is None:
            return self.acquire(outcome) if precharge else self.recover()
        self.frozen = True
        return self.prepare(precharge, outcome)

    def prepare(self, precharge, outcome):
        entries = self.aged(self.live, self.now)
        current = self.origin + floor(Fraction(self.now - self.origin, SPACING)) * SPACING
        if self.grant is not None:
            old = self.grant
            settled = old['baseline'] + old['increment'] - old['available']
            entries = [(settled if start == old['start'] else charge, start, end)
                       for charge, start, end in entries]
        granted = None
        result = 'STATE_READY'
        if precharge:
            deadline = current + floor(WIDTH * SLOW)
            if self.now >= deadline:
                result = 'GRANT_EXPIRED'
            else:
                baseline = sum(c for c, start, _ in entries if start == current)
                increment = min(LIMIT - baseline, BUDGET - sum(c for c, _, _ in entries))
                if increment < ACK:
                    result = 'BUDGET_EXHAUSTED'
                else:
                    retention = next((end for _, start, end in entries if start == current),
                                     current + ceil((WINDOW + WIDTH + TAIL) * FAST))
                    entries = [(c, start, end) for c, start, end in entries if start != current]
                    entries.append((baseline + increment, current, retention))
                    granted = dict(start=current, baseline=baseline, increment=increment,
                                   available=increment, deadline=deadline)
                    result = 'ALLOWED'
            if granted is None and self.grant is None:
                return result
        charges = tuple(sum(c for c, start, _ in entries
                            if start == current - (61 - index) * SPACING)
                        for index in range(62))
        requested = (self.durable[0] + 1, self.evidence(), charges)
        transition = dict(requested=requested, entries=entries, grant=granted,
                          previous_live=deepcopy(self.live), previous_grant=deepcopy(self.grant),
                          next_reason=result)
        if outcome in ('committed', 'unknown_installed'):
            self.durable = requested
        if outcome.startswith('unknown'):
            self.pending = transition
            return 'PERSISTENCE_PENDING'
        self.finish(transition)
        return 'PERSISTENCE_FAILED' if outcome == 'not_installed' else self.finished_reason(transition)

    def finish(self, transition):
        installed = self.durable == transition['requested']
        self.live = transition['entries' if installed else 'previous_live']
        self.grant = transition['grant' if installed else 'previous_grant']
        self.pending = None
        self.frozen = False

    def finished_reason(self, transition):
        if self.durable == transition['requested'] and transition['grant'] is None:
            return transition['next_reason']
        return self.reason()

    def reconcile(self, fail_load=False):
        if self.pending is not None:
            if fail_load:
                return 'PERSISTENCE_PENDING'
            transition = self.pending
            self.finish(transition)
            return self.finished_reason(transition)
        if self.grant is not None:
            return self.reason()
        return self.recover()
