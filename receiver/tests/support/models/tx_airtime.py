"""Independent list-based grant promises and exact continuous-window observations.

Inputs are primitive integers/strings. Fractions express physical-duration bounds;
no production state, encoding, builder, clock or policy algorithm is imported.
"""

from copy import deepcopy
from fractions import Fraction
from math import ceil, floor

WINDOW = 3_600_000_000
WIDTH = 60_000_000
GUARD = 120_000_000
BUDGET = 36_000_000
LIMIT = 8_000_000
ACK = 67_866
FAST = Fraction(10037, 10000)
SLOW = Fraction(9963, 10000)


class LedgerModel:
    def __init__(self, entries=()):
        self.now = 100
        self.durable = (1, 0, tuple(entries))
        self.attempts = []
        self.restart(True, 0)

    def restart(self, enabled, bias):
        self.live = None
        self.anchor = None
        self.grant = None
        self.pending = None
        self.sample = None
        self.epoch = 0
        self.rebase = False
        self.frozen = False
        self.set_trust(enabled, bias)
        if self.utc() is not None:
            self.restore()

    def set_trust(self, enabled, bias):
        new = (
            (self.now, self.now - 100 + bias, abs(bias) + 1, self.now + 10_000_000_000)
            if enabled
            else None
        )
        if new is None or (
            self.sample is not None
            and new[1] - new[0] != self.sample[1] - self.sample[0]
        ):
            self.epoch += 1
            self.rebase = True
        self.sample = new

    def utc(self):
        if self.sample is None:
            return None
        mono, utc, error, horizon = self.sample
        growth = ceil(Fraction((self.now - mono) * 37, 9963))
        if self.now >= horizon or error + growth >= 40_000_000:
            self.sample = None
            self.epoch += 1
            self.rebase = True
            return None
        return utc + self.now - mono

    def restore(self):
        utc = self.utc()
        self.live = [
            (charge, expiration)
            for charge, expiration in self.durable[2]
            if expiration > utc
        ]
        self.anchor = (self.now, utc)
        self.rebase = False

    @staticmethod
    def aged(entries, anchor, now):
        mono, utc = anchor
        return [
            (charge, expiration)
            for charge, expiration in entries
            if now < mono + ceil(Fraction(expiration - utc) * FAST)
        ]

    def advance(self, duration):
        self.now += duration

    def reason(self):
        if self.pending is not None:
            return "PERSISTENCE_PENDING"
        if self.utc() is None:
            return "UNTRUSTED_TIME"
        if self.rebase or self.frozen or self.grant is None:
            return "GRANT_REQUIRED"
        if self.grant["offset"] != self.sample[1] - self.sample[0]:
            return "GRANT_REQUIRED"
        if self.now >= self.grant["deadline"]:
            return "GRANT_EXPIRED"
        if self.grant["available"] < ACK:
            return "BUDGET_EXHAUSTED"
        return "ALLOWED"

    @property
    def available(self):
        return self.grant["available"] if self.reason() == "ALLOWED" else 0

    @property
    def total(self):
        return (
            None
            if self.live is None or self.pending is not None
            else sum(charge for charge, _ in self.live)
        )

    def recover(self):
        if self.utc() is None:
            return "UNTRUSTED_TIME"
        if self.live is None or self.rebase:
            self.restore()
        else:
            self.live = self.aged(self.live, self.anchor, self.now)
        return "STATE_READY"

    def acquire(self, outcome="committed"):
        if self.pending is not None:
            return self.reconcile()
        if self.reason() == "ALLOWED":
            return "ALLOWED"
        if self.grant is not None:
            return self.settle(True, outcome)
        result = self.recover()
        if result != "STATE_READY":
            return result
        return self.prepare(True, outcome)

    def spend(self, certainty):
        reason = self.reason()
        if reason != "ALLOWED":
            return reason
        self.grant["available"] -= ACK
        if certainty == "NOT_STARTED":
            self.grant["available"] += ACK
        else:
            self.attempts.append((self.now - 100, ACK))
            # This oracle observes actual attempts independently of any bucket decision.
            observed = sum(
                charge
                for actual, charge in self.attempts
                if self.now - 100 - WINDOW <= actual <= self.now - 100
            )
            assert observed <= BUDGET, (
                "continuous physical window exceeded",
                observed,
                self.attempts,
            )
        return reason

    def settle(self, precharge, outcome="committed"):
        if self.pending is not None:
            return self.reconcile()
        if self.grant is None:
            return self.acquire(outcome) if precharge else self.recover()
        self.frozen = True
        return self.prepare(precharge, outcome)

    def prepare(self, precharge, outcome):
        utc = self.utc()
        if utc is None:
            return "UNTRUSTED_TIME"
        if self.rebase:
            entries = [(c, e) for c, e in self.durable[2] if e > utc]
            anchor = (self.now, utc)
        else:
            entries = self.aged(self.live, self.anchor, self.now)
            anchor = self.anchor
        if self.grant is not None:
            old = self.grant
            settled = old["baseline"] + old["increment"] - old["available"]
            entries = [
                (settled if e == old["expiration"] else c, e) for c, e in entries
            ]
            entries = [(c, e) for c, e in entries if c]
        if not entries:
            anchor = (self.now, utc)
        granted = None
        result = "STATE_READY"
        if precharge:
            if entries:
                reference_end = min(e for _, e in entries) - WINDOW - GUARD
                index = floor(Fraction(utc - reference_end, WIDTH)) + 1
                end = reference_end + index * WIDTH
            else:
                end = utc + WIDTH
            expiration = end + WINDOW + GUARD
            all_expirations = [e for _, e in entries] + [expiration]
            if (max(all_expirations) - min(all_expirations)) // WIDTH >= 64:
                result = "CAPACITY_EXCEEDED"
            else:
                baseline = sum(c for c, e in entries if e == expiration)
                increment = min(LIMIT - baseline, BUDGET - sum(c for c, _ in entries))
                if increment < ACK:
                    result = "BUDGET_EXHAUSTED"
                else:
                    entries = [(c, e) for c, e in entries if e != expiration]
                    entries.append((baseline + increment, expiration))
                    granted = dict(
                        expiration=expiration,
                        baseline=baseline,
                        increment=increment,
                        available=increment,
                        deadline=self.now + floor(Fraction(end - utc) * SLOW),
                        offset=self.sample[1] - self.sample[0],
                    )
                    result = "ALLOWED"
            if granted is None and self.grant is None:
                return result
        if any(expiration <= utc for _, expiration in entries):
            return "SNAPSHOT_DEFERRED"
        entries.sort(key=lambda item: item[1])
        requested = (self.durable[0] + 1, utc, tuple(entries))
        transition = dict(
            requested=requested,
            entries=entries,
            anchor=anchor,
            grant=granted,
            previous_live=deepcopy(self.live),
            previous_anchor=self.anchor,
            previous_grant=deepcopy(self.grant),
            epoch=self.epoch,
            next_reason=result,
        )
        if outcome in ("committed", "unknown_installed"):
            self.durable = requested
        if outcome.startswith("unknown"):
            self.pending = transition
            return "PERSISTENCE_PENDING"
        self.finish(transition)
        return (
            "PERSISTENCE_FAILED"
            if outcome == "not_installed"
            else self.finished_reason(transition)
        )

    def finish(self, transition):
        installed = self.durable == transition["requested"]
        if installed:
            self.live, self.anchor, self.grant = (
                transition["entries"],
                transition["anchor"],
                transition["grant"],
            )
            self.rebase = self.epoch != transition["epoch"]
        else:
            self.live, self.anchor, self.grant = (
                transition["previous_live"],
                transition["previous_anchor"],
                transition["previous_grant"],
            )
        self.pending = None
        self.frozen = False
        if self.utc() is not None and self.grant is not None:
            self.rebase |= self.grant["offset"] != self.sample[1] - self.sample[0]

    def finished_reason(self, transition):
        result = self.reason()
        if (
            self.durable == transition["requested"]
            and transition["grant"] is None
            and self.utc() is not None
        ):
            return transition["next_reason"]
        return result

    def reconcile(self, fail_load=False):
        if self.pending is not None:
            if fail_load:
                return "PERSISTENCE_PENDING"
            transition = self.pending
            self.finish(transition)
            return self.finished_reason(transition)
        if self.grant is not None:
            return self.reason()
        if self.utc() is None:
            return "UNTRUSTED_TIME"
        self.restore()
        return "STATE_READY"
