#!/usr/bin/env python3
"""Independent, standalone airtime recovery coverage checks; no receiver imports.

Reproduce:
  PYTHONPATH=receiver .venv/bin/python -m tests.support.models.tx_airtime
  PYTHONPATH=receiver .venv/bin/python -m tests.support.models.tx_airtime --trials 1000 --steps 500

Physical time is exact Fraction arithmetic. Each process has a fresh integer
monotonic origin; rate may change between any two events, within +/-3700 ppm.
Event readings are integer microseconds, with no additional sampling error.
UTC readings are rounded integers with honest bounds including that rounding.

The oracle sees only real attempted TX charges and physical charge times. It
does not use ledger entries, snapshots, expiry calculations or commit status.
It counts each charge through charge_time + W + T, a stronger reference than
the physical rolling-hour occupancy. Coverage is checked at every ledger
expiration between actions, as well as every state transition and TX.

This checks the airtime recovery algorithm, not production serialization, threading,
hardware timing, arithmetic overflow, or enforcement of the clock envelope.
The finite recovery-envelope enumeration is exhaustive in its stated domain;
the event simulation is bounded, reproducible sampling, not an exhaustive proof.
"""

import argparse
from collections import Counter, deque
from dataclasses import dataclass
from fractions import Fraction as F
import itertools
import json
from pathlib import Path
import random


P, R = 1_000_000, 3700
W, T, H = 3_600_000_000, 250_000, 3_600_250_000
C, B, N, ACK = 2_000_000, 36_000_000, 18, 67_866
RATES = (P - R, P - R // 2, P, P + R // 2, P + R)


def ceildiv(a, b):
    return -(-a // b)


def wait(d):
    return ceildiv(d * (P + R), P)


HOLD = wait(H)


@dataclass(frozen=True)
class Snapshot:
    entries: tuple
    utc: int
    bound: int
    trusted: bool
    generation: int
    built_physical: F  # instrumentation only, never used for recovery


class Violation(Exception):
    pass


class Oracle:
    def __init__(self):
        self.tx = deque()
        self.total = 0
        self.last_time = F(0)

    def at(self, physical):
        assert physical >= self.last_time
        self.last_time = physical
        while self.tx and self.tx[0][0] + H <= physical:
            self.total -= self.tx.popleft()[1]
        return self.total

    def transmit(self, physical, charge):
        self.at(physical)
        self.tx.append((physical, charge))
        self.total += charge


class Model:
    def __init__(self, seed=0, mutation="design", longest=ACK):
        self.rng = random.Random(seed)
        self.mutation, self.longest = mutation, longest
        self.t, self.m, self.rate = F(0), 1_000_000_000, P
        self.entries, self.current, self.used = [0] * N, None, 0
        self.required, self.pending, self.starting = False, None, False
        self.disk = Snapshot((0,) * N, 0, P, True, 1, F(0))
        self.oracle, self.stats = Oracle(), Counter()
        self.trace = deque(maxlen=80)
        self.tx_ready = F(0)

    def record(self, event, **extra):
        self.trace.append(dict(event=event, physical_us=str(self.t), mono_us=self.m,
                               current=self.current, used=self.used, **extra))

    def sample(self, trusted=True, bound=P, sign=None):
        # The sub-us floor error is included explicitly in the declared bound.
        offset = self.rng.choice((-bound, 0, bound)) if sign is None else sign * bound
        utc = self.t.numerator // self.t.denominator + offset
        declared = bound + 1
        assert abs(F(utc) - self.t) <= declared
        return utc, declared, trusted

    def check(self, at=None, mono=None):
        at = self.t if at is None else at
        mono = self.m if mono is None else mono
        actual = self.oracle.at(at)
        accounted = C * sum(d > mono for d in self.entries)
        self.stats["coverage_checks"] += 1
        if accounted < actual or actual > B:
            raise Violation(json.dumps(dict(
                physical_us=str(at), mono_us=mono, accounted_us=accounted,
                reference_us=actual, entries=self.entries,
                mutation=self.mutation, trace=list(self.trace)), indent=2))

    def expire(self):
        for i, deadline in enumerate(self.entries):
            if deadline and deadline <= self.m:
                self.entries[i] = 0
                self.stats["expired_entries"] += 1
                if self.current == i:
                    self.current = None
                    self.stats["expired_current"] += 1
        # In particular, do not clear self.used.

    def advance(self, delta, rate=None):
        assert isinstance(delta, int) and delta >= 0
        rho = self.rate if rate is None else rate
        assert P - R <= rho <= P + R
        self.stats["rate_changes"] += rho != self.rate
        finish = self.m + delta
        for deadline in sorted(set(d for d in self.entries if self.m < d <= finish)):
            physical = self.t + F((deadline - self.m) * P, rho)
            self.check(physical, deadline)
            self.stats["expiration_edges"] += 1
        self.t += F(delta * P, rho)
        self.m, self.rate = finish, rho
        self.check()
        self.expire()

    def snapshot(self, correlation=None, trusted=True, bound=P):
        assert self.t >= self.tx_ready
        utc, error, eligible = correlation or self.sample(trusted, bound)
        values = tuple(ceildiv(max(0, d - self.m) * P, P - R) if d else 0
                       for d in self.entries)
        generation = 1 if self.disk is None else self.disk.generation + 1
        self.stats["snapshots"] += 1
        return Snapshot(values, utc, error, eligible, generation, self.t)

    def begin_save(self, correlation=None, trusted=True, bound=P):
        assert self.pending is None
        self.pending = self.snapshot(correlation, trusted, bound)
        self.record("begin_save", generation=self.pending.generation)

    def commit(self):
        assert self.pending is not None
        self.disk = self.pending
        self.stats["disk_commits"] += 1
        self.record("commit", generation=self.disk.generation)

    def confirm(self, receipt=None):
        assert self.pending is not None
        expected = self.pending
        got = self.disk if receipt is None else receipt
        if got != expected:
            self.stats["rejected_receipts"] += 1
            return False
        self.pending = None
        self.used, self.required, self.starting = 0, False, False
        if self.mutation != "reuse_closed":
            self.current = None
        self.stats["confirmed_saves"] += 1
        self.record("confirm")
        self.check()
        return True

    def fail_save(self):
        assert self.pending is not None and self.disk != self.pending
        self.pending = None
        self.stats["definite_failures"] += 1
        self.record("definite_failure")
        self.check()

    def save_ok(self, trusted=True):
        self.begin_save(trusted=trusted)
        self.advance(1000)
        self.commit()
        assert self.confirm()

    def tx(self, charge=None, real=True, refund=False):
        charge = ACK if charge is None else charge
        assert 0 < charge <= self.longest <= C
        assert not (real and refund)
        if self.starting or self.pending is not None or self.required:
            self.stats["barrier_denials"] += 1
            return False
        assert self.t >= self.tx_ready
        self.expire()
        if self.current is None:
            if 0 not in self.entries:
                self.stats["budget_denials"] += 1
                return False
            self.current = self.entries.index(0)
            self.stats["opened_groups"] += 1
        assert self.used + charge <= C
        self.used += charge
        duration = H if self.mutation == "nominal_wait" else HOLD
        self.entries[self.current] = max(self.entries[self.current], self.m + duration)
        if real:
            self.oracle.transmit(self.t, charge)
            self.stats["real_tx"] += 1
        else:
            self.stats["no_rf"] += 1
        if refund:
            # Models a definite non-start; never releases a historical entry.
            self.used -= charge
            self.stats["refunds"] += 1
        self.required = self.used + self.longest >= C
        self.tx_ready = self.t + T
        self.record("tx", charge=charge, real=real, refund=refund)
        self.check()
        return True

    def recover(self, down=0, trusted=True, bound=P, corrupt=False):
        assert isinstance(down, int) and down >= 0
        self.t += down  # no receiver process during this physical interval
        self.oracle.at(self.t)
        self.tx_ready = self.t
        self.m = self.rng.randrange(1, 10**12)
        self.entries, self.current, self.used = [0] * N, None, 0
        self.required, self.pending, self.starting = False, None, True
        correlation = self.sample(trusted, bound)
        utc, error, eligible = correlation
        old = None if corrupt else self.disk
        good = old is not None and old.trusted and eligible
        if good:
            minimum = max(0, utc - old.utc - error - old.bound)
            maximum = utc - old.utc + error + old.bound
            assert F(minimum) <= self.t - old.built_physical
            good = maximum >= 0
        if good:
            self.entries = [self.m + wait(d - minimum) if d > minimum else 0
                            for d in old.entries]
            if self.mutation != "omit_recovery":
                if 0 in self.entries:
                    i = self.entries.index(0)
                    self.entries[i] = self.m + HOLD
                    self.stats["recovery_added"] += 1
                else:
                    i = min(range(N), key=self.entries.__getitem__)
                    self.entries[i] = max(self.entries[i], self.m + HOLD)
                    self.stats["recovery_replaced"] += 1
            self.stats["trusted_recoveries"] += 1
        else:
            self.entries = [self.m + HOLD] * N
            self.stats["fallbacks"] += 1
        self.record("recover", trusted=good)
        self.check()
        # Startup recovery and its snapshot share the exact same correlation.
        self.begin_save(correlation)


def recovery_envelopes():
    """All sorted 18-entry vectors over six abstract expiration values.

    The abstract missing-use horizon is 3. At each future time x, the most
    that past TX can need is min(B, old_ledger(x) + C * (x < 3)). This already
    grants all earlier executions their inductive 36-second limit. Checking
    this envelope is stronger than sampling particular transmission histories.
    """
    vectors = comparisons = 0
    latest_mutation_failures = 0
    for old in itertools.combinations_with_replacement(range(6), N):
        new = list(old)
        index = new.index(0) if 0 in new else 0
        new[index] = max(new[index], 3)
        mutant = list(old)
        index_bad = mutant.index(0) if 0 in mutant else N - 1
        mutant[index_bad] = max(mutant[index_bad], 3)
        vectors += 1
        for x in range(7):
            before = C * sum(d > x for d in old)
            need = min(B, before + C * (x < 3))
            after = C * sum(d > x for d in new)
            assert after >= need, (old, x, after, need)
            comparisons += 1
            latest_mutation_failures += C * sum(d > x for d in mutant) < need
    assert latest_mutation_failures > 0
    return dict(vectors=vectors, comparisons=comparisons,
                latest_replacement_envelope_failures=latest_mutation_failures)


def clock_edges():
    checks = 0
    # Cover zero, 1-us rounding, the retention edge, and expired snapshots.
    for elapsed_m in (0, 1, 2, 999_999, HOLD - 1, HOLD, HOLD + 1):
        for split in (0, elapsed_m // 2, elapsed_m):
            for a, b in itertools.product(RATES, repeat=2):
                elapsed = F(split * P, a) + F((elapsed_m - split) * P, b)
                saved = ceildiv(max(0, HOLD - elapsed_m) * P, P - R)
                for down in (0, 1, H - 1, H, H + 1):
                    for uncertainty in (0, 1, 2 * P, 78 * P):
                        minimum = max(0, down - uncertainty)
                        restored = wait(max(0, saved - minimum))
                        earliest = F(restored * P, P + R)
                        needed = max(F(0), H - elapsed - down)
                        assert earliest >= needed
                        checks += 1
    return dict(exact_rational_checks=checks)


def directed():
    stats = Counter()
    # Save threshold, failed receipt, definite failure and lost acknowledgement.
    a = Model()
    for _ in range(29):
        assert a.tx()
        a.advance(wait(T))
    assert a.used == 1_968_114 and a.required
    assert not a.tx()
    a.begin_save()
    assert not a.confirm(a.disk)
    a.fail_save()
    assert a.used == 1_968_114 and a.required
    a.begin_save()
    a.commit()  # acknowledgement lost; TX remains blocked until reconciliation
    assert not a.tx()
    a.advance(3 * P)
    assert a.confirm()
    assert a.current is None and a.used == 0
    assert a.tx()
    a.advance(wait(T))
    a.save_ok()  # RTC-style early covering save closes a partial group
    stats.update(a.stats)

    # Current expiry during idle preserves the counter, then allocates anew.
    a = Model()
    assert a.tx()
    counter = a.used
    a.advance(HOLD + 1, P + R)
    assert a.current is None and a.used == counter
    assert a.tx()
    assert a.used == 2 * ACK
    a.advance(wait(T))
    a.save_ok()
    stats.update(a.stats)

    # A truly full ledger, then repeated recovery, max(old,new), and tied minima.
    a = Model()
    for _ in range(N):
        for _ in range(29):
            assert a.tx()
            a.advance(wait(T))
        a.save_ok()
    assert not a.tx()
    for i in range(40):
        a.recover(down=i % 3, bound=39 * P)
        assert not a.tx()
        a.commit()
        assert a.confirm()
    a.recover(trusted=False)
    a.commit()
    assert a.confirm()
    a.recover(trusted=True)  # previous snapshot ineligible -> fallback again
    a.commit()
    assert a.confirm()
    a.advance(HOLD + 1, P + R)
    assert a.tx()
    a.advance(wait(T))
    stats.update(a.stats)

    # Equality at the threshold, variable charges, refunds and uncertain no-RF.
    a = Model(longest=100_000)
    for _ in range(19):
        assert a.tx(charge=100_000)
        a.advance(wait(T))
    assert a.used + a.longest == C and a.required
    a.save_ok()
    assert a.tx(charge=100_000, real=False, refund=True)
    a.advance(wait(T))
    assert a.used == 0
    assert a.tx(charge=50_000, real=False)
    a.advance(wait(T))
    assert a.used == 50_000
    stats.update(a.stats)
    return dict(stats)


def mutations():
    detected = {}
    for mutation in ("omit_recovery", "reuse_closed", "nominal_wait"):
        a = Model(mutation=mutation)
        try:
            if mutation == "omit_recovery":
                assert a.tx()
                a.advance(wait(T))
                a.recover()  # disk is still the empty initial state
            elif mutation == "reuse_closed":
                for _ in range(31):
                    assert a.tx()
                    a.advance(wait(T))
                    if a.required:
                        a.save_ok()
            else:
                assert a.tx()
                a.advance(H, P + R)
        except Violation as error:
            failure = json.loads(str(error))
            detected[mutation] = {k: failure[k] for k in (
                "physical_us", "accounted_us", "reference_us")}
        else:
            raise AssertionError("oracle failed to detect " + mutation)
    return detected


def lost_save_restart_trace():
    source = json.loads((Path(__file__).parents[1] / "data" / "airtime_regression_requests.json").read_text())
    result, events = source["result"], source["events"]
    assert result["contract_violated"] and result["acks_in_window"] == 609
    a = Model()
    # Translate the old trace's negative quiet bootstrap into nonnegative time.
    origin = events[0][1]
    a.disk = None
    for kind, physical, *extra in events:
        target = physical - origin
        assert a.t.denominator == 1 and target >= a.t
        a.advance(target - int(a.t), P)
        if kind == "boot":
            a.recover(trusted=extra[0])
        elif kind == "save":
            if a.pending is None:
                a.begin_save()
            a.commit()
            assert a.confirm()
        elif kind == "lost_save":
            if a.pending is None:
                a.begin_save()
        elif kind == "tx":
            a.tx()  # Same requests; the current policy may refuse them.
    a.check()
    return dict(old_nominal_rf_us=result["nominal_rf_airtime_us"],
                old_admissions=result["acks_in_window"], stats=dict(a.stats))


def random_trial(seed, steps, mode):
    a = Model(seed, longest=(ACK if mode != "variable" else 200_000))
    rng = a.rng
    a.recover(trusted=mode != "fallback")
    a.commit()
    a.confirm()
    for _ in range(steps):
        # All pauses inspect every expiration crossed, including with rate changes.
        rate = rng.choice(RATES)
        if a.pending is not None:
            if rng.random() < .2:
                a.recover(rng.choice((0, 1, 2 * P, 4000 * P)),
                          trusted=rng.random() < .85, bound=rng.choice((P, 39 * P)))
                continue
            a.advance(rng.choice((1000, 2 * P, 5 * P)), rate)
            if rng.random() < .65:
                a.commit()
                if rng.random() < .8:
                    a.confirm()
                else:
                    a.stats["lost_acknowledgements"] += 1
            elif a.disk != a.pending:
                a.fail_save()
            else:
                a.confirm()
            continue
        if a.required or a.starting:
            a.begin_save(trusted=rng.random() < .95, bound=rng.choice((P, 39 * P)))
            continue
        pick = rng.random()
        if pick < (.16 if mode == "restart" else .045):
            a.recover(rng.choice((0, 1, P, 60 * P, 4000 * P)),
                      trusted=rng.random() < (.5 if mode == "fallback" else .92),
                      bound=rng.choice((P, 10 * P, 39 * P)), corrupt=rng.random() < .03)
        elif pick < .13:
            a.begin_save(trusted=rng.random() < .95, bound=rng.choice((P, 39 * P)))
        elif pick < (.43 if mode == "idle" else .26):
            if any(a.entries) and rng.random() < .5:
                delta = max(0, min(d for d in a.entries if d) - a.m)
                delta = max(0, delta + rng.choice((-1, 0, 1)))
            else:
                delta = rng.choice((1, 60 * P, 1800 * P, HOLD, 2 * HOLD))
            a.advance(delta, rate)
        else:
            for _ in range(rng.randint(1, 35)):
                charge = ACK if mode != "variable" else rng.choice((50_000, 100_000, 200_000))
                certainty = rng.random()
                if not a.tx(charge, real=certainty >= .04, refund=certainty < .02):
                    break
                a.advance(wait(T) + rng.choice((0, 1, 10_000)), rate)
                if a.required:
                    break
    # Drain the final ledger, checking all expiry edges and eventual reference zero.
    delta = max(a.entries + [a.m + HOLD]) - a.m + 1
    a.advance(delta, rng.choice(RATES))
    assert a.oracle.total == 0
    return a.stats


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=100)
    parser.add_argument("--steps", type=int, default=250)
    parser.add_argument("--seed", type=int, default=4004)
    args = parser.parse_args()
    report = dict(seed=args.seed, trials_per_mode=args.trials, steps_per_trial=args.steps)
    report["recovery_envelopes"] = recovery_envelopes()
    report["clock_edges"] = clock_edges()
    report["directed"] = directed()
    report["mutation_controls"] = mutations()
    report["lost_save_restart_trace"] = lost_save_restart_trace()
    report["randomized"] = {}
    for mode_index, mode in enumerate(("mixed", "restart", "fallback", "idle", "variable")):
        stats = Counter()
        for i in range(args.trials):
            seed = args.seed + mode_index * 1_000_000 + i
            try:
                stats.update(random_trial(seed, args.steps, mode))
            except Violation as error:
                report["failure"] = dict(mode=mode, seed=seed, counterexample=json.loads(str(error)))
                print(json.dumps(report, indent=2))
                raise SystemExit(1)
        report["randomized"][mode] = dict(stats)
    report["result"] = "PASS: no design undercount found in the stated checks"
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
