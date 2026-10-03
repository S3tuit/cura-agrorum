#!/usr/bin/env python3
"""Adversarial airtime recovery model check; imports no receiver code.

Run from the repository root:
  PYTHONPATH=receiver .venv/bin/python -m tests.support.models.tx_airtime_adversarial
  PYTHONPATH=receiver .venv/bin/python -m tests.support.models.tx_airtime_adversarial --trials 3000 --seed 7

Goal: find a violation of receiver/INTERFACE.md's airtime recovery contract:
any run in which the ledger (live entries x 2 s) is lower than charged airtime that is
really still inside the protected window, or in which real airtime in a rolling
physical hour exceeds 36 s.

Model (integer microseconds):
  * Physical time p is the reference. Each process has a fresh monotonic origin
    and a piecewise-constant rate inside +/-3700 ppm that may change mid-run.
  * UTC readings are p + e with |e| <= the advertised error bound; the adversary
    picks e at either bound or anywhere in between. Time may be ineligible.
  * Saves: ok, definite failure, unknown outcome (committed or not, reconciled
    later), crash before commit, crash after commit but before confirmation.
    Replacement snapshots at startup are frozen at (m_b, U_b), committed later.
  * State on disk may be missing or corrupt (fallback path).
  * Every TX starts >= T after the previous one; its RF occupies the full charge
    and ends exactly T after admission (the latest the envelope allows).

Properties, checked independently of the ledger's own bookkeeping:
  P1 coverage: at every admission attempt, every recovery and every replacement
     commit, sum(charge of TX with s + W + T > p) <= 2 s x live entries.
  P2 limit: max RF airtime over every 3600-s physical window <= 36 s.

Controls (each disables one design rule) must FAIL, proving the checker can see
the defects the rules exist to prevent.
"""

import argparse
import bisect
import json
import random

US = P = 1_000_000
R = 3_700
N = 18
Q = 2 * US
B = N * Q
MSABP = 2 * US
W = 3600 * US
T = 250_000
H = W + T
ACK = 67_866
CHARGES = (ACK, ACK, ACK, 31_000, 120_000)  # variable packet charges, all < T
LONGEST = max(CHARGES)


def cdiv(a, b):
    return -(-a // b)


def wait(d):
    return cdiv(d * (P + R), P)


HOLD = wait(H)
assert HOLD == 3_613_570_925
assert LONGEST <= MSABP


class Violation(Exception):
    pass


class Clock:
    """Integer monotonic readings; each segment's rate lies within +/-R ppm."""

    def __init__(self, boot, m0, rate):
        self.segs = [(boot, m0, rate)]

    def set_rate(self, p, rate):
        self.segs.append((p, self.mono(p), rate))

    def mono(self, p):
        for ps, ms, r in reversed(self.segs):
            if p >= ps:
                return ms + (p - ps) * (P + r) // P
        raise AssertionError("time before boot")

    def phys(self, m):
        """Earliest physical instant whose reading is >= m."""
        for ps, ms, r in reversed(self.segs):
            if m >= ms:
                return ps + cdiv((m - ms) * P, P + r)
        return self.segs[0][0]


class Snapshot:
    __slots__ = ("utc", "err", "eligible", "lifetimes")

    def __init__(self, utc, err, eligible, lifetimes):
        self.utc, self.err, self.eligible, self.lifetimes = utc, err, eligible, lifetimes


class Proc:
    def __init__(self, sim, clock):
        self.sim, self.clock = sim, clock
        self.entries = [0] * N         # monotonic deadlines, 0 = empty
        self.current = None
        self.used = 0
        self.barrier = False
        self.enabled = False
        self.last_tx = None

    def expire(self, now):
        for i, d in enumerate(self.entries):
            if d and d <= now:
                self.entries[i] = 0
                if self.current == i:
                    self.current = None

    def live(self, p):
        now = self.clock.mono(p)
        self.expire(now)
        return sum(1 for d in self.entries if d)

    def admit(self, p, charge):
        sim = self.sim
        live = self.live(p)
        sim.check_cover(p, live, "pre-admit")
        if not self.enabled or self.barrier:
            return False
        if self.last_tx is not None and p - self.last_tx < T:
            return False
        now = self.clock.mono(p)
        if self.current is None:
            free = [i for i, d in enumerate(self.entries) if not d]
            if not free:
                return False
            if self.used + charge > MSABP:
                return False
            self.current = free[0]
        if self.used + charge > MSABP:
            return False
        self.used += charge
        self.entries[self.current] = max(self.entries[self.current], now + sim.hold)
        self.last_tx = p
        sim.record_tx(p, charge)
        sim.check_cover(p, self.live(p), "post-admit")
        if self.used + LONGEST >= MSABP:
            self.barrier = True
        return True

    def snapshot(self, p, eligible, err):
        now = self.clock.mono(p)
        self.expire(now)
        life = tuple(cdiv((d - now) * P, P - R) if d > now else 0 for d in self.entries)
        return Snapshot(self.sim.utc(p, err), err, eligible, life)

    def confirm(self):
        self.used = 0
        if not self.sim.ctl.get("keep_group_open"):
            self.current = None
        self.barrier = False

    def next_expiry_phys(self, p):
        live = [d for d in self.entries if d]
        if not live:
            return p
        return self.clock.phys(min(live))


class Sim:
    def __init__(self, rng, ctl=None):
        self.rng = rng
        self.ctl = ctl or {}
        self.hold = wait(W) if self.ctl.get("nominal_hour") else HOLD
        self.disk = None             # None missing, "corrupt", or Snapshot
        self.tx_s, self.tx_c, self.prefix = [], [], [0]
        self.stats = dict(boots=0, fallbacks=0, full_replacements=0, absorbed=0,
                          tx=0, saves=0, lost_saves=0, max_live_after_recovery=0)

    # ---------------- reference ----------------
    def record_tx(self, p, c):
        assert not self.tx_s or p >= self.tx_s[-1] + T
        self.tx_s.append(p)
        self.tx_c.append(c)
        self.prefix.append(self.prefix[-1] + c)
        self.stats["tx"] += 1

    def obligation(self, p):
        i = bisect.bisect_right(self.tx_s, p - H)
        return self.prefix[-1] - self.prefix[i]

    def check_cover(self, p, live, where):
        ob = self.obligation(p)
        if ob > Q * live:
            raise Violation(dict(property="P1", where=where, at_us=p,
                                 real_charged_in_window_us=ob, ledger_us=Q * live))

    def max_rolling_rf(self):
        """Exact max over windows [x - W, x] of RF on [s + T - c, s + T]."""
        iv = [(s + T - c, s + T) for s, c in zip(self.tx_s, self.tx_c)]
        if not iv:
            return 0
        starts = [a for a, _ in iv]
        ends = [b for _, b in iv]
        pre = [0]
        for a, b in iv:
            pre.append(pre[-1] + b - a)

        def total(x):
            lo = x - W
            i = bisect.bisect_right(ends, lo)       # first interval ending after lo
            j = bisect.bisect_left(starts, x)       # intervals starting before x
            if j <= i:
                return 0
            s = pre[j] - pre[i]
            s -= max(0, lo - iv[i][0])
            s -= max(0, iv[j - 1][1] - x)
            return s
        best = 0
        for a, b in iv:
            best = max(best, total(b), total(a + W))
        return best

    # ---------------- adversarial inputs ----------------
    def utc(self, p, err):
        r = self.rng.random()
        e = -err if r < 0.35 else err if r < 0.7 else self.rng.randint(-err, err)
        return p + e

    def rate(self):
        return self.rng.choice([R, R, -R, 0, self.rng.randint(-R, R)])

    def err(self):
        return self.rng.choice([0, 0, US, 5 * US, 60 * US, 600 * US])

    # ---------------- persistence ----------------
    def save(self, proc, p, outcome, eligible=True, err=None):
        """Synchronous save; returns (p_after, alive)."""
        err = self.err() if err is None else err
        snap = proc.snapshot(p, eligible, err)
        dur = self.rng.choice([1_000, 50_000, 1_800_000])
        p += dur
        self.stats["saves"] += 1
        if outcome == "ok":
            self.disk = snap
            proc.confirm()
            return p, True
        if outcome == "fail":
            return p + 500_000, True
        if outcome == "unknown":
            committed = self.rng.random() < 0.5
            if committed:
                self.disk = snap
            p += self.rng.choice([1_000_000, 30_000_000])   # TX blocked meanwhile
            if committed:
                proc.confirm()
            return p, True
        if outcome == "crash_before":
            self.stats["lost_saves"] += 1
            return p, False
        if outcome == "crash_after":
            self.disk = snap
            return p, False
        raise AssertionError(outcome)

    # ---------------- startup ----------------
    def boot(self, p, *, rate=None, eligible=None, err=None, e_override=None,
             replacement=None):
        """Returns (proc, p_enabled) or (None, p_crash) if it died in recovery."""
        self.stats["boots"] += 1
        rng = self.rng
        clock = Clock(p, rng.choice([0, rng.randint(1, 10**13)]),
                      self.rate() if rate is None else rate)
        proc = Proc(self, clock)
        m_b = clock.mono(p)
        err_b = self.err() if err is None else err
        startup_ok = (rng.random() > 0.05) if eligible is None else eligible
        if e_override is not None:
            u_b = p + e_override
        else:
            u_b = self.utc(p, err_b)
        d = self.disk
        fallback = (not isinstance(d, Snapshot) or not d.eligible or not startup_ok)
        if not fallback:
            e_err = d.err + err_b
            elapsed_min = max(0, u_b - d.utc - e_err)
            elapsed_max = u_b - d.utc + e_err
            fallback = elapsed_max < 0
        if fallback:
            self.stats["fallbacks"] += 1
            proc.entries = [m_b + self.hold] * N
        else:
            for i, life in enumerate(d.lifetimes):
                rem = max(0, life - elapsed_min)
                proc.entries[i] = 0 if rem == 0 else m_b + wait(rem)
            if not self.ctl.get("no_recovery_charge"):
                free = [i for i, x in enumerate(proc.entries) if not x]
                if free:
                    proc.entries[free[0]] = m_b + self.hold
                else:
                    self.stats["full_replacements"] += 1
                    k = min(range(N), key=lambda i: proc.entries[i])
                    if proc.entries[k] >= m_b + self.hold:
                        self.stats["absorbed"] += 1
                    if self.ctl.get("replace_latest"):
                        k = max(range(N), key=lambda i: proc.entries[i])
                    proc.entries[k] = max(proc.entries[k], m_b + self.hold)
        live = sum(1 for x in proc.entries if x)
        self.stats["max_live_after_recovery"] = max(self.stats["max_live_after_recovery"], live)
        self.check_cover(p, live, "recovery")
        # Frozen replacement snapshot at (m_b, U_b).
        frozen = Snapshot(u_b, err_b, startup_ok,
                          tuple(cdiv((x - m_b) * P, P - R) if x > m_b else 0
                                for x in proc.entries))
        while True:
            outcome = replacement or rng.choice(
                ["ok"] * 8 + ["fail", "unknown", "crash_before", "crash_after"])
            replacement = None
            p += rng.choice([1_000, 50_000, 1_800_000])
            if outcome in ("ok", "crash_after") or (outcome == "unknown" and rng.random() < 0.5):
                self.disk = frozen
                if outcome == "unknown":
                    p += 1_000_000
                if outcome == "crash_after":
                    return None, p
                proc.enabled = True
                proc.live(p)
                self.check_cover(p, proc.live(p), "replacement-commit")
                return proc, p
            if outcome == "crash_before":
                return None, p
            p += 500_000


# ---------------------------------------------------------------- drivers
def greedy(sim, proc, p, until, charge=ACK):
    """Transmit at the maximum rate allowed; jump to expiries when full."""
    while p < until:
        if proc.barrier:
            return p
        if proc.admit(p, charge):
            p += T
            continue
        if proc.current is None and all(proc.entries):
            p = max(p + 1, proc.next_expiry_phys(p))
        else:
            p += T
    return p


def random_run(seed, hours, ctl=None):
    rng = random.Random(seed)
    sim = Sim(rng, ctl)
    if rng.random() < 0.7:
        sim.disk = Snapshot(0, 0, True, (0,) * N)   # valid empty state
    p = rng.randint(0, 10 * US)
    end = hours * 3600 * US
    proc = None
    while p < end:
        if proc is None:
            r = rng.random()
            if r < 0.03:
                sim.disk = "corrupt"
            p += rng.choice([1_000, US, 10 * US, 61 * US,
                             rng.randint(0, 2 * W), rng.randint(W - 300 * US, W + 300 * US)])
            proc, p = sim.boot(p)
            if proc is None:
                continue
        r = rng.random()
        if proc.barrier:
            outcome = rng.choice(["ok"] * 10 + ["fail", "unknown", "crash_before",
                                                 "crash_before", "crash_after"])
            p, alive = sim.save(proc, p, outcome, eligible=rng.random() > 0.05)
        elif r < 0.04:   # RTC/provenance save, closes the current group
            outcome = rng.choice(["ok"] * 6 + ["fail", "unknown", "crash_before", "crash_after"])
            p, alive = sim.save(proc, p, outcome, eligible=rng.random() > 0.05)
        elif r < 0.08:
            alive = False
            p += rng.randint(0, T)
        elif r < 0.11:
            proc.clock.set_rate(p, sim.rate())
            alive = True
        elif r < 0.25:
            p += rng.choice([rng.randint(0, 600 * US), rng.randint(0, 2 * W)])
            alive = True
        elif r < 0.35:
            p = max(p, proc.next_expiry_phys(p)) + rng.choice([0, 1, 1000, T])
            alive = True
        else:
            n = rng.randint(1, 40)
            for _ in range(n):
                if proc.barrier:
                    break
                c = rng.choice(CHARGES)
                if proc.admit(p, c):
                    p += T + rng.choice([0, 0, 0, 1, 1000, 100_000])
                elif proc.current is None and all(proc.entries):
                    p = max(p + 1, proc.next_expiry_phys(p))
                else:
                    p += T
            alive = True
        if not alive:
            proc = None
    rf = sim.max_rolling_rf()
    if rf > B:
        raise Violation(dict(property="P2", max_rolling_rf_us=rf, budget_us=B))
    return sim, rf


def lost_save_restart_schedule(ctl=None):
    """Lost threshold saves across five trusted
    restarts, eight more groups, then an untrusted fallback and greedy TX."""
    rng = random.Random(3)
    sim = Sim(rng, ctl)
    sim.disk = Snapshot(0, 0, True, (0,) * N)
    p = 0
    proc, p = sim.boot(p, rate=R, err=US, e_override=US, replacement="ok")
    for _ in range(5):
        p = greedy(sim, proc, p, p + 60 * US)
        assert proc.barrier
        p += 300_000
        p, _ = sim.save(proc, p, "crash_before", err=US)
        p += 9 * US
        proc, p = sim.boot(p, rate=R, err=US, e_override=US, replacement="ok")
        p = greedy(sim, proc, p, p + 60 * US)
        p, _ = sim.save(proc, p + 300_000, "ok", err=US)
    for _ in range(8):
        p += 60 * US
        p = greedy(sim, proc, p, p + 60 * US)
        if proc.barrier:
            p, _ = sim.save(proc, p + 300_000, "ok", err=US)
    p += 500_000
    proc, p = sim.boot(p, rate=R, eligible=False, replacement="ok")
    p = greedy(sim, proc, p, p + 3 * W)
    rf = sim.max_rolling_rf()
    if rf > B:
        raise Violation(dict(property="P2", max_rolling_rf_us=rf))
    return sim, rf


def full_array_zombie(ctl=None, err=60 * US):
    """Fill all 18 entries, reuse the first slot to expire for an unsaved
    1.968-s group, crash, recover with every entry still live, transmit greedily
    at each later expiry. Repeat crash/recover cycles to stack replacements."""
    rng = random.Random(4)
    sim = Sim(rng, ctl)
    sim.disk = Snapshot(0, 0, True, (0,) * N)
    p = 0
    proc, p = sim.boot(p, rate=R, err=0, e_override=0, replacement="ok")
    cycles = 0
    for step in range(400):
        p = greedy(sim, proc, p, p + 8 * W)
        if not proc.barrier:
            break
        live_before = proc.live(p)
        if live_before == N and proc.current is not None and step % 2 == 0:
            # Maximum unsaved use in a reused slot; crash instead of saving.
            p, _ = sim.save(proc, p + 300_000, "crash_before", err=err)
            p += 1
            # Tight recovery: elapsed_min equals the true elapsed time.
            proc, p = sim.boot(p, rate=R, err=err, e_override=err, replacement="ok")
            cycles += 1
        else:
            p, _ = sim.save(proc, p + 300_000, "ok", err=err)
    rf = sim.max_rolling_rf()
    if rf > B:
        raise Violation(dict(property="P2", max_rolling_rf_us=rf))
    return sim, rf


def no_tx_crash_storm(ctl=None):
    """Full ledger, then 40 rapid no-TX crashes with error bounds that inflate
    round-tripped deadlines; afterwards transmit greedily for three hours."""
    rng = random.Random(5)
    sim = Sim(rng, ctl)
    sim.disk = Snapshot(0, 0, True, (0,) * N)
    proc, p = sim.boot(0, rate=R, err=0, e_override=0, replacement="ok")
    for _ in range(N + 1):
        p = greedy(sim, proc, p, p + W)
        if not proc.barrier:
            break
        p, _ = sim.save(proc, p + 300_000, "ok", err=0)
        if proc.live(p) == N:
            break
    p = greedy(sim, proc, p, p + 3 * US)
    for _ in range(40):
        p += rng.choice([1_000, US, 30 * US])
        proc, p = sim.boot(p, rate=-R, err=60 * US, e_override=-60 * US, replacement="ok")
    for _ in range(12):
        p = greedy(sim, proc, p, p + 3 * W)
        if proc.barrier:
            p, _ = sim.save(proc, p + 300_000, "ok")
    rf = sim.max_rolling_rf()
    if rf > B:
        raise Violation(dict(property="P2", max_rolling_rf_us=rf))
    return sim, rf


def idle_expiry_unsaved(ctl=None):
    """A current group of 28 ACKs expires during idle time without a save; the
    retained counter must still force the threshold save; then crash/recover."""
    rng = random.Random(6)
    sim = Sim(rng, ctl)
    sim.disk = Snapshot(0, 0, True, (0,) * N)
    proc, p = sim.boot(0, rate=R, err=0, e_override=0, replacement="ok")
    for _ in range(28):
        assert proc.admit(p, ACK)
        p += T
    p += HOLD + US
    assert proc.live(p) == 0 or proc.current is None
    for _ in range(5):
        if proc.barrier:
            break
        assert proc.admit(p, ACK)
        p += T
    p, _ = sim.save(proc, p, "crash_before")
    proc, p = sim.boot(p + 1, rate=R, err=0, e_override=0, replacement="ok")
    p = greedy(sim, proc, p, p + 3 * W)
    rf = sim.max_rolling_rf()
    if rf > B:
        raise Violation(dict(property="P2", max_rolling_rf_us=rf))
    return sim, rf


SCENARIOS = {
    "lost_save_restart_schedule": lost_save_restart_schedule,
    "full_array_zombie": full_array_zombie,
    "no_tx_crash_storm": no_tx_crash_storm,
    "idle_expiry_unsaved": idle_expiry_unsaved,
}

CONTROLS = {
    "no_recovery_charge": {"no_recovery_charge": True},
    "keep_group_open_after_save": {"keep_group_open": True},
    "nominal_hour_hold": {"nominal_hour": True},
    "replace_latest_entry": {"replace_latest": True},
}


def attempt(fn, *a, **kw):
    try:
        sim, rf = fn(*a, **kw)
        return dict(violation=None, max_rolling_rf_us=rf, **sim.stats)
    except Violation as v:
        return dict(violation=v.args[0])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--trials", type=int, default=800)
    ap.add_argument("--hours", type=int, default=12)
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()

    out = {"design": {}, "controls": {}}

    for name, fn in SCENARIOS.items():
        out["design"][name] = attempt(fn)
    worst, found = 0, []
    agg = dict(boots=0, fallbacks=0, full_replacements=0, absorbed=0, tx=0, lost_saves=0)
    for k in range(args.trials):
        seed = args.seed * 1_000_003 + k
        res = attempt(random_run, seed, args.hours)
        if res["violation"]:
            found.append(dict(seed=seed, **res["violation"]))
            continue
        worst = max(worst, res["max_rolling_rf_us"])
        for key in agg:
            agg[key] += res[key]
    out["design"]["random"] = dict(trials=args.trials, hours=args.hours,
                                   violations=found[:5], violation_count=len(found),
                                   worst_rolling_rf_us=worst, **agg)

    for cname, ctl in CONTROLS.items():
        res = {n: attempt(fn, ctl)["violation"] is not None for n, fn in SCENARIOS.items()}
        hits = 0
        for k in range(min(200, args.trials)):
            if attempt(random_run, args.seed * 7 + k, args.hours, ctl)["violation"]:
                hits += 1
        res["random_hits_of_200"] = hits
        out["controls"][cname] = res
    out["design_violated"] = any(v.get("violation") or v.get("violation_count")
                                 for v in out["design"].values())
    print(json.dumps(out, indent=2))
    if out["design_violated"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
