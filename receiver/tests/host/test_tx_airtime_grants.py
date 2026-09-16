"""Write-ahead grants and process-local allowance through the production worker."""

from dataclasses import replace
import sqlite3
from threading import Event, current_thread

import pytest

from cura_receiver.airtime_ledger import AirtimeCorrelation
from cura_receiver.communicator_state_owner import CommunicatorStateOwner
from cura_receiver.generated.receiver_entities_generated import (
    TxAirtimeBucketV1 as Bucket,
)
from cura_receiver.generated.receiver_enums_generated import (
    RtcHealth as RH,
    SystemTimeQuality as Q,
)
from cura_receiver.time_observations import TrustedTimeSample
from cura_receiver.tx_airtime import TxAirtimePolicy, AirtimeReason as R
from tests.support.builders.persistence_control import state
from tests.support.coordination.state_commit import LostStateReply
from tests.support.coordination.threads import (
    start_checked_threads,
    join_checked_threads,
)


def populated(*pairs):
    return state(
        buckets=tuple(Bucket(*pair) for pair in pairs)
        + (Bucket(0, 0),) * (64 - len(pairs))
    )


def acquire(policy, clock):
    return policy.acquire_grant(
        deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000
    )


# Descheduling after the clock sample cannot pair old UTC with a later monotonic origin and extend permission.
def test_grant_preparation_uses_one_clock_sample(airtime_component, monkeypatch):
    policy, worker, database, clock, loaded = airtime_component(initial_state=state())
    assert policy.recover(deadline_monotonic_us=5_000_100).reason is R.STATE_READY
    arrived, release = Event(), Event()
    armed = False
    read = clock.now_monotonic_us
    prepare = policy._prepare_transition

    def delayed_clock_read():
        nonlocal armed
        captured = read()
        if armed and current_thread().name == "airtime-preemption":
            armed = False
            arrived.set()
            assert release.wait(5)
        return captured

    def prepare_at_barrier(**kwargs):
        nonlocal armed
        armed = True
        return prepare(**kwargs)

    monkeypatch.setattr(clock, "now_monotonic_us", delayed_clock_read)
    monkeypatch.setattr(policy, "_prepare_transition", prepare_at_barrier)
    results = []
    threads = start_checked_threads(
        [
            (
                "airtime-preemption",
                lambda: results.append(
                    policy.acquire_grant(deadline_monotonic_us=100_000_100)
                ),
            )
        ]
    )
    try:
        assert arrived.wait(5)
        clock.advance_elapsed_us(30_000_000)
    finally:
        release.set()
        join_checked_threads(threads)
    assert results[0].reason is R.ALLOWED
    spend = policy.try_spend()
    assert spend.grant_deadline_monotonic_us == 59_778_100
    assert policy._ledger.retention_deadline(3_780_000_000) == 3_793_986_100
    clock.advance_elapsed_us(29_778_000)
    assert policy.try_spend().reason is R.GRANT_EXPIRED


# Reconstruction pairs the UTC sample with its captured monotonic value even if preparation is delayed.
def test_reconstruction_uses_one_clock_sample(airtime_component, monkeypatch):
    policy, _, _, clock, _ = airtime_component(
        initial_state=populated((1, 3_780_000_000))
    )
    restore = policy._restore

    def delayed_restore(value, utc, monotonic):
        clock.advance_elapsed_us(30_000_000)
        return restore(value, utc, monotonic)

    monkeypatch.setattr(policy, "_restore", delayed_restore)
    assert policy.recover(deadline_monotonic_us=40_000_100).reason is R.STATE_READY
    assert policy._ledger.retention_deadline(3_780_000_000) == 3_793_986_100


class ObserveCommit:
    def __init__(self, control, before=lambda: None, after=lambda: None):
        self.control, self.before, self.after = control, before, after

    def commit_communicator_state(self, value, **kwargs):
        self.before()
        result = self.control.commit_communicator_state(value, **kwargs)
        self.after()
        return result

    def load_communicator_state(self, **kwargs):
        return self.control.load_communicator_state(**kwargs)


# Only a durable increment creates allowance; commit latency consumes the original shortened lifetime.
def test_durable_grant_before_allowance_and_exact_deadline(airtime_component):
    policy, worker, database, clock, loaded = airtime_component(initial_state=state())
    checked = []

    def before():
        checked.append(policy.available_charge_us)
        assert policy.owner.pending is not None

    channel = ObserveCommit(
        worker.control, before, lambda: clock.advance_elapsed_us(1_000_000)
    )
    policy.owner = CommunicatorStateOwner.from_load(control=channel, loaded=loaded)
    assert policy.available_charge_us == 0
    assert acquire(policy, clock).reason is R.ALLOWED
    assert checked == [0] and policy.available_charge_us == 8_000_000
    assert policy.state.generation == 2
    with sqlite3.connect(database) as connection:
        assert connection.execute(
            "SELECT generation FROM communicator_state"
        ).fetchone() == (2,)
    clock.advance_elapsed_us(58_777_999)
    assert policy.available_charge_us == 8_000_000
    clock.advance_elapsed_us(1)
    assert policy.available_charge_us == 0
    assert policy.try_spend().reason is R.GRANT_EXPIRED
    assert policy.state.generation == 2


# A restart baseline retains its exact expiration and only an acknowledged top-up becomes spendable.
def test_loaded_current_bucket_top_up(airtime_component):
    original = populated((4_000_000, 3_780_000_000))
    policy, _, _, clock, _ = airtime_component(initial_state=original, utc=30_000_000)
    assert policy.available_charge_us == 0
    assert acquire(policy, clock).reason is R.ALLOWED
    assert policy.available_charge_us == 4_000_000 and policy.total_used == 8_000_000
    assert policy.state.buckets[0] == Bucket(8_000_000, 3_780_000_000)
    clock.advance_elapsed_us(29_889_000)
    assert policy.available_charge_us == 0


# Bucket and global headroom independently require the complete 67,866-us ACK charge at equality.
@pytest.mark.parametrize("delta", [-1, 0, 1])
@pytest.mark.parametrize("global_limited", [False, True])
def test_complete_ack_headroom_boundaries(airtime_component, delta, global_limited):
    headroom = 67_866 + delta
    original = (
        populated(
            (4_000_000 - headroom, 3_480_000_000),
            (8_000_000, 3_540_000_000),
            (8_000_000, 3_600_000_000),
            (8_000_000, 3_660_000_000),
            (8_000_000, 3_720_000_000),
        )
        if global_limited
        else populated((8_000_000 - headroom, 3_780_000_000))
    )
    policy, _, _, clock, _ = airtime_component(initial_state=original)
    initial = policy.state
    result = acquire(policy, clock)
    if delta < 0:
        assert result.reason is R.BUDGET_EXHAUSTED and policy.state is initial
        assert policy.state == original
        assert policy.available_charge_us == 0
    else:
        assert result.reason is R.ALLOWED and policy.available_charge_us == headroom
        assert policy.total_used == (36_000_000 if global_limited else 8_000_000)


# A fresh current bucket never acquires an earlier process's old charge under a new expiration.
def test_restart_continues_grid_without_relabeling(airtime_component):
    original = populated((8_000_000, 3_720_000_000))
    policy, _, _, clock, _ = airtime_component(initial_state=original, utc=20_000_000)
    assert acquire(policy, clock).reason is R.ALLOWED
    assert policy.state.buckets[:2] == (
        Bucket(8_000_000, 3_720_000_000),
        Bucket(8_000_000, 3_780_000_000),
    )
    assert policy.total_used == 16_000_000 and policy.available_charge_us == 8_000_000


# Crossing either the bucket or source-validity boundary during commit cannot enable an expired grant.
@pytest.mark.parametrize("trust_expired", [False, True])
def test_commit_completion_rechecks_time(airtime_component, trust_expired):
    policy, worker, _, clock, loaded = airtime_component(initial_state=state())
    if trust_expired:
        policy.update_time(
            AirtimeCorrelation(
                TrustedTimeSample(100, 0, 1, Q.NETWORK_SYNCED, 1), 1, 101
            ),
            rtc_health=RH.PRESENT,
        )
    channel = ObserveCommit(
        worker.control, after=lambda: clock.advance_elapsed_us(60_000_000)
    )
    policy.owner = CommunicatorStateOwner.from_load(control=channel, loaded=loaded)
    assert acquire(policy, clock).reason is (
        R.UNTRUSTED_TIME if trust_expired else R.GRANT_EXPIRED
    )
    assert policy.state.generation == 2 and policy.available_charge_us == 0
    assert policy.total_used == 8_000_000


# An unknown increment is unusable until exact reconciliation; a replacement process owns none of it.
@pytest.mark.parametrize("installed", [False, True])
def test_unknown_grant_and_replacement_baseline(airtime_component, installed):
    policy, worker, _, clock, loaded = airtime_component(initial_state=state())
    channel = LostStateReply(worker.control, installed=installed)
    policy.owner = CommunicatorStateOwner.from_load(control=channel, loaded=loaded)
    assert acquire(policy, clock).reason is R.PERSISTENCE_PENDING
    assert policy.available_charge_us == 0
    channel.fail_load = True
    assert acquire(policy, clock).reason is R.PERSISTENCE_PENDING
    channel.fail_load = False
    assert acquire(policy, clock).reason is (
        R.ALLOWED if installed else R.GRANT_REQUIRED
    )
    assert policy.available_charge_us == (8_000_000 if installed else 0)
    actual = worker.control.load_communicator_state(deadline_monotonic_us=5_000_100)
    replacement = TxAirtimePolicy(
        state_owner=CommunicatorStateOwner.from_load(
            control=worker.control, loaded=actual
        ),
        clock=clock,
    )
    replacement.update_time(
        AirtimeCorrelation(
            TrustedTimeSample(100, 0, 1, Q.NETWORK_SYNCED, 1), 1, 1_000_000
        ),
        rtc_health=RH.PRESENT,
    )
    assert replacement.available_charge_us == 0
    assert acquire(replacement, clock).reason is (
        R.BUDGET_EXHAUSTED if installed else R.ALLOWED
    )


# Exhausted generation authority is reported before a pending grant or database mutation exists.
def test_generation_exhaustion_fails_closed(airtime_component):
    original = state(generation=(1 << 63) - 1)
    policy, _, _, clock, _ = airtime_component(initial_state=original)
    assert acquire(policy, clock).reason is R.INVALID_STATE
    assert policy.owner.pending is None and policy.available_charge_us == 0
