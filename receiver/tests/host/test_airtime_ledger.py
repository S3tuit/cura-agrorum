"""Exact virtual-time boundaries for the production airtime ring."""

from dataclasses import replace

import pytest

from cura_receiver.airtime_ledger import (
    AirtimeCorrelation,
    AirtimeLedger,
    SnapshotDeferred,
)
from cura_receiver.communicator_state_persistence import CommunicatorStatePolicy
from cura_receiver.generated.receiver_entities_generated import (
    TxAirtimeBucketV1 as Bucket,
)
from cura_receiver.generated.receiver_enums_generated import SystemTimeQuality as Q
from cura_receiver.time_observations import TrustedTimeSample

POLICY = CommunicatorStatePolicy()


def ledger(*pairs, utc=0, monotonic=0):
    buckets = tuple(Bucket(*pair) for pair in pairs)
    return AirtimeLedger(
        POLICY,
        buckets + (Bucket(0, 0),) * (64 - len(buckets)),
        utc_us=utc,
        monotonic_us=monotonic,
    )


# A partially overlapping oldest bucket retains its entire charge through the conservative boundary.
def test_exact_retention_and_partial_oldest():
    value = ledger((4_000_000, 10_000_000), (8_000_000, 70_000_000))
    assert value.total_used == 12_000_000
    value.advance(10_036_999)
    assert value.total_used == 12_000_000
    value.advance(10_037_000)
    assert value.total_used == 8_000_000
    assert value.entries() == (Bucket(8_000_000, 70_000_000),)
    value.advance(70_259_000)
    assert value.total_used == 0 and value.empty


# Restarts discard guarded UTC expirations at equality and reconstruct totals from retained entries only.
def test_load_pruning_grid_and_total_reconstruction():
    value = ledger(
        (4_000_000, 10_000_000), (8_000_000, 70_000_000), utc=10_000_000, monotonic=7
    )
    assert value.total_used == 8_000_000
    assert value.retention_deadline(70_000_000) == 60_222_007
    assert value.current_bucket_end(10_000_000) == 70_000_000
    assert value.current_bucket_end(70_000_000) == 130_000_000
    assert value.entries() == (Bucket(8_000_000, 70_000_000),)


# New grids begin one width after the snapshot only after every preceding entry expires.
def test_long_idle_bulk_reset_and_new_grid():
    value = ledger((1, 3_780_000_000), (2, 3_900_000_000))
    value.advance(10_000_000_000)
    assert value.entries() == () and value.total_used == 0
    assert value.current_bucket_end(10_000_000_123) == 10_060_000_123


# Sparse insertion, top-up and zero-charge settlement preserve their grid and cached total across wraparound.
def test_ring_wraparound_and_no_charge_relocation():
    value = ledger((2, 120_000_000), (3, 180_000_000))
    value.advance(120_444_000)
    value.set_charge(240_000_000, 4)
    value.set_charge(180_000_000, 5)
    assert value.entries() == (Bucket(5, 180_000_000), Bucket(4, 240_000_000))
    assert value.total_used == 9
    for minute in range(5, 80):
        value.advance((minute - 2) * 60_000_000 * 10037 // 10000)
        value.set_charge(minute * 60_000_000, 1)
    assert value.total_used == 2
    assert value.entries() == (Bucket(1, 78 * 60_000_000), Bucket(1, 79 * 60_000_000))
    value.set_charge(78 * 60_000_000, 0)
    assert value.total_used == 1


# Receive-to-ACK reads and head aging do not iterate the full charge array or construct a snapshot.
def test_admission_path_never_scans_ring():
    value = ledger((2, 10_000_000), (3, 70_000_000), (4, 130_000_000))

    class NoIteration(list):
        def __iter__(self):
            raise AssertionError("admission scanned the ring")

    value._charges = NoIteration(value._charges)
    value.advance(10_037_000)
    assert value.total_used == 7 and value.charge_at(70_000_000) == 3
    assert value.charge_at(190_000_000) == 0


# Snapshot deferral preserves charge until monotonic retention permits canonical removal.
def test_snapshot_waits_for_conservative_aging():
    value = ledger((1, 10_000_000))
    value.advance(10_000_000)
    with pytest.raises(SnapshotDeferred):
        value.snapshot_buckets(10_000_000)
    assert value.total_used == 1
    value.advance(10_037_000)
    assert value.snapshot_buckets(10_037_000) == (Bucket(0, 0),) * 64


# Corrupt ledgers are rejected before UTC pruning can hide excess charge or malformed entries.
@pytest.mark.parametrize(
    "pairs",
    [
        [(0, 1)],
        [(8_000_001, 1)],
        [(1, 2), (1, 1)],
        [(1, 1), (1, 2)],
        [(1, 1), (1, 1 + 64 * 60_000_000)],
        [(8_000_000, n * 60_000_000) for n in range(5)],
        [(1, -(1 << 63) + 1)],
    ],
)
def test_inconsistent_ledgers_rejected(pairs):
    with pytest.raises((ValueError, OverflowError)):
        ledger(*pairs, utc=10_000_000_000)


# Failed insertion is atomic and cannot corrupt cached totals or reuse a colliding physical slot.
def test_capacity_and_headroom_failures_leave_ledger_unchanged():
    value = ledger((8_000_000, 120_000_000), (8_000_000, 180_000_000))
    original = value.entries()
    for expiration, charge in [
        (120_000_000, 8_000_001),
        (121_000_000, 1),
        (120_000_000 + 64 * 60_000_000, 1),
    ]:
        with pytest.raises(ValueError):
            value.set_charge(expiration, charge)
        assert value.entries() == original and value.total_used == 16_000_000
    assert value.copy().entries() == original


# Trust generation, source validity and upward-rounded error growth all gate correlated UTC.
def test_correlation_exact_trust_boundaries():
    sample = TrustedTimeSample(100, 1000, 39_999_999, Q.RTC_HOLDOVER, 2)
    correlation = AirtimeCorrelation(sample, 2, 1000)
    assert correlation.utc_at(100, policy=POLICY, rate_bound_ppm=3700) == 1000
    with pytest.raises(ValueError):
        correlation.utc_at(101, policy=POLICY, rate_bound_ppm=3700)
    for changed in [
        replace(correlation, clock_state_generation=3),
        replace(correlation, valid_until_monotonic_us=100),
    ]:
        with pytest.raises(ValueError):
            changed.utc_at(100, policy=POLICY, rate_bound_ppm=3700)
    with pytest.raises(ValueError):
        correlation.utc_at(99, policy=POLICY, rate_bound_ppm=3700)
