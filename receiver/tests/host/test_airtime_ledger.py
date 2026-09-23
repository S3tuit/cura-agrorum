"""Independent boundary checks for positional history and immutable deadlines."""
from dataclasses import replace
import pytest
from cura_receiver.airtime_ledger import AirtimeLedger, AirtimeCorrelation
from cura_receiver.communicator_state_persistence import CommunicatorStatePolicy
from cura_receiver.generated.receiver_entities_generated import TxAirtimeBucketV1 as Bucket
from cura_receiver.generated.receiver_enums_generated import SystemTimeQuality as Q
from cura_receiver.time_observations import TrustedTimeSample

POLICY = CommunicatorStatePolicy()
SPACING = 60_222_000
HORIZON = 3_673_792_925


def ledger(*charges, monotonic=0, elapsed=0):
    return AirtimeLedger(POLICY, (Bucket(0),) * (62 - len(charges)) +
                        tuple(Bucket(c) for c in charges),
                        monotonic_us=monotonic, elapsed_us=elapsed)


def test_exact_retention_including_tx_tail_and_current_empty_slot():
    value = ledger(1, *([0] * 61))
    assert value.snapshot_buckets()[0] == Bucket(1)
    value.advance(250_924)
    assert value.total_used == 1
    value.advance(250_925)
    assert value.total_used == 0


def test_snapshot_and_copy_preserve_sparse_positions_and_deadlines():
    value = ledger(2, 0, 3, monotonic=100)
    oldest = 100 - 2 * SPACING
    deadlines = (value.retention_deadline(oldest), value.retention_deadline(100))
    value.advance(50_000_100)
    clone = value.copy()
    clone.set_charge(100, 4)
    assert clone.snapshot_buckets()[-3:] == (Bucket(2), Bucket(0), Bucket(4))
    assert value.snapshot_buckets()[-1] == Bucket(3)
    assert (clone.retention_deadline(oldest), clone.retention_deadline(100)) == deadlines
    assert clone.total_used == 6 and value.total_used == 5


def test_trusted_elapsed_shifts_positions_and_preserves_partial_phase():
    value = ledger(2, 0, 3, monotonic=100, elapsed=150_000_000)
    assert value.snapshot_buckets()[-5:] == (Bucket(2), Bucket(0), Bucket(3), Bucket(0), Bucket(0))
    assert value.current_start == 100 - 30_111_000
    assert value.grant_deadline == 29_667_100
    assert value.total_used == 5


def test_recovered_top_up_cannot_extend_retention_or_spending():
    value = ledger(2, monotonic=100, elapsed=30_000_000)
    identity = value.current_start
    deadline = value.retention_deadline(identity)
    value.set_charge(identity, 4)
    assert value.retention_deadline(identity) == deadline == HORIZON - 30_111_000 + 100
    assert value.grant_deadline == 29_667_100


def test_wrap_and_long_idle_without_scanning_for_admission():
    value = ledger()
    class NoIteration(list):
        def __iter__(self):
            raise AssertionError("admission scanned the ring")
    value._ring = NoIteration(value._ring)
    for interval in range(140):
        value.advance(interval * SPACING)
        value.set_charge(value.current_start, 1)
        assert value.charge_at(value.current_start) == 1
        assert value.total_used == min(interval + 1, 62)
        assert value.retention_deadline(value.current_start) == interval * SPACING + HORIZON
    value.advance(100_000_000_000)
    assert value.total_used == 0
    assert value.snapshot_buckets() == (Bucket(0),) * 62


@pytest.mark.parametrize("charges", [(8_000_001,), (8_000_000,) * 5, (True,), (-1,)])
def test_malformed_history_cannot_hide_behind_elapsed_pruning(charges):
    with pytest.raises((ValueError, OverflowError, TypeError)):
        ledger(*charges, elapsed=100_000_000_000)


def test_failed_updates_are_atomic():
    value = ledger(1)
    before = value.snapshot_buckets()
    for identity, charge in [(1, 2), (0, 8_000_001), (-SPACING, 1)]:
        with pytest.raises(ValueError):
            value.set_charge(identity, charge)
        assert value.snapshot_buckets() == before and value.total_used == 1
    high = ledger(monotonic=(1 << 64) - 1)
    with pytest.raises(OverflowError):
        high.set_charge(high.current_start, 1)
    assert high.total_used == 0


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
