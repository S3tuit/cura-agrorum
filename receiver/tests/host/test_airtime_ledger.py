"""Fixed reservations: boundaries, rounding and ordinary group invariants."""
from dataclasses import replace
from fractions import Fraction
import pytest
from cura_receiver.airtime_ledger import AirtimeLedger, AirtimeCorrelation
from cura_receiver.communicator_state_persistence import CommunicatorStatePolicy
from cura_receiver.elapsed_duration import maximum_physical_duration_us
from cura_receiver.generated.receiver_entities_generated import AirtimeEntryV2 as Entry
from cura_receiver.generated.receiver_enums_generated import SystemTimeQuality as Q
from cura_receiver.time_observations import TrustedTimeSample

POLICY = CommunicatorStatePolicy()
HOLD = 3_613_570_925


def ledger(*durations, now=0, elapsed=0):
    return AirtimeLedger(POLICY, tuple(Entry(d) for d in durations) +
        (Entry(0),) * (18-len(durations)), monotonic_us=now, elapsed_us=elapsed)


def test_reserve_extend_expire_and_retain_unsaved_usage():
    a = ledger()
    assert a.reserve(500_000, 100)
    assert a.deadlines[0] == 100 + HOLD and a.total_used == 2_000_000
    assert a.reserve(500_000, 200)
    assert a.deadlines[0] == 200 + HOLD
    a.advance(199 + HOLD)
    assert a.current_entry == 0
    a.advance(200 + HOLD)
    assert a.total_used == 0 and a.current_entry is None
    assert a.used_since_save_us == 1_000_000
    assert a.reserve(1_000_000, 201 + HOLD)
    assert not a.reserve(1, 202 + HOLD)


@pytest.mark.parametrize('occupied', range(19))
def test_historical_entries_are_not_spendable(occupied):
    a = ledger(*([10_000_000]*occupied))
    assert a.total_used == occupied*2_000_000
    assert a.reserve(100, 0) is (occupied < 18)
    if occupied == 17:
        assert a.reserve(100, 1)  # full array, existing current group


def test_partial_coverage_closes_preserving_deadline_and_remainder():
    a = ledger()
    a.reserve(1_200_000, 0)
    a.confirm_coverage(800_000)
    assert a.current_entry is None and a.used_since_save_us == 400_000
    assert a.deadlines[0] == HOLD
    assert a.reserve(100_000, 1)
    assert a.current_entry == 1 and a.used_since_save_us == 500_000
    a.confirm_coverage(500_000)
    assert a.total_used == 4_000_000


def test_refund_does_not_free_or_reopen_a_reservation():
    a = ledger()
    a.reserve(100, 0)
    a.refund(100)
    assert a.total_used == 2_000_000 and a.used_since_save_us == 0
    a.reserve(100, 1)
    a.confirm_coverage(100)
    with pytest.raises(ValueError):
        a.refund(100)
    assert a.current_entry is None


@pytest.mark.parametrize('remaining', [0, 1, 999_999, HOLD, HOLD*2])
def test_saved_lifetime_covers_slowest_physical_clock(remaining):
    value = maximum_physical_duration_us(remaining)
    assert value >= Fraction(remaining*1_000_000, 996_300)
    if value:
        assert value-1 < Fraction(remaining*1_000_000, 996_300)


def test_round_trips_inflate_retention_instead_of_capping_it():
    a = ledger()
    a.add_recovery()
    saved = a.snapshot_entries(0)
    assert saved[0].remaining_us > HOLD
    b = AirtimeLedger(POLICY, saved, monotonic_us=0)
    assert b.deadlines[0] > a.deadlines[0]
    assert a.deadlines[0] == HOLD  # saving never rebases the live ledger


@pytest.mark.parametrize('value', [True, -1, 1 << 64])
def test_bad_history_is_rejected_before_elapsed_pruning(value):
    with pytest.raises((TypeError, ValueError, OverflowError)):
        ledger(value, elapsed=(1 << 64)-1)


def test_checked_deadline_and_product_overflow():
    a = ledger(now=(1 << 64)-1)
    with pytest.raises(OverflowError):
        a.reserve(1, (1 << 64)-1)
    assert a.total_used == 0 and a.current_entry is None
    with pytest.raises(OverflowError):
        maximum_physical_duration_us((1 << 64)-1)


def test_recovery_never_shortens_and_ties_are_safe():
    a = ledger(*([HOLD*2]*18))
    before = a.deadlines.copy()
    a.add_recovery()
    assert a.deadlines == before and a.current_entry is None
    b = ledger(*([1]*18))
    b.add_recovery()
    assert b.deadlines[0] == HOLD and all(d == 2 for d in b.deadlines[1:])


def test_correlation_exact_trust_boundaries():
    sample = TrustedTimeSample(100, 1000, 39_999_999, Q.RTC_HOLDOVER, 2)
    correlation = AirtimeCorrelation(sample, 2, 1000)
    assert correlation.utc_at(100, policy=POLICY, rate_bound_ppm=3700) == 1000
    with pytest.raises(ValueError):
        correlation.utc_at(101, policy=POLICY, rate_bound_ppm=3700)
    for changed in [replace(correlation, clock_state_generation=3),
                    replace(correlation, valid_until_monotonic_us=100)]:
        with pytest.raises(ValueError):
            changed.utc_at(100, policy=POLICY, rate_bound_ppm=3700)
    with pytest.raises(ValueError):
        correlation.utc_at(99, policy=POLICY, rate_bound_ppm=3700)
