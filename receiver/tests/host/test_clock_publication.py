from dataclasses import replace
import random

import pytest

from cura_receiver.clock_publication import (
    clock_boundary_required, publication_deadline_us, should_publish_clock_sample,
)
from cura_receiver.generated.receiver_entities_generated import ClockObservationV1
from cura_receiver.generated.receiver_enums_generated import RtcHealth as H, SystemTimeQuality as Q
from cura_receiver.time_observations import TrustedTimeSample
from cura_receiver.time_policy import TimePolicy

POLICY = TimePolicy()


def anchor(error=2_000_000):
    return ClockObservationV1(b"i" * 16, 1, 1, 0, 0, False, Q.CHRONY_SYNCED,
                             H.PRESENT, error, 0, 0, None, None, None)


@pytest.mark.parametrize("error", [38_000_000, 38_999_999, 39_000_000, 39_000_001, 39_950_000])
def test_guard_exact_boundary(error):
    due = publication_deadline_us(0, error, POLICY)
    if error >= 39_000_000:
        assert due == 0
    else:
        growth = lambda dt: (3700 * dt + 996299) // 996300
        assert error + growth(due - 1) < 39_000_000
        assert error + growth(due) >= 39_000_000


def test_half_margin_and_cap_independent_oracle():
    rng = random.Random(3900)
    for error in [0, 4_000_000, 30_000_000, 39_999_999] + [rng.randrange(40_000_000) for _ in range(1000)]:
        thresholds = [error + (40_000_000 - error + 1) // 2, 39_000_000]
        distances = [3600_000_000 * 996300 // 1_000_000]
        distances += [0 if error >= t else ((t - error - 1) * 996300) // 3700 + 1 for t in thresholds]
        assert publication_deadline_us(123, error, POLICY) == 123 + min(distances)


def test_unpublished_samples_cannot_reset_anchor_deadline():
    saved = anchor()
    for at in range(60_000_000, 3500_000_000, 60_000_000):
        live = TrustedTimeSample(at, at, 1_000_000, Q.CHRONY_SYNCED, 20)
        assert not should_publish_clock_sample(saved, live, rtc_health=H.PRESENT,
            next_check_at_monotonic_us=at + 60_000_000, policy=POLICY)
    assert should_publish_clock_sample(saved, live, rtc_health=H.PRESENT,
        next_check_at_monotonic_us=3600_000_000, policy=POLICY)


def test_boundaries_ignore_generation_but_preserve_initial_health_and_step():
    saved = anchor()
    assert clock_boundary_required(None, Q.UNTRUSTED, H.MISSING)
    assert not clock_boundary_required(saved, Q.CHRONY_SYNCED, H.PRESENT)
    assert clock_boundary_required(saved, Q.CHRONY_SYNCED, H.MISSING)
    assert clock_boundary_required(saved, Q.UNTRUSTED, H.PRESENT)
    assert clock_boundary_required(replace(saved, system_time_quality=Q.UNTRUSTED),
                                   Q.UNTRUSTED, H.PRESENT, step=True)


def test_rtc_reads_retain_publication_cadence():
    saved = replace(anchor(), system_time_quality=Q.RTC_HOLDOVER)
    sample = TrustedTimeSample(1, 1, 2_000_000, Q.RTC_HOLDOVER, 2)
    assert should_publish_clock_sample(saved, sample, rtc_health=H.PRESENT,
                                      next_check_at_monotonic_us=None, policy=POLICY)


def test_expired_or_unrepresentable_anchor_rejected():
    with pytest.raises(ValueError):
        publication_deadline_us(0, 40_000_000, POLICY)
    with pytest.raises(OverflowError):
        publication_deadline_us((1 << 64) - 1, 1_000_000, POLICY)
