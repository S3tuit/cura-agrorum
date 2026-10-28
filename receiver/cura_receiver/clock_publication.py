"""Publication over saved evidence; fresh live checks retain their own deadlines.

Only successful FIFO admission replaces the saved anchor. These decisions do
not initiate clock reads, extend trust, or imply a SQLite commit acknowledgement.
"""
from .elapsed_duration import (
    checked_duration_us,
    checked_monotonic_deadline,
    exclusive_trust_distance_us,
    maximum_lifetime_monotonic_us,
)
from .generated.receiver_entities_generated import ClockObservationV1
from .generated.receiver_enums_generated import RtcHealth, SystemTimeQuality
from .time_observations import TrustedTimeSample
from .time_policy import TimePolicy

PUBLICATION_ERROR_RESERVE_US = 1_000_000


def publication_deadline_us(
    sampled_at_monotonic_us: int, error_bound_us: int, policy: TimePolicy
) -> int:
    """First preferred refresh instant, including an already-consumed reserve.

    Half the unused budget is a preference; the reserve avoids planning an
    ordinary refresh at the hard trust boundary. Integer threshold inversion
    preserves the same upward rounding as event correlation.
    """
    start = checked_duration_us(sampled_at_monotonic_us)
    error = checked_duration_us(error_bound_us)
    budget = policy.receiver_utc_error_budget_us
    if error >= budget:
        raise ValueError("publication anchor has no trusted interval")
    rate = policy.monotonic_elapsed_rate_bound_ppm
    distances = [maximum_lifetime_monotonic_us(
        policy.clock_observation_period_cap_us, rate_bound_ppm=rate
    )]
    for threshold in (
        error + (budget - error + 1) // 2,
        max(0, budget - PUBLICATION_ERROR_RESERVE_US),
    ):
        distance = exclusive_trust_distance_us(error, threshold, rate_bound_ppm=rate)
        if distance is not None:
            distances.append(distance)
    return checked_monotonic_deadline(start, min(distances))


def clock_boundary_required(
    published: ClockObservationV1 | None,
    quality: SystemTimeQuality,
    rtc_health: RtcHealth,
    *,
    step: bool = False,
) -> bool:
    return published is None or step or (
        published.system_time_quality, published.rtc_health
    ) != (quality, rtc_health)


def should_publish_clock_sample(
    published: ClockObservationV1 | None,
    sample: TrustedTimeSample,
    *,
    rtc_health: RtcHealth,
    next_check_at_monotonic_us: int | None,
    policy: TimePolicy,
) -> bool:
    """Called once per newly accepted sample, never as a timer on cached evidence.

    Anticipate the next required check, not its unknown result. Failed or late
    checks still expire live trust independently. RTC reads keep their existing
    publication cadence, including their independent provenance-age accounting.
    """
    if clock_boundary_required(published, sample.quality, rtc_health):
        return True
    if sample.quality is SystemTimeQuality.RTC_HOLDOVER:
        return True
    if published.error_bound_us is None or next_check_at_monotonic_us is None:
        raise ValueError("network publication needs its anchor bound and next check")
    return checked_duration_us(next_check_at_monotonic_us) >= publication_deadline_us(
        published.sampled_at_monotonic_us, published.error_bound_us, policy
    )
