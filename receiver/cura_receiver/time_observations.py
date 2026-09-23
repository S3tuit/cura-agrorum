"""Pure candidate observations and provenance; publication and I/O belong to callers."""

from __future__ import annotations

from dataclasses import dataclass

from .elapsed_duration import (
    checked_absolute_us,
    checked_correlated_utc,
    checked_duration_us,
    checked_monotonic_deadline,
    checked_monotonic_elapsed,
    checked_monotonic_midpoint,
    checked_utc_difference,
    checked_utc_us,
    checked_whole_second_midpoint,
    exclusive_trust_distance_us,
    maximum_physical_half_bracket_us,
    rate_growth_us,
)
from .generated.receiver_entities_generated import RtcProvenanceV1
from .generated.receiver_enums_generated import RtcHealth, SystemTimeQuality
from .time_policy import (
    ClockState,
    NetworkEstimate,
    TimePolicy,
    network_tracking_decision,
)


@dataclass(frozen=True, slots=True)
class TrustedTimeSample:
    """Bounded candidate data, before a caller's atomic transition/publication."""

    monotonic_us: int
    utc_us: int
    error_bound_us: int
    quality: SystemTimeQuality
    generation: int

    def __post_init__(self) -> None:
        checked_duration_us(self.monotonic_us)
        checked_utc_us(self.utc_us)
        checked_duration_us(self.error_bound_us)
        checked_duration_us(self.generation)
        if self.generation > (1 << 63) - 1:
            raise OverflowError(
                "sample generation exceeds the persisted observation range"
            )
        if self.quality not in (
            SystemTimeQuality.NETWORK_SYNCED,
            SystemTimeQuality.RTC_HOLDOVER,
        ):
            raise ValueError("a trusted sample must have a trusted quality")


def _bracket(start_us: int, finish_us: int, policy: TimePolicy) -> tuple[int, int]:
    width = checked_monotonic_elapsed(start_us, finish_us)
    if width > policy.time_sampling_margin_us:
        raise ValueError("observation bracket exceeds the fixed sampling margin")
    return checked_monotonic_midpoint(start_us, finish_us), width


def rtc_read_uncertainty_us(start_us: int, finish_us: int, policy: TimePolicy) -> int:
    _, width = _bracket(start_us, finish_us, policy)
    half = maximum_physical_half_bracket_us(
        width, rate_bound_ppm=policy.monotonic_elapsed_rate_bound_ppm
    )
    return checked_monotonic_deadline(
        checked_monotonic_deadline(500_000, half), policy.time_sampling_margin_us
    )


def network_observation(
    before: ClockState,
    after: ClockState,
    estimate: NetworkEstimate | None,
    *,
    operation_started_at_monotonic_us: int,
    operation_finished_at_monotonic_us: int,
    sampled_utc_us: int,
    kernel_sample_usable: bool,
    required_poll_deadline_us: int,
    policy: TimePolicy,
    report_calculation_failure: bool = False,
) -> TrustedTimeSample | None:
    """Consume normalized kernel acceptance; this function never interprets raw timex bits."""
    try:
        if before.generation != after.generation or kernel_sample_usable is not True:
            return None
        midpoint, _ = _bracket(
            operation_started_at_monotonic_us,
            operation_finished_at_monotonic_us,
            policy,
        )
        decision = network_tracking_decision(
            before,
            estimate,
            now_monotonic_us=operation_finished_at_monotonic_us,
            required_poll_deadline_us=required_poll_deadline_us,
            policy=policy,
        )
        if (
            decision.candidate_quality is not SystemTimeQuality.NETWORK_SYNCED
            or decision.error_bound_us is None
        ):
            return None
        if decision.error_bound_us >= policy.receiver_utc_error_budget_us:
            return None
        checked_monotonic_elapsed(
            estimate.sample_finished_at_monotonic_us, operation_started_at_monotonic_us
        )
        sample = TrustedTimeSample(
            midpoint,
            sampled_utc_us,
            estimate.error_at(midpoint, policy),
            SystemTimeQuality.NETWORK_SYNCED,
            before.generation,
        )
        schedule = observation_schedule(
            sample,
            policy,
            tracking_started_at_monotonic_us=estimate.sample_started_at_monotonic_us,
        )
        if (
            schedule.trust_expires_at_monotonic_us is not None
            and operation_finished_at_monotonic_us
            >= schedule.trust_expires_at_monotonic_us
        ):
            return None
        return sample
    except OverflowError:
        if report_calculation_failure:
            raise
        return None
    except (TypeError, ValueError):
        return None


def rtc_observation(
    before: ClockState,
    after: ClockState,
    provenance: RtcProvenanceV1 | None,
    *,
    rtc_health: RtcHealth,
    rtc_utc_seconds: int,
    operation_started_at_monotonic_us: int,
    operation_finished_at_monotonic_us: int,
    policy: TimePolicy,
    report_calculation_failure: bool = False,
) -> TrustedTimeSample | None:
    """Provenance must already be durably loaded/acknowledged and its instance resolved."""
    try:
        if before.generation != after.generation or rtc_health is not RtcHealth.PRESENT:
            return None
        if type(provenance) is not RtcProvenanceV1:
            return None
        if (
            type(provenance.verified_by_receiver_instance_id) is not bytes
            or len(provenance.verified_by_receiver_instance_id) != 16
        ):
            return None
        if (
            type(provenance.drift_bound_ppm) is not int
            or not 0 < provenance.drift_bound_ppm < 1_000_000
        ):
            return None
        checked_utc_us(provenance.network_utc_at_verification_us)
        if (
            checked_absolute_us(
                checked_utc_difference(
                    provenance.network_utc_at_verification_us,
                    provenance.rtc_readback_utc_us,
                )
            )
            > policy.time_sampling_margin_us
        ):
            return None
        midpoint, _ = _bracket(
            operation_started_at_monotonic_us,
            operation_finished_at_monotonic_us,
            policy,
        )
        utc = checked_whole_second_midpoint(rtc_utc_seconds)
        age = checked_duration_us(
            checked_utc_difference(utc, provenance.rtc_readback_utc_us)
        )
        growth = rate_growth_us(provenance.drift_bound_ppm, age)
        error = checked_monotonic_deadline(
            provenance.verification_uncertainty_us, growth
        )
        error = checked_monotonic_deadline(
            error,
            rtc_read_uncertainty_us(
                operation_started_at_monotonic_us,
                operation_finished_at_monotonic_us,
                policy,
            ),
        )
        if error >= policy.receiver_utc_error_budget_us:
            return None
        sample = TrustedTimeSample(
            midpoint, utc, error, SystemTimeQuality.RTC_HOLDOVER, before.generation
        )
        schedule = observation_schedule(sample, policy)
        if (
            schedule.trust_expires_at_monotonic_us is not None
            and operation_finished_at_monotonic_us
            >= schedule.trust_expires_at_monotonic_us
        ):
            return None
        return sample
    except OverflowError:
        if report_calculation_failure:
            raise
        return None
    except (TypeError, ValueError):
        return None


def advanced_error_us(
    sample: TrustedTimeSample, event_monotonic_us: int, policy: TimePolicy
) -> int:
    event = checked_duration_us(event_monotonic_us)
    distance = abs(event - sample.monotonic_us)
    return checked_monotonic_deadline(
        sample.error_bound_us,
        rate_growth_us(policy.monotonic_elapsed_rate_bound_ppm, distance),
    )


@dataclass(frozen=True, slots=True)
class ObservationSchedule:
    trust_expires_at_monotonic_us: int | None
    observation_due_at_monotonic_us: int
    tracking_poll_due_at_monotonic_us: int | None


def _trust_expiry_us(sample: TrustedTimeSample, policy: TimePolicy) -> int | None:
    distance = exclusive_trust_distance_us(
        sample.error_bound_us,
        policy.receiver_utc_error_budget_us,
        rate_bound_ppm=policy.monotonic_elapsed_rate_bound_ppm,
    )
    if distance == 0:
        raise ValueError("observation has no trusted interval")
    return (
        None
        if distance is None
        else checked_monotonic_deadline(sample.monotonic_us, distance)
    )


def observation_schedule(
    sample: TrustedTimeSample,
    policy: TimePolicy,
    *,
    tracking_started_at_monotonic_us: int | None = None,
) -> ObservationSchedule:
    """Upper due-time bounds; network polls are measured from their actual query start."""
    expiry = _trust_expiry_us(sample, policy)
    cap = (
        policy.clock_observation_period_cap_us
        if sample.quality is SystemTimeQuality.NETWORK_SYNCED
        else policy.rtc_holdover_observation_period_cap_us
    )
    observation = checked_monotonic_deadline(sample.monotonic_us, cap)
    if expiry is not None:
        observation = min(observation, expiry)
    poll = None
    if sample.quality is SystemTimeQuality.NETWORK_SYNCED:
        if tracking_started_at_monotonic_us is None:
            raise ValueError("network scheduling requires the supporting query start")
        checked_monotonic_elapsed(tracking_started_at_monotonic_us, sample.monotonic_us)
        poll = checked_monotonic_deadline(
            tracking_started_at_monotonic_us, policy.chrony_tracking_poll_period_cap_us
        )
        if expiry is not None:
            poll = min(poll, expiry)
    return ObservationSchedule(expiry, observation, poll)


def rtc_refresh_source_error_us(
    sample: TrustedTimeSample,
    current: ClockState,
    *,
    now_monotonic_us: int,
    required_poll_deadline_us: int,
    policy: TimePolicy,
    report_calculation_failure: bool = False,
) -> int | None:
    try:
        if (
            sample.generation != current.generation
            or current.quality is not SystemTimeQuality.NETWORK_SYNCED
            or sample.quality is not SystemTimeQuality.NETWORK_SYNCED
        ):
            return None
        checked_monotonic_elapsed(sample.monotonic_us, now_monotonic_us)
        checked_duration_us(required_poll_deadline_us)
        if now_monotonic_us >= required_poll_deadline_us:
            return None
        error = advanced_error_us(sample, now_monotonic_us, policy)
        if (
            error <= policy.network_rtc_write_error_threshold_us
            and error < policy.receiver_utc_error_budget_us
        ):
            return error
        return None
    except OverflowError:
        if report_calculation_failure:
            raise
        return None
    except (TypeError, ValueError):
        return None


def rtc_refresh_due_us(
    sample: TrustedTimeSample,
    *,
    last_refresh_monotonic_us: int | None,
    policy: TimePolicy,
) -> int | None:
    """Initial stable qualifying samples are due now; subsequent caps use the last refresh."""
    if (
        sample.quality is not SystemTimeQuality.NETWORK_SYNCED
        or sample.error_bound_us > policy.network_rtc_write_error_threshold_us
        or sample.error_bound_us >= policy.receiver_utc_error_budget_us
    ):
        return None
    expiry = _trust_expiry_us(sample, policy)
    if last_refresh_monotonic_us is None:
        return sample.monotonic_us
    # The supporting sample can still be current after a refresh completes.
    checked_duration_us(last_refresh_monotonic_us)
    due = checked_monotonic_deadline(
        last_refresh_monotonic_us, policy.network_rtc_refresh_period_us
    )
    if expiry is not None:
        due = min(due, expiry)
    return max(sample.monotonic_us, due)


def rtc_provenance_candidate(
    sample: TrustedTimeSample,
    current: ClockState,
    *,
    verified_by_receiver_instance_id: bytes,
    episode_started_at_monotonic_us: int,
    read_started_at_monotonic_us: int,
    read_finished_at_monotonic_us: int,
    commit_at_monotonic_us: int,
    rtc_utc_seconds: int,
    required_poll_deadline_us: int,
    policy: TimePolicy,
    report_calculation_failure: bool = False,
) -> RtcProvenanceV1 | None:
    """Compute a proposal only; it is not durable provenance until commit acknowledgement."""
    try:
        if (
            type(verified_by_receiver_instance_id) is not bytes
            or len(verified_by_receiver_instance_id) != 16
        ):
            return None
        checked_monotonic_elapsed(
            episode_started_at_monotonic_us, read_started_at_monotonic_us
        )
        checked_monotonic_elapsed(read_finished_at_monotonic_us, commit_at_monotonic_us)
        for instant in (episode_started_at_monotonic_us, commit_at_monotonic_us):
            if (
                rtc_refresh_source_error_us(
                    sample,
                    current,
                    now_monotonic_us=instant,
                    required_poll_deadline_us=required_poll_deadline_us,
                    policy=policy,
                )
                is None
            ):
                return None
        midpoint, _ = _bracket(
            read_started_at_monotonic_us, read_finished_at_monotonic_us, policy
        )
        network_utc = checked_correlated_utc(
            sample.utc_us, sample.monotonic_us, midpoint
        )
        rtc_utc = checked_whole_second_midpoint(rtc_utc_seconds)
        difference = checked_absolute_us(checked_utc_difference(network_utc, rtc_utc))
        if difference > policy.time_sampling_margin_us:
            return None
        uncertainty = checked_monotonic_deadline(
            advanced_error_us(sample, midpoint, policy), difference
        )
        uncertainty = checked_monotonic_deadline(
            uncertainty,
            rtc_read_uncertainty_us(
                read_started_at_monotonic_us, read_finished_at_monotonic_us, policy
            ),
        )
        if uncertainty >= policy.receiver_utc_error_budget_us:
            return None
        return RtcProvenanceV1(
            verified_by_receiver_instance_id,
            network_utc,
            rtc_utc,
            uncertainty,
            policy.rtc_drift_bound_ppm,
        )
    except OverflowError:
        if report_calculation_failure:
            raise
        return None
    except (TypeError, ValueError):
        return None
