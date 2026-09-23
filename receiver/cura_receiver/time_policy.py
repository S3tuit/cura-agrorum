"""Pure time decisions over supplied evidence; no service, device or queue operations."""

from __future__ import annotations

from dataclasses import dataclass, field

from .elapsed_duration import (
    checked_absolute_us,
    checked_duration_us,
    checked_monotonic_deadline,
    checked_monotonic_elapsed,
    rate_growth_us,
)
from .generated.receiver_enums_generated import RtcHealth, SystemTimeQuality

TIME_SAMPLING_MARGIN_US = 1_000_000


@dataclass(frozen=True, slots=True)
class TimePolicy:
    """Deployment error budgets and shared elapsed-clock assumptions."""

    chrony_max_slew_rate_ppm: int = 3500
    monotonic_elapsed_rate_bound_ppm: int = 3700
    rtc_drift_bound_ppm: int = 10
    network_trust_error_threshold_us: int = 35_000_000
    network_step_error_threshold_us: int = 40_000_000
    receiver_utc_error_budget_us: int = 40_000_000
    network_rtc_write_error_threshold_us: int = 5_000_000
    chrony_tracking_poll_period_cap_us: int = 60_000_000
    clock_observation_period_cap_us: int = 10_800_000_000
    network_rtc_refresh_period_us: int = 10_800_000_000
    rtc_holdover_observation_period_cap_us: int = 3_600_000_000
    time_sampling_margin_us: int = field(default=TIME_SAMPLING_MARGIN_US, init=False)

    def __post_init__(self) -> None:
        for name in self.__dataclass_fields__:
            checked_duration_us(getattr(self, name))
        if not (
            self.chrony_max_slew_rate_ppm
            <= self.monotonic_elapsed_rate_bound_ppm
            < 1_000_000
            and 0 < self.rtc_drift_bound_ppm < 1_000_000
            and TIME_SAMPLING_MARGIN_US
            < self.network_rtc_write_error_threshold_us
            <= self.network_trust_error_threshold_us
            < self.network_step_error_threshold_us
            and 0
            < self.receiver_utc_error_budget_us
            <= self.network_step_error_threshold_us
            and self.receiver_utc_error_budget_us <= 40_000_000
        ):
            raise ValueError("inconsistent time-policy bounds")
        for period in (
            self.chrony_tracking_poll_period_cap_us,
            self.clock_observation_period_cap_us,
            self.network_rtc_refresh_period_us,
            self.rtc_holdover_observation_period_cap_us,
        ):
            if period == 0:
                raise ValueError("time-policy periods must be positive")


@dataclass(frozen=True, slots=True)
class ClockState:
    quality: SystemTimeQuality
    rtc_health: RtcHealth
    generation: int

    def __post_init__(self) -> None:
        if (
            type(self.quality) is not SystemTimeQuality
            or type(self.rtc_health) is not RtcHealth
        ):
            raise TypeError("clock state requires quality and RTC-health enums")
        checked_duration_us(self.generation)
        if self.generation > (1 << 63) - 1:
            raise OverflowError(
                "clock generation exceeds the persisted observation range"
            )
        if (
            self.quality is SystemTimeQuality.RTC_HOLDOVER
            and self.rtc_health is not RtcHealth.PRESENT
        ):
            raise ValueError("holdover requires a present RTC")


def startup_clock_state(probed_rtc_health: RtcHealth) -> ClockState:
    """A current probe supplies health; no persisted quality snapshot is an input."""
    return ClockState(SystemTimeQuality.UNTRUSTED, probed_rtc_health, 0)


def advance_clock_state(
    current: ClockState,
    *,
    quality: SystemTimeQuality,
    rtc_health: RtcHealth,
    tracking_processed: bool = False,
    step_boundary: bool = False,
) -> ClockState:
    """One atomic policy update advances at most once, including equal-value tracking."""
    if type(tracking_processed) is not bool or type(step_boundary) is not bool:
        raise TypeError("clock update flags must be Boolean")
    if step_boundary and quality is not SystemTimeQuality.UNTRUSTED:
        raise ValueError("an intentional step first requires untrusted quality")
    changed = (
        tracking_processed
        or step_boundary
        or (quality, rtc_health) != (current.quality, current.rtc_health)
    )
    generation = checked_monotonic_deadline(current.generation, int(changed))
    return ClockState(quality, rtc_health, generation)


@dataclass(frozen=True, slots=True)
class NetworkEvidence:
    """Normalized successful tracking facts; unavailable/invalid responses use None.

    Constructing this value neither parses chronyc output nor proves it valid.
    Evaluation checks every field before it can support a policy decision.
    """

    sample_started_at_monotonic_us: int
    sample_finished_at_monotonic_us: int
    source_selected: bool
    synchronized: bool
    remaining_correction_us: int
    root_distance_us: int
    estimated_skew_ppb: int


def network_root_distance_us(root_delay_us: int, root_dispersion_us: int) -> int:
    """Conservatively combine integral microsecond inputs; adapter parsing is separate."""
    delay = checked_duration_us(root_delay_us)
    half_delay = delay // 2 + delay % 2
    return checked_monotonic_deadline(half_delay, root_dispersion_us)


def network_error_bound_us(remaining_correction_us: int, root_distance_us: int) -> int:
    return checked_monotonic_deadline(
        checked_monotonic_deadline(
            checked_absolute_us(remaining_correction_us), root_distance_us
        ),
        TIME_SAMPLING_MARGIN_US,
    )


@dataclass(frozen=True, slots=True)
class NetworkEstimate:
    """Validated source facts with one initial bound anchored at query start."""

    sample_started_at_monotonic_us: int
    sample_finished_at_monotonic_us: int
    initial_error_bound_us: int

    def __post_init__(self) -> None:
        checked_monotonic_elapsed(
            self.sample_started_at_monotonic_us, self.sample_finished_at_monotonic_us
        )
        checked_duration_us(self.initial_error_bound_us)

    def error_at(self, now_monotonic_us: int, policy: TimePolicy) -> int:
        age = checked_monotonic_elapsed(
            self.sample_started_at_monotonic_us, now_monotonic_us
        )
        return checked_monotonic_deadline(
            self.initial_error_bound_us,
            rate_growth_us(policy.monotonic_elapsed_rate_bound_ppm, age),
        )


def network_estimate(
    evidence: NetworkEvidence | None, *, report_calculation_failure: bool = False
) -> NetworkEstimate | None:
    """Validate and calculate once; a valid skew is diagnostic, not a cutoff."""
    try:
        if type(evidence) is not NetworkEvidence:
            return None
        if evidence.source_selected is not True or evidence.synchronized is not True:
            return None
        checked_monotonic_elapsed(
            evidence.sample_started_at_monotonic_us,
            evidence.sample_finished_at_monotonic_us,
        )
        checked_duration_us(evidence.estimated_skew_ppb)
        error = network_error_bound_us(
            evidence.remaining_correction_us, evidence.root_distance_us
        )
        return NetworkEstimate(
            evidence.sample_started_at_monotonic_us,
            evidence.sample_finished_at_monotonic_us,
            error,
        )
    except OverflowError:
        if report_calculation_failure:
            raise
        return None
    except (TypeError, ValueError):
        return None


def usable_network_error_us(
    estimate: NetworkEstimate | None,
    *,
    now_monotonic_us: int,
    required_poll_deadline_us: int,
    policy: TimePolicy,
) -> int | None:
    """Advance qualified evidence to use time, retaining both freshness limits."""
    try:
        checked_duration_us(now_monotonic_us)
        checked_duration_us(required_poll_deadline_us)
        if (
            type(estimate) is not NetworkEstimate
            or now_monotonic_us >= required_poll_deadline_us
        ):
            return None
        checked_monotonic_elapsed(
            estimate.sample_finished_at_monotonic_us, now_monotonic_us
        )
        if checked_monotonic_elapsed(
            estimate.sample_started_at_monotonic_us, now_monotonic_us
        ) > TIME_SAMPLING_MARGIN_US:
            return None
        return estimate.error_at(now_monotonic_us, policy)
    except (TypeError, ValueError, OverflowError):
        return None


@dataclass(frozen=True, slots=True)
class NetworkDecision:
    """Candidate tracking decision; promotion still requires a usable clock bracket."""

    candidate_quality: SystemTimeQuality
    error_bound_us: int | None
    step_required: bool


def network_tracking_decision(
    current: ClockState,
    estimate: NetworkEstimate | None,
    *,
    now_monotonic_us: int,
    required_poll_deadline_us: int,
    policy: TimePolicy,
) -> NetworkDecision:
    error = usable_network_error_us(
        estimate,
        now_monotonic_us=now_monotonic_us,
        required_poll_deadline_us=required_poll_deadline_us,
        policy=policy,
    )
    if error is None:
        quality = (
            SystemTimeQuality.UNTRUSTED
            if current.quality is SystemTimeQuality.NETWORK_SYNCED
            else current.quality
        )
        return NetworkDecision(quality, None, False)
    if error > policy.network_step_error_threshold_us:
        return NetworkDecision(SystemTimeQuality.UNTRUSTED, error, True)
    quality = (
        SystemTimeQuality.NETWORK_SYNCED
        if error <= policy.network_trust_error_threshold_us
        else current.quality
    )
    return NetworkDecision(quality, error, False)


def expire_clock_trust(current: ClockState) -> ClockState:
    """Expiry is an ordinary transition; it supplies no authorization to step."""
    return advance_clock_state(
        current, quality=SystemTimeQuality.UNTRUSTED, rtc_health=current.rtc_health
    )
