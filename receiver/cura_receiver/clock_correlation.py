"""Read-only UTC analysis of immutable receiver-instance and observation history."""

from __future__ import annotations

from bisect import bisect_right
from collections.abc import Iterable
from dataclasses import dataclass

from .elapsed_duration import (
    checked_correlated_utc,
    checked_duration_us,
    checked_utc_us,
)
from .generated.receiver_entities_generated import ClockObservationV1
from .generated.receiver_enums_generated import RtcHealth, SystemTimeQuality


def _identity(value: bytes) -> None:
    if type(value) is not bytes or len(value) != 16:
        raise ValueError("receiver and Linux boot identities must contain 16 bytes")


def _persisted_unsigned(value: int) -> None:
    checked_duration_us(value)
    if value > (1 << 63) - 1:
        raise OverflowError("persisted unsigned value exceeds SQLite's integer range")


@dataclass(frozen=True, slots=True)
class AnalysisInstance:
    """The immutable columns of a durable receiver_instances row needed by analysis."""

    instance_ordinal: int
    receiver_instance_id: bytes
    linux_boot_id: bytes
    started_at_monotonic_us: int

    def __post_init__(self) -> None:
        _persisted_unsigned(self.instance_ordinal)
        if self.instance_ordinal == 0:
            raise ValueError("instance ordinals start at one")
        _identity(self.receiver_instance_id)
        _identity(self.linux_boot_id)
        _persisted_unsigned(self.started_at_monotonic_us)


@dataclass(frozen=True, slots=True)
class CorrelatedUtc:
    utc_us: int
    clock_observation_receiver_instance_id: bytes
    clock_observation_sequence: int


class ClockCorrelation:
    """Index a supplied history snapshot; never compare clocks across instances or boots.

    Invalid history raises an input error. An event without an eligible, representable
    UTC has no result. Exact per-observation uncertainty is deliberately not persisted;
    analysis follows the recorded segment boundaries asserted by trusted publication.
    """

    def __init__(
        self,
        instances: Iterable[AnalysisInstance],
        observations: Iterable[ClockObservationV1],
    ) -> None:
        self._instances: dict[bytes, AnalysisInstance] = {}
        ordinals: set[int] = set()
        for instance in instances:
            if type(instance) is not AnalysisInstance:
                raise TypeError("analysis requires immutable instance rows")
            if (
                instance.receiver_instance_id in self._instances
                or instance.instance_ordinal in ordinals
            ):
                raise ValueError("duplicate receiver-instance identity or ordinal")
            self._instances[instance.receiver_instance_id] = instance
            ordinals.add(instance.instance_ordinal)
        groups: dict[bytes, list[ClockObservationV1]] = {
            key: [] for key in self._instances
        }
        for observation in observations:
            if type(observation) is not ClockObservationV1:
                raise TypeError("analysis requires immutable clock observations")
            _identity(observation.receiver_instance_id)
            instance = self._instances.get(observation.receiver_instance_id)
            if instance is None:
                raise ValueError("observation has no durable receiver instance")
            for value in (
                observation.observation_sequence,
                observation.clock_state_generation,
                observation.sampled_at_monotonic_us,
            ):
                _persisted_unsigned(value)
            if observation.sampled_at_monotonic_us < instance.started_at_monotonic_us:
                raise ValueError("observation precedes its receiver-instance start")
            if (
                type(observation.system_time_quality) is not SystemTimeQuality
                or type(observation.rtc_health) is not RtcHealth
            ):
                raise TypeError("observation requires canonical enums")
            if type(observation.step_discontinuity_boundary) is not bool:
                raise TypeError("step boundary must be Boolean")
            trusted = observation.system_time_quality is not SystemTimeQuality.UNTRUSTED
            if trusted != (observation.sampled_at_utc_us is not None) or (
                trusted and observation.step_discontinuity_boundary
            ):
                raise ValueError("observation quality, UTC and step boundary disagree")
            if (
                observation.system_time_quality is SystemTimeQuality.RTC_HOLDOVER
                and observation.rtc_health is not RtcHealth.PRESENT
            ):
                raise ValueError("holdover observation requires a present RTC")
            if trusted:
                checked_utc_us(observation.sampled_at_utc_us)
            groups[observation.receiver_instance_id].append(observation)
        self._observations: dict[bytes, tuple[ClockObservationV1, ...]] = {}
        self._times: dict[bytes, tuple[int, ...]] = {}
        self._step_gaps: dict[bytes, tuple[bool, ...]] = {}
        for identity, group in groups.items():
            ordered = tuple(
                sorted(
                    group,
                    key=lambda o: (o.sampled_at_monotonic_us, o.observation_sequence),
                )
            )
            gaps: list[bool] = []
            step_gap = False
            previous_sequence = previous_generation = -1
            for observation in ordered:
                if (
                    observation.observation_sequence <= previous_sequence
                    or observation.clock_state_generation < previous_generation
                ):
                    raise ValueError(
                        "observation sequence or generation moves backwards"
                    )
                previous_sequence = observation.observation_sequence
                previous_generation = observation.clock_state_generation
                if observation.step_discontinuity_boundary:
                    step_gap = True
                elif observation.sampled_at_utc_us is not None:
                    step_gap = False
                gaps.append(step_gap)
            self._observations[identity] = ordered
            self._times[identity] = tuple(o.sampled_at_monotonic_us for o in ordered)
            self._step_gaps[identity] = tuple(gaps)

    def instance(self, receiver_instance_id: bytes) -> AnalysisInstance | None:
        _identity(receiver_instance_id)
        return self._instances.get(receiver_instance_id)

    def correlate(
        self, receiver_instance_id: bytes, event_monotonic_us: int
    ) -> CorrelatedUtc | None:
        instance = self.instance(receiver_instance_id)
        _persisted_unsigned(event_monotonic_us)
        if instance is None or event_monotonic_us < instance.started_at_monotonic_us:
            return None
        ordered = self._observations[receiver_instance_id]
        previous = (
            bisect_right(self._times[receiver_instance_id], event_monotonic_us) - 1
        )
        selected = None
        if previous >= 0:
            if self._step_gaps[receiver_instance_id][previous]:
                return None
            if ordered[previous].sampled_at_utc_us is not None:
                selected = ordered[previous]
        if selected is None:
            for observation in ordered[previous + 1 :]:
                if observation.step_discontinuity_boundary:
                    return None
                if observation.sampled_at_utc_us is not None:
                    selected = observation
                    break
        if selected is None:
            return None
        try:
            utc = checked_correlated_utc(
                selected.sampled_at_utc_us,
                selected.sampled_at_monotonic_us,
                event_monotonic_us,
            )
        except OverflowError:
            return None
        return CorrelatedUtc(utc, receiver_instance_id, selected.observation_sequence)
