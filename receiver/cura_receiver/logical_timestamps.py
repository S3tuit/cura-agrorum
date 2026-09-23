"""Logical reading timestamps from immutable history; no output-storage operations."""

from __future__ import annotations

from bisect import bisect_left
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from enum import Enum

from .clock_correlation import ClockCorrelation
from .elapsed_duration import (
    checked_duration_product,
    checked_duration_us,
    checked_monotonic_deadline,
    checked_utc_difference,
    checked_utc_offset,
    checked_utc_us,
)
from .generated.protocol_v2_lora_generated import Domain, ReadingFlag
from .generated.receiver_entities_generated import (
    MessageProfileRowV1,
    ReadingMessageRowV1,
)
from .generated.receiver_enums_generated import (
    PersistenceClassification,
    ProcessingResult,
)

READING_AIRTIME_US = 102_656
RADIO_CYCLE_US = 30_000_000
DEEP_SLEEP_US = 900_000_000
SampleKey = tuple[bytes, int]
_ANCHOR_CLASSIFICATIONS = frozenset(
    (
        PersistenceClassification.FIRST_SEEN,
        PersistenceClassification.RETRANSMISSION,
        PersistenceClassification.DUPLICATE_SAME_CONTENT,
    )
)


class TimestampSource(Enum):
    DIRECT = "DIRECT"
    EXTRAPOLATED = "EXTRAPOLATED"


@dataclass(frozen=True, slots=True)
class ReadingTimestamp:
    timestamp_utc_us: int
    timestamp_source: TimestampSource
    anchor_sample_id: int
    clock_observation_receiver_instance_id: bytes
    clock_observation_sequence: int

    def __post_init__(self) -> None:
        checked_utc_us(self.timestamp_utc_us)
        if type(self.timestamp_source) is not TimestampSource:
            raise TypeError("logical timestamp requires an analysis source")
        _bounded(self.anchor_sample_id, 32)
        _identity(self.clock_observation_receiver_instance_id, 16)
        _bounded(self.clock_observation_sequence, 63)


def _bounded(value: int, bits: int) -> None:
    checked_duration_us(value)
    if value >= 1 << bits:
        raise OverflowError("reading-history integer is out of range")


def _identity(value: bytes, size: int) -> None:
    if type(value) is not bytes or len(value) != size:
        raise ValueError("reading-history identity has an invalid size")


def direct_anchor_midpoint_us(
    rx_done_utc_us: int, finalized_plus_airtime_us: int
) -> int | None:
    """Estimate the bounded application start; input duration is run_ms*1000 + Tair."""
    checked_duration_us(finalized_plus_airtime_us)
    if finalized_plus_airtime_us > RADIO_CYCLE_US:
        return None
    minimum = checked_utc_offset(rx_done_utc_us, -RADIO_CYCLE_US)
    maximum = checked_utc_offset(rx_done_utc_us, -finalized_plus_airtime_us)
    return checked_utc_offset(minimum, checked_utc_difference(maximum, minimum) // 2)


def _continuous(earlier: ReadingMessageRowV1, later: ReadingMessageRowV1) -> bool:
    required = int(
        ReadingFlag.DEEP_SLEEP_BOOT | ReadingFlag.PREVIOUS_CYCLE_METRICS_VALID
    )
    return (
        earlier.node_id == later.node_id
        and later.sample_id == earlier.sample_id + 1
        and later.flags & required == required
    )


def analyze_reading_timestamps(
    readings: Iterable[ReadingMessageRowV1],
    profiles: Iterable[MessageProfileRowV1],
    correlation: ClockCorrelation,
    *,
    materialized: Mapping[SampleKey, ReadingTimestamp] | None = None,
) -> dict[SampleKey, ReadingTimestamp]:
    """Return preserved output plus new estimates, keyed by (node_id, sample_id).

    Inputs are a complete supplied history snapshot. Node IDs scope identity
    lifetimes; receiver-instance ordinals order occurrences across clock domains.
    Direct anchors come from currently eligible occurrences, not from prior
    extrapolated output. Existing materialized entries always win for their keys.
    """
    messages: dict[tuple[bytes, int], ReadingMessageRowV1] = {}
    canonical: dict[SampleKey, ReadingMessageRowV1] = {}
    for reading in readings:
        if type(reading) is not ReadingMessageRowV1:
            raise TypeError("analysis requires immutable reading rows")
        _identity(reading.node_id, 8)
        for value in (reading.message_id, reading.sample_id):
            _bounded(value, 32)
        for value in (reading.run_ms, reading.previous_awake_ms, reading.flags):
            _bounded(value, 16)
        if type(reading.is_canonical_for_sample) is not bool:
            raise TypeError("canonical marker must be Boolean")
        message = (reading.node_id, reading.message_id)
        key = (reading.node_id, reading.sample_id)
        if message in messages or (
            reading.is_canonical_for_sample and key in canonical
        ):
            raise ValueError("duplicate reading-message or canonical-sample identity")
        messages[message] = reading
        if reading.is_canonical_for_sample:
            canonical[key] = reading
    direct: dict[SampleKey, tuple[tuple[int, int], ReadingTimestamp]] = {}
    occurrences: set[tuple[bytes, int]] = set()
    for stored in profiles:
        if type(stored) is not MessageProfileRowV1:
            raise TypeError("analysis requires immutable stored profile rows")
        profile = stored.profile
        identity = (profile.receiver_instance_id, profile.occurrence_sequence)
        _identity(identity[0], 16)
        _bounded(identity[1], 63)
        if identity in occurrences:
            raise ValueError("duplicate occurrence identity")
        occurrences.add(identity)
        if (
            profile.processing_result is not ProcessingResult.ACCEPTED
            or profile.header_authenticated is not True
            or profile.claimed_domain != Domain.CURRENT_READING_UPLINK
            or stored.persistence_classification not in _ANCHOR_CLASSIFICATIONS
        ):
            continue
        message = messages.get((profile.claimed_node_id, profile.claimed_message_id))
        key = (profile.claimed_node_id, profile.decoded_sample_id)
        sample = canonical.get(key)
        if (
            message is None
            or sample is None
            or message.sample_id != profile.decoded_sample_id
            or message.reading_body != sample.reading_body
        ):
            raise ValueError(
                "eligible occurrence does not resolve to its canonical reading"
            )
        instance = correlation.instance(profile.receiver_instance_id)
        if instance is None:
            raise ValueError("eligible occurrence has no durable receiver instance")
        event = correlation.correlate(
            profile.receiver_instance_id, profile.received_at_monotonic_us
        )
        if event is None:
            continue
        try:
            elapsed = checked_monotonic_deadline(
                checked_duration_product(message.run_ms, 1000), READING_AIRTIME_US
            )
            timestamp = direct_anchor_midpoint_us(event.utc_us, elapsed)
        except OverflowError:
            continue
        if timestamp is None:
            continue
        order = (instance.instance_ordinal, profile.occurrence_sequence)
        if key not in direct or order < direct[key][0]:
            direct[key] = (
                order,
                ReadingTimestamp(
                    timestamp,
                    TimestampSource.DIRECT,
                    sample.sample_id,
                    event.clock_observation_receiver_instance_id,
                    event.clock_observation_sequence,
                ),
            )
    output = dict(materialized or {})
    for key, value in output.items():
        _identity(key[0], 8)
        _bounded(key[1], 32)
        if type(value) is not ReadingTimestamp:
            raise TypeError("materialized output requires immutable analysis values")
    # Partition by real protocol continuity. Prefix durations permit either direction
    # without propagating another sample's rounded or already materialized estimate.
    components: list[list[ReadingMessageRowV1]] = []
    for key in sorted(canonical):
        reading = canonical[key]
        if not components or not _continuous(components[-1][-1], reading):
            components.append([])
        components[-1].append(reading)
    for component in components:
        positions = [0]
        for reading in component[1:]:
            cycle = checked_monotonic_deadline(
                DEEP_SLEEP_US, checked_duration_product(reading.previous_awake_ms, 1000)
            )
            positions.append(checked_monotonic_deadline(positions[-1], cycle))
        anchors = [
            i
            for i, reading in enumerate(component)
            if (reading.node_id, reading.sample_id) in direct
        ]
        for index, reading in enumerate(component):
            key = (reading.node_id, reading.sample_id)
            if key in output or not anchors:
                continue
            insertion = bisect_left(anchors, index)
            candidates = anchors[max(0, insertion - 1) : insertion + 1]
            anchor_index = min(
                candidates, key=lambda i: (abs(index - i), -component[i].sample_id)
            )
            anchor_reading = component[anchor_index]
            anchor = direct[(anchor_reading.node_id, anchor_reading.sample_id)][1]
            try:
                utc = checked_utc_offset(
                    anchor.timestamp_utc_us, positions[index] - positions[anchor_index]
                )
            except OverflowError:
                continue
            output[key] = ReadingTimestamp(
                utc,
                (
                    TimestampSource.DIRECT
                    if index == anchor_index
                    else TimestampSource.EXTRAPOLATED
                ),
                anchor.anchor_sample_id,
                anchor.clock_observation_receiver_instance_id,
                anchor.clock_observation_sequence,
            )
    return output
