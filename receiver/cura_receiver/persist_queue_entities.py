"""Immutable logical values that may cross :class:`PersistQueue`.

These types describe ownership and composition only.  They deliberately do not
encode, copy, coerce, or semantically validate values; producer-side validators
belong to the components constructing them. Publication requires deep immutability
even though Python aliases remain reachable after ownership transfers.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TypeAlias

from .generated.receiver_entities_generated import (
    ClockObservationV1,
    DiagnosticV1,
    PacketReceptionV1,
    RadioObservationV1,
)
from .generated.receiver_enums_generated import (
    PersistQueueEntityKind,
    HealthConsistency,
    RtcReadStatus,
    RtcHealth,
    SystemTimeQuality,
)


PERSIST_QUEUE_ENTITY_SCHEMA_VERSION = 3


@dataclass(frozen=True, slots=True)
class PersistQueueEntitySpec:
    """Opaque logical contract attached to one queue slot."""

    kind: PersistQueueEntityKind
    schema_version: int


READING_RECEPTION_V2_SPEC = PersistQueueEntitySpec(
    kind=PersistQueueEntityKind.MEASUREMENT_PROFILE,
    schema_version=PERSIST_QUEUE_ENTITY_SCHEMA_VERSION,
)
RECEPTION_ONLY_V2_SPEC = PersistQueueEntitySpec(
    kind=PersistQueueEntityKind.PROFILE_ONLY,
    schema_version=PERSIST_QUEUE_ENTITY_SCHEMA_VERSION,
)
RECEIVER_HEALTH_REQUEST_V1_SPEC = PersistQueueEntitySpec(
    kind=PersistQueueEntityKind.RECEIVER_HEALTH_REQUEST,
    schema_version=PERSIST_QUEUE_ENTITY_SCHEMA_VERSION,
)
DIAGNOSTIC_V1_SPEC = PersistQueueEntitySpec(
    kind=PersistQueueEntityKind.DIAGNOSTIC,
    schema_version=PERSIST_QUEUE_ENTITY_SCHEMA_VERSION,
)
CLOCK_OBSERVATION_V1_SPEC = PersistQueueEntitySpec(
    kind=PersistQueueEntityKind.CLOCK_OBSERVATION,
    schema_version=PERSIST_QUEUE_ENTITY_SCHEMA_VERSION,
)

RADIO_OBSERVATION_V1_SPEC = PersistQueueEntitySpec(
    kind=PersistQueueEntityKind.RADIO_OBSERVATION,
    schema_version=PERSIST_QUEUE_ENTITY_SCHEMA_VERSION,
)

SUPPORTED_PERSIST_QUEUE_ENTITY_SPECS = frozenset(
    {
        READING_RECEPTION_V2_SPEC,
        RECEPTION_ONLY_V2_SPEC,
        RECEIVER_HEALTH_REQUEST_V1_SPEC,
        DIAGNOSTIC_V1_SPEC,
        CLOCK_OBSERVATION_V1_SPEC,
        RADIO_OBSERVATION_V1_SPEC,
    }
)


@dataclass(frozen=True, slots=True)
class AuthenticatedReadingCandidateV1:
    """Authenticated reading facts known before SQLite classification."""

    node_id: bytes
    message_id: int
    domain: int
    sample_id: int
    reading_body: bytes


@dataclass(frozen=True, slots=True)
class ReadingReceptionUnitV2:
    """Indivisible admitted reading, reception and transient raw evidence."""

    candidate: AuthenticatedReadingCandidateV1
    profile: PacketReceptionV1
    received_frame: bytes


@dataclass(frozen=True, slots=True)
class ReceptionOnlyUnitV2:
    """Authenticated reception and transient raw evidence without an admitted reading."""

    profile: PacketReceptionV1
    received_frame: bytes


@dataclass(frozen=True, slots=True)
class ReceiverHealthRequestV1:
    """Immutable health and radio evidence, committed as one persistence unit."""

    receiver_instance_id: bytes
    health_sequence: int
    communicator_sampled_at_monotonic_us: int
    radio_observation: RadioObservationV1
    consistency: HealthConsistency
    radio_header_errors_observed: int
    radio_header_errors_handled: int
    radio_crc_errors_observed: int
    radio_crc_errors_handled: int
    radio_observations_dropped: int
    radio_recovery_attempts: int
    radio_recovery_successes: int
    radio_recovery_failures: int
    radio_recovery_attempts_by_reason: tuple[int, ...]
    system_time_quality: SystemTimeQuality
    rtc_health: RtcHealth
    rtc_read_status: RtcReadStatus
    rtc_read_errno: int | None
    rtc_read_started_at_monotonic_us: int
    rtc_read_finished_at_monotonic_us: int
    rtc_read_utc_s: int | None
    time_quality_transition_count: int
    rtc_health_transition_count: int
    last_time_quality_transition_monotonic_us: int | None
    last_rtc_health_transition_monotonic_us: int | None
    chrony_step_command_results: tuple[int, ...]
    rtc_write_results: tuple[int, ...]
    rtc_write_readback_verified_count: int
    rtc_write_trust_invalidated_count: int
    persist_queue_admission_counts: tuple[tuple[int, ...], ...]


PersistQueueEntityV2: TypeAlias = (
    ReadingReceptionUnitV2
    | ReceptionOnlyUnitV2
    | RadioObservationV1
    | ReceiverHealthRequestV1
    | DiagnosticV1
    | ClockObservationV1
)


__all__ = [
    "PERSIST_QUEUE_ENTITY_SCHEMA_VERSION",
    "PersistQueueEntitySpec",
    "READING_RECEPTION_V2_SPEC",
    "RECEPTION_ONLY_V2_SPEC",
    "RECEIVER_HEALTH_REQUEST_V1_SPEC",
    "DIAGNOSTIC_V1_SPEC",
    "CLOCK_OBSERVATION_V1_SPEC",
    "RADIO_OBSERVATION_V1_SPEC",
    "SUPPORTED_PERSIST_QUEUE_ENTITY_SPECS",
    "AuthenticatedReadingCandidateV1",
    "ReadingReceptionUnitV2",
    "ReceptionOnlyUnitV2",
    "ReceiverHealthRequestV1",
    "PersistQueueEntityV2",
]
