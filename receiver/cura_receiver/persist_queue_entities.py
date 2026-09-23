"""Immutable logical values that may cross :class:`PersistQueue`.

These types describe ownership and composition only.  They deliberately do not
encode, copy, coerce, or semantically validate values; producer-side validators
belong to later receiver stages.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TypeAlias

from .generated.receiver_entities_generated import (
    ClockObservationV1,
    DiagnosticV1,
    MessageProfilingV1,
)
from .generated.receiver_enums_generated import (
    PersistQueueEntityKind,
    RadioState,
    RtcHealth,
    SystemTimeQuality,
)


PERSIST_QUEUE_ENTITY_SCHEMA_VERSION = 1


@dataclass(frozen=True, slots=True)
class PersistQueueEntitySpec:
    """Opaque logical contract attached to one queue slot."""

    kind: PersistQueueEntityKind
    schema_version: int


MEASUREMENT_PROFILE_V1_SPEC = PersistQueueEntitySpec(
    kind=PersistQueueEntityKind.MEASUREMENT_PROFILE,
    schema_version=PERSIST_QUEUE_ENTITY_SCHEMA_VERSION,
)
PROFILE_ONLY_V1_SPEC = PersistQueueEntitySpec(
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

SUPPORTED_PERSIST_QUEUE_ENTITY_SPECS = frozenset(
    {
        MEASUREMENT_PROFILE_V1_SPEC,
        PROFILE_ONLY_V1_SPEC,
        RECEIVER_HEALTH_REQUEST_V1_SPEC,
        DIAGNOSTIC_V1_SPEC,
        CLOCK_OBSERVATION_V1_SPEC,
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
class MeasurementProfileUnitV1:
    """Indivisible accepted-reading and occurrence-profile queue unit."""

    candidate: AuthenticatedReadingCandidateV1
    profile: MessageProfilingV1


@dataclass(frozen=True, slots=True)
class ProfileOnlyUnitV1:
    """One complete occurrence profile without an accepted reading."""

    profile: MessageProfilingV1


@dataclass(frozen=True, slots=True)
class ReceiverHealthRequestV1:
    """Immutable communicator-owned portion of one receiver-health row."""

    receiver_instance_id: bytes
    health_sequence: int
    communicator_sampled_at_monotonic_us: int
    radio_state: RadioState
    radio_recovery_attempts: int
    radio_recovery_successes: int
    radio_recovery_failures: int
    radio_recovery_attempts_by_reason: tuple[int, ...]
    system_time_quality: SystemTimeQuality
    rtc_health: RtcHealth
    time_quality_transition_count: int
    rtc_health_transition_count: int
    last_time_quality_transition_monotonic_us: int | None
    last_rtc_health_transition_monotonic_us: int | None
    chrony_step_command_results: tuple[int, ...]
    rtc_write_results: tuple[int, ...]
    rtc_write_readback_verified_count: int
    rtc_write_trust_invalidated_count: int
    persist_queue_admission_counts: tuple[tuple[int, ...], ...]


PersistQueueEntityV1: TypeAlias = (
    MeasurementProfileUnitV1
    | ProfileOnlyUnitV1
    | ReceiverHealthRequestV1
    | DiagnosticV1
    | ClockObservationV1
)


__all__ = [
    "PERSIST_QUEUE_ENTITY_SCHEMA_VERSION",
    "PersistQueueEntitySpec",
    "MEASUREMENT_PROFILE_V1_SPEC",
    "PROFILE_ONLY_V1_SPEC",
    "RECEIVER_HEALTH_REQUEST_V1_SPEC",
    "DIAGNOSTIC_V1_SPEC",
    "CLOCK_OBSERVATION_V1_SPEC",
    "SUPPORTED_PERSIST_QUEUE_ENTITY_SPECS",
    "AuthenticatedReadingCandidateV1",
    "MeasurementProfileUnitV1",
    "ProfileOnlyUnitV1",
    "ReceiverHealthRequestV1",
    "PersistQueueEntityV1",
]
