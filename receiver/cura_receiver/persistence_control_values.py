"""Immutable control values; semantic state validation lives in handwritten persistence."""

from __future__ import annotations

from dataclasses import dataclass, fields
from enum import Enum, auto

from .generated.receiver_entities_generated import (
    CommunicatorStateV1,
    RtcProvenanceV1,
    TxAirtimeBucketV1,
)
from .generated.receiver_enums_generated import DiagnosticOperation as Operation
from .generated.receiver_enums_generated import RtcHealth, SystemTimeQuality
from .receiver_configuration import PersistenceControlInterfaceViolation as Violation


class CommunicatorStateCondition(Enum):
    NONE = auto()
    MISSING = auto()
    CORRUPT = auto()
    UNSUPPORTED_VERSION = auto()
    POLICY_MISMATCH = auto()


class CommunicatorStateLoadStatus(Enum):
    LOADED = auto()
    STATE_UNAVAILABLE = auto()
    INTERFACE_VIOLATION = auto()
    DATABASE_ERROR = auto()
    DEADLINE_EXCEEDED = auto()
    CHANNEL_CLOSED = auto()


class CommunicatorStateCommitDisposition(Enum):
    COMMITTED = auto()
    ALREADY_COMMITTED = auto()
    NOT_INSTALLED = auto()
    OUTCOME_UNKNOWN = auto()


class CommunicatorStateCommitFailureKind(Enum):
    NONE = auto()
    INTERFACE_VIOLATION = auto()
    STATE_UNAVAILABLE = auto()
    DATABASE_ERROR = auto()
    DEADLINE_EXCEEDED = auto()
    CHANNEL_CLOSED = auto()


class ReceiverCleanStopCommitDisposition(Enum):
    COMMITTED = auto()
    ALREADY_COMMITTED = auto()
    NOT_COMMITTED = auto()
    OUTCOME_UNKNOWN = auto()


class ReceiverCleanStopCommitFailureKind(Enum):
    NONE = auto()
    INTERFACE_VIOLATION = auto()
    DATABASE_ERROR = auto()
    DEADLINE_EXCEEDED = auto()
    CHANNEL_CLOSED = auto()


def _enum(value, expected):
    if type(value) is not expected:
        raise TypeError(f"expected {expected.__name__}")


def require_immutable_state(state: CommunicatorStateV1) -> None:
    """Reject mutable/foreign logical values before publishing their references."""
    _enum(state, CommunicatorStateV1)
    for field in fields(state):
        expected = {
            "last_observed_system_time_quality": SystemTimeQuality,
            "last_observed_rtc_health": RtcHealth,
            "rtc_provenance": type(None)
            if state.rtc_provenance is None
            else RtcProvenanceV1,
            "buckets": tuple,
        }.get(field.name, int)
        _enum(getattr(state, field.name), expected)
    if state.rtc_provenance is not None:
        for field in fields(state.rtc_provenance):
            _enum(
                getattr(state.rtc_provenance, field.name),
                bytes if field.name == "verified_by_receiver_instance_id" else int,
            )
    for bucket in state.buckets:
        _enum(bucket, TxAirtimeBucketV1)
        _enum(bucket.charged_airtime_us, int)
        _enum(bucket.expires_at_utc_us, int)


def _evidence(result, *, failure: str, operation: Operation) -> None:
    _enum(result.operation, Operation)
    _enum(result.interface_violation, Violation)
    if result.operation is not (Operation.NONE if failure == "NONE" else operation):
        raise ValueError("operation does not match the result")
    if (result.interface_violation is not Violation.NONE) != (
        failure == "INTERFACE_VIOLATION"
    ):
        raise ValueError("interface violation does not match the failure")
    for name in ("sqlite_primary_code", "sqlite_extended_code", "os_errno"):
        value = getattr(result, name)
        if value is not None:
            if type(value) is not int or not -(1 << 31) <= value < (1 << 31):
                raise ValueError("failure evidence must be an i32 or absent")
            if failure != "DATABASE_ERROR":
                raise ValueError("database evidence requires a database failure")
    if result.sqlite_extended_code is not None and (
        result.sqlite_primary_code is None
        or result.sqlite_primary_code != (result.sqlite_extended_code & 0xFF)
    ):
        raise ValueError("SQLite extended evidence requires its matching primary code")


@dataclass(frozen=True, slots=True)
class CommunicatorStateLoadResult:
    status: CommunicatorStateLoadStatus
    operation: Operation
    interface_violation: Violation = Violation.NONE
    state_condition: CommunicatorStateCondition = CommunicatorStateCondition.NONE
    sqlite_primary_code: int | None = None
    sqlite_extended_code: int | None = None
    os_errno: int | None = None
    state: CommunicatorStateV1 | None = None

    def __post_init__(self) -> None:
        _enum(self.status, CommunicatorStateLoadStatus)
        _enum(self.state_condition, CommunicatorStateCondition)
        loaded = self.status is CommunicatorStateLoadStatus.LOADED
        if loaded != (self.state is not None):
            raise ValueError("state is present exactly for LOADED")
        if loaded:
            require_immutable_state(self.state)
        if (self.state_condition is not CommunicatorStateCondition.NONE) != (
            self.status is CommunicatorStateLoadStatus.STATE_UNAVAILABLE
        ):
            raise ValueError("state condition does not match load status")
        _evidence(
            self,
            failure="NONE" if loaded else self.status.name,
            operation=Operation.READ,
        )


@dataclass(frozen=True, slots=True)
class CommunicatorStateCommitResult:
    disposition: CommunicatorStateCommitDisposition
    failure_kind: CommunicatorStateCommitFailureKind
    operation: Operation
    interface_violation: Violation = Violation.NONE
    state_condition: CommunicatorStateCondition = CommunicatorStateCondition.NONE
    sqlite_primary_code: int | None = None
    sqlite_extended_code: int | None = None
    os_errno: int | None = None

    def __post_init__(self) -> None:
        _enum(self.disposition, CommunicatorStateCommitDisposition)
        _enum(self.failure_kind, CommunicatorStateCommitFailureKind)
        _enum(self.state_condition, CommunicatorStateCondition)
        if (self.state_condition is not CommunicatorStateCondition.NONE) != (
            self.failure_kind is CommunicatorStateCommitFailureKind.STATE_UNAVAILABLE
        ):
            raise ValueError("state condition does not match commit failure")
        _commit_result(self, Operation.WRITE)


@dataclass(frozen=True, slots=True)
class ReceiverCleanStopV1:
    receiver_instance_id: bytes
    stopped_at_monotonic_us: int
    communicator_state_generation: int

    def __post_init__(self) -> None:
        if (
            type(self.receiver_instance_id) is not bytes
            or len(self.receiver_instance_id) != 16
        ):
            raise ValueError("receiver_instance_id must be exactly 16 bytes")
        for value in (self.stopped_at_monotonic_us, self.communicator_state_generation):
            if type(value) is not int or not 0 <= value <= (1 << 63) - 1:
                raise ValueError("clean-stop values must be integers in 0..INT64_MAX")


@dataclass(frozen=True, slots=True)
class ReceiverCleanStopCommitResult:
    disposition: ReceiverCleanStopCommitDisposition
    failure_kind: ReceiverCleanStopCommitFailureKind
    operation: Operation
    interface_violation: Violation = Violation.NONE
    sqlite_primary_code: int | None = None
    sqlite_extended_code: int | None = None
    os_errno: int | None = None

    def __post_init__(self) -> None:
        _enum(self.disposition, ReceiverCleanStopCommitDisposition)
        _enum(self.failure_kind, ReceiverCleanStopCommitFailureKind)
        _commit_result(self, Operation.CLEANUP)


def _commit_result(result, operation):
    failure = result.failure_kind.name
    success = result.disposition.name in ("COMMITTED", "ALREADY_COMMITTED")
    if success != (failure == "NONE"):
        raise ValueError("disposition does not match failure kind")
    if result.disposition.name == "OUTCOME_UNKNOWN" and failure not in (
        "DATABASE_ERROR",
        "DEADLINE_EXCEEDED",
        "CHANNEL_CLOSED",
    ):
        raise ValueError("unknown outcome requires an operational failure")
    _evidence(result, failure=failure, operation=operation)
