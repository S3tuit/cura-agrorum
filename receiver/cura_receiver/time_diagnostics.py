"""Bounded TIME evidence and encoding; the communicator owns admission/identity."""

from dataclasses import dataclass, replace
from enum import IntFlag
import struct

from .elapsed_duration import checked_monotonic_elapsed
from .generated import receiver_enums_generated as E
from .generated.receiver_entities_generated import DiagnosticV1


class TimeFlags(IntFlag):
    SOURCE_SELECTED = 1
    SYNCHRONIZED = 2
    COMMAND_MAY_HAVE_APPLIED = 4
    QUALITY_CHANGED = 8
    RTC_HEALTH_CHANGED = 16
    GENERATION_RECHECK_MATCHED = 32
    READBACK_MATCHED = 64
    TRUST_SUPPRESSED = 128


_STATUS_ENUMS = {
    E.TimeBackendStatusKind.CHRONY_QUERY_STATUS: E.ChronyQueryStatus,
    E.TimeBackendStatusKind.CHRONY_STEP_DISPOSITION: E.ChronyStepDisposition,
    E.TimeBackendStatusKind.DS3231_READ_STATUS: E.Ds3231ReadStatus,
    E.TimeBackendStatusKind.DS3231_WRITE_DISPOSITION: E.Ds3231WriteDisposition,
    E.TimeBackendStatusKind.DS3231_FAILURE: E.Ds3231Failure,
    E.TimeBackendStatusKind.ADJTIMEX_RETURN: E.AdjtimexReturn,
}
_ALLOWED = {
    E.TimeDiagnosticErrorCode.IO: (
        E.DiagnosticOperation.INITIALIZE,
        E.DiagnosticOperation.READ,
        E.DiagnosticOperation.SYNC,
    ),
    E.TimeDiagnosticErrorCode.DEADLINE: (
        E.DiagnosticOperation.READ,
        E.DiagnosticOperation.SYNC,
    ),
    E.TimeDiagnosticErrorCode.INVALID_RESPONSE: (
        E.DiagnosticOperation.INITIALIZE,
        E.DiagnosticOperation.READ,
        E.DiagnosticOperation.SYNC,
    ),
    E.TimeDiagnosticErrorCode.COMMAND_REJECTED: (E.DiagnosticOperation.SYNC,),
    E.TimeDiagnosticErrorCode.OUTCOME_UNKNOWN: (E.DiagnosticOperation.SYNC,),
    E.TimeDiagnosticErrorCode.CLOCK_INTERFERENCE: (
        E.DiagnosticOperation.READ,
        E.DiagnosticOperation.VALIDATE,
    ),
    E.TimeDiagnosticErrorCode.RTC_READBACK_MISMATCH: (E.DiagnosticOperation.SYNC,),
    E.TimeDiagnosticErrorCode.CALCULATION_RANGE: (
        E.DiagnosticOperation.VALIDATE,
        E.DiagnosticOperation.SYNC,
    ),
}
_CONTEXT = struct.Struct("<I10BHiIQQQqqQQ")


def integer(value, low=0, high=(1 << 64) - 1):
    if type(value) is not int:
        raise TypeError("expected an exact integer")
    if not low <= value <= high:
        raise ValueError("integer outside interface range")
    return value


def _member(value, enum, *, nonzero=False):
    if type(value) is not enum or (nonzero and value.value == 0):
        raise ValueError("undefined interface enum")


@dataclass(frozen=True, slots=True)
class BackendStatus:
    kind: E.TimeBackendStatusKind
    value: int

    def __post_init__(self):
        _member(self.kind, E.TimeBackendStatusKind, nonzero=True)
        integer(self.value, 0, 255)
        _STATUS_ENUMS[self.kind](self.value)


def encoded_status(kind, runtime_status):
    """Map names explicitly through the diagnostic catalogue, never Enum.value."""
    _member(kind, E.TimeBackendStatusKind, nonzero=True)
    return BackendStatus(kind, _STATUS_ENUMS[kind][runtime_status.name].value)


@dataclass(frozen=True, slots=True)
class ReceiverTimeEpisodeContextV1:
    component: E.TimeComponent
    stage: E.TimeFailureStage
    operation_duration_us: int
    primary_status: BackendStatus | None = None
    secondary_status: BackendStatus | None = None
    quality: tuple[E.SystemTimeQuality, E.SystemTimeQuality] | None = None
    rtc_health: tuple[E.RtcHealth, E.RtcHealth] | None = None
    flags: int = 0
    os_errno: int | None = None
    kernel_status_bits: int | None = None
    clock_state_generation: int | None = None
    operation_generation: int | None = None
    related_clock_observation_sequence: int | None = None
    observed_value_us: int | None = None
    comparison_value_us: int | None = None
    threshold_us: int | None = None

    def __post_init__(self):
        _member(self.component, E.TimeComponent, nonzero=True)
        _member(self.stage, E.TimeFailureStage, nonzero=True)
        integer(self.operation_duration_us)
        integer(self.flags, 0, 255)
        for value in (self.primary_status, self.secondary_status):
            if value is not None and type(value) is not BackendStatus:
                raise TypeError("expected a bounded backend status")
        for pair, enum in (
            (self.quality, E.SystemTimeQuality),
            (self.rtc_health, E.RtcHealth),
        ):
            if pair is not None:
                if type(pair) is not tuple or len(pair) != 2:
                    raise ValueError("transition requires both values")
                for value in pair:
                    _member(value, enum)
        for name in (
            "os_errno",
            "kernel_status_bits",
            "clock_state_generation",
            "operation_generation",
            "related_clock_observation_sequence",
            "observed_value_us",
            "comparison_value_us",
            "threshold_us",
        ):
            value = getattr(self, name)
            if value is None:
                continue
            low, high = (0, (1 << 64) - 1)
            if name == "os_errno":
                low, high = -(1 << 31), (1 << 31) - 1
            elif name == "kernel_status_bits":
                high = (1 << 32) - 1
            elif name in ("observed_value_us", "comparison_value_us"):
                low, high = -(1 << 63), (1 << 63) - 1
            integer(value, low, high)


_OPTIONALS = (
    "primary_status",
    "secondary_status",
    "quality",
    "rtc_health",
    "os_errno",
    "kernel_status_bits",
    "clock_state_generation",
    "operation_generation",
    "related_clock_observation_sequence",
    "observed_value_us",
    "comparison_value_us",
    "threshold_us",
)


def encode_time_context(context):
    if type(context) is not ReceiverTimeEpisodeContextV1:
        raise TypeError("expected TIME context")
    mask = sum(
        1 << i
        for i, name in enumerate(_OPTIONALS)
        if getattr(context, name) is not None
    )
    statuses = []
    for value in (context.primary_status, context.secondary_status):
        statuses.extend((0, 0) if value is None else (value.kind.value, value.value))
    pairs = []
    for pair in (context.quality, context.rtc_health):
        pairs.extend((0, 0) if pair is None else (pair[0].value, pair[1].value))
    return _CONTEXT.pack(
        mask,
        context.component.value,
        context.stage.value,
        *statuses,
        *pairs,
        context.flags,
        *(getattr(context, name) or 0 for name in _OPTIONALS[4:]),
        context.operation_duration_us
    )


def decode_time_context(data):
    if type(data) is not bytes or len(data) != 80:
        raise ValueError("TIME context requires exactly 80 bytes")
    values = _CONTEXT.unpack(data)
    mask, component, stage = values[:3]
    if mask & ~0xFFF:
        raise ValueError("reserved validity bits")
    kwargs = {}
    for i, start in enumerate((3, 5, 7, 9)):
        first, second = values[start : start + 2]
        if mask & (1 << i):
            if i < 2:
                value = BackendStatus(E.TimeBackendStatusKind(first), second)
            else:
                enum = E.SystemTimeQuality if i == 2 else E.RtcHealth
                value = (enum(first), enum(second))
            kwargs[_OPTIONALS[i]] = value
        elif first or second:
            raise ValueError("nonzero absent pair")
    for i, value in enumerate(values[12:20], 4):
        if mask & (1 << i):
            kwargs[_OPTIONALS[i]] = value
        elif value:
            raise ValueError("nonzero absent scalar")
    return ReceiverTimeEpisodeContextV1(
        E.TimeComponent(component),
        E.TimeFailureStage(stage),
        values[20],
        flags=values[11],
        **kwargs
    )


@dataclass(frozen=True, slots=True)
class TimeFailureEpisode:
    operation: E.DiagnosticOperation
    error_code: E.TimeDiagnosticErrorCode
    sampled_at_monotonic_us: int
    context: ReceiverTimeEpisodeContextV1

    def __post_init__(self):
        _member(self.error_code, E.TimeDiagnosticErrorCode, nonzero=True)
        if self.operation not in _ALLOWED[self.error_code]:
            raise ValueError("invalid TIME error/operation combination")
        integer(self.sampled_at_monotonic_us, 0, (1 << 63) - 1)
        if type(self.context) is not ReceiverTimeEpisodeContextV1:
            raise TypeError("expected TIME episode context")

    def finish(
        self,
        *,
        started_at_monotonic_us,
        finished_at_monotonic_us,
        secondary_status=None,
        quality=None,
        rtc_health=None,
        flags=None
    ):
        """Freeze the trigger; fill only the documented bounded outcome fields."""
        return replace(
            self,
            context=replace(
                self.context,
                operation_duration_us=checked_monotonic_elapsed(
                    started_at_monotonic_us, finished_at_monotonic_us
                ),
                secondary_status=(
                    secondary_status
                    if secondary_status is not None
                    else self.context.secondary_status
                ),
                quality=quality if quality is not None else self.context.quality,
                rtc_health=(
                    rtc_health if rtc_health is not None else self.context.rtc_health
                ),
                flags=self.context.flags if flags is None else flags,
            ),
        )


def time_diagnostic(episode, *, receiver_instance_id, diagnostic_sequence):
    if type(episode) is not TimeFailureEpisode:
        raise TypeError("expected TIME failure episode")
    if type(receiver_instance_id) is not bytes or len(receiver_instance_id) != 16:
        raise ValueError("invalid receiver identity")
    integer(diagnostic_sequence, 1, (1 << 63) - 1)
    return DiagnosticV1(
        receiver_instance_id,
        diagnostic_sequence,
        episode.sampled_at_monotonic_us,
        E.DiagnosticSeverity.ERROR,
        E.DiagnosticErrorDomain.TIME,
        episode.operation,
        episode.error_code.value,
        1,
        80,
        encode_time_context(episode.context) + bytes(48),
    )


class TimeFailureLatch:
    def __init__(self):
        self._signature = None

    def failed(self, episode):
        signature = (
            episode.context.component,
            episode.operation,
            episode.error_code,
            episode.context.stage,
        )
        repeated = signature == self._signature
        self._signature = signature
        return None if repeated else episode

    def succeeded(self):
        self._signature = None
