"""Closed RADIO catalogue and bounded episode values; no queue admission."""

from dataclasses import dataclass
import struct

from .elapsed_duration import checked_monotonic_elapsed
from .generated import receiver_enums_generated as E
from .generated.receiver_entities_generated import DiagnosticV1
from .ports.radio import RadioFailure, integer

State = E.RadioState
Code = E.RadioDiagnosticErrorCode
Operation = E.DiagnosticOperation
Level = E.RadioRecoveryLevelResult
Reason = E.RadioRecoveryReason
Severity = E.DiagnosticSeverity
_DETAIL = struct.Struct("<5BiBHH")
_TAIL = struct.Struct("<H6BH2xQqQ")
_SEMANTIC = (Operation.INITIALIZE, Operation.TRANSMIT, Operation.RECEIVE,
             Operation.RECOVER, Operation.CLEANUP)
_FINISHED = (State.RX_SINGLE, State.RX_EVENT_PENDING, State.SHUTDOWN,
             State.INITIALIZATION_FAILED, State.RECOVERY_EXHAUSTED, State.HARDWARE_MISSING)


def member(value, enum):
    if type(value) is not enum:
        raise ValueError("undefined radio diagnostic enum")


def _pair(operation, code):
    member(operation, Operation)
    member(code, Code)
    allowed = (Operation.VALIDATE,) if code in (Code.INVALID_ARGUMENT, Code.INVALID_STATE) else _SEMANTIC
    if code is Code.NONE or operation not in allowed:
        raise ValueError("undefined radio operation/error pair")


@dataclass(frozen=True, slots=True)
class RadioFailureDetailV1:
    state: State
    command_opcode: int
    stage: E.RadioFailureStage
    backend_status_kind: E.RadioBackendStatusKind = E.RadioBackendStatusKind.NONE
    backend_status: int = 0
    chip_status: int | None = None
    irq_status: int | None = None
    device_errors: int | None = None
    hardware_touched: bool = False

    def __post_init__(self):
        member(self.state, State)
        member(self.stage, E.RadioFailureStage)
        member(self.backend_status_kind, E.RadioBackendStatusKind)
        integer(self.command_opcode, 0, 255)
        integer(self.backend_status, -(1 << 31), (1 << 31) - 1)
        if self.backend_status_kind is E.RadioBackendStatusKind.NONE and self.backend_status:
            raise ValueError("absent backend status must be zero")
        if self.backend_status_kind is E.RadioBackendStatusKind.ERRNO and self.backend_status <= 0:
            raise ValueError("errno must be positive")
        for value, maximum in ((self.chip_status, 255), (self.irq_status, 65535), (self.device_errors, 65535)):
            if value is not None:
                integer(value, 0, maximum)
        if type(self.hardware_touched) is not bool:
            raise TypeError("hardware touch requires a Boolean")

    @classmethod
    def from_failure(cls, state, failure):
        if type(failure) is not RadioFailure:
            raise TypeError("expected normalized radio failure")
        return cls(
            state, failure.opcode, failure.stage,
            E.RadioBackendStatusKind.NONE if failure.os_errno is None else E.RadioBackendStatusKind.ERRNO,
            failure.os_errno or 0, failure.chip_status, failure.irq_status,
            failure.device_errors, failure.hardware_touched,
        )


def encode_failure_detail(detail):
    if type(detail) is not RadioFailureDetailV1:
        raise TypeError("expected radio failure detail")
    flags = (int(detail.chip_status is not None) | (int(detail.irq_status is not None) << 1)
             | (int(detail.device_errors is not None) << 2) | (int(detail.hardware_touched) << 3))
    return _DETAIL.pack(
        detail.state.value, detail.command_opcode, detail.stage.value, flags,
        detail.backend_status_kind.value, detail.backend_status,
        detail.chip_status or 0, detail.irq_status or 0, detail.device_errors or 0,
    )


def decode_failure_detail(data):
    if type(data) is not bytes or len(data) != 14:
        raise ValueError("radio failure detail requires 14 bytes")
    state, opcode, stage, flags, kind, backend, chip, irq, errors = _DETAIL.unpack(data)
    if flags & ~15:
        raise ValueError("reserved detail flags")
    values = []
    for bit, value in enumerate((chip, irq, errors)):
        if not flags & (1 << bit) and value:
            raise ValueError("nonzero absent failure field")
        values.append(value if flags & (1 << bit) else None)
    return RadioFailureDetailV1(State(state), opcode, E.RadioFailureStage(stage),
                                E.RadioBackendStatusKind(kind), backend, *values, bool(flags & 8))


@dataclass(frozen=True, slots=True)
class ReceiverRadioEpisodeContextV1:
    trigger_detail: RadioFailureDetailV1
    recovery_reason: Reason
    trigger_command_outcome: E.RadioCommandOutcome
    soft_recovery_result: Level
    hard_recovery_result: Level
    terminal_state: State
    episode_duration_us: int
    last_recovery_failure_detail: RadioFailureDetailV1 | None = None
    last_recovery_error_code: Code | None = None
    related_occurrence_sequence: int | None = None
    airtime_bucket_expiration_utc_us: int | None = None

    def __post_init__(self):
        if type(self.trigger_detail) is not RadioFailureDetailV1:
            raise TypeError("expected trigger detail")
        for value, enum in ((self.recovery_reason, Reason), (self.trigger_command_outcome, E.RadioCommandOutcome),
                            (self.soft_recovery_result, Level), (self.hard_recovery_result, Level), (self.terminal_state, State)):
            member(value, enum)
        if self.terminal_state not in _FINISHED:
            raise ValueError("episode is not completed")
        integer(self.episode_duration_us)
        if (self.last_recovery_failure_detail is None) != (self.last_recovery_error_code is None):
            raise ValueError("last failure requires both detail and code")
        if self.last_recovery_failure_detail is not None:
            if type(self.last_recovery_failure_detail) is not RadioFailureDetailV1:
                raise TypeError("invalid last failure detail")
            _pair(Operation.RECOVER, self.last_recovery_error_code)
            if self.last_recovery_failure_detail.state is not State.RECOVERING:
                raise ValueError("last recovery failure must describe recovery")
        if self.related_occurrence_sequence is not None:
            integer(self.related_occurrence_sequence, 1)
        if self.airtime_bucket_expiration_utc_us is not None:
            integer(self.airtime_bucket_expiration_utc_us, -(1 << 63), (1 << 63) - 1)
        soft, hard = self.soft_recovery_result, self.hard_recovery_result
        if self.recovery_reason is Reason.NONE:
            if (soft, hard) != (Level.NOT_APPLICABLE, Level.NOT_APPLICABLE) or self.last_recovery_error_code is not None:
                raise ValueError("direct episode has recovery evidence")
        else:
            if Level.NOT_APPLICABLE in (soft, hard):
                raise ValueError("recovery levels cannot be not applicable")
            if hard is not Level.NOT_ATTEMPTED and soft is not Level.FAILED:
                raise ValueError("hard recovery requires failed soft recovery")
            if self.terminal_state is State.RX_SINGLE:
                if Level.SUCCEEDED not in (soft, hard):
                    raise ValueError("RX recovery requires a successful level")
            elif self.terminal_state not in (State.SHUTDOWN, State.HARDWARE_MISSING, State.RECOVERY_EXHAUSTED):
                raise ValueError("invalid recovery final state")
            elif Level.SUCCEEDED in (soft, hard):
                raise ValueError("successful recovery must finish at RX")
            if self.terminal_state is State.RECOVERY_EXHAUSTED and (soft, hard) != (Level.FAILED, Level.FAILED):
                raise ValueError("exhaustion requires both levels to fail")
            if self.last_recovery_error_code is not None and Level.FAILED not in (soft, hard):
                raise ValueError("last failure without a failed level")


def encode_radio_context(context):
    if type(context) is not ReceiverRadioEpisodeContextV1:
        raise TypeError("expected radio context")
    last = context.last_recovery_failure_detail
    mask = int(last is not None) | (int(context.related_occurrence_sequence is not None) << 1) | (int(context.airtime_bucket_expiration_utc_us is not None) << 2)
    return encode_failure_detail(context.trigger_detail) + (encode_failure_detail(last) if last else bytes(14)) + _TAIL.pack(
        mask, context.recovery_reason.value, context.trigger_command_outcome.value,
        context.soft_recovery_result.value, context.hard_recovery_result.value,
        context.terminal_state.value, 0,
        context.last_recovery_error_code.value if last else 0,
        context.related_occurrence_sequence or 0, context.airtime_bucket_expiration_utc_us or 0,
        context.episode_duration_us,
    )


def decode_radio_context(data):
    if type(data) is not bytes or len(data) != 64:
        raise ValueError("radio context requires exactly 64 bytes")
    mask, reason, outcome, soft, hard, terminal, reserved, last_code, sequence, bucket, duration = _TAIL.unpack(data[28:])
    if mask & ~7 or reserved or data[38:40] != bytes(2):
        raise ValueError("reserved radio context bits")
    if not mask & 1 and (data[14:28] != bytes(14) or last_code):
        raise ValueError("nonzero absent recovery failure")
    if (not mask & 2 and sequence) or (not mask & 4 and bucket):
        raise ValueError("nonzero absent correlation")
    return ReceiverRadioEpisodeContextV1(
        decode_failure_detail(data[:14]), Reason(reason), E.RadioCommandOutcome(outcome),
        Level(soft), Level(hard), State(terminal), duration,
        decode_failure_detail(data[14:28]) if mask & 1 else None,
        Code(last_code) if mask & 1 else None, sequence if mask & 2 else None,
        bucket if mask & 4 else None,
    )


@dataclass(frozen=True, slots=True)
class CompletedRadioEpisode:
    operation: Operation
    error_code: Code
    severity: Severity
    sampled_at_monotonic_us: int
    context: ReceiverRadioEpisodeContextV1

    def __post_init__(self):
        _pair(self.operation, self.error_code)
        member(self.severity, Severity)
        integer(self.sampled_at_monotonic_us)
        if type(self.context) is not ReceiverRadioEpisodeContextV1:
            raise TypeError("expected completed radio context")
        state = self.context.terminal_state
        if self.severity is Severity.FATAL and state in (State.RX_SINGLE, State.RX_EVENT_PENDING):
            raise ValueError("fatal radio diagnostic requires a terminal state")
        if self.error_code is Code.INVALID_ARGUMENT:
            allowed = (Severity.FATAL,)
        elif self.operation is Operation.INITIALIZE:
            if state not in (State.INITIALIZATION_FAILED, State.HARDWARE_MISSING):
                raise ValueError("initialization diagnostic requires startup failure")
            allowed = (Severity.FATAL,)
        elif state in (State.HARDWARE_MISSING, State.RECOVERY_EXHAUSTED, State.INITIALIZATION_FAILED):
            allowed = (Severity.FATAL,)
        elif state is State.SHUTDOWN:
            allowed = (Severity.FATAL,) if self.operation is Operation.VALIDATE and self.context.recovery_reason is Reason.NONE else (Severity.ERROR, Severity.FATAL)
        elif self.error_code is Code.UNEXPECTED_IRQ and self.context.recovery_reason is Reason.NONE:
            allowed = (Severity.WARN,)
        else:
            allowed = (Severity.ERROR,)
        if self.operation is Operation.CLEANUP and state not in (
            State.SHUTDOWN, State.INITIALIZATION_FAILED, State.HARDWARE_MISSING,
        ):
            raise ValueError("cleanup diagnostic requires shutdown or failed startup")
        if self.severity not in allowed:
            raise ValueError("invalid radio diagnostic severity")


def radio_diagnostic(episode, *, receiver_instance_id, diagnostic_sequence):
    if type(episode) is not CompletedRadioEpisode:
        raise TypeError("expected completed radio episode")
    if type(receiver_instance_id) is not bytes or len(receiver_instance_id) != 16:
        raise ValueError("invalid receiver identity")
    integer(diagnostic_sequence, 1, (1 << 63) - 1)
    return DiagnosticV1(
        receiver_instance_id, diagnostic_sequence, episode.sampled_at_monotonic_us,
        episode.severity, E.DiagnosticErrorDomain.RADIO, episode.operation,
        episode.error_code.value, 1, 64, encode_radio_context(episode.context) + bytes(64),
    )


def decode_radio_diagnostic(diagnostic):
    """Validate a stored RADIO entity against this closed component catalogue."""
    if type(diagnostic) is not DiagnosticV1:
        raise TypeError("expected DiagnosticV1")
    if (diagnostic.error_domain is not E.DiagnosticErrorDomain.RADIO
            or type(diagnostic.context_schema) is not int or diagnostic.context_schema != 1
            or type(diagnostic.context_length) is not int or diagnostic.context_length != 64
            or type(diagnostic.context) is not bytes or len(diagnostic.context) != 128
            or diagnostic.context[64:] != bytes(64)):
        raise ValueError("invalid RADIO domain/schema/context")
    integer(diagnostic.error_code, 1, 9)
    result = CompletedRadioEpisode(
        diagnostic.operation, Code(diagnostic.error_code), diagnostic.severity,
        diagnostic.sampled_at_monotonic_us, decode_radio_context(diagnostic.context[:64]),
    )
    radio_diagnostic(result, receiver_instance_id=diagnostic.receiver_instance_id,
                     diagnostic_sequence=diagnostic.diagnostic_sequence)
    return result


@dataclass(frozen=True, slots=True)
class _Trigger:
    operation: Operation
    failure: RadioFailure
    detail: RadioFailureDetailV1
    at_us: int
    sequence: int | None
    bucket: int | None


class RadioEpisodeBuilder:
    """Owner-local bounded fields; no history, identity allocation or admission."""

    def __init__(self, operation, failure, state, at_us, *, sequence=None, bucket=None):
        _pair(operation, failure.code)
        integer(at_us)
        self._trigger = _Trigger(operation, failure, RadioFailureDetailV1.from_failure(state, failure), at_us, sequence, bucket)
        self.reason = Reason.NONE
        self.soft = self.hard = Level.NOT_APPLICABLE
        self.last_failure = None
        self.active_level = None
        self._finished = False

    def _live(self):
        if self._finished:
            raise RuntimeError("radio episode already finalized")

    @property
    def trigger_failure(self):
        return self._trigger.failure

    def enter_recovery(self, reason):
        self._live()
        member(reason, Reason)
        if reason is Reason.NONE or self.reason is not Reason.NONE:
            raise ValueError("recovery may be entered once")
        self.reason = reason
        self.soft = self.hard = Level.NOT_ATTEMPTED

    def start_level(self, *, hard=False):
        self._live()
        if self.active_level is not None or self.reason is Reason.NONE:
            raise RuntimeError("invalid recovery level start")
        name = "hard" if hard else "soft"
        if getattr(self, name) is not Level.NOT_ATTEMPTED or (hard and self.soft is not Level.FAILED):
            raise RuntimeError("recovery level cannot be repeated")
        self.active_level = name

    def finish_level(self, failure=None):
        self._live()
        if self.active_level is None:
            raise RuntimeError("no active recovery level")
        if failure is not None:
            if type(failure) is not RadioFailure:
                raise TypeError("expected normalized failure")
            self.last_failure = failure
        setattr(self, self.active_level, Level.SUCCEEDED if failure is None else Level.FAILED)
        self.active_level = None

    def interrupt(self):
        self._live()
        if self.active_level is not None:
            setattr(self, self.active_level, Level.FAILED)
            self.active_level = None

    def finish(self, state, at_us, *, safe=True):
        self._live()
        if self.active_level is not None:
            raise RuntimeError("active level cannot be finalized")
        t = self._trigger
        context = ReceiverRadioEpisodeContextV1(
            t.detail, self.reason, t.failure.outcome, self.soft, self.hard, state,
            checked_monotonic_elapsed(t.at_us, at_us),
            RadioFailureDetailV1.from_failure(State.RECOVERING, self.last_failure) if self.last_failure else None,
            self.last_failure.code if self.last_failure else None, t.sequence, t.bucket,
        )
        if t.failure.code is Code.INVALID_ARGUMENT or (t.operation is Operation.VALIDATE and self.reason is Reason.NONE):
            severity = Severity.FATAL
        elif state in (State.INITIALIZATION_FAILED, State.HARDWARE_MISSING, State.RECOVERY_EXHAUSTED) or not safe:
            severity = Severity.FATAL
        elif t.failure.code is Code.UNEXPECTED_IRQ and self.reason is Reason.NONE and state is not State.SHUTDOWN:
            severity = Severity.WARN
        else:
            severity = Severity.ERROR
        result = CompletedRadioEpisode(t.operation, t.failure.code, severity, t.at_us, context)
        self._finished = True
        return result
