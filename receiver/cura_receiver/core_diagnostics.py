"""Closed CORE context encoding; no exception text crosses this boundary."""

from dataclasses import dataclass
import struct

from .generated import receiver_enums_generated as E
from .generated.receiver_entities_generated import DiagnosticV1
from .time_diagnostics import integer

_CONTEXT = struct.Struct('<HBBBBHiiQQQQqQ')
_GENERIC_OPERATIONS = tuple(op.name for op in E.DiagnosticOperation if op is not E.DiagnosticOperation.NONE)
_ALLOWED = {
    E.CoreDiagnosticErrorCode.INVALID_ARGUMENT: ('VALIDATE',),
    E.CoreDiagnosticErrorCode.INVALID_STATE: ('INITIALIZE', 'VALIDATE', 'CLEANUP'),
    E.CoreDiagnosticErrorCode.REPRESENTATION_INVARIANT: ('ENCODE', 'DECODE', 'APPEND'),
    E.CoreDiagnosticErrorCode.PERSIST_QUEUE_CONTRACT: ('APPEND', 'CLEANUP'),
    E.CoreDiagnosticErrorCode.PERSISTENCE_CONTROL_CONTRACT: ('READ', 'WRITE', 'CLEANUP'),
    E.CoreDiagnosticErrorCode.MEMORY_EXHAUSTED: _GENERIC_OPERATIONS,
    E.CoreDiagnosticErrorCode.CODEC_BACKEND: ('ENCODE', 'DECODE'),
    E.CoreDiagnosticErrorCode.CRYPTO_BACKEND: ('ENCODE', 'DECODE'),
    E.CoreDiagnosticErrorCode.ARITHMETIC_RANGE: ('VALIDATE', 'ENCODE', 'DECODE'),
    E.CoreDiagnosticErrorCode.UNEXPECTED_EXCEPTION: _GENERIC_OPERATIONS,
}


@dataclass(frozen=True, slots=True)
class ReceiverCoreFailureContextV1:
    phase: E.CorePhase
    stage: E.CoreFailureStage
    operation_duration_us: int
    flags: int = 0
    related_entity_kind: E.PersistQueueEntityKind | None = None
    detail_kind: E.CoreDetailKind | None = None
    detail_code: int | None = None
    os_errno: int | None = None
    related_occurrence_sequence: int | None = None
    related_health_sequence: int | None = None
    related_clock_observation_sequence: int | None = None
    communicator_state_generation: int | None = None
    airtime_bucket_expiration_utc_us: int | None = None

    def __post_init__(self):
        for value, kind in ((self.phase, E.CorePhase), (self.stage, E.CoreFailureStage)):
            if type(value) is not kind or value.value == 0:
                raise ValueError('invalid core phase/stage')
        integer(self.flags, 0, 255)
        integer(self.operation_duration_us)
        if bool(self.flags & 1) != (self.related_entity_kind is not None):
            raise ValueError('entity flag does not match context')
        if self.related_entity_kind is not None and type(self.related_entity_kind) is not E.PersistQueueEntityKind:
            raise ValueError('invalid entity kind')
        if (self.detail_kind is None) != (self.detail_code is None):
            raise ValueError('incomplete core detail')
        if self.detail_kind is not None:
            catalogues = {E.CoreDetailKind.PERSIST_QUEUE_VIOLATION: E.PersistQueueViolationDetailCode,
                          E.CoreDetailKind.PERSISTENCE_CONTROL_VIOLATION: E.PersistenceControlViolationDetailCode}
            if self.detail_kind not in catalogues:
                raise ValueError('invalid core detail kind')
            integer(self.detail_code, 1, (1 << 31) - 1)
            catalogues[self.detail_kind](self.detail_code)
        if self.os_errno is not None:
            integer(self.os_errno, -(1 << 31), (1 << 31) - 1)
        for value in (self.related_occurrence_sequence, self.related_health_sequence,
                      self.related_clock_observation_sequence, self.communicator_state_generation):
            if value is not None:
                integer(value)
        if self.airtime_bucket_expiration_utc_us is not None:
            integer(self.airtime_bucket_expiration_utc_us, -(1 << 63), (1 << 63) - 1)


def encode_core_context(c):
    if type(c) is not ReceiverCoreFailureContextV1:
        raise TypeError('core context required')
    c.__post_init__()
    optional = (c.detail_code, c.os_errno, c.related_occurrence_sequence, c.related_health_sequence,
                c.related_clock_observation_sequence, c.communicator_state_generation, c.airtime_bucket_expiration_utc_us)
    mask = sum(1 << i for i, v in enumerate(optional) if v is not None)
    return _CONTEXT.pack(mask, c.phase.value, c.stage.value,
        0 if c.detail_kind is None else c.detail_kind.value,
        0 if c.related_entity_kind is None else c.related_entity_kind.value,
        c.flags, *(0 if v is None else v for v in optional), c.operation_duration_us)


def decode_core_context(data):
    if type(data) is not bytes or len(data) != 64:
        raise ValueError('core context requires 64 bytes')
    mask, phase, stage, detail_kind, entity, flags, *values = _CONTEXT.unpack(data)
    if mask & ~127:
        raise ValueError('reserved validity bits')
    optional = values[:-1]
    for i, value in enumerate(optional):
        if not mask & (1 << i) and value:
            raise ValueError('nonzero absent field')
    if not mask & 1 and detail_kind:
        raise ValueError('nonzero absent detail kind')
    c = ReceiverCoreFailureContextV1(
        E.CorePhase(phase), E.CoreFailureStage(stage), values[-1], flags,
        E.PersistQueueEntityKind(entity) if flags & 1 else None,
        E.CoreDetailKind(detail_kind) if mask & 1 else None,
        *(value if mask & (1 << i) else None for i, value in enumerate(optional)))
    if encode_core_context(c) != data:
        raise ValueError('noncanonical core context')
    return c


@dataclass(frozen=True, slots=True)
class CoreFailureEpisode:
    error_code: E.CoreDiagnosticErrorCode
    operation: E.DiagnosticOperation
    sampled_at_monotonic_us: int
    context: ReceiverCoreFailureContextV1

    def __post_init__(self):
        if type(self.error_code) is not E.CoreDiagnosticErrorCode or self.error_code not in _ALLOWED:
            raise ValueError('invalid core error')
        if type(self.operation) is not E.DiagnosticOperation or self.operation.name not in _ALLOWED[self.error_code]:
            raise ValueError('invalid core operation')
        integer(self.sampled_at_monotonic_us)
        encode_core_context(self.context)


def core_diagnostic(episode, *, receiver_instance_id, diagnostic_sequence):
    if type(episode) is not CoreFailureEpisode:
        raise TypeError('core episode required')
    if type(receiver_instance_id) is not bytes or len(receiver_instance_id) != 16:
        raise ValueError('invalid receiver identity')
    integer(diagnostic_sequence, 1, (1 << 63) - 1)
    return DiagnosticV1(receiver_instance_id, diagnostic_sequence, episode.sampled_at_monotonic_us,
        E.DiagnosticSeverity.FATAL, E.DiagnosticErrorDomain.CORE, episode.operation,
        episode.error_code.value, 1, 64, encode_core_context(episode.context) + bytes(64))


class CoreFault(RuntimeError):
    """A normalized internal failure; source exception remains only in __cause__."""

    def __init__(self, code, operation, phase, stage, *, detail_kind=None, detail_code=None):
        super().__init__('receiver core failure')
        self.code, self.operation, self.phase, self.stage = code, operation, phase, stage
        self.detail_kind, self.detail_code = detail_kind, detail_code


def exception_episode(error, *, phase, stage, operation, started, finished, safe_radio=False,
                      related_occurrence_sequence=None, profile_published=False,
                      related_entity_kind=None, occurrence_accepted=False, ack_selected=False,
                      tx_may_have_started=False, airtime_grant_outstanding=False,
                      communicator_state_generation=None, airtime_bucket_expiration_utc_us=None):
    """None means no sound diagnostic allocation/admission is possible."""
    from .persist_queue import PersistQueueInterfaceError
    from .elapsed_duration import checked_monotonic_elapsed
    if isinstance(error, MemoryError):
        return None
    flags = (128 if safe_radio else 0) | (32 if profile_published else 0)
    flags |= (1 if related_entity_kind is not None else 0) | (2 if airtime_grant_outstanding else 0)
    flags |= (4 if occurrence_accepted else 0) | (8 if ack_selected else 0) | (16 if tx_may_have_started else 0)
    detail_kind = detail_code = None
    if isinstance(error, PersistQueueInterfaceError):
        if not error.queue_known_sound:
            return None
        flags |= 64
        code, operation = E.CoreDiagnosticErrorCode.PERSIST_QUEUE_CONTRACT, E.DiagnosticOperation.APPEND
        if error.detail_code is not None:
            detail_kind, detail_code = E.CoreDetailKind.PERSIST_QUEUE_VIOLATION, error.detail_code.value
    elif isinstance(error, CoreFault):
        code, operation, phase, stage = error.code, error.operation, error.phase, error.stage
        detail_kind, detail_code = error.detail_kind, error.detail_code
    elif isinstance(error, OverflowError):
        code, operation = E.CoreDiagnosticErrorCode.ARITHMETIC_RANGE, E.DiagnosticOperation.VALIDATE
    else:
        code = E.CoreDiagnosticErrorCode.UNEXPECTED_EXCEPTION
    context = ReceiverCoreFailureContextV1(phase, stage, checked_monotonic_elapsed(started, finished),
        flags=flags, detail_kind=detail_kind, detail_code=detail_code,
        related_occurrence_sequence=related_occurrence_sequence, related_entity_kind=related_entity_kind,
        communicator_state_generation=communicator_state_generation,
        airtime_bucket_expiration_utc_us=airtime_bucket_expiration_utc_us)
    return CoreFailureEpisode(code, operation, finished, context)
