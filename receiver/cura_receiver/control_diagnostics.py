"""Normalized synchronous-control failure evidence owned by the communicator."""

from dataclasses import dataclass
import struct

from .generated import receiver_enums_generated as E
from .generated.receiver_entities_generated import DiagnosticV1
from .time_diagnostics import integer

_CONTEXT = struct.Struct('<H6BHHiiiQQQqQ')
_FIELDS = ('disposition', 'state_condition', 'protocol_rejection_code', 'sqlite_primary_code',
           'sqlite_extended_code', 'os_errno', 'requested_generation',
           'authoritative_generation_before', 'related_occurrence_sequence', 'airtime_bucket_expiration_utc_us')


@dataclass(frozen=True, slots=True)
class ReceiverPersistenceControlContextV1:
    control_command: E.PersistenceControlCommand
    purpose: E.PersistenceControlPurpose
    failure_kind: E.PersistenceControlFailureKind
    operation_duration_us: int
    flags: int = 0
    disposition_kind: E.PersistenceControlDispositionKind | None = None
    disposition: E.PersistenceControlDisposition | None = None
    state_condition: E.PersistenceControlStateCondition | None = None
    protocol_rejection_code: int | None = None
    sqlite_primary_code: int | None = None
    sqlite_extended_code: int | None = None
    os_errno: int | None = None
    requested_generation: int | None = None
    authoritative_generation_before: int | None = None
    related_occurrence_sequence: int | None = None
    airtime_bucket_expiration_utc_us: int | None = None

    def __post_init__(self):
        for name, kind in (('control_command', E.PersistenceControlCommand), ('purpose', E.PersistenceControlPurpose),
                           ('failure_kind', E.PersistenceControlFailureKind)):
            value = getattr(self, name)
            if type(value) is not kind or value.value == 0:
                raise ValueError('undefined control context enum')
        integer(self.flags, 0, 63)
        integer(self.operation_duration_us)
        command = self.control_command
        mutating = command in (E.PersistenceControlCommand.COMMIT_COMMUNICATOR_STATE, E.PersistenceControlCommand.COMMIT_RECEIVER_CLEAN_STOP)
        if bool(self.flags & 1) != mutating or mutating != (self.disposition is not None):
            raise ValueError('mutating/disposition mismatch')
        if (self.disposition is None) != (self.disposition_kind is None):
            raise ValueError('incomplete disposition')
        if self.disposition is not None:
            expected = (E.PersistenceControlDispositionKind.COMMUNICATOR_STATE_COMMIT
                        if command is E.PersistenceControlCommand.COMMIT_COMMUNICATOR_STATE
                        else E.PersistenceControlDispositionKind.RECEIVER_CLEAN_STOP_COMMIT)
            if self.disposition_kind is not expected or type(self.disposition) is not E.PersistenceControlDisposition or not self.disposition.value:
                raise ValueError('invalid disposition')
        if self.flags & 2 and self.disposition is not E.PersistenceControlDisposition.OUTCOME_UNKNOWN:
            raise ValueError('commit uncertainty requires unknown outcome')
        if (self.state_condition is not None) != (self.failure_kind is E.PersistenceControlFailureKind.STATE_UNAVAILABLE):
            raise ValueError('state condition does not match failure')
        if self.state_condition is not None and (type(self.state_condition) is not E.PersistenceControlStateCondition or not self.state_condition.value):
            raise ValueError('invalid state condition')
        if command is E.PersistenceControlCommand.LOAD_RECEIVER_CONFIGURATION and self.purpose is not E.PersistenceControlPurpose.STARTUP_CONFIGURATION:
            raise ValueError('configuration purpose mismatch')
        if command is E.PersistenceControlCommand.LOAD_COMMUNICATOR_STATE and self.purpose not in (E.PersistenceControlPurpose.STARTUP_STATE, E.PersistenceControlPurpose.RECONCILIATION):
            raise ValueError('state-load purpose mismatch')
        if command is E.PersistenceControlCommand.COMMIT_RECEIVER_CLEAN_STOP and self.purpose is not E.PersistenceControlPurpose.CLEAN_STOP:
            raise ValueError('clean-stop purpose mismatch')
        if command is E.PersistenceControlCommand.COMMIT_COMMUNICATOR_STATE and self.purpose not in (
                E.PersistenceControlPurpose.AIRTIME_BUCKET_GRANT, E.PersistenceControlPurpose.AIRTIME_BUCKET_SETTLEMENT,
                E.PersistenceControlPurpose.RTC_PROVENANCE, E.PersistenceControlPurpose.AIRTIME_HISTORY_RECOVERY):
            raise ValueError('state mutation purpose mismatch')
        if self.protocol_rejection_code is not None:
            integer(self.protocol_rejection_code, 0, 65535)
        for value in (self.sqlite_primary_code, self.sqlite_extended_code, self.os_errno):
            if value is not None:
                integer(value, -(1 << 31), (1 << 31) - 1)
        if self.sqlite_extended_code is not None and self.sqlite_primary_code is None:
            raise ValueError('extended SQLite evidence requires primary')
        for value in (self.requested_generation, self.authoritative_generation_before, self.related_occurrence_sequence):
            if value is not None:
                integer(value)
        if self.airtime_bucket_expiration_utc_us is not None:
            integer(self.airtime_bucket_expiration_utc_us, -(1 << 63), (1 << 63) - 1)


def encode_control_context(c):
    if type(c) is not ReceiverPersistenceControlContextV1:
        raise TypeError('control context required')
    c.__post_init__()
    mask = sum(1 << i for i, name in enumerate(_FIELDS) if getattr(c, name) is not None)
    def number(value):
        return 0 if value is None else value.value if hasattr(value, 'value') else value
    return _CONTEXT.pack(mask, c.control_command.value, c.purpose.value,
        number(c.disposition_kind), number(c.disposition), c.failure_kind.value, number(c.state_condition),
        number(c.protocol_rejection_code), c.flags, number(c.sqlite_primary_code), number(c.sqlite_extended_code),
        number(c.os_errno), number(c.requested_generation), number(c.authoritative_generation_before),
        number(c.related_occurrence_sequence), number(c.airtime_bucket_expiration_utc_us), c.operation_duration_us)


def decode_control_context(data):
    if type(data) is not bytes or len(data) != 64:
        raise ValueError('control context requires 64 bytes')
    mask, command, purpose, disposition_kind, disposition, failure, condition, rejection, flags, *rest = _CONTEXT.unpack(data)
    if mask & ~1023:
        raise ValueError('reserved control validity bits')
    optional = [disposition, condition, rejection, *rest[:-1]]
    for i, value in enumerate(optional):
        if not mask & (1 << i) and value:
            raise ValueError('nonzero absent field')
    if not mask & 1 and disposition_kind:
        raise ValueError('nonzero absent disposition kind')
    values = {name: value if mask & (1 << i) else None for i, (name, value) in enumerate(zip(_FIELDS, optional))}
    if values['disposition'] is not None:
        values['disposition'] = E.PersistenceControlDisposition(values['disposition'])
    if values['state_condition'] is not None:
        values['state_condition'] = E.PersistenceControlStateCondition(values['state_condition'])
    c = ReceiverPersistenceControlContextV1(E.PersistenceControlCommand(command), E.PersistenceControlPurpose(purpose),
        E.PersistenceControlFailureKind(failure), rest[-1], flags,
        E.PersistenceControlDispositionKind(disposition_kind) if mask & 1 else None, **values)
    if encode_control_context(c) != data:
        raise ValueError('noncanonical control context')
    return c


@dataclass(frozen=True, slots=True)
class ControlFailureEpisode:
    error_code: E.PersistenceControlDiagnosticErrorCode
    operation: E.DiagnosticOperation
    severity: E.DiagnosticSeverity
    sampled_at_monotonic_us: int
    context: ReceiverPersistenceControlContextV1

    def __post_init__(self):
        if type(self.error_code) is not E.PersistenceControlDiagnosticErrorCode or not self.error_code.value:
            raise ValueError('invalid control error')
        code = self.error_code.name
        allowed = (('READ',) if code in ('CONFIGURATION_REJECTED', 'HOST_IDENTITY_REJECTED', 'STATE_MISSING', 'STATE_CORRUPT')
                   else ('READ', 'WRITE') if code in ('UNSUPPORTED_STATE_VERSION', 'STATE_POLICY_MISMATCH')
                   else ('READ', 'WRITE', 'CLEANUP'))
        if type(self.operation) is not E.DiagnosticOperation or self.operation.name not in allowed:
            raise ValueError('invalid control error/operation')
        if self.severity not in (E.DiagnosticSeverity.WARN, E.DiagnosticSeverity.ERROR, E.DiagnosticSeverity.FATAL):
            raise ValueError('invalid control severity')
        integer(self.sampled_at_monotonic_us)
        encode_control_context(self.context)


def control_failure(result, *, command, purpose, started, finished, fatal=False, requested_generation=None,
                    authoritative_generation_before=None, related_occurrence_sequence=None,
                    airtime_bucket_expiration_utc_us=None):
    """Normalize an operational result; interface violations belong to CORE."""
    if result.interface_violation.name != 'NONE':
        raise ValueError('caller violation requires CORE')
    failure = result.failure_kind.name if hasattr(result, 'failure_kind') else result.status.name
    if failure in ('NONE', 'LOADED'):
        return None
    normalized = 'IO_ERROR' if failure == 'OS_ERROR' else failure
    condition = getattr(result, 'state_condition', None)
    code = ({'MISSING': 'STATE_MISSING', 'CORRUPT': 'STATE_CORRUPT', 'UNSUPPORTED_VERSION': 'UNSUPPORTED_STATE_VERSION',
             'POLICY_MISMATCH': 'STATE_POLICY_MISMATCH'}[condition.name] if failure == 'STATE_UNAVAILABLE' else
            {'CONFIGURATION_REJECTED': 'CONFIGURATION_REJECTED', 'HOST_IDENTITY_REJECTED': 'HOST_IDENTITY_REJECTED',
             'OS_ERROR': 'IO', 'DATABASE_ERROR': 'DATABASE', 'DEADLINE_EXCEEDED': 'DEADLINE', 'CHANNEL_CLOSED': 'CHANNEL_CLOSED'}[failure])
    disposition = getattr(result, 'disposition', None)
    kind = value = None
    flags = 0
    if disposition is not None:
        flags |= 1
        kind = (E.PersistenceControlDispositionKind.COMMUNICATOR_STATE_COMMIT
                if command is E.PersistenceControlCommand.COMMIT_COMMUNICATOR_STATE
                else E.PersistenceControlDispositionKind.RECEIVER_CLEAN_STOP_COMMIT)
        value = E.PersistenceControlDisposition[{'NOT_INSTALLED': 'DEFINITELY_NOT_COMMITTED', 'NOT_COMMITTED': 'DEFINITELY_NOT_COMMITTED'}.get(disposition.name, disposition.name)]
        if value is E.PersistenceControlDisposition.OUTCOME_UNKNOWN:
            flags |= 2
    if purpose is E.PersistenceControlPurpose.RECONCILIATION:
        flags |= 4
    if purpose in (E.PersistenceControlPurpose.STARTUP_CONFIGURATION, E.PersistenceControlPurpose.STARTUP_STATE):
        flags |= 8
    if purpose is E.PersistenceControlPurpose.CLEAN_STOP:
        flags |= 16
    rejection = getattr(result, 'protocol_rejection', None)
    rejection = rejection.value if rejection is not None and type(rejection.value) is int else None
    from .elapsed_duration import checked_monotonic_elapsed
    context = ReceiverPersistenceControlContextV1(command, purpose, E.PersistenceControlFailureKind[normalized],
        checked_monotonic_elapsed(started, finished), flags, kind, value,
        E.PersistenceControlStateCondition[condition.name] if failure == 'STATE_UNAVAILABLE' else None,
        rejection, getattr(result, 'sqlite_primary_code', None), getattr(result, 'sqlite_extended_code', None),
        getattr(result, 'os_errno', None), requested_generation, authoritative_generation_before,
        related_occurrence_sequence, airtime_bucket_expiration_utc_us)
    severity = E.DiagnosticSeverity.FATAL if fatal else E.DiagnosticSeverity.WARN if code == 'STATE_MISSING' else E.DiagnosticSeverity.ERROR
    operation = E.DiagnosticOperation.CLEANUP if purpose is E.PersistenceControlPurpose.CLEAN_STOP else E.DiagnosticOperation.WRITE if disposition is not None else E.DiagnosticOperation.READ
    return ControlFailureEpisode(E.PersistenceControlDiagnosticErrorCode[code], operation, severity, finished, context)


def control_diagnostic(episode, *, receiver_instance_id, diagnostic_sequence):
    if type(episode) is not ControlFailureEpisode:
        raise TypeError('control episode required')
    if type(receiver_instance_id) is not bytes or len(receiver_instance_id) != 16:
        raise ValueError('invalid receiver identity')
    integer(diagnostic_sequence, 1, (1 << 63) - 1)
    return DiagnosticV1(receiver_instance_id, diagnostic_sequence, episode.sampled_at_monotonic_us,
        episode.severity, E.DiagnosticErrorDomain.PERSISTENCE_CONTROL, episode.operation,
        episode.error_code.value, 1, 64, encode_control_context(episode.context) + bytes(64))


class ControlEpisodeTracker:
    """One outstanding complete-state command; observations never touch the queue.

    The application drains completed evidence after each bounded policy action.
    An unknown command and its reconciliation retain one original root.
    """

    def __init__(self):
        self._root = None
        self._root_reported = False
        self._ready = ()
        self._last_reconciliation_failure = None

    def _complete(self, episode):
        if episode is not None:
            if len(self._ready) >= 4:
                raise RuntimeError('control evidence must be drained at each scheduling boundary')
            self._ready += (episode,)

    def __call__(self, event):
        result = event.result
        if result.interface_violation.name != 'NONE':
            from .core_diagnostics import CoreFault
            raise CoreFault(E.CoreDiagnosticErrorCode.PERSISTENCE_CONTROL_CONTRACT, result.operation,
                E.CorePhase.CONTROL_OPERATION,
                E.CoreFailureStage.LOAD_STATE if event.command is E.PersistenceControlCommand.LOAD_COMMUNICATOR_STATE else E.CoreFailureStage.COMMIT_STATE,
                detail_kind=E.CoreDetailKind.PERSISTENCE_CONTROL_VIOLATION,
                detail_code=E.PersistenceControlViolationDetailCode[result.interface_violation.name].value)
        episode = control_failure(result, command=event.command, purpose=event.purpose,
            started=event.started, finished=event.finished,
            requested_generation=event.requested_generation,
            authoritative_generation_before=event.authoritative_generation_before,
            airtime_bucket_expiration_utc_us=event.bucket_expiration_utc_us)
        reconciling = event.purpose is E.PersistenceControlPurpose.RECONCILIATION
        if self._root is not None:
            if not self._root_reported:
                self._complete(self._root)
                self._root_reported = True
            if episode is not None and episode.error_code is not self._root.error_code:
                signature = (episode.error_code, episode.context.state_condition)
                if signature != self._last_reconciliation_failure:
                    self._complete(episode)
                    self._last_reconciliation_failure = signature
            if event.reconciliation_complete:
                self._root = None
                self._root_reported = False
                self._last_reconciliation_failure = None
            return
        if episode is None:
            return
        if episode.context.disposition is E.PersistenceControlDisposition.OUTCOME_UNKNOWN and not reconciling:
            self._root = episode
            self._root_reported = False
        else:
            self._complete(episode)

    def take_ready(self, *, terminal=False):
        if terminal and self._root is not None and not self._root_reported:
            self._complete(self._root)
            self._root_reported = True
        ready, self._ready = self._ready, ()
        return ready
