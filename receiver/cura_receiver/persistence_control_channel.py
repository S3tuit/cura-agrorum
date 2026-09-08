"""Synchronous in-memory control mailbox; all filesystem work stays with its worker."""

from threading import current_thread

from .generated.receiver_enums_generated import DiagnosticOperation as Op
from .persistence_control_execution import (
    ControlCommand,
    ControlRequest,
    ControlCommandKind as Kind,
    ControlExecutionState as Phase,
)
from .persistence_control_values import (
    CommunicatorStateCommitDisposition as SD,
    CommunicatorStateCommitFailureKind as SF,
    CommunicatorStateCommitResult as StateResult,
    CommunicatorStateLoadStatus as LS,
    CommunicatorStateLoadResult as LoadResult,
    ReceiverCleanStopCommitDisposition as CD,
    ReceiverCleanStopCommitFailureKind as CF,
    ReceiverCleanStopCommitResult as CleanResult,
    ReceiverCleanStopV1,
    require_immutable_state,
)
from .receiver_configuration import (
    ReceiverConfigurationLoadResult as ConfigResult,
    ReceiverConfigurationLoadStatus as CS,
    PersistenceControlInterfaceViolation as Violation,
)


def control_failure(
    kind, reason, *, unknown=False, violation=Violation.NONE, evidence=None
):
    """Construct only command-specific closed results, never diagnostic records."""
    values = dict(interface_violation=violation, **(evidence or {}))
    if kind is Kind.CONFIGURATION:
        return ConfigResult(CS[reason], Op.READ, interface_violation=violation)
    if kind is Kind.LOAD_STATE:
        return LoadResult(LS[reason], Op.READ, **values)
    if kind is Kind.COMMIT_STATE:
        return StateResult(
            SD.OUTCOME_UNKNOWN if unknown else SD.NOT_INSTALLED,
            SF[reason],
            Op.WRITE,
            **values,
        )
    return CleanResult(
        CD.OUTCOME_UNKNOWN if unknown else CD.NOT_COMMITTED,
        CF[reason],
        Op.CLEANUP,
        **values,
    )


class PersistenceControlChannel:
    """One communicator caller; immutable requests/results, blocking completion events."""

    def __init__(self, worker):
        self._worker = worker
        self._owner = None

    def load_receiver_configuration(self, *, deadline_monotonic_us):
        return self._call(Kind.CONFIGURATION, None, deadline_monotonic_us)

    def load_communicator_state(self, *, deadline_monotonic_us):
        return self._call(Kind.LOAD_STATE, None, deadline_monotonic_us)

    def commit_communicator_state(self, state, *, deadline_monotonic_us):
        return self._call(Kind.COMMIT_STATE, state, deadline_monotonic_us)

    def commit_receiver_clean_stop(self, marker, *, deadline_monotonic_us):
        return self._call(Kind.CLEAN_STOP, marker, deadline_monotonic_us)

    def close(self):
        """Close new submissions; queued commands are completed by the disk owner."""
        with self._worker._scheduler_lock:
            self._worker._channel_closed = True
            self._worker._wake.set()

    def _wait_for_completion(self, command, remaining_seconds):
        return command.completion.wait(remaining_seconds)

    def _call(self, kind, payload, deadline):
        worker = self._worker
        with worker._scheduler_lock:
            if self._owner is None:
                self._owner = current_thread()
            if current_thread() is not self._owner:
                return control_failure(
                    kind, "INTERFACE_VIOLATION", violation=Violation.WRONG_CALLER
                )
        if type(deadline) is not int or not 0 <= deadline <= (1 << 64) - 1:
            return control_failure(
                kind, "INTERFACE_VIOLATION", violation=Violation.INVALID_DEADLINE
            )
        if kind is Kind.COMMIT_STATE:
            try:
                require_immutable_state(payload)
            except (TypeError, ValueError):
                return control_failure(
                    kind, "INTERFACE_VIOLATION", violation=Violation.INVALID_STATE
                )
        elif kind is Kind.CLEAN_STOP and type(payload) is not ReceiverCleanStopV1:
            return control_failure(
                kind, "INTERFACE_VIOLATION", violation=Violation.INVALID_ARGUMENT
            )
        if worker._clock.now_monotonic_us() >= deadline:
            return control_failure(kind, "DEADLINE_EXCEEDED")
        command = ControlCommand(ControlRequest(kind, deadline, payload), worker._clock)
        with worker._scheduler_lock:
            if worker._channel_closed:
                return control_failure(kind, "CHANNEL_CLOSED")
            worker._mailbox.append(command)
            worker._wake.set()
        remaining = max(0, deadline - worker._clock.now_monotonic_us()) / 1_000_000
        self._wait_for_completion(command, remaining)
        phase, result = command.expire()
        if phase is Phase.DONE:
            return result
        return control_failure(
            kind, "DEADLINE_EXCEEDED", unknown=phase is Phase.COMMIT_MAY_HAVE_RUN
        )
