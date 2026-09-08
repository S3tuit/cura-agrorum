from dataclasses import FrozenInstanceError, replace

import pytest

from cura_receiver.generated.receiver_enums_generated import DiagnosticOperation as Op
from cura_receiver.persistence_control_values import (
    CommunicatorStateCommitDisposition as D,
    CommunicatorStateCommitFailureKind as F,
    CommunicatorStateCommitResult as Result,
    CommunicatorStateCondition as Condition,
    CommunicatorStateLoadResult as Load,
    CommunicatorStateLoadStatus as Status,
    ReceiverCleanStopV1,
    ReceiverCleanStopCommitDisposition as StopD,
    ReceiverCleanStopCommitFailureKind as StopF,
    ReceiverCleanStopCommitResult as Stop,
)
from cura_receiver.receiver_configuration import (
    PersistenceControlInterfaceViolation as Violation,
)


# Operational uncertainty retains its precise failure evidence in an immutable result.
def test_unknown_control_result_is_bounded_and_frozen():
    result = Result(
        D.OUTCOME_UNKNOWN,
        F.DATABASE_ERROR,
        Op.WRITE,
        sqlite_primary_code=10,
        sqlite_extended_code=266,
        os_errno=5,
    )
    assert (
        result.sqlite_primary_code,
        result.sqlite_extended_code,
        result.os_errno,
    ) == (10, 266, 5)
    with pytest.raises(FrozenInstanceError):
        result.disposition = D.COMMITTED
    with pytest.raises(ValueError):
        replace(result, os_errno=OSError("secret path"))
    with pytest.raises(ValueError):
        replace(result, sqlite_primary_code=True)


# Impossible status/effect combinations cannot cross the channel as legitimate outcomes.
@pytest.mark.parametrize(
    "changes",
    [
        {"failure_kind": F.NONE},
        {"disposition": D.COMMITTED},
        {"operation": Op.READ},
        {"interface_violation": Violation.INVALID_STATE},
        {"state_condition": Condition.CORRUPT},
        {"sqlite_extended_code": 1 << 31},
        {"failure_kind": F.STATE_UNAVAILABLE, "state_condition": Condition.CORRUPT},
    ],
)
def test_invalid_commit_combinations(changes):
    with pytest.raises((ValueError, TypeError)):
        replace(Result(D.OUTCOME_UNKNOWN, F.DEADLINE_EXCEEDED, Op.WRITE), **changes)


# Definite interface failures and successful idempotency use the command's exact operation.
def test_clean_stop_and_state_result_operations():
    state = Result(
        D.NOT_INSTALLED,
        F.INTERFACE_VIOLATION,
        Op.WRITE,
        interface_violation=Violation.GENERATION_GAP,
    )
    marker = Stop(
        StopD.NOT_COMMITTED,
        StopF.INTERFACE_VIOLATION,
        Op.CLEANUP,
        interface_violation=Violation.CLEAN_STOP_PRECONDITION,
    )
    assert state.operation is Op.WRITE and marker.operation is Op.CLEANUP
    assert Stop(StopD.ALREADY_COMMITTED, StopF.NONE, Op.NONE).operation is Op.NONE
    with pytest.raises(ValueError):
        Stop(StopD.COMMITTED, StopF.NONE, Op.CLEANUP)


# Failed loads carry a single state condition and cannot retain a state payload.
def test_load_field_presence():
    result = Load(Status.STATE_UNAVAILABLE, Op.READ, state_condition=Condition.MISSING)
    assert result.state is None
    with pytest.raises(ValueError):
        replace(result, state_condition=Condition.NONE)
    with pytest.raises(ValueError):
        replace(result, state=object())
    with pytest.raises(ValueError):
        Load(Status.LOADED, Op.NONE)


# Clean-stop requests cannot retain a mutable identity or values outside SQLite's range.
@pytest.mark.parametrize(
    "changes",
    [
        {"receiver_instance_id": bytearray(16)},
        {"receiver_instance_id": b"short"},
        {"stopped_at_monotonic_us": True},
        {"stopped_at_monotonic_us": -1},
        {"communicator_state_generation": 1 << 63},
    ],
)
def test_clean_stop_request_bounds(changes):
    with pytest.raises((TypeError, ValueError)):
        replace(ReceiverCleanStopV1(bytes(16), 1, 0), **changes)
