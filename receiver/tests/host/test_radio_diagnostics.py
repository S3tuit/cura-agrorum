from dataclasses import FrozenInstanceError, replace

import pytest

from cura_receiver.generated import receiver_enums_generated as E
from cura_receiver.ports.radio import RadioFailure
from cura_receiver.radio_diagnostics import (
    Code, CompletedRadioEpisode, Level, Operation, RadioEpisodeBuilder,
    RadioFailureDetailV1, Reason, ReceiverRadioEpisodeContextV1, Severity, State,
    decode_failure_detail, decode_radio_context, decode_radio_diagnostic,
    encode_failure_detail, encode_radio_context, radio_diagnostic,
)


def context(**changes):
    values = dict(
        trigger_detail=RadioFailureDetailV1(State.RX_SINGLE, 0, E.RadioFailureStage.WAIT_IRQ),
        recovery_reason=Reason.NONE, trigger_command_outcome=E.RadioCommandOutcome.NOT_APPLICABLE,
        soft_recovery_result=Level.NOT_APPLICABLE, hard_recovery_result=Level.NOT_APPLICABLE,
        terminal_state=State.RX_SINGLE, episode_duration_us=9,
    )
    return ReceiverRadioEpisodeContextV1(**(values | changes))


def builder(**changes):
    return RadioEpisodeBuilder(
        Operation.TRANSMIT, RadioFailure(Code.BUSY_TIMEOUT, E.RadioFailureStage.WAIT_BUSY,
        E.RadioCommandOutcome.UNCERTAIN, 0x83, hardware_touched=True), State.TX_ACTIVE, 100,
        **changes,
    )


# Reviewed literal fields fix both 14-byte slots and every offset of the 64-byte context.
def test_literal_context():
    trigger = RadioFailureDetailV1(State.TX_ACTIVE, 0x83, E.RadioFailureStage.WRITE_COMMAND,
        E.RadioBackendStatusKind.ERRNO, 5, 0x64, 1, 0x20, True)
    last = RadioFailureDetailV1(State.RECOVERING, 0xC0, E.RadioFailureStage.READ_COMMAND,
        E.RadioBackendStatusKind.SX1262_DRIVER_STATUS, -7, 0x28, None, None, True)
    value = context(trigger_detail=trigger, recovery_reason=Reason.TX_OUTCOME_UNCERTAIN,
        trigger_command_outcome=E.RadioCommandOutcome.UNCERTAIN,
        soft_recovery_result=Level.FAILED, hard_recovery_result=Level.SUCCEEDED,
        last_recovery_failure_detail=last, last_recovery_error_code=Code.COMMAND_STATUS,
        related_occurrence_sequence=0x0102030405060708, airtime_bucket_expiration_utc_us=-2,
        episode_duration_us=0x1112131415161718)
    expected = bytes.fromhex(
        "0483080f01050000006401002000"
        "05c0090902f9ffffff2800000000"
        "070004030302020005000000"
        "0807060504030201feffffffffffffff1817161514131211"
    )
    assert len(expected) == 64
    assert encode_radio_context(value) == expected
    assert decode_radio_context(expected) == value


# Failed-startup cleanup uses the same closed catalogue, with fatal severity and original terminal.
@pytest.mark.parametrize("state", [State.INITIALIZATION_FAILED, State.HARDWARE_MISSING])
@pytest.mark.parametrize("code", [Code.IO, Code.BUSY_TIMEOUT, Code.COMMAND_STATUS,
    Code.DEADLINE, Code.UNEXPECTED_IRQ, Code.DEVICE_ERROR, Code.MALFORMED_RESPONSE])
def test_failed_startup_cleanup_catalogue(state, code):
    value = context(terminal_state=state)
    episode = CompletedRadioEpisode(Operation.CLEANUP, code, Severity.FATAL, 100, value)
    diagnostic = radio_diagnostic(episode, receiver_instance_id=bytes(16), diagnostic_sequence=1)
    assert decode_radio_diagnostic(diagnostic) == episode
    for severity in (Severity.WARN, Severity.ERROR):
        with pytest.raises(ValueError):
            replace(episode, severity=severity)


# Extending cleanup to startup does not authorize other operational or recovery terminals.
@pytest.mark.parametrize("state", [State.INITIALIZING, State.RX_EVENT_PENDING, State.RECOVERY_EXHAUSTED])
def test_cleanup_rejects_unrelated_terminal(state):
    with pytest.raises(ValueError):
        CompletedRadioEpisode(Operation.CLEANUP, Code.IO, Severity.FATAL, 100, context(terminal_state=state))


# Absence zeroes complete fields; a present zero-valued bucket stays distinguishable.
@pytest.mark.parametrize("sequence", [None, 1, (1 << 64) - 1])
@pytest.mark.parametrize("bucket", [None, 0, -(1 << 63), (1 << 63) - 1])
def test_optional_correlations(sequence, bucket):
    value = context(related_occurrence_sequence=sequence, airtime_bucket_expiration_utc_us=bucket)
    data = encode_radio_context(value)
    assert data[14:28] == bytes(14)
    assert data[28] == (2 if sequence is not None else 0) + (4 if bucket is not None else 0)
    assert decode_radio_context(data) == value


# All operation/code pairs are independently enumerated; unused operations are rejected.
@pytest.mark.parametrize("operation", list(Operation))
@pytest.mark.parametrize("code", list(Code))
def test_complete_catalogue_matrix(operation, code):
    allowed = (
        code.value in (1, 2) and operation.name == "VALIDATE"
        or code.value in range(3, 10) and operation.name in {"INITIALIZE", "TRANSMIT", "RECEIVE", "RECOVER", "CLEANUP"}
    )
    state = State.INITIALIZATION_FAILED if operation is Operation.INITIALIZE else State.SHUTDOWN if operation in (Operation.VALIDATE, Operation.CLEANUP) else State.RX_SINGLE
    severity = Severity.FATAL if operation in (Operation.INITIALIZE, Operation.VALIDATE) else Severity.WARN if code is Code.UNEXPECTED_IRQ and state is State.RX_SINGLE else Severity.ERROR
    if not allowed:
        with pytest.raises(ValueError):
            CompletedRadioEpisode(operation, code, severity, 100, context(terminal_state=state))
    else:
        episode = CompletedRadioEpisode(operation, code, severity, 100, context(terminal_state=state))
        entity = radio_diagnostic(episode, receiver_instance_id=bytes(range(16)), diagnostic_sequence=1)
        assert entity.context_length == 64
        assert entity.context[64:] == bytes(64)
        assert decode_radio_diagnostic(entity) == episode


# Each reserved bit, absent nonzero field and undefined enum is rejected at decode.
@pytest.mark.parametrize("offset,value", [
    (0, 0), (0, 10), (2, 19), (3, 16), (3, 32), (3, 64), (3, 128),
    (4, 3), (5, 1), (9, 1), (10, 1), (12, 1), (14, 1),
    (28, 8), (29, 1), (30, 9), (31, 4), (32, 4), (33, 4),
    (34, 0), (34, 10), (35, 1), (36, 1), (38, 1), (39, 1), (40, 1), (48, 1),
])
def test_context_rejects_invalid_bytes(offset, value):
    data = bytearray(encode_radio_context(context()))
    data[offset] = value
    with pytest.raises(ValueError):
        decode_radio_context(bytes(data))


# A stored entity cannot bypass the domain, schema, length, padding or scalar checks.
@pytest.mark.parametrize("changes", [
    {"error_domain": E.DiagnosticErrorDomain.TIME}, {"error_domain": 1},
    {"context_schema": 0}, {"context_schema": 2}, {"context_schema": True},
    {"context_length": 63}, {"context": bytes(127)}, {"context": bytes(127) + b"x"},
    {"error_code": 10}, {"error_code": True}, {"operation": 11}, {"severity": 2},
    {"diagnostic_sequence": 0}, {"receiver_instance_id": bytes(15)},
])
def test_entity_validation(changes):
    episode = CompletedRadioEpisode(Operation.RECEIVE, Code.IO, Severity.ERROR, 100, context())
    entity = radio_diagnostic(episode, receiver_instance_id=bytes(16), diagnostic_sequence=1)
    with pytest.raises((TypeError, ValueError)):
        decode_radio_diagnostic(replace(entity, **changes))


# Direct warnings are restricted to successfully handled unexpected IRQs.
@pytest.mark.parametrize("code", [Code.IO, Code.BUSY_TIMEOUT, Code.DEADLINE, Code.INVALID_ARGUMENT])
@pytest.mark.parametrize("severity", [Severity.WARN, Severity.ERROR, Severity.FATAL])
def test_severity_matrix(code, severity):
    operation = Operation.VALIDATE if code is Code.INVALID_ARGUMENT else Operation.RECEIVE
    state = State.SHUTDOWN if operation is Operation.VALIDATE else State.RX_SINGLE
    allowed = Severity.FATAL if operation is Operation.VALIDATE else Severity.ERROR
    if severity is allowed:
        CompletedRadioEpisode(operation, code, severity, 100, context(terminal_state=state))
    else:
        with pytest.raises(ValueError):
            CompletedRadioEpisode(operation, code, severity, 100, context(terminal_state=state))


# One builder yields one frozen result and preserves the original trigger on escalation.
@pytest.mark.parametrize("path", ["soft", "hard", "exhausted", "missing"])
def test_recovery_episode(path):
    episode = builder(sequence=17, bucket=300)
    episode.enter_recovery(Reason.TX_OUTCOME_UNCERTAIN)
    episode.start_level()
    first = RadioFailure(Code.IO, E.RadioFailureStage.WRITE_COMMAND, os_errno=5)
    second = RadioFailure(Code.DEVICE_ERROR, E.RadioFailureStage.READ_COMMAND, device_errors=0x40)
    episode.finish_level(None if path == "soft" else first)
    if path in ("hard", "exhausted"):
        episode.start_level(hard=True)
        episode.finish_level(None if path == "hard" else second)
    state = State.HARDWARE_MISSING if path == "missing" else State.RECOVERY_EXHAUSTED if path == "exhausted" else State.RX_SINGLE
    result = episode.finish(state, 140)
    assert result.error_code is Code.BUSY_TIMEOUT
    assert result.operation is Operation.TRANSMIT
    assert result.context.trigger_detail.command_opcode == 0x83
    assert result.context.episode_duration_us == 40
    assert result.context.related_occurrence_sequence == 17
    assert result.context.airtime_bucket_expiration_utc_us == 300
    assert result.context.last_recovery_error_code is (None if path == "soft" else Code.DEVICE_ERROR if path == "exhausted" else Code.IO)
    assert result.severity is (Severity.ERROR if path in ("soft", "hard") else Severity.FATAL)
    with pytest.raises(RuntimeError):
        episode.finish(state, 150)
    with pytest.raises(FrozenInstanceError):
        result.severity = Severity.WARN


# Cancellation records attempted levels truthfully without inventing a backend error.
@pytest.mark.parametrize("phase", ["before", "soft", "hard"])
@pytest.mark.parametrize("safe", [True, False])
def test_interrupted_recovery(phase, safe):
    episode = builder()
    episode.enter_recovery(Reason.TX_OUTCOME_UNCERTAIN)
    if phase != "before":
        episode.start_level()
    if phase == "hard":
        episode.finish_level(RadioFailure(Code.IO, E.RadioFailureStage.WRITE_COMMAND, os_errno=5))
        episode.start_level(hard=True)
    episode.interrupt()
    result = episode.finish(State.SHUTDOWN, 100, safe=safe)
    assert result.context.soft_recovery_result is (Level.NOT_ATTEMPTED if phase == "before" else Level.FAILED)
    assert result.context.hard_recovery_result is (Level.FAILED if phase == "hard" else Level.NOT_ATTEMPTED)
    assert result.context.last_recovery_error_code is (Code.IO if phase == "hard" else None)
    assert result.severity is (Severity.ERROR if safe else Severity.FATAL)
    assert result.context.episode_duration_us == 0


# Backward timestamps and impossible level histories cannot be emitted as completed.
def test_builder_and_context_invariants():
    episode = builder()
    with pytest.raises(ValueError):
        episode.finish(State.RX_SINGLE, 99)
    episode.enter_recovery(Reason.BUSY_TIMEOUT)
    with pytest.raises(RuntimeError):
        episode.start_level(hard=True)
    with pytest.raises(ValueError):
        episode.finish(State.RX_SINGLE, 101)
    episode.start_level()
    with pytest.raises(RuntimeError):
        episode.finish(State.SHUTDOWN, 102)
    with pytest.raises(ValueError):
        context(last_recovery_error_code=Code.IO)
    with pytest.raises(ValueError):
        context(soft_recovery_result=Level.SUCCEEDED)
    with pytest.raises(ValueError):
        context(related_occurrence_sequence=0)


# Driver-status signed boundaries and all present raw bits round trip losslessly.
@pytest.mark.parametrize("status", [-(1 << 31), -1, 0, (1 << 31) - 1])
def test_driver_status(status):
    detail = RadioFailureDetailV1(State.RECOVERING, 255, E.RadioFailureStage.NONE,
        E.RadioBackendStatusKind.SX1262_DRIVER_STATUS, status, 255, 65535, 65535, True)
    assert decode_failure_detail(encode_failure_detail(detail)) == detail


# Fatal diagnostics cannot claim that the instance remains in an operational RX state.
def test_fatal_invalid_argument_requires_terminal():
    with pytest.raises(ValueError, match="terminal"):
        CompletedRadioEpisode(Operation.VALIDATE, Code.INVALID_ARGUMENT, Severity.FATAL, 100, context())
