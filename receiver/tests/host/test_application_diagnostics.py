"""Independent byte-layout and closed-catalogue diagnostic boundaries."""

from dataclasses import replace
import pytest

from cura_receiver.generated import receiver_enums_generated as E
from cura_receiver.core_diagnostics import (ReceiverCoreFailureContextV1, CoreFailureEpisode,
    encode_core_context, decode_core_context, core_diagnostic)
from cura_receiver.control_diagnostics import (ReceiverPersistenceControlContextV1,
    encode_control_context, decode_control_context, control_failure, control_diagnostic)
from cura_receiver.persistence_control_values import (CommunicatorStateLoadResult, CommunicatorStateLoadStatus,
    CommunicatorStateCondition, CommunicatorStateCommitResult, CommunicatorStateCommitDisposition,
    CommunicatorStateCommitFailureKind)
from tests.support.builders.persistence import INSTANCE


def test_core_literal_layout_and_padding():
    context = ReceiverCoreFailureContextV1(E.CorePhase.POST_RESPONSE_FINALIZATION,
        E.CoreFailureStage.CONSTRUCT_ENTITY, 27, flags=0x65,
        related_entity_kind=E.PersistQueueEntityKind.MEASUREMENT_PROFILE,
        related_occurrence_sequence=123, communicator_state_generation=7,
        airtime_bucket_expiration_utc_us=-99)
    encoded = encode_core_context(context)
    assert encoded[:8] == bytes.fromhex('6400050900016500')
    assert encoded[8:16] == bytes(8)
    assert int.from_bytes(encoded[16:24], 'little') == 123
    assert encoded[24:40] == bytes(16)
    assert int.from_bytes(encoded[40:48], 'little') == 7
    assert int.from_bytes(encoded[48:56], 'little', signed=True) == -99
    assert int.from_bytes(encoded[56:64], 'little') == 27
    assert decode_core_context(encoded) == context
    episode = CoreFailureEpisode(E.CoreDiagnosticErrorCode.REPRESENTATION_INVARIANT,
                                 E.DiagnosticOperation.APPEND, 200, context)
    diagnostic = core_diagnostic(episode, receiver_instance_id=INSTANCE, diagnostic_sequence=4)
    assert diagnostic.context == encoded + bytes(64)
    assert diagnostic.severity is E.DiagnosticSeverity.FATAL


@pytest.mark.parametrize('offset', [0, 24, 25, 31])
def test_core_reserved_and_absent_bytes_rejected(offset):
    encoded = bytearray(encode_core_context(ReceiverCoreFailureContextV1(E.CorePhase.IDLE,
        E.CoreFailureStage.INVOKE_ADAPTER, 0)))
    encoded[offset] |= 0x80
    with pytest.raises(ValueError):
        decode_core_context(bytes(encoded))


@pytest.mark.parametrize('condition,code', [('MISSING', 'STATE_MISSING'), ('CORRUPT', 'STATE_CORRUPT'),
    ('UNSUPPORTED_VERSION', 'UNSUPPORTED_STATE_VERSION'), ('POLICY_MISMATCH', 'STATE_POLICY_MISMATCH')])
def test_control_state_conditions_exact(condition, code):
    result = CommunicatorStateLoadResult(CommunicatorStateLoadStatus.STATE_UNAVAILABLE,
        E.DiagnosticOperation.READ, state_condition=CommunicatorStateCondition[condition])
    episode = control_failure(result, command=E.PersistenceControlCommand.LOAD_COMMUNICATOR_STATE,
        purpose=E.PersistenceControlPurpose.STARTUP_STATE, started=10, finished=19)
    assert episode.error_code is E.PersistenceControlDiagnosticErrorCode[code]
    assert episode.severity is (E.DiagnosticSeverity.WARN if condition == 'MISSING' else E.DiagnosticSeverity.ERROR)
    raw = encode_control_context(episode.context)
    assert raw[:8] == bytes((2, 0, 2, 2, 0, 0, 3, E.PersistenceControlStateCondition[condition].value))
    assert raw[8:12] == bytes((0, 0, 8, 0))
    assert raw[12:56] == bytes(44)
    assert raw[56:] == (9).to_bytes(8, 'little')
    assert decode_control_context(raw) == episode.context
    diagnostic = control_diagnostic(episode, receiver_instance_id=INSTANCE, diagnostic_sequence=1)
    assert diagnostic.context == raw + bytes(64)


def test_control_unknown_commit_preserves_failure_and_disposition():
    result = CommunicatorStateCommitResult(CommunicatorStateCommitDisposition.OUTCOME_UNKNOWN,
        CommunicatorStateCommitFailureKind.DATABASE_ERROR, E.DiagnosticOperation.WRITE,
        sqlite_primary_code=10, sqlite_extended_code=778, os_errno=5)
    episode = control_failure(result, command=E.PersistenceControlCommand.COMMIT_COMMUNICATOR_STATE,
        purpose=E.PersistenceControlPurpose.AIRTIME_BUCKET_GRANT, started=100, finished=125,
        requested_generation=8, authoritative_generation_before=7, airtime_bucket_expiration_utc_us=999)
    assert episode.error_code is E.PersistenceControlDiagnosticErrorCode.DATABASE
    assert episode.context.disposition is E.PersistenceControlDisposition.OUTCOME_UNKNOWN
    raw = encode_control_context(episode.context)
    assert raw[:8] == bytes.fromhex('f902030301040500')
    assert int.from_bytes(raw[10:12], 'little') == 3
    assert int.from_bytes(raw[12:16], 'little') == 10
    assert int.from_bytes(raw[16:20], 'little') == 778
    assert int.from_bytes(raw[24:32], 'little') == 8
    assert decode_control_context(raw) == episode.context


def test_core_closed_operation_catalogue():
    context = ReceiverCoreFailureContextV1(E.CorePhase.IDLE, E.CoreFailureStage.VALIDATE_ARGUMENT, 0)
    with pytest.raises(ValueError):
        CoreFailureEpisode(E.CoreDiagnosticErrorCode.INVALID_ARGUMENT, E.DiagnosticOperation.RECEIVE, 0, context)
    with pytest.raises(ValueError):
        replace(context, flags=256)
    with pytest.raises(ValueError):
        replace(context, detail_kind=E.CoreDetailKind.PERSIST_QUEUE_VIOLATION, detail_code=99)


def test_queue_violation_soundness_is_explicit():
    from cura_receiver.persist_queue import PersistQueue, PersistQueueInterfaceError
    from cura_receiver.core_diagnostics import exception_episode
    queue = PersistQueue()
    with pytest.raises(PersistQueueInterfaceError) as caught:
        queue.try_reserve_one(object())
    assert caught.value.detail_code is E.PersistQueueViolationDetailCode.INVALID_SPEC
    assert caught.value.queue_known_sound
    episode = exception_episode(caught.value, phase=E.CorePhase.PACKET_PROCESSING,
        stage=E.CoreFailureStage.RESERVE_QUEUE, operation=E.DiagnosticOperation.APPEND, started=10, finished=20)
    assert episode.context.flags & 64
    assert episode.context.detail_code == 1
    queue.close()
    with pytest.raises(PersistQueueInterfaceError) as closed:
        queue.try_reserve_one(object())
    assert not closed.value.queue_known_sound
    assert exception_episode(closed.value, phase=E.CorePhase.SHUTDOWN,
        stage=E.CoreFailureStage.RESERVE_QUEUE, operation=E.DiagnosticOperation.APPEND, started=10, finished=20) is None


def test_crypto_backend_escape_is_not_authentication_failure(monkeypatch):
    from cura_receiver import protocol_v2_lora_crypto as crypto
    from cura_receiver.core_diagnostics import CoreFault
    from tests.support.builders.protocol_ingress import REVIEWED_NODE_KEY, REVIEWED_CURRENT_FRAME
    def fail_backend(*args, **kwargs):
        raise RuntimeError('injected backend implementation error')
    monkeypatch.setattr(crypto, 'AESCCM', fail_backend)
    with pytest.raises(CoreFault) as caught:
        crypto.open_frame(REVIEWED_NODE_KEY, REVIEWED_CURRENT_FRAME)
    assert caught.value.code is E.CoreDiagnosticErrorCode.CRYPTO_BACKEND
    assert caught.value.operation is E.DiagnosticOperation.DECODE
    assert str(caught.value) == 'receiver core failure'


@pytest.mark.parametrize('code', [E.CoreDiagnosticErrorCode.MEMORY_EXHAUSTED,
                                  E.CoreDiagnosticErrorCode.UNEXPECTED_EXCEPTION])
@pytest.mark.parametrize('operation', list(E.DiagnosticOperation))
def test_generic_core_operations_preserve_actual_action(code, operation):
    context = ReceiverCoreFailureContextV1(E.CorePhase.PERIODIC_TIME,
        E.CoreFailureStage.INVOKE_ADAPTER, 7)
    if operation is E.DiagnosticOperation.NONE:
        with pytest.raises(ValueError, match='invalid core operation'):
            CoreFailureEpisode(code, operation, 20, context)
    else:
        episode = CoreFailureEpisode(code, operation, 20, context)
        record = core_diagnostic(episode, receiver_instance_id=INSTANCE, diagnostic_sequence=1)
        assert record.operation is operation
        assert record.error_code == code.value
        assert decode_core_context(record.context[:64]) == context


@pytest.mark.parametrize('operation', [E.DiagnosticOperation.READ, E.DiagnosticOperation.SYNC])
def test_chrony_implementation_escape_keeps_actual_operation(monkeypatch, operation):
    from cura_receiver.platform import linux_chrony as L
    from cura_receiver.core_diagnostics import exception_episode
    from tests.host.test_runtime_time import runtime, sample, tracking
    from tests.support.fakes.chrony import FakeChronyControl
    rt, clock, kernel, _ = runtime()
    sample(rt, kernel)
    rt.sample_network(tracking(rt))
    if operation is E.DiagnosticOperation.SYNC:
        source = FakeChronyControl()
        source.tracking_results.append(tracking(rt, correction=40_000_000))
        rt.poll_chrony(source)
    else:
        clock.advance_elapsed_us(rt.next_tracking_start() - clock.now_monotonic_us())
    monkeypatch.setattr(L, '_run_child', lambda *args: L._ChildResult(
        0, b'chronyc (chrony) version 4.6.1 (+READLINE)\n', started=True))
    adapter = L.LinuxChronyControl(clock, socket_path='/run/chrony/chronyd.sock',
        deadline_monotonic_us=clock.now_monotonic_us() + 1000)
    calls = []
    def escape(argv, *args):
        calls.append(argv[-1])
        raise RuntimeError('private implementation detail')
    monkeypatch.setattr(L, '_run_child', escape)
    started = clock.now_monotonic_us()
    with pytest.raises(RuntimeError) as caught:
        rt.poll_chrony(adapter)
    assert calls == ['makestep' if operation is E.DiagnosticOperation.SYNC else 'tracking']
    episode = exception_episode(caught.value, phase=E.CorePhase.PERIODIC_TIME,
        stage=E.CoreFailureStage.INVOKE_ADAPTER, operation=operation,
        started=started, finished=clock.now_monotonic_us())
    assert episode.operation is operation
    assert episode.error_code is E.CoreDiagnosticErrorCode.UNEXPECTED_EXCEPTION
    record = core_diagnostic(episode, receiver_instance_id=INSTANCE, diagnostic_sequence=1)
    assert b'private' not in record.context
