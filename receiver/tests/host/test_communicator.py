"""Real packet composition over SPI/GPIO fakes, real policy and real SQLite."""

from types import SimpleNamespace
import sqlite3

import pytest

from cura_receiver.producer_admission import ProducerAdmission
from cura_receiver.communicator import Communicator, CommunicatorFailure
from cura_receiver.generated import receiver_enums_generated as E
from cura_receiver.persist_queue import PersistQueue, PersistenceAdmissionSnapshot
from cura_receiver.ports.chrony import ChronyTrackingResult, ChronyQueryStatus as CQ
from cura_receiver.ports.ds3231 import Ds3231ReadResult, Ds3231ReadStatus as DR
from cura_receiver.ports.kernel_clock import KernelClockResult, KernelSampleStatus as KS
from cura_receiver.ports.radio import Dio1Edge
from cura_receiver.protocol_ingress import ProtocolIngress
from cura_receiver.radio import Radio
from cura_receiver.runtime_time import RuntimeTime
from cura_receiver.sx1262 import Sx1262
from cura_receiver.time_policy import TimePolicy
from tests.support.builders.persistence import INSTANCE
from tests.support.builders.persistence_control import state
from tests.support.builders.protocol_ingress import REVIEWED_CURRENT_FRAME, REVIEWED_ACCEPTED_ACK, REVIEWED_NODE_ID, REVIEWED_NODE_KEY
from tests.support.fakes.kernel_clock import FakeKernelClock
from tests.support.fakes.radio_io import PhysicalPort, Wait

UTC = 1_800_000_000_000_000


@pytest.fixture
def composition(airtime_component):
    def create(*, isolated_queue=False, grant=True):
        airtime, worker, database, clock, _ = airtime_component(initial_state=state(), utc=UTC)
        queue = PersistQueue(capacity_entities=1) if isolated_queue else worker.queue
        if isolated_queue:
            queue.publish_admission_state(PersistenceAdmissionSnapshot(1, E.PersistenceAdmissionState.AVAILABLE, 0))
        producer = ProducerAdmission(queue)
        io = PhysicalPort(clock)
        radio = Radio(Sx1262(io, clock, Wait(clock)))
        radio.initialize()
        now = clock.now_monotonic_us()
        kernel = FakeKernelClock()
        runtime = RuntimeTime(
            receiver_instance_id=INSTANCE, clock=clock, kernel=kernel, queue=producer,
            policy=TimePolicy(),
            startup_rtc_result=Ds3231ReadResult(DR.OK, now, now, UTC // 1_000_000),
            state_owner=airtime.owner,
        )
        kernel.results.append(KernelClockResult(KS.OK, now, now, UTC + now - 100, 5, 0x2040))
        runtime.sample_network(ChronyTrackingResult(CQ.OK, now, now, True, True, 0, 0, 0))
        airtime.update_time(runtime.airtime_correlation(), rtc_health=runtime.state.rtc_health)
        if grant:
            airtime.acquire_grant(deadline_monotonic_us=now + 5_000_000)
        ingress = ProtocolIngress(queue=producer, monotonic_clock=clock, auth_node_keys={REVIEWED_NODE_ID: REVIEWED_NODE_KEY})
        communicator = Communicator(instance_id=INSTANCE, clock=clock, radio=radio, ingress=ingress, runtime_time=runtime, airtime=airtime, queue=producer)
        return SimpleNamespace(communicator=communicator, radio=radio, io=io, clock=clock, queue=queue, time=runtime, airtime=airtime, worker=worker, database=database)
    return create


def deliver(c, frame=REVIEWED_CURRENT_FRAME, *, tx_irq=1):
    c.io.buffer[:] = frame
    c.io.irq = 2
    c.io.status = 0x24
    c.edge_sequence = getattr(c, "edge_sequence", 0) + 1
    c.io.edges.append(Dio1Edge(c.clock.now_monotonic_us() * 1000, c.edge_sequence))
    def after(command):
        if command[0] == 0x83:
            c.clock.advance_elapsed_us(61_696)
            c.io.irq = tx_irq
            c.io.status = 0x24
            c.edge_sequence += 1
            c.io.edges.append(Dio1Edge(c.clock.now_monotonic_us() * 1000, c.edge_sequence))
    c.io.after_transfer = after
    return c.communicator.receive_once(deadline_monotonic_us=c.clock.now_monotonic_us() + 500_000)


# A real accepted exchange transmits exact reviewed bytes and persists one complete pair.
def test_valid_packet_to_real_sqlite(composition):
    c = composition()
    result = deliver(c)
    entity = result.finalization.published_entity
    assert entity.profile.ack_frame == REVIEWED_ACCEPTED_ACK
    assert entity.profile.processing_result is E.ProcessingResult.ACCEPTED
    assert entity.profile.ack_tx_result is E.AckTxResult.TX_DONE
    assert entity.profile.t5_tx_done_monotonic_us - entity.profile.t4_set_tx_attempted_monotonic_us == 61_696
    assert b'\x0e\x00' + REVIEWED_ACCEPTED_ACK in c.io.commands
    c.worker.request_stop(deadline_monotonic_us=c.clock.now_monotonic_us() + 5_000_000)
    c.worker.join(5)
    assert not c.worker.is_alive() and c.worker.failure is None
    with sqlite3.connect(c.database) as db:
        assert db.execute('SELECT count(*) FROM reading_messages').fetchone() == (1,)
        assert db.execute('SELECT count(*) FROM message_profiles').fetchone() == (1,)


# With no acknowledged grant acceptance survives without issuing SetTx or a state commit.
def test_airtime_suppression_keeps_acceptance(composition):
    c = composition(grant=False)
    generation = c.airtime.owner.state.generation
    result = deliver(c)
    assert result.finalization.published_entity.profile.ack_tx_result is E.AckTxResult.SUPPRESSED_AIRTIME_BUDGET
    assert result.finalization.published_entity.candidate is not None
    assert not any(command[0] == 0x83 for command in c.io.commands)
    assert c.airtime.owner.state.generation == generation


# A pending clock boundary prevents ingress even when the consumer frees capacity.
def test_pending_boundary_precedes_new_packet(composition):
    c = composition(isolated_queue=True)
    c.clock.advance_elapsed_us(60_000_000)
    assert c.time.expire_due().admission_result is E.AdmissionResult.QUEUE_FULL
    before = len(c.io.calls)
    blocked = c.communicator.receive_once(deadline_monotonic_us=c.clock.now_monotonic_us())
    assert blocked.boundary_blocked
    assert len(c.io.calls) == before and c.communicator.occurrence_sequence == 0
    c.queue.claim_batch(max_entities=1).acknowledge_durable(completed_entities=1)
    result = deliver(c)
    assert not result.boundary_blocked
    assert not c.time.ordinary_admission_blocked
    lease = c.queue.claim_batch(max_entities=1)
    assert lease.entries[0].entity.system_time_quality is E.SystemTimeQuality.UNTRUSTED
    assert result.finalization.published_entity is None  # Boundary owns the only slot.


# Expiry after ReadBuffer drops only the unaccepted snapshot, restores RX and sends nothing.
def test_boundary_closes_after_copy(composition):
    c = composition(isolated_queue=True)
    c.clock.advance_elapsed_us(59_999_950)
    c.io.buffer[:] = REVIEWED_CURRENT_FRAME
    c.io.irq = 2
    c.io.status = 0x24
    c.io.edges.append(Dio1Edge(c.clock.now_monotonic_us() * 1000, 1))
    c.io.hooks[0x1E] = lambda command: c.clock.advance_elapsed_us(100)
    result = c.communicator.receive_once(deadline_monotonic_us=c.clock.now_monotonic_us() + 500_000)
    assert result.boundary_blocked and result.discarded_unaccepted_packet
    assert c.communicator.occurrence_sequence == 0
    assert c.queue.snapshot().reserved_entities == 0
    assert not any(command[0] == 0x83 for command in c.io.commands)
    assert result.radio_result.state in (E.RadioState.RX_SINGLE, E.RadioState.RX_EVENT_PENDING)


# A confirmed timeout is retained as such; it is not an uncertain host deadline.
def test_confirmed_tx_timeout(composition):
    c = composition()
    result = deliver(c, tx_irq=0x200)
    assert result.finalization.published_entity.profile.ack_tx_result is E.AckTxResult.TX_TIMEOUT
    assert result.finalization.published_entity.profile.t5_tx_done_monotonic_us is None


# Fatal failure after confirmed TxDone retains the edge and finalizes after bounded shutdown.
def test_fatal_after_confirmed_tx_done(composition):
    c = composition()
    def fail_restore(command):
        raise RuntimeError('injected implementation failure')
    def after(command):
        if command[0] == 0x83:
            c.clock.advance_elapsed_us(61_696)
            c.io.irq = 1
            c.io.status = 0x24
            c.io.edges.append(Dio1Edge(c.clock.now_monotonic_us() * 1000, 2))
            c.io.hooks[0x82] = fail_restore
    c.io.buffer[:] = REVIEWED_CURRENT_FRAME
    c.io.irq = 2
    c.io.status = 0x24
    c.io.edges.append(Dio1Edge(c.clock.now_monotonic_us() * 1000, 1))
    c.io.after_transfer = after
    with pytest.raises(CommunicatorFailure) as failure:
        c.communicator.receive_once(deadline_monotonic_us=c.clock.now_monotonic_us() + 500_000)
    evidence = failure.value.result
    assert evidence.radio_result.state is E.RadioState.SHUTDOWN
    assert evidence.finalization.published_entity.profile.ack_tx_result is E.AckTxResult.TX_DONE
    assert evidence.finalization.published_entity.profile.t5_tx_done_monotonic_us is not None


# Receive errors never authenticate/admit a reading, including valid copied bytes.
@pytest.mark.parametrize("fault", ["crc", "header", "read_buffer", "packet_status", "clear_irq"])
def test_failed_receive_is_profile_only(composition, fault):
    from cura_receiver.ports.radio import RadioBackendError, RadioFailure, Error, Stage
    c = composition()
    c.io.buffer[:] = REVIEWED_CURRENT_FRAME
    c.io.irq = {"crc": 0x42, "header": 0x22}.get(fault, 2)
    c.io.status = 0x24
    t0 = c.clock.now_monotonic_us()
    c.io.edges.append(Dio1Edge(t0 * 1000, 1))
    opcode = {"read_buffer": 0x1e, "packet_status": 0x14, "clear_irq": 0x02}.get(fault)
    if opcode is not None:
        def fail_once(command):
            del c.io.hooks[opcode]
            raise RadioBackendError(RadioFailure(Error.IO, Stage.READ_BUFFER, os_errno=5))
        c.io.hooks[opcode] = fail_once
    result = c.communicator.receive_once(deadline_monotonic_us=t0 + 500_000)
    entity = result.finalization.published_entity
    profile = entity.profile
    assert not hasattr(entity, "candidate")
    assert profile.processing_result is E.ProcessingResult.RADIO_ERROR
    assert profile.ack_selected is E.AckSelection.NONE
    assert profile.ack_tx_result is E.AckTxResult.NOT_APPLICABLE
    assert profile.ack_frame is None and not profile.header_authenticated
    assert profile.decoded_sample_id is None
    assert profile.received_at_monotonic_us == t0
    assert profile.t1_handler_started_monotonic_us >= t0
    assert profile.t3_authentication_completed_monotonic_us is None
    assert profile.t4_set_tx_attempted_monotonic_us is None
    assert profile.t5_tx_done_monotonic_us is None
    assert profile.t6_set_rx_issued_monotonic_us is not None
    copied = fault in ("packet_status", "clear_irq")
    assert profile.received_frame_length == (len(REVIEWED_CURRENT_FRAME) if copied else None)
    assert profile.received_frame == (REVIEWED_CURRENT_FRAME.ljust(255, b"\0") if copied else None)
    assert (profile.t2_packet_copied_monotonic_us is not None) is copied
    assert profile.claimed_node_id == (REVIEWED_NODE_ID if copied else None)
    assert not any(command[0] in (0x0e, 0x83) for command in c.io.commands)
    assert c.communicator.occurrence_sequence == 1
    assert c.communicator.queue.counts[1] == (1, 0, 0)
    assert c.communicator.queue.counts[4] == (1, 0, 0)
    c.worker.request_stop(deadline_monotonic_us=c.clock.now_monotonic_us() + 5_000_000)
    c.worker.join(5)
    assert not c.worker.is_alive() and c.worker.failure is None
    with sqlite3.connect(c.database) as db:
        assert db.execute('SELECT count(*) FROM reading_messages').fetchone() == (0,)
        assert db.execute('SELECT count(*) FROM message_profiles').fetchone() == (1,)


# Best-effort failed-event profiles do not reserve over the initial clock boundary.
def test_failed_receive_profile_queue_full(composition):
    c = composition(isolated_queue=True)
    c.io.irq, c.io.status = 0x42, 0x24
    c.io.edges.append(Dio1Edge(c.clock.now_monotonic_us() * 1000, 1))
    result = c.communicator.receive_once(deadline_monotonic_us=c.clock.now_monotonic_us() + 500_000)
    assert result.finalization.admission.result is E.AdmissionResult.QUEUE_FULL
    assert result.finalization.published_entity is None
    assert c.queue.snapshot().reserved_entities == 0
    assert not any(command[0] == 0x83 for command in c.io.commands)


# When radio and health are ready together the complete exchange wins.
def test_scheduler_radio_precedes_health(composition):
    from cura_receiver.communicator_scheduler import CommunicatorScheduler, Work
    c = composition()
    scheduler = CommunicatorScheduler(c.communicator, chrony=None, rtc=None, health_interval_us=60_000_000)
    scheduler.next_airtime = c.clock.now_monotonic_us() + 1_000_000
    c.time.last_refresh_monotonic_us = c.clock.now_monotonic_us()
    c.io.buffer[:] = b'bad'
    c.io.irq, c.io.status = 2, 0x24
    c.io.edges.append(Dio1Edge(c.clock.now_monotonic_us() * 1000, 1))
    turn = scheduler.run_once()
    assert turn.work is Work.RADIO
    assert turn.exchange.finalization.published_entity.profile.processing_result is E.ProcessingResult.REJECTED_MALFORMED_LENGTH
    assert scheduler.run_once().work is Work.HEALTH


# Grant commits happen in a separate idle turn; an earlier packet cannot wait for one.
def test_scheduler_precharges_only_outside_exchange(composition):
    from cura_receiver.communicator_scheduler import CommunicatorScheduler, Work
    c = composition(grant=False)
    first = deliver(c)
    assert first.finalization.published_entity.profile.ack_tx_result is E.AckTxResult.SUPPRESSED_AIRTIME_BUDGET
    scheduler = CommunicatorScheduler(c.communicator, chrony=None, rtc=None, health_interval_us=60_000_000)
    generation = c.airtime.owner.state.generation
    assert scheduler.run_once().work is Work.AIRTIME
    assert c.airtime.owner.state.generation > generation
    assert deliver(c).finalization.published_entity.profile.ack_tx_result is E.AckTxResult.TX_DONE


# A full observation queue uses a bounded retry deadline rather than busy-spinning.
def test_scheduler_boundary_retry_is_paced(composition):
    from cura_receiver.communicator_scheduler import CommunicatorScheduler, Work
    c = composition(isolated_queue=True)
    scheduler = CommunicatorScheduler(c.communicator, chrony=None, rtc=None, health_interval_us=60_000_000)
    c.clock.advance_elapsed_us(60_000_000)
    first = scheduler.run_once()
    counts = c.communicator.queue.counts
    assert first.work is Work.TIME
    second = scheduler.run_once()
    assert second.work is Work.WAIT
    assert second.wait_until_monotonic_us > c.clock.now_monotonic_us()
    assert c.communicator.queue.counts == counts


# The real health request includes its own admission and persists through worker enrichment.
def test_health_uses_shared_admission_matrix(composition):
    from cura_receiver.communicator_telemetry import CommunicatorTelemetry
    c = composition()
    deliver(c)
    telemetry = CommunicatorTelemetry(c.communicator)
    assert telemetry.health() is E.AdmissionResult.RESERVED
    assert c.communicator.queue.counts == ((1, 0, 0), (0, 0, 0), (1, 0, 0), (0, 0, 0), (1, 0, 0))
    c.worker.request_stop(deadline_monotonic_us=c.clock.now_monotonic_us() + 5_000_000)
    c.worker.join(5)
    assert c.worker.failure is None
    with sqlite3.connect(c.database) as db:
        assert db.execute('SELECT count(*) FROM receiver_health').fetchone() == (1,)


# Gate closure is not a reservation attempt; actual failed health attempts advance sequence.
def test_health_gate_and_failed_attempt_sequence(composition):
    from cura_receiver.communicator_telemetry import CommunicatorTelemetry
    c = composition(isolated_queue=True)
    telemetry = CommunicatorTelemetry(c.communicator)
    assert telemetry.health() is E.AdmissionResult.QUEUE_FULL
    assert telemetry.health_sequence == 1
    assert c.communicator.queue.counts[2] == (0, 0, 1)
    c.clock.advance_elapsed_us(60_000_000)
    c.time.expire_due()
    counts = c.communicator.queue.counts
    assert telemetry.health() is None
    assert telemetry.health_sequence == 1 and c.communicator.queue.counts == counts


# Each new receive poll owns its stream failure after the previous recovery returns.
def test_idle_recovery_continues_at_next_turn(composition):
    from cura_receiver.ports.radio import RadioBackendError, RadioFailure, Error, Stage
    c = composition()
    original = c.io.wait_edge
    failures = 2
    def wait_edge(*, deadline_monotonic_us):
        nonlocal failures
        if failures:
            failures -= 1
            raise RadioBackendError(RadioFailure(Error.IO, Stage.WAIT_IRQ, os_errno=5))
        return original(deadline_monotonic_us=deadline_monotonic_us)
    c.io.wait_edge = wait_edge
    first = c.communicator.receive_once(deadline_monotonic_us=c.clock.now_monotonic_us())
    assert first.radio_result.state is E.RadioState.RX_SINGLE
    second = c.communicator.receive_once(deadline_monotonic_us=c.clock.now_monotonic_us())
    assert second.radio_result.state is E.RadioState.RX_SINGLE
    assert c.radio.counters.recovery_attempts == 2
    assert c.radio.counters.recovery_successes == 2
    assert all(episode.error_code is Error.IO for episode in first.radio_episodes + second.radio_episodes)


# Producer closure terminates diagnostic emission even with a pending clock boundary.
@pytest.mark.parametrize('pending_clock', [False, True])
def test_queue_closure_ends_diagnostics(composition, pending_clock):
    from cura_receiver.communicator_telemetry import CommunicatorTelemetry, EmissionSkipped
    c = composition(isolated_queue=True)
    telemetry = CommunicatorTelemetry(c.communicator)
    if pending_clock:
        c.clock.advance_elapsed_us(60_000_000)
        c.time.expire_due()
    counts = c.communicator.queue.counts
    c.queue.close()
    def forbidden_factory(*args, **kwargs):
        pytest.fail('closed diagnostics must not construct a diagnostic')
    assert telemetry.diagnostic(forbidden_factory, object()) is EmissionSkipped.CLOSED
    assert telemetry.health() is EmissionSkipped.CLOSED
    assert telemetry.diagnostic_sequence == telemetry.health_sequence == 0
    assert c.communicator.queue.counts == counts
    assert c.queue.snapshot().closed
    # Closing the producer does not discard existing FIFO work.
    batch = c.queue.claim_batch(max_entities=1)
    assert batch is not None
    batch.acknowledge_durable(completed_entities=1)
    assert c.queue.snapshot().closed_and_drained


def test_begin_exception_after_binding_retains_accepted_completion(composition, monkeypatch):
    c = composition()
    ingress = c.communicator.ingress
    original = ingress.begin
    def escape(packet):
        original(packet)
        raise RuntimeError('injected exceptional return after binding')
    monkeypatch.setattr(ingress, 'begin', escape)
    with pytest.raises(CommunicatorFailure) as caught:
        deliver(c)
    result = caught.value.result
    assert result.radio_result.state is E.RadioState.SHUTDOWN
    assert result.finalization.published_entity.candidate is not None
    profile = result.finalization.published_entity.profile
    assert profile.processing_result is E.ProcessingResult.ACCEPTED
    assert profile.ack_tx_result is E.AckTxResult.SET_TX_FAILED
    assert profile.t4_set_tx_attempted_monotonic_us is None
    assert profile.t5_tx_done_monotonic_us is None
    assert c.queue.snapshot().reserved_entities == 0
    assert ingress.active_occurrence is None
    assert c.communicator.queue.counts[0] == (1, 0, 0)
    assert not any(command[0] == 0x83 for command in c.io.commands)


def runtime_dispatch(c):
    from cura_receiver.communicator_runtime import CommunicatorRuntime
    from cura_receiver.communicator_scheduler import CommunicatorScheduler
    from cura_receiver.control_diagnostics import ControlEpisodeTracker
    scheduler = CommunicatorScheduler(c.communicator, chrony=None, rtc=None, health_interval_us=60_000_000)
    scheduler.next_airtime = c.clock.now_monotonic_us() + 10_000_000
    c.time.last_refresh_monotonic_us = c.clock.now_monotonic_us()
    return CommunicatorRuntime(scheduler, control_episodes=ControlEpisodeTracker())


def test_runtime_fatal_packet_publishes_profile_before_core(composition, monkeypatch):
    from cura_receiver.core_diagnostics import decode_core_context
    c = composition()
    runtime = runtime_dispatch(c)
    original = c.communicator.ingress.begin
    def escape(packet):
        original(packet)
        raise RuntimeError('private failure')
    monkeypatch.setattr(c.communicator.ingress, 'begin', escape)
    c.io.buffer[:] = REVIEWED_CURRENT_FRAME
    c.io.irq, c.io.status = 2, 0x24
    c.io.edges.append(Dio1Edge(c.clock.now_monotonic_us() * 1000, 1))
    assert runtime.step() is None
    assert runtime.terminal
    assert runtime.failure.exchange.finalization.published_entity.candidate is not None
    assert runtime.failure.diagnostic_result is E.AdmissionResult.RESERVED
    assert c.queue.snapshot().reserved_entities == 0
    c.worker.request_stop(deadline_monotonic_us=c.clock.now_monotonic_us() + 5_000_000)
    c.worker.join(5)
    with sqlite3.connect(c.database) as db:
        assert db.execute('SELECT count(*) FROM reading_messages').fetchone() == (1,)
        row = db.execute('SELECT error_domain_id, error_code_id, context FROM diagnostics').fetchone()
    assert row[:2] == (E.DiagnosticErrorDomain.CORE.value, E.CoreDiagnosticErrorCode.UNEXPECTED_EXCEPTION.value)
    context = decode_core_context(row[2][:64])
    assert context.flags & 32  # Profile completed before diagnostic construction.
    assert context.flags & 128


def test_runtime_diagnostic_construction_failure_does_not_recurse(composition, monkeypatch):
    c = composition()
    runtime = runtime_dispatch(c)
    def broken(episode):
        raise ValueError('injected diagnostic construction failure')
    runtime._pending.append((broken, object()))
    assert runtime.step() is None
    assert runtime.terminal
    assert runtime.telemetry.diagnostic_sequence == 0
    assert runtime.failure.diagnostic_result is None
    assert c.radio.state is E.RadioState.SHUTDOWN


def test_health_construction_failure_releases_reservation(composition, monkeypatch):
    from cura_receiver import communicator_telemetry as module
    c = composition()
    runtime = runtime_dispatch(c)
    def broken(*args, **kwargs):
        raise RuntimeError('injected health construction failure')
    monkeypatch.setattr(module, 'ReceiverHealthRequestV1', broken)
    assert runtime.step() is None
    assert runtime.terminal
    assert c.queue.snapshot().reserved_entities == 0
    assert runtime.telemetry.health_sequence == 1
    assert runtime.failure.diagnostic_result is E.AdmissionResult.RESERVED
    assert runtime.telemetry.diagnostic_sequence == 1


def test_runtime_radio_failure_emits_one_operational_diagnostic(composition):
    from cura_receiver.ports.radio import RadioBackendError, RadioFailure, Error, Stage
    c = composition()
    runtime = runtime_dispatch(c)
    c.io.buffer[:] = REVIEWED_CURRENT_FRAME
    c.io.irq, c.io.status = 2, 0x24
    c.io.edges.append(Dio1Edge(c.clock.now_monotonic_us() * 1000, 1))
    def fail_once(command):
        del c.io.hooks[0x14]
        raise RadioBackendError(RadioFailure(Error.IO, Stage.READ_PACKET_STATUS, os_errno=5))
    c.io.hooks[0x14] = fail_once
    turn = runtime.step()
    assert not runtime.terminal
    assert turn.exchange.finalization.published_entity.profile.processing_result is E.ProcessingResult.RADIO_ERROR
    assert runtime.telemetry.diagnostic_sequence == 1
    c.worker.request_stop(deadline_monotonic_us=c.clock.now_monotonic_us() + 5_000_000)
    c.worker.join(5)
    with sqlite3.connect(c.database) as db:
        assert db.execute('SELECT error_domain_id, error_code_id FROM diagnostics').fetchall() == [(E.DiagnosticErrorDomain.RADIO.value, Error.IO.value)]
        assert db.execute('SELECT count(*) FROM reading_messages').fetchone() == (0,)


def test_runtime_time_failure_emits_once_after_boundary(composition):
    from tests.support.fakes.chrony import FakeChronyControl
    c = composition()
    runtime = runtime_dispatch(c)
    chrony = FakeChronyControl()
    runtime.scheduler.chrony = chrony
    c.clock.advance_elapsed_us(c.time.next_tracking_start() - c.clock.now_monotonic_us())
    now = c.clock.now_monotonic_us()
    chrony.tracking_results.append(ChronyTrackingResult(CQ.DEADLINE_EXCEEDED, now, now))
    turn = runtime.step()
    assert not runtime.terminal
    assert turn.update.failure is not None
    assert c.time.state.quality is E.SystemTimeQuality.UNTRUSTED
    assert runtime.telemetry.diagnostic_sequence == 1
    c.worker.request_stop(deadline_monotonic_us=c.clock.now_monotonic_us() + 5_000_000)
    c.worker.join(5)
    with sqlite3.connect(c.database) as db:
        assert db.execute('SELECT error_domain_id FROM diagnostics').fetchall() == [(E.DiagnosticErrorDomain.TIME.value,)]


def test_runtime_defers_diagnostics_until_packet_releases_reservation(composition):
    from cura_receiver.core_diagnostics import CoreFailureEpisode, ReceiverCoreFailureContextV1
    from tests.support.builders.protocol_ingress import ingress_packet
    from cura_receiver.protocol_ingress import ProtocolIngressTerminalV1
    c = composition()
    runtime = runtime_dispatch(c)
    occurrence = c.communicator.ingress.begin(ingress_packet())
    episode = CoreFailureEpisode(E.CoreDiagnosticErrorCode.UNEXPECTED_EXCEPTION, E.DiagnosticOperation.READ,
        c.clock.now_monotonic_us(), ReceiverCoreFailureContextV1(E.CorePhase.IDLE, E.CoreFailureStage.INVOKE_ADAPTER, 0))
    runtime._pending.append((runtime.telemetry.core, episode))
    runtime.flush_diagnostics()
    assert runtime.telemetry.diagnostic_sequence == 0
    assert c.queue.snapshot().reserved_entities == 1
    c.communicator.ingress.finalize(occurrence, ProtocolIngressTerminalV1(
        E.AckTxResult.SET_TX_FAILED, None, None, None, radio_state=E.RadioState.SHUTDOWN))
    runtime.flush_diagnostics()
    assert runtime.telemetry.diagnostic_sequence == 1
    assert not runtime._pending


def test_repeated_stream_failures_cannot_hold_previous_accepted_packet(composition):
    from cura_receiver.ports.radio import RadioBackendError, RadioFailure, Error, Stage
    c = composition()
    runtime = runtime_dispatch(c)
    c.io.buffer[:] = REVIEWED_CURRENT_FRAME
    c.io.irq, c.io.status = 2, 0x24
    c.io.edges.append(Dio1Edge(c.clock.now_monotonic_us() * 1000, 1))
    polls = []
    def broken_stream(*, deadline_monotonic_us):
        polls.append(c.communicator.ingress.active_occurrence)
        raise RadioBackendError(RadioFailure(Error.IO, Stage.WAIT_IRQ, os_errno=5))
    def fail_tx_preparation(command):
        del c.io.hooks[0x0e]
        def after_restore(command):
            if command[0] == 0x82:
                c.io.wait_edge = broken_stream
        c.io.after_transfer = after_restore
        raise RadioBackendError(RadioFailure(Error.IO, Stage.WRITE_BUFFER, os_errno=5))
    c.io.hooks[0x0e] = fail_tx_preparation
    first = runtime.step()
    assert first.exchange.finalization.published_entity.candidate is not None
    assert first.exchange.finalization.published_entity.profile.ack_tx_result is E.AckTxResult.SET_TX_FAILED
    assert not polls
    for count in range(1, 5):
        assert c.queue.snapshot().reserved_entities == 0
        assert not runtime._pending
        assert c.radio.state is E.RadioState.RX_SINGLE
        assert runtime.telemetry.diagnostic_sequence == count
        if count < 4:
            runtime.step()
    assert polls == [None, None, None]
    assert c.communicator.occurrence_sequence == 1
    assert not any(command[0] == 0x83 for command in c.io.commands)


@pytest.mark.parametrize('stage,expected', [
    ('prepare', E.AckTxResult.SET_TX_FAILED),
    ('set_tx', E.AckTxResult.UNKNOWN_INTERRUPTED),
    ('tx_done', E.AckTxResult.TX_DONE),
    ('tx_timeout', E.AckTxResult.TX_TIMEOUT),
])
def test_runtime_fatal_completion_preserves_actual_tx_facts(composition, stage, expected):
    c = composition()
    runtime = runtime_dispatch(c)
    def fail(command):
        raise RuntimeError('injected implementation failure')
    if stage == 'prepare':
        c.io.hooks[0x0e] = fail
    elif stage == 'set_tx':
        c.io.hooks[0x83] = fail
    else:
        def after(command):
            if command[0] == 0x83:
                c.clock.advance_elapsed_us(61_696)
                c.io.irq = 1 if stage == 'tx_done' else 0x200
                c.io.status = 0x24
                c.io.edges.append(Dio1Edge(c.clock.now_monotonic_us() * 1000, 2))
                c.io.hooks[0x82] = fail
        c.io.after_transfer = after
    c.io.buffer[:] = REVIEWED_CURRENT_FRAME
    c.io.irq, c.io.status = 2, 0x24
    c.io.edges.append(Dio1Edge(c.clock.now_monotonic_us() * 1000, 1))
    assert runtime.step() is None
    assert runtime.terminal
    result = runtime.failure.exchange
    assert result.radio_result.state is E.RadioState.SHUTDOWN
    profile = result.finalization.published_entity.profile
    assert result.finalization.published_entity.candidate is not None
    assert profile.ack_tx_result is expected
    assert (profile.t4_set_tx_attempted_monotonic_us is not None) == (stage != 'prepare')
    assert (profile.t5_tx_done_monotonic_us is not None) == (stage == 'tx_done')
    assert runtime.failure.diagnostic_result is E.AdmissionResult.RESERVED
    assert runtime.telemetry.diagnostic_sequence == 1
    assert c.queue.snapshot().reserved_entities == 0


def test_incremental_rtc_yields_to_packet_before_next_action(composition):
    from cura_receiver.communicator_scheduler import CommunicatorScheduler, Work
    from tests.support.fakes.ds3231 import FakeDs3231Control
    c = composition()
    rtc = FakeDs3231Control()
    now = c.clock.now_monotonic_us()
    rtc.read_results.append(Ds3231ReadResult(DR.OK, now, now, UTC // 1_000_000))
    scheduler = CommunicatorScheduler(c.communicator, chrony=None, rtc=rtc, health_interval_us=60_000_000)
    scheduler.next_airtime = now + 10_000_000
    assert scheduler.run_once().work is Work.RTC
    assert c.time.rtc_refresh_episode is not None
    # No precharged ACK here: RX and finalization still use the production path.
    c.io.buffer[:] = REVIEWED_CURRENT_FRAME
    c.io.irq = 2
    c.io.status = 0x24
    c.io.edges.append(Dio1Edge(c.clock.now_monotonic_us() * 1000, 1))
    turn = scheduler.run_once()
    assert turn.work is Work.RADIO and turn.exchange.finalization is not None
    assert len(rtc.calls) == 1
    scheduler.stop_requested = lambda: True
    turn = scheduler.run_once()
    assert turn.work is Work.TERMINAL
    assert turn.update.status.name == 'SHUTDOWN_CANCELLED'
    assert len(rtc.calls) == 1 and c.time.rtc_refresh_episode is None


@pytest.mark.parametrize('tick_us', [0, 1, 100])
def test_scheduler_rtc_refresh_reaches_durable_verification(composition, monkeypatch, tick_us):
    from cura_receiver.communicator_scheduler import CommunicatorScheduler, Work
    from cura_receiver.generated.receiver_entities_generated import decode_communicator_state_v1
    from cura_receiver.ports.ds3231 import Ds3231WriteResult, Ds3231WriteDisposition, Ds3231Failure
    from cura_receiver.runtime_time import RtcRefreshStatus
    from tests.support.fakes.ds3231 import FakeDs3231Control

    c = composition()
    read_clock = c.clock.now_monotonic_us

    def advancing_read():
        c.clock.advance_elapsed_us(tick_us)
        return read_clock()

    monkeypatch.setattr(c.clock, 'now_monotonic_us', advancing_read)
    rtc = FakeDs3231Control()

    def read_rtc():
        started = c.clock.now_monotonic_us()
        return Ds3231ReadResult(DR.OK, started, c.clock.now_monotonic_us(),
                               (UTC + started - 100) // 1_000_000)

    def write_rtc():
        return Ds3231WriteResult(Ds3231WriteDisposition.COMPLETED, Ds3231Failure.NONE,
                                c.clock.now_monotonic_us(), c.clock.now_monotonic_us())

    rtc.read_results.extend([read_rtc, read_rtc])
    rtc.write_results.append(write_rtc)
    scheduler = CommunicatorScheduler(c.communicator, chrony=None, rtc=rtc,
                                      health_interval_us=60_000_000)
    scheduler.next_airtime = c.clock.now_monotonic_us() + 10_000_000
    initial_generation = c.airtime.owner.state.generation
    assert c.time.state.quality is E.SystemTimeQuality.NETWORK_SYNCED
    assert c.time.sample.error_bound_us == 1_000_000
    assert c.airtime.owner.state.rtc_provenance is None

    turns = []
    for _ in range(12):
        previous_calls = len(rtc.calls)
        turn = scheduler.run_once()
        turns.append(turn)
        assert len(rtc.calls) - previous_calls <= 1
        if turn.work is Work.RTC and turn.update is not None:
            break

    # A frozen clock is the control: both advancing cases must complete too.
    assert [call[0] for call in rtc.calls] == ['read', 'write', 'read'], turns
    assert turns[-1].update.status is RtcRefreshStatus.VERIFIED
    assert c.time.rtc_refresh_episode is None
    assert c.time.last_refresh_monotonic_us is not None
    assert c.airtime.owner.state.generation == initial_generation + 1
    with sqlite3.connect(c.database) as db:
        blob = db.execute('SELECT state_blob FROM communicator_state').fetchone()[0]
    persisted = decode_communicator_state_v1(blob)
    assert persisted == c.airtime.owner.state
    assert persisted.rtc_provenance.verified_by_receiver_instance_id == INSTANCE
    assert persisted.rtc_provenance.drift_bound_ppm == 10


@pytest.mark.parametrize('invalidation', ['generation', 'source_error'])
def test_scheduler_rtc_continuation_rechecks_source_before_write(composition, invalidation):
    from dataclasses import replace
    from cura_receiver.communicator_scheduler import CommunicatorScheduler, Work
    from cura_receiver.runtime_time import RtcRefreshStatus
    from tests.support.fakes.ds3231 import FakeDs3231Control

    c = composition()
    rtc = FakeDs3231Control()
    now = c.clock.now_monotonic_us()
    rtc.read_results.append(Ds3231ReadResult(DR.OK, now, now, UTC // 1_000_000))
    scheduler = CommunicatorScheduler(c.communicator, chrony=None, rtc=rtc,
                                      health_interval_us=60_000_000)
    scheduler.next_airtime = now + 10_000_000
    assert scheduler.run_once().work is Work.RTC
    assert c.time.rtc_refresh_episode is not None
    generation = c.airtime.owner.state.generation
    if invalidation == 'generation':
        c.time.state = replace(c.time.state, generation=c.time.state.generation + 1)
    else:
        c.time.sample = replace(c.time.sample, error_bound_us=5_000_001)

    turn = scheduler.run_once()
    assert turn.work is Work.RTC
    assert turn.update.status is RtcRefreshStatus.TRUST_INVALIDATED
    assert [call[0] for call in rtc.calls] == ['read']
    assert c.time.rtc_refresh_episode is None
    assert c.airtime.owner.state.generation == generation
    assert c.airtime.owner.state.rtc_provenance is None


@pytest.mark.parametrize('future', ['refresh', 'retry'])
def test_scheduler_new_rtc_work_respects_future_deadline(composition, future):
    from cura_receiver.communicator_scheduler import CommunicatorScheduler, Work
    from tests.support.fakes.ds3231 import FakeDs3231Control

    c = composition()
    rtc = FakeDs3231Control()
    now = c.clock.now_monotonic_us()
    if future == 'refresh':
        c.time.last_refresh_monotonic_us = now
    else:
        c.time.next_rtc_attempt_monotonic_us = now + 10_000_000
    scheduler = CommunicatorScheduler(c.communicator, chrony=None, rtc=rtc,
                                      health_interval_us=60_000_000)
    scheduler.next_airtime = scheduler.next_health = now + 10_000_000
    assert scheduler.run_once().work is not Work.RTC
    assert rtc.calls == []
    assert c.time.rtc_refresh_episode is None


def test_duplicate_after_lost_ack_and_backlog_keep_all_occurrences(composition):
    from tests.support.builders.protocol_ingress import REVIEWED_BACKLOG_FRAME
    c = composition()
    # Peer receipt is not visible here: repeat after a confirmed physical TxDone.
    for frame in (REVIEWED_CURRENT_FRAME, REVIEWED_CURRENT_FRAME, REVIEWED_BACKLOG_FRAME):
        result = deliver(c, frame)
        assert result.finalization.published_entity.profile.ack_frame == REVIEWED_ACCEPTED_ACK
        assert result.finalization.published_entity.candidate is not None
    c.worker.request_stop(deadline_monotonic_us=c.clock.now_monotonic_us()+5_000_000)
    c.worker.join(5)
    assert not c.worker.is_alive() and c.worker.failure is None
    with sqlite3.connect(c.database) as db:
        assert db.execute('SELECT count(*) FROM reading_messages').fetchone() == (1,)
        assert db.execute('SELECT count(*) FROM message_profiles').fetchone() == (3,)


@pytest.mark.parametrize('frame,expected_ack', [
    (b'bad', None),
    (REVIEWED_CURRENT_FRAME[:-1]+bytes([REVIEWED_CURRENT_FRAME[-1]^1]), None),
])
def test_unauthenticated_packets_remain_silent(composition, frame, expected_ack):
    c = composition()
    result = deliver(c, frame)
    profile = result.finalization.published_entity.profile
    assert profile.ack_frame is expected_ack
    assert not any(command[0] == 0x83 for command in c.io.commands)
    assert c.communicator.ingress.active_occurrence is None


def test_queue_pressure_retry_then_recovery_accepts_same_packet(composition):
    from tests.support.builders.protocol_ingress import REVIEWED_RETRY_LATER_ACK
    c = composition(isolated_queue=True)
    first = deliver(c)
    assert first.finalization.published_entity is None
    assert b'\x0e\x00' + REVIEWED_RETRY_LATER_ACK in c.io.commands
    assert c.communicator.ingress.active_occurrence is None
    lease = c.queue.claim_batch(max_entities=1)
    lease.acknowledge_durable(completed_entities=1)
    second = deliver(c)
    assert second.finalization.published_entity.candidate is not None
    assert second.finalization.published_entity.profile.ack_frame == REVIEWED_ACCEPTED_ACK
    assert c.communicator.occurrence_sequence == 2


@pytest.mark.parametrize('malformed', [False, True])
def test_authenticated_rejection_transmits_exact_reviewed_ack(composition, malformed):
    from tests.support.builders.protocol_ingress import (
        authenticated_frame, REVIEWED_READING_BODY,
        REVIEWED_REJECTED_MALFORMED_ACK, REVIEWED_REJECTED_UNSUPPORTED_ACK)
    c = composition()
    frame = (authenticated_frame(body=REVIEWED_READING_BODY[:-1]) if malformed
             else authenticated_frame(control=0x30))
    expected = REVIEWED_REJECTED_MALFORMED_ACK if malformed else REVIEWED_REJECTED_UNSUPPORTED_ACK
    result = deliver(c, frame)
    entity = result.finalization.published_entity
    assert not hasattr(entity, 'candidate')
    assert entity.profile.ack_frame == expected
    assert b'\x0e\x00' + expected in c.io.commands


def test_unpublishable_accepted_occurrence_does_not_start_diagnostic_reservation(composition, monkeypatch):
    c = composition()
    runtime = runtime_dispatch(c)
    def broken_finalize(*args, **kwargs):
        raise RuntimeError('injected profile representation escape')
    monkeypatch.setattr(c.communicator.ingress, 'finalize', broken_finalize)
    c.io.buffer[:] = REVIEWED_CURRENT_FRAME
    c.io.irq = 2
    c.io.status = 0x24
    c.io.edges.append(Dio1Edge(c.clock.now_monotonic_us()*1000, 1))
    runtime.step()
    assert runtime.failure is not None
    assert c.communicator.ingress.active_occurrence is not None
    assert c.queue.snapshot().reserved_entities == 1
    assert sum(c.communicator.queue.counts[E.PersistQueueEntityKind.DIAGNOSTIC.value-1]) == 0
    assert c.radio.state is E.RadioState.SHUTDOWN
