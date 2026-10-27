import json

import pytest

from cura_receiver.generated import receiver_enums_generated as E
from cura_receiver.health_assessment import assess_health
from cura_receiver.ports.radio import Dio1Edge, Error, RadioBackendError, RadioFailure, Stage
from cura_receiver.radio import Radio
from cura_receiver.sx1262 import Sx1262, RadioEventObservation
from tests.support.fakes.os_clock import FakeOsClock
from tests.support.fakes.radio_io import PhysicalPort, Wait


@pytest.fixture
def rig():
    clock = FakeOsClock(monotonic_us=1000)
    io = PhysicalPort(clock)
    backend = Sx1262(io, clock, Wait(clock))
    radio = Radio(backend)
    radio.initialize()
    io.commands.clear()
    return radio, backend, io, clock


def test_health_reads_do_not_modify_operational_state_or_drain_events(rig):
    radio, backend, io, clock = rig
    edge = Dio1Edge(clock.now_monotonic_us()*1000, 1)
    io.edges.append(edge)
    backend_before, radio_before = vars(backend).copy(), vars(radio).copy()
    snapshot = radio.observe(E.RadioObservationTrigger.HEALTH)
    assert vars(backend) == backend_before and vars(radio) == radio_before
    assert list(io.edges) == [edge]
    assert io.commands == [b'\xc0\x00', b'\x12\x00\x00\x00', b'\x17\x00\x00\x00']
    assert snapshot.chip_mode is E.RadioChipMode.RX
    assert snapshot.capture_status is E.RadioCaptureStatus.COMPLETE


def test_busy_is_observed_without_waiting_or_spi(rig):
    radio, backend, io, clock = rig
    io.busy_forever = True
    before = clock.now_monotonic_us()
    history_before = (backend.history.sequence, backend.history.dropped, backend.history.snapshot())
    snapshot = radio.observe(E.RadioObservationTrigger.HEALTH)
    assert snapshot.busy is True and snapshot.chip_status is None
    assert snapshot.capture_status is E.RadioCaptureStatus.PARTIAL
    assert clock.now_monotonic_us() == before and io.commands == []
    assert (backend.history.sequence, backend.history.dropped, snapshot.history) == history_before


def test_snapshot_does_not_hide_packet_arrival_between_reads(rig):
    radio, _, io, clock = rig

    def incoming(command):
        if command[0] == 0xC0:
            io.irq = 2
            io.status = 0x24
            io.edges.append(Dio1Edge(clock.now_monotonic_us()*1000, 1))

    io.after_transfer = incoming
    snapshot = radio.observe(E.RadioObservationTrigger.HEALTH)
    assert snapshot.dio1_before is False and snapshot.dio1_after is True
    assert snapshot.irq_status == 2 and len(io.edges) == 1
    assert assess_health(snapshot, E.RtcHealth.PRESENT, E.RtcReadStatus.OK) is E.HealthConsistency.INDETERMINATE


def test_failed_irq_read_retains_partial_fields_and_errno_without_retry(rig):
    radio, backend, io, _ = rig
    before = vars(backend).copy()

    def failed(command):
        raise RadioBackendError(RadioFailure(Error.IO, Stage.READ_IRQ, os_errno=5))

    io.hooks[0x12] = failed
    snapshot = radio.observe(E.RadioObservationTrigger.HEALTH)
    assert snapshot.irq_status is None and snapshot.device_errors == 0
    assert snapshot.os_errno == 5 and snapshot.failure_opcode == 0x12
    assert snapshot.capture_status is E.RadioCaptureStatus.PARTIAL
    assert len([x for x in io.commands if x[0] == 0x12]) == 1
    assert vars(backend) == before


@pytest.mark.parametrize('failed_opcode', [None, 0xc0, 0x12, 0x17])
def test_later_snapshot_retains_probe_attempts_with_matching_history_loss(rig, failed_opcode):
    radio, backend, io, clock = rig
    history = backend.history
    sequence, dropped, before = history.sequence, history.dropped, history.snapshot()
    assert len(json.loads(before)) == 32
    operational_before = vars(backend).copy()

    if failed_opcode is not None:
        def failed(command):
            raise RadioBackendError(RadioFailure(Error.IO, Stage.READ_COMMAND, os_errno=5))
        io.hooks[failed_opcode] = failed

    first = radio.observe(E.RadioObservationTrigger.HEALTH)
    second = radio.observe(E.RadioObservationTrigger.RECOVERY)
    commands = [b'\xc0\x00', b'\x12\x00\x00\x00', b'\x17\x00\x00\x00']
    assert io.commands == commands * 2
    assert first.capture_status is (E.RadioCaptureStatus.COMPLETE if failed_opcode is None
                                    else E.RadioCaptureStatus.PARTIAL)
    # A snapshot describes its context before sampling. Every physical probe
    # attempt, including a failed one, must be visible in the next snapshot.
    assert first.history == before and first.history_dropped == dropped
    entries = json.loads(second.history)
    assert len(entries) == 32
    assert entries[-3:] == [[sequence+i, clock.now_monotonic_us(), 'command', opcode, 0]
                            for i, opcode in enumerate((0xc0, 0x12, 0x17), 1)]
    assert second.history_dropped == dropped+3
    assert history.sequence == sequence+6 and history.dropped == dropped+6
    assert vars(backend) == operational_before


def test_bad_status_is_preserved_instead_of_decoded_as_a_mode(rig):
    radio, _, io, _ = rig
    io.status = 0xFF
    snapshot = radio.observe(E.RadioObservationTrigger.HEALTH)
    assert snapshot.chip_status == 0xFF and snapshot.chip_mode is None
    assert snapshot.failure_code == Error.MALFORMED_RESPONSE.value


def test_event_snapshot_reuses_original_facts_without_extra_spi(rig):
    radio, _, io, clock = rig
    event = RadioEventObservation(0x20, 0x52, 0)
    edge = Dio1Edge(clock.now_monotonic_us()*1000, 1)
    snapshot = radio.observe(E.RadioObservationTrigger.PHY_REJECTION, event=event, edge=edge,
                             started=clock.now_monotonic_us()-10)
    assert snapshot.irq_status == 0x20 and snapshot.chip_status == 0x52
    assert snapshot.event_sequence == 1
    assert io.commands == []


def test_history_is_bounded_and_records_gpio_drain_and_sequence_gaps(rig):
    radio, backend, io, clock = rig
    for index in range(40):
        backend.history.record('command', index)
    io.edges.extend([Dio1Edge(clock.now_monotonic_us()*1000, 1),
                     Dio1Edge(clock.now_monotonic_us()*1000, 4)])
    io.resynchronize_events(deadline_monotonic_us=clock.now_monotonic_us()+1000)
    snapshot = radio.observe(E.RadioObservationTrigger.HEALTH)
    entries = json.loads(snapshot.history)
    assert len(entries) == 32 and snapshot.history_dropped > 0
    assert any(entry[2:] == ['edge_gap', 2, 0] for entry in entries)
    assert entries[-1][2:] == ['edge', 4, clock.now_monotonic_us()*1000]


def test_closed_radio_reports_unavailable_without_reopening(rig):
    radio, backend, io, _ = rig
    backend.close()
    before = list(io.calls)
    snapshot = radio.observe(E.RadioObservationTrigger.FINAL_FAILURE)
    assert snapshot.capture_status is E.RadioCaptureStatus.UNAVAILABLE
    assert snapshot.chip_status is None and snapshot.dio1_before is None
    assert io.calls == before


@pytest.mark.parametrize('irq', [0x20, 0x40, 0x60, 0x22, 0x42])
def test_each_rejection_has_one_pre_mutation_snapshot_and_separate_counts(rig, irq):
    radio, _, io, clock = rig
    saved = []
    radio.observation_sink = saved.append
    io.irq, io.status = irq, 0x24
    io.edges.append(Dio1Edge(clock.now_monotonic_us()*1000, 1))

    def before_standby(command):
        assert len(saved) == 1 and saved[0].irq_status == irq
        assert saved[0].chip_status == 0x24
        assert io.irq == irq
        del io.hooks[0x80]

    io.hooks[0x80] = before_standby
    result = radio.receive(deadline_monotonic_us=clock.now_monotonic_us()+500000)
    assert result.state is E.RadioState.RX_SINGLE
    assert len(saved) == 1 and saved[0].trigger is E.RadioObservationTrigger.PHY_REJECTION
    counters = radio.counters
    assert counters.header_errors_observed == counters.header_errors_handled == bool(irq & 0x20)
    assert counters.crc_errors_observed == counters.crc_errors_handled == bool(irq & 0x40)
    assert counters.recovery_attempts == 0


def test_failed_rearm_is_observed_but_not_counted_as_handled(rig):
    radio, _, io, clock = rig
    radio.observation_sink = lambda snapshot: None
    io.irq, io.status = 0x20, 0x52
    io.edges.append(Dio1Edge(clock.now_monotonic_us()*1000, 1))

    def failed(command):
        raise RadioBackendError(RadioFailure(Error.IO, Stage.WRITE_COMMAND, os_errno=5))

    io.hooks[0x82] = failed
    result = radio.receive(deadline_monotonic_us=clock.now_monotonic_us()+500000)
    assert result.state is E.RadioState.RECOVERING
    assert radio.counters.header_errors_observed == 1
    assert radio.counters.header_errors_handled == 0


def test_zero_irq_snapshot_precedes_receiver_rearm(rig):
    radio, _, io, clock = rig
    saved = []
    radio.observation_sink = saved.append
    io.edges.append(Dio1Edge(clock.now_monotonic_us()*1000, 1))
    radio.receive(deadline_monotonic_us=clock.now_monotonic_us()+500000)
    assert len(saved) == 1
    assert saved[0].trigger is E.RadioObservationTrigger.UNEXPECTED_IRQ
    assert saved[0].irq_status == 0 and saved[0].software_state is E.RadioState.RX_EVENT_PENDING


def test_recovery_records_before_soft_restore_and_hard_reset(rig):
    radio, _, io, clock = rig
    saved = []
    radio.observation_sink = saved.append
    io.irq, io.status = 0x20, 0x52
    io.edges.append(Dio1Edge(clock.now_monotonic_us()*1000, 1))

    def failed(command):
        raise RadioBackendError(RadioFailure(Error.IO, Stage.WRITE_COMMAND, os_errno=5))

    io.hooks[0x82] = failed
    radio.receive(deadline_monotonic_us=clock.now_monotonic_us()+500000)
    radio.recover()
    assert [x.trigger for x in saved] == [E.RadioObservationTrigger.PHY_REJECTION,
        E.RadioObservationTrigger.RECOVERY, E.RadioObservationTrigger.RECOVERY,
        E.RadioObservationTrigger.FINAL_FAILURE]
    assert saved[1].software_state is E.RadioState.RECOVERING
    assert saved[2].software_state is E.RadioState.RECOVERING
    assert radio.counters.header_errors_handled == 0


@pytest.mark.parametrize('soft_failure_opcode', [None, 0x80, 0x82])
def test_recovery_snapshot_retains_prior_set_rx_before_resetting_attempt_timing(
        rig, monkeypatch, soft_failure_opcode):
    radio, backend, io, clock = rig
    saved, failed_attempts = [], []
    radio.observation_sink = saved.append
    previous = backend.last_set_rx_issued_us
    wait_edge = io.wait_edge

    def failed_wait(*, deadline_monotonic_us):
        monkeypatch.setattr(io, 'wait_edge', wait_edge)
        raise RadioBackendError(RadioFailure(Error.IO, Stage.WAIT_IRQ, os_errno=5))

    monkeypatch.setattr(io, 'wait_edge', failed_wait)
    clock.advance_elapsed_us(10)
    assert radio.receive(deadline_monotonic_us=clock.now_monotonic_us()+500_000).state is E.RadioState.RECOVERING
    assert backend.last_set_rx_issued_us == previous

    if soft_failure_opcode is not None:
        def failed_restore(command):
            del io.hooks[soft_failure_opcode]
            failed_attempts.append(backend.last_set_rx_issued_us)
            raise RadioBackendError(RadioFailure(Error.IO, Stage.WRITE_COMMAND, os_errno=5))
        io.hooks[soft_failure_opcode] = failed_restore

    result = radio.recover()
    assert result.state is E.RadioState.RX_SINGLE
    assert saved[0].last_set_rx_issued_monotonic_us == previous
    assert len(saved) == (1 if soft_failure_opcode is None else 2)
    if failed_attempts:
        # A failed pre-SetRx attempt has no new timestamp; a failed SetRx
        # command retains its actual issue time for the next recovery snapshot.
        assert failed_attempts[0] == (None if soft_failure_opcode == 0x80 else previous+10)
        assert saved[1].last_set_rx_issued_monotonic_us == failed_attempts[0]
    assert result.t6_set_rx_issued_monotonic_us == backend.last_set_rx_issued_us
    assert result.t6_set_rx_issued_monotonic_us > previous


@pytest.mark.parametrize('opcode', [0x17, 0xc0])
def test_partial_event_retains_original_irq_and_counts_before_later_read_failure(rig, opcode):
    radio, _, io, clock = rig
    saved = []
    radio.observation_sink = saved.append
    io.irq, io.status = 0x60, 0x24
    io.edges.append(Dio1Edge(clock.now_monotonic_us()*1000, 1))
    def fail(command):
        del io.hooks[opcode]
        io.irq = 0  # A later recovery read cannot reconstruct original evidence.
        raise RadioBackendError(RadioFailure(Error.IO, Stage.READ_COMMAND, os_errno=5))
    io.hooks[opcode] = fail
    radio.receive(deadline_monotonic_us=clock.now_monotonic_us()+500_000)
    assert len(saved) == 1
    assert saved[0].irq_status == 0x60
    assert saved[0].capture_status is E.RadioCaptureStatus.PARTIAL
    assert saved[0].failure_code == Error.IO.value and saved[0].os_errno == 5
    assert saved[0].device_errors == (None if opcode == 0x17 else 0)
    assert radio.counters.header_errors_observed == radio.counters.crc_errors_observed == 1
    radio.recover()
    assert radio.counters.header_errors_observed == radio.counters.crc_errors_observed == 1
    assert radio.counters.header_errors_handled == radio.counters.crc_errors_handled == 0
