"""Owner tests run the real command layer over its physical SPI/GPIO port."""

import errno

import pytest

from cura_receiver.ports.radio import Dio1Edge, Error, Outcome, RadioBackendError, RadioFailure, RadioTxAuthorization, Stage
from cura_receiver.generated.receiver_enums_generated import AckTxResult
from cura_receiver.radio import Radio, State
from cura_receiver.radio_diagnostics import Reason, Severity
from cura_receiver.sx1262 import Sx1262
from tests.support.fakes.os_clock import FakeOsClock
from tests.support.fakes.radio_io import PhysicalPort, Wait


@pytest.fixture
def owner():
    clock = FakeOsClock(monotonic_us=10000)
    io = PhysicalPort(clock)
    radio = Radio(Sx1262(io, clock, Wait(clock)))
    return radio, io, clock


def signal(io, clock, irq=2):
    io.irq = irq
    io.status = 0x24
    io.edges.append(Dio1Edge(clock.now_monotonic_us() * 1000, 1))


def fail(code=Error.IO, *, opcode=0, outcome=Outcome.DEFINITELY_NOT_APPLIED, missing=False):
    def raise_error(_=None):
        raise RadioBackendError(RadioFailure(code, Stage.WRITE_COMMAND, outcome, opcode,
            os_errno=errno.ENODEV if missing else errno.EIO if code is Error.IO else None,
            hardware_missing=missing))
    return raise_error


# RX_SINGLE is observable only after the complete literal profile and confirmed SetRx.
def test_initialization_state(owner):
    radio, io, _ = owner
    seen = []
    io.after_transfer = lambda command: seen.append((command, radio.state))
    result = radio.initialize()
    assert result.state is State.RX_SINGLE
    assert all(state is State.INITIALIZING for _, state in seen)
    assert [command for command, _ in seen][-2:] == [b"\x82\x00\x00\x00", b"\xc0\x00"]
    assert io.registers[0x0736] & 4
    assert io.registers[0x08AC] == 0x96
    assert result.t6_set_rx_issued_monotonic_us == 11004


# Missing hardware and reachable initialization failures remain distinct immutable terminals.
@pytest.mark.parametrize("missing", [False, True])
def test_initialization_terminal(owner, missing):
    radio, io, _ = owner
    io.hooks[0x8A] = fail(missing=missing)
    result = radio.initialize()
    expected = State.HARDWARE_MISSING if missing else State.INITIALIZATION_FAILED
    assert result.state is expected
    assert len(result.episodes) == 1
    assert result.episodes[0].severity is Severity.FATAL
    calls = io.calls.copy()
    for operation in (radio.initialize, radio.rearm, lambda: radio.receive(deadline_monotonic_us=999999)):
        with pytest.raises(RuntimeError):
            operation()
    assert io.calls == calls
    assert radio.state is expected
    assert io.calls[-1] == ("close",)


# Failed startup preserves confirmed cleanup, or every distinct cleanup/close failure.
@pytest.mark.parametrize("fault", ["safe", "standby", "close", "both"])
def test_startup_cleanup_result_and_diagnostics(owner, monkeypatch, fault):
    radio, io, clock = owner
    def initialization_failure(_):
        if fault in ("standby", "both"):
            io.hooks[0x80] = fail(Error.BUSY_TIMEOUT, opcode=0x80)
        fail(opcode=0x8A)()
    io.hooks[0x8A] = initialization_failure
    original_close = io.close
    def close():
        original_close()
        if fault in ("close", "both"):
            raise RadioBackendError(RadioFailure(Error.IO, Stage.DETACH_IRQ, os_errno=errno.EBADF))
    monkeypatch.setattr(io, "close", close)
    result = radio.initialize()
    assert result.state is State.INITIALIZATION_FAILED
    assert result.safe_shutdown is (fault == "safe")
    assert result.episodes[0].operation.name == "INITIALIZE"
    assert result.episodes[0].context.trigger_detail.command_opcode == 0x8A
    expected = [Error.IO]
    if fault in ("standby", "both"):
        expected.append(Error.BUSY_TIMEOUT)
    if fault in ("close", "both"):
        expected.append(Error.IO)
        assert result.episodes[-1].context.trigger_detail.backend_status == errno.EBADF
        assert result.episodes[-1].context.trigger_detail.stage is Stage.DETACH_IRQ
    assert [e.error_code for e in result.episodes] == expected
    assert all(e.operation.name == "CLEANUP" for e in result.episodes[1:])
    assert all(e.severity is Severity.FATAL and e.context.terminal_state is State.INITIALIZATION_FAILED for e in result.episodes)
    assert not any(c[0] in (0x82, 0x83) for c in io.commands)
    calls = io.calls.copy()
    later = radio.shutdown()
    assert later.state is result.state and later.safe_shutdown is result.safe_shutdown
    assert later.episodes == () and io.calls == calls
    assert io.calls.count(("close",)) == 1


# Missing resources have no safety assessment, but a release failure remains a separate fact.
@pytest.mark.parametrize("close_fails", [False, True])
def test_missing_startup_cleanup_assessment(owner, monkeypatch, close_fails):
    radio, io, _ = owner
    monkeypatch.setattr(io, "open", lambda *a, **kw: fail(missing=True)())
    if close_fails:
        monkeypatch.setattr(io, "close", fail())
    result = radio.initialize()
    assert result.state is State.HARDWARE_MISSING
    assert result.safe_shutdown is (False if close_fails else None)
    assert len(result.episodes) == (2 if close_fails else 1)
    if close_fails:
        assert result.episodes[-1].operation.name == "CLEANUP"
        assert result.episodes[-1].severity is Severity.FATAL
    assert io.commands == []


# Exhausting startup leaves no new cleanup budget or extra reset, but still releases handles.
def test_startup_cleanup_uses_remaining_deadline(owner):
    radio, io, clock = owner
    def exhaust(_):
        clock.advance_elapsed_us(2000000)
        fail(opcode=0x8A)()
    io.hooks[0x8A] = exhaust
    result = radio.initialize()
    assert result.state is State.INITIALIZATION_FAILED and result.safe_shutdown is False
    assert result.episodes[0].error_code is Error.IO
    assert all(e.error_code is Error.DEADLINE and e.operation.name == "CLEANUP" for e in result.episodes[1:])
    assert io.commands[-1] == b"\x8a\x01"
    assert len([c for c in io.calls if c[:2] == ("reset", True)]) == 1
    assert io.calls[-1] == ("close",)


# A static BUSY-high input blocks startup and SPI; only a fresh owner can start after restoration.
def test_manual_busy_held_startup_and_fresh_nominal_restoration(owner):
    radio, io, clock = owner
    io.busy_forever = True
    started = clock.now_monotonic_us()
    result = radio.initialize()
    assert result.state is State.INITIALIZATION_FAILED
    assert len(result.episodes) == 2
    assert result.episodes[0].error_code is Error.BUSY_TIMEOUT
    assert result.episodes[0].severity is Severity.FATAL
    assert result.episodes[0].context.trigger_detail.stage is Stage.WAIT_BUSY
    assert result.safe_shutdown is False
    assert result.episodes[1].operation.name == "CLEANUP"
    assert result.episodes[1].error_code is Error.BUSY_TIMEOUT
    assert result.episodes[1].severity is Severity.FATAL
    assert result.busy.timeout_count == 2
    assert result.tx is None and result.t6_set_rx_issued_monotonic_us is None
    assert 0 < clock.now_monotonic_us() - started <= 2_000_000
    assert io.commands == [] and io.calls[-1] == ("close",)
    calls = io.calls.copy()
    assert radio.shutdown().safe_shutdown is False
    io.busy_forever = False
    with pytest.raises(RuntimeError, match="terminal"):
        radio.initialize()
    assert io.calls == calls
    restored_io = PhysicalPort(clock)
    restored = Radio(Sx1262(restored_io, clock, Wait(clock)))
    assert restored.initialize().state is State.RX_SINGLE
    assert restored.shutdown().safe_shutdown is True


# An idle single-receive wait is normal and does not invent a radio deadline failure.
def test_idle_receive(owner):
    radio, io, _ = owner
    radio.initialize()
    before = len(io.commands)
    result = radio.receive(deadline_monotonic_us=20000)
    assert result.state is State.RX_SINGLE and result.packet is None and result.episodes == ()
    assert len(io.commands) == before


# Packet bytes are independent before later commands, with kernel T0 and exact copy T2.
def test_packet_snapshot_and_rearm(owner):
    radio, io, clock = owner
    radio.initialize()
    signal(io, clock)

    def mutate(command):
        if command[0] == 0x1E:
            io.buffer[:] = b"mutate"

    io.after_transfer = mutate
    result = radio.receive(deadline_monotonic_us=20000)
    assert result.state is State.RX_EVENT_PENDING
    assert result.packet.frame == b"packet"
    assert result.packet.received_at_monotonic_us == 11004
    assert result.packet.rssi_dbm_x2 == -200 and result.packet.snr_db_x4 == -12
    assert result.packet.t1_handler_started_monotonic_us <= result.packet.t2_packet_copied_monotonic_us
    assert io.irq == 0
    result2 = radio.rearm()
    assert result2.state is State.RX_SINGLE
    assert result2.t6_set_rx_issued_monotonic_us == 11004
    assert result.packet.frame == b"packet"


# Header/CRC combinations discard bytes, clear exact bits and rearm without diagnostics.
@pytest.mark.parametrize("irq", [0x20, 0x40, 0x42, 0x60, 0x62, 0x22])
def test_normal_rx_error_irq(owner, irq):
    radio, io, clock = owner
    radio.initialize()
    io.commands.clear()
    signal(io, clock, irq)
    result = radio.receive(deadline_monotonic_us=20000)
    assert result.state is State.RX_SINGLE and result.packet is None and result.episodes == ()
    assert b"\x02" + irq.to_bytes(2, "big") in io.commands
    assert not any(command[0] == 0x1E for command in io.commands)
    assert radio.counters.header_errors == bool(irq & 0x20)
    assert radio.counters.crc_errors == bool(irq & 0x40)


# Impossible RX IRQ combinations produce one direct warning after confirmed RX restoration.
@pytest.mark.parametrize("irq", [0, 1, 3, 0x200, 0x202, 0x8000, 0xFFFF])
def test_unexpected_rx_irq(owner, irq):
    radio, io, clock = owner
    radio.initialize()
    signal(io, clock, irq)
    result = radio.receive(deadline_monotonic_us=20000)
    assert result.state is State.RX_SINGLE
    assert len(result.episodes) == 1
    assert result.episodes[0].error_code is Error.UNEXPECTED_IRQ
    assert result.episodes[0].severity is Severity.WARN
    assert radio.counters.recovery_attempts == 0


# Every receive primitive failure suppresses copying/continuation and enters one recovery.
@pytest.mark.parametrize("opcode", [0x12, 0x17, 0x13, 0x1E, 0x14, 0x0D, 0x1D, 0x02])
def test_receive_fault_enters_recovery(owner, opcode):
    radio, io, clock = owner
    radio.initialize()
    signal(io, clock)
    io.hooks[opcode] = fail(outcome=Outcome.UNCERTAIN)
    result = radio.receive(deadline_monotonic_us=20000)
    assert result.state is State.RECOVERING and not result.episodes
    # A complete copied snapshot survives a subsequent IRQ-clear failure.
    assert (result.packet is not None) is (opcode == 0x02)
    if result.packet is not None:
        assert result.packet.frame == b"packet"
    assert radio.counters.recovery_attempts == 1
    assert radio.counters.recovery_attempts_by_reason == (0, 1, 0, 0, 0, 0, 0, 0)


# Re-arm failure selects the SetRx/profile reason ahead of the underlying SPI trigger.
@pytest.mark.parametrize("opcode,reason", [(0x82, Reason.SET_RX_FAILED), (0x8C, Reason.RX_PROFILE_RESTORE_FAILED)])
def test_rearm_failure_reason(owner, opcode, reason):
    radio, io, clock = owner
    radio.initialize()
    signal(io, clock)
    radio.receive(deadline_monotonic_us=20000)
    io.hooks[opcode] = fail()
    result = radio.rearm()
    assert result.state is State.RECOVERING
    assert radio.counters.recovery_attempts_by_reason[reason.value - 1] == 1


# A new edge raised by SetRx survives its confirmation and becomes pending immediately.
def test_immediate_rx_edge(owner):
    radio, io, clock = owner

    def complete(command):
        if command[0] == 0x82:
            signal(io, clock)

    io.after_transfer = complete
    assert radio.initialize().state is State.RX_EVENT_PENDING
    assert radio.receive(deadline_monotonic_us=20000).packet.frame == b"packet"


def received(owner):
    radio, io, clock = owner
    radio.initialize()
    signal(io, clock)
    radio.receive(deadline_monotonic_us=20000)
    return radio, io, clock


def transmitting(owner):
    radio, io, clock = received(owner)
    radio.prepare_ack(b"selected ACK", occurrence_sequence=17)
    assert radio.start_ack(RadioTxAuthorization(1000000, 17, 500)).state is State.TX_ACTIVE
    return radio, io, clock


# Caller charging separates exact WriteBuffer bytes from every TX-profile command.
def test_ack_order_and_complete_rx_restoration(owner):
    radio, io, clock = received(owner)
    io.calls.clear()
    radio.prepare_ack(b"selected ACK")
    io.calls.append(("caller charged allowance",))
    assert radio.start_ack(RadioTxAuthorization(1000000)).state is State.TX_ACTIVE
    commands = [call[1] for call in io.calls if call[0] == "spi"]
    assert commands[0] == b"\x0e\x00selected ACK"
    charged = io.calls.index(("caller charged allowance",))
    assert all(call[1][0] not in (0x8C, 0x83) for call in io.calls[:charged] if call[0] == "spi")
    assert b"\x8c\x00\x08\x00\x0c\x01\x01" in commands
    assert io.registers[0x0736] & 4 == 0
    clock.advance_elapsed_us(62000)
    signal(io, clock, 1)
    result = radio.finish_ack()
    assert result.state is State.RX_SINGLE and not result.episodes
    assert result.tx.ack_tx_result is AckTxResult.TX_DONE
    assert result.tx.t4_set_tx_attempted_monotonic_us == 11004
    assert result.tx.t5_tx_done_monotonic_us == 73004
    assert result.t6_set_rx_issued_monotonic_us == 73004
    assert io.registers[0x0736] & 4 and io.registers[0x08AC] == 0x96


# Definite profile failures stop before SetTx and return refund-eligible facts.
@pytest.mark.parametrize("opcode", [0x80, 0x8A, 0x86, 0x8B, 0x8C, 0x8E, 0x93, 0x9F, 0xA0, 0x08, 0x0D, 0x1D, 0x17, 0x83])
def test_definite_pre_settx_failure(owner, opcode):
    radio, io, _ = received(owner)
    radio.prepare_ack(b"selected ACK")

    def once(_):
        del io.hooks[opcode]
        fail()()

    io.hooks[opcode] = once
    result = radio.start_ack(RadioTxAuthorization(1000000))
    assert result.state is State.RX_SINGLE
    assert result.tx.ack_tx_result is AckTxResult.SET_TX_FAILED
    assert result.tx.facts.set_tx_outcome is Outcome.DEFINITELY_NOT_APPLIED
    assert not result.tx.facts.profile_uncertain
    assert (result.tx.t4_set_tx_attempted_monotonic_us is not None) is (opcode == 0x83)
    assert result.tx.t5_tx_done_monotonic_us is None
    assert len(result.episodes) == 1 and result.episodes[0].severity is Severity.ERROR


# Every uncertain profile primitive retains charge without inventing a SetTx timestamp.
@pytest.mark.parametrize("opcode", [0x80, 0x8A, 0x86, 0x8B, 0x8C, 0x8E, 0x93, 0x9F, 0xA0, 0x08, 0x0D, 0x1D, 0x17, 0x83])
def test_uncertain_profile_or_settx(owner, opcode):
    radio, io, _ = received(owner)
    radio.prepare_ack(b"selected ACK")
    io.commands.clear()
    io.hooks[opcode] = fail(outcome=Outcome.UNCERTAIN)
    result = radio.start_ack(RadioTxAuthorization(1000000, 17, 500))
    assert result.state is State.RECOVERING and not result.episodes
    assert result.tx.ack_tx_result is AckTxResult.TX_UNCONFIRMED
    assert result.tx.facts.profile_uncertain is (opcode != 0x83)
    assert (result.tx.t4_set_tx_attempted_monotonic_us is not None) is (opcode == 0x83)
    assert radio.counters.recovery_attempts_by_reason[3] == 1
    assert not any(command[0] == 0x82 for command in io.commands)


# The earlier caller grant deadline prevents SetTx even when profile setup has room.
def test_earlier_authorization_deadline(owner):
    radio, io, clock = received(owner)
    radio.prepare_ack(b"selected ACK")
    io.commands.clear()
    result = radio.start_ack(RadioTxAuthorization(clock.now_monotonic_us()))
    assert result.tx.ack_tx_result is AckTxResult.SET_TX_FAILED
    assert result.tx.t4_set_tx_attempted_monotonic_us is None
    assert not any(command[0] == 0x83 for command in io.commands)


# Terminal kernel edges are inclusive at the bound and late by even one ns are unknown.
@pytest.mark.parametrize("offset,result_code", [(0, AckTxResult.TX_DONE), (1, AckTxResult.TX_UNCONFIRMED)])
def test_tx_deadline_edge(owner, offset, result_code):
    radio, io, clock = transmitting(owner)
    clock.advance_elapsed_us(249075 + 100)  # Kernel edge can be serviced later.
    io.irq = 1
    io.status = 0x24
    io.edges.append(Dio1Edge((11004 + 249075) * 1000 + offset, 2))
    result = radio.finish_ack()
    assert result.tx.ack_tx_result is result_code
    assert result.state is (State.RX_SINGLE if offset == 0 else State.RECOVERING)


# A chip timeout is a normal terminal event; absence of an event remains uncertain.
@pytest.mark.parametrize("timeout_irq", [False, True])
def test_timeout_vs_missing_irq(owner, timeout_irq):
    radio, io, clock = transmitting(owner)
    clock.advance_elapsed_us(249075)
    if timeout_irq:
        signal(io, clock, 0x200)
    result = radio.finish_ack()
    assert result.tx.ack_tx_result is (AckTxResult.TX_TIMEOUT if timeout_irq else AckTxResult.TX_UNCONFIRMED)
    assert result.tx.t5_tx_done_monotonic_us is None
    assert result.state is (State.RX_SINGLE if timeout_irq else State.RECOVERING)
    assert not result.episodes


# Terminal success remains known when IRQ clearing subsequently fails.
def test_known_tx_done_survives_cleanup_fault(owner):
    radio, io, clock = transmitting(owner)
    signal(io, clock, 1)
    io.hooks[0x02] = fail(outcome=Outcome.UNCERTAIN)
    result = radio.finish_ack()
    assert result.state is State.RECOVERING
    assert result.tx.ack_tx_result is AckTxResult.TX_DONE
    assert result.tx.t5_tx_done_monotonic_us == 11004


# Replay the Pi timeout pair through the owner; fresh standby precedes all later writes.
@pytest.mark.parametrize("transmit", [False, True])
def test_captured_timeout_observation(owner, transmit):
    radio, io, clock = transmitting(owner) if transmit else owner
    if not transmit:
        radio.initialize()
        radio.backend.arm_receive(1000000, timeout_ticks=6400)
    clock.advance_elapsed_us(100000)
    signal(io, clock, 0x200)
    io.status = 0x26
    io.commands.clear()
    result = radio.finish_ack() if transmit else radio.receive(deadline_monotonic_us=1000000)
    assert result.state is State.RX_SINGLE
    assert [c.hex() for c in io.commands[:5]] == ["12000000", "17000000", "c000", "8000", "c000"]
    assert io.irq == 0 and io.status == 0x54
    assert io.registers[0x0736] & 4 and io.registers[0x08AC] == 0x96
    assert radio.counters.recovery_attempts == 0
    if transmit:
        assert result.tx.ack_tx_result is AckTxResult.TX_TIMEOUT
        assert result.tx.facts.set_tx_outcome is Outcome.CONFIRMED_APPLIED
        assert result.tx.t5_tx_done_monotonic_us is None
        assert result.episodes == ()
    else:
        assert result.tx is None and result.packet is None
        assert len(result.episodes) == 1
        assert result.episodes[0].error_code is Error.UNEXPECTED_IRQ
        assert result.episodes[0].context.trigger_detail.irq_status == 0x200
        assert result.episodes[0].severity is Severity.WARN


# Both immediate RX and TX completion keep their kernel edge until normal owner handling.
@pytest.mark.parametrize("transmit", [False, True])
def test_immediate_timeout_reaches_owner_once(owner, transmit):
    radio, io, clock = received(owner) if transmit else owner
    if transmit:
        radio.prepare_ack(b"selected ACK")
    edge = None
    def complete(command):
        nonlocal edge
        if command[0] == (0x83 if transmit else 0x82):
            io.after_transfer = None
            io.status, io.irq = 0x26, 0x200
            edge = Dio1Edge(clock.now_monotonic_us() * 1000, 1)
            io.edges.append(edge)
    io.after_transfer = complete
    started = radio.start_ack(RadioTxAuthorization(1000000)) if transmit else radio.initialize()
    assert started.state is (State.TX_ACTIVE if transmit else State.RX_EVENT_PENDING)
    assert not io.edges  # The backend/owner has retained, not lost, this edge.
    result = radio.finish_ack() if transmit else radio.receive(deadline_monotonic_us=1000000)
    assert result.state is State.RX_SINGLE
    if transmit:
        assert result.tx.ack_tx_result is AckTxResult.TX_TIMEOUT
    else:
        assert result.episodes[0].context.trigger_detail.irq_status == 0x200
    assert radio.backend.wait_edge(deadline_monotonic_us=1000000) is None


# Runtime event correlation rejects unexplained timeout, failure status, mode and error conflicts.
@pytest.mark.parametrize("transmit", [False, True])
@pytest.mark.parametrize("status,irq,errors,code", [
    (0x26, 0, 0, Error.COMMAND_STATUS), (0x26, 0x201, 0, Error.COMMAND_STATUS),
    (0x26, 2, 0, Error.COMMAND_STATUS), (0x56, 0x200, 0, Error.COMMAND_STATUS),
    (0x28, 0x200, 0, Error.COMMAND_STATUS), (0x2A, 0x200, 0, Error.COMMAND_STATUS),
    (0x2C, 0x200, 0, Error.COMMAND_STATUS), (0x26, 0x200, 0x40, Error.DEVICE_ERROR),
    (0xFF, 0x200, 0, Error.MALFORMED_RESPONSE),
])
def test_runtime_event_conflicting_evidence(owner, transmit, status, irq, errors, code):
    radio, io, clock = transmitting(owner) if transmit else owner
    if not transmit:
        radio.initialize()
    signal(io, clock, irq)
    io.status, io.errors = status, errors
    io.commands.clear()
    result = radio.finish_ack() if transmit else radio.receive(deadline_monotonic_us=1000000)
    assert result.state is State.RECOVERING
    assert all(c[0] in (0x12, 0x17, 0xC0) for c in io.commands)
    if transmit:
        assert result.tx.ack_tx_result is AckTxResult.TX_UNCONFIRMED
    episode = radio.shutdown().episodes[0]
    assert episode.error_code is code
    assert episode.context.trigger_detail.chip_status == status
    if code is not Error.MALFORMED_RESPONSE:
        assert episode.context.trigger_detail.irq_status == irq
        assert episode.context.trigger_detail.device_errors == errors


# Shutdown intent at the immediate-event wait boundary suppresses further normal operation.
def test_shutdown_during_immediate_completion_wait(owner):
    radio, io, clock = owner
    original = io.wait_edge
    def complete(command):
        if command[0] == 0x82:
            io.status, io.irq = 0x26, 0x200
            io.edges.append(Dio1Edge(clock.now_monotonic_us() * 1000, 1))
    def wait(*, deadline_monotonic_us):
        if io.irq == 0x200:
            radio.request_shutdown()
        return original(deadline_monotonic_us=deadline_monotonic_us)
    io.after_transfer, io.wait_edge = complete, wait
    result = radio.initialize()
    assert result.state is State.SHUTDOWN and result.safe_shutdown is True
    assert sum(c[0] == 0x82 for c in io.commands) == 1
    assert not any(c[0] == 0x83 for c in io.commands)


# The timeout pair does not excuse stale/future RX edges or stale/late TX edges.
@pytest.mark.parametrize("transmit,offset,code", [
    (False, -1, Error.MALFORMED_RESPONSE), (False, 1000, Error.MALFORMED_RESPONSE),
    (True, -1, Error.UNEXPECTED_IRQ), (True, 249075001, Error.DEADLINE),
])
def test_timeout_pair_edge_rejection(owner, transmit, offset, code):
    radio, io, clock = transmitting(owner) if transmit else owner
    if not transmit:
        radio.initialize()
    io.status, io.irq = 0x26, 0x200
    io.edges.append(Dio1Edge(clock.now_monotonic_us() * 1000 + offset, 1))
    if transmit and offset > 0:
        clock.advance_elapsed_us(250000)
    io.commands.clear()
    result = radio.finish_ack() if transmit else radio.receive(deadline_monotonic_us=1000000)
    assert result.state is State.RECOVERING and io.commands == []
    if transmit:
        assert result.tx.ack_tx_result is AckTxResult.TX_UNCONFIRMED
    assert radio.shutdown().episodes[0].error_code is code


# A confirmed timeout remains known even if standby, IRQ clearing or profile restoration fails.
@pytest.mark.parametrize("opcode", [0x80, 0x02, 0x8A])
def test_known_tx_timeout_survives_restore_fault(owner, opcode):
    radio, io, clock = transmitting(owner)
    signal(io, clock, 0x200)
    io.status = 0x26
    io.hooks[opcode] = fail(outcome=Outcome.UNCERTAIN)
    result = radio.finish_ack()
    assert result.state is State.RECOVERING
    assert result.tx.ack_tx_result is AckTxResult.TX_TIMEOUT
    assert result.tx.t5_tx_done_monotonic_us is None
    assert result.tx.facts.set_tx_outcome is Outcome.CONFIRMED_APPLIED


# External real airtime policy refunds only definite non-start and durably retains uncertainty.
@pytest.mark.parametrize("uncertain", [False, True])
def test_external_airtime_settlement(airtime_component, uncertain):
    from cura_receiver.tx_airtime import AirtimeReason, TxCertainty
    from tests.support.builders.persistence_control import state

    policy, _, _, clock, _ = airtime_component(initial_state=state())
    assert policy.acquire_grant(deadline_monotonic_us=5000100).reason is AirtimeReason.ALLOWED
    io = PhysicalPort(clock)
    radio = Radio(Sx1262(io, clock, Wait(clock)))
    radio.initialize()
    signal(io, clock)
    radio.receive(deadline_monotonic_us=clock.now_monotonic_us() + 100)
    radio.prepare_ack(b"selected ACK")
    spend = policy.try_spend()
    assert spend.token is not None

    def once(_):
        del io.hooks[0x8C]
        fail(outcome=Outcome.UNCERTAIN if uncertain else Outcome.DEFINITELY_NOT_APPLIED)()

    io.hooks[0x8C] = once
    result = radio.start_ack(RadioTxAuthorization(clock.now_monotonic_us() + 10000))
    facts = result.tx.facts
    certainty = TxCertainty.UNCERTAIN if facts.profile_uncertain or facts.set_tx_outcome is Outcome.UNCERTAIN else TxCertainty.NOT_STARTED
    policy.report_tx(spend.token, certainty)
    assert policy.settle(precharge=False, deadline_monotonic_us=clock.now_monotonic_us() + 5000000).reason is AirtimeReason.STATE_READY
    assert sum(bucket.charged_airtime_us for bucket in policy.state.buckets) == (67866 if uncertain else 0)


def needs_recovery(owner):
    radio, io, _ = received(owner)
    radio.prepare_ack(b"selected ACK", occurrence_sequence=17)
    io.hooks[0x8C] = fail(outcome=Outcome.UNCERTAIN)
    radio.start_ack(RadioTxAuthorization(1000000, 17, 500))
    del io.hooks[0x8C]
    return owner


# Soft success, hard success and exhaustion each finalize exactly one original episode.
@pytest.mark.parametrize("path", ["soft", "hard", "exhausted", "missing"])
def test_bounded_recovery(owner, path):
    radio, io, _ = needs_recovery(owner)
    resets = sum(call[0] == "reset" for call in io.calls)
    count = 0

    def recovery_fault(_):
        nonlocal count
        count += 1
        if path == "hard":
            del io.hooks[0x80]
        fail(missing=path == "missing")()

    if path != "soft":
        io.hooks[0x80] = recovery_fault
    result = radio.recover()
    expected = State.RECOVERY_EXHAUSTED if path == "exhausted" else State.HARDWARE_MISSING if path == "missing" else State.RX_SINGLE
    assert result.state is expected
    assert len(result.episodes) == 1
    episode = result.episodes[0]
    assert episode.error_code is Error.IO
    assert episode.context.trigger_detail.command_opcode == 0x8C
    assert episode.context.recovery_reason is Reason.TX_OUTCOME_UNCERTAIN
    assert episode.context.related_occurrence_sequence == 17
    assert episode.context.airtime_bucket_expiration_utc_us == 500
    assert (episode.context.last_recovery_failure_detail is None) is (path == "soft")
    assert radio.counters.recovery_attempts == 1
    assert radio.counters.recovery_successes == (expected is State.RX_SINGLE)
    assert radio.counters.recovery_failures == (expected is not State.RX_SINGLE)
    assert sum(call[0] == "reset" for call in io.calls) - resets == (2 if path in ("hard", "exhausted") else 0)
    assert result.tx.ack_tx_result is AckTxResult.TX_UNCONFIRMED
    if expected is State.RX_SINGLE:
        assert result.t6_set_rx_issued_monotonic_us is not None
        assert io.registers[0x0736] & 4
    else:
        before = io.calls.copy()
        with pytest.raises(RuntimeError):
            radio.recover()
        assert io.calls == before


# A new event after recovery SetRx is retained for the next packet instead of cleared.
def test_event_immediately_after_recovery(owner):
    radio, io, clock = needs_recovery(owner)
    io.after_transfer = lambda command: signal(io, clock) if command[0] == 0x82 else None
    result = radio.recover()
    assert result.state is State.RX_EVENT_PENDING
    assert result.episodes[0].context.terminal_state is State.RX_SINGLE
    assert io.irq == 2
    assert radio.receive(deadline_monotonic_us=clock.now_monotonic_us()).packet is not None


# A permanently high BUSY exhausts one soft and one hard attempt within their bounds.
def test_busy_recovery_is_bounded(owner):
    radio, io, clock = needs_recovery(owner)
    start = clock.now_monotonic_us()
    io.busy_forever = True
    result = radio.recover()
    assert result.state is State.RECOVERY_EXHAUSTED
    assert clock.now_monotonic_us() - start <= 2490750
    assert result.episodes[0].context.last_recovery_error_code is Error.BUSY_TIMEOUT
    assert result.episodes[0].error_code is Error.IO


# Missing hardware outranks profile/TX uncertainty and skips futile recovery commands.
def test_missing_hardware_reason_precedence(owner):
    radio, io, _ = received(owner)
    radio.prepare_ack(b"ACK")
    io.hooks[0x8C] = fail(outcome=Outcome.UNCERTAIN, missing=True)
    assert radio.start_ack(RadioTxAuthorization(1000000)).state is State.RECOVERING
    before = io.commands.copy()
    result = radio.recover()
    assert result.state is State.HARDWARE_MISSING
    assert result.episodes[0].context.recovery_reason is Reason.HARDWARE_UNREACHABLE
    assert io.commands == before


# Each nonterminal state shuts down synchronously, confirms standby and detaches once.
@pytest.mark.parametrize("initial", ["INITIALIZING", "RX_SINGLE", "RX_EVENT_PENDING", "TX_ACTIVE", "RECOVERING"])
def test_shutdown_each_state(owner, initial):
    radio, io, _ = owner
    if initial == "RX_SINGLE":
        radio.initialize()
    elif initial == "RX_EVENT_PENDING":
        received(owner)
    elif initial == "TX_ACTIVE":
        transmitting(owner)
    elif initial == "RECOVERING":
        needs_recovery(owner)
    assert radio.state.name == initial
    result = radio.shutdown()
    assert result.state is State.SHUTDOWN and result.safe_shutdown is True
    assert io.status == 0x24 and io.irq == 0
    assert b"\x08" + bytes(8) in io.commands
    assert io.calls[-1] == ("close",)
    if initial == "TX_ACTIVE":
        assert result.tx.ack_tx_result is AckTxResult.UNKNOWN_INTERRUPTED
        assert result.tx.facts.set_tx_outcome is Outcome.CONFIRMED_APPLIED
    elif initial == "RECOVERING":
        assert len(result.episodes) == 1 and result.episodes[0].severity is Severity.ERROR
        assert radio.counters.recovery_failures == 1
    before = io.calls.copy()
    assert radio.shutdown().state is State.SHUTDOWN
    with pytest.raises(RuntimeError):
        radio.initialize()
    assert io.calls == before


# Shutdown fallback retries only once, shares one bound, and reports safety truthfully.
@pytest.mark.parametrize("fault", ["once", "always", "busy", "close"])
def test_shutdown_failure_and_fallback(owner, fault):
    radio, io, clock = owner
    radio.initialize()
    start = clock.now_monotonic_us()
    if fault in ("once", "always"):
        def fail_standby(_):
            if fault == "once":
                del io.hooks[0x80]
            fail(outcome=Outcome.UNCERTAIN)()
        io.hooks[0x80] = fail_standby
    elif fault == "busy":
        io.busy_forever = True
    else:
        io.close = fail()
    result = radio.shutdown()
    assert result.state is State.SHUTDOWN
    assert result.safe_shutdown is (fault == "once")
    assert len(result.episodes) == 1
    assert result.episodes[0].operation.name == "CLEANUP"
    assert result.episodes[0].severity is (Severity.ERROR if fault == "once" else Severity.FATAL)
    assert clock.now_monotonic_us() - start <= 498150


# Cancellation before/during either recovery level completes the approved original episode.
@pytest.mark.parametrize("phase", ["before", "soft", "hard"])
def test_shutdown_interrupts_recovery_at_boundary(owner, phase):
    radio, io, _ = needs_recovery(owner)
    if phase == "before":
        radio.request_shutdown()
    else:
        if phase == "hard":
            def fail_soft(_):
                del io.hooks[0x80]
                fail()()
            io.hooks[0x80] = fail_soft
        def cancel(command):
            if command[0] == (0x80 if phase == "soft" else 0x96):
                radio.request_shutdown()
        io.after_transfer = cancel
    result = radio.recover()
    assert result.state is State.SHUTDOWN and result.safe_shutdown
    assert len(result.episodes) == 1
    context = result.episodes[0].context
    assert context.trigger_detail.command_opcode == 0x8C
    assert context.soft_recovery_result.name == ("NOT_ATTEMPTED" if phase == "before" else "FAILED")
    assert context.hard_recovery_result.name == ("FAILED" if phase == "hard" else "NOT_ATTEMPTED")
    assert (context.last_recovery_failure_detail is None) is (phase != "hard")
    assert radio.counters.recovery_failures == 1 and radio.counters.recovery_successes == 0


# Invalid inputs terminate after a safe-state attempt and carry VALIDATE, never fake errno.
@pytest.mark.parametrize("value", [None, b"", bytearray(b"a"), bytes(256)])
def test_invalid_ack_argument(owner, value):
    radio, _, _ = received(owner)
    result = radio.prepare_ack(value)
    assert result.state is State.SHUTDOWN and result.safe_shutdown
    assert result.episodes[0].error_code is Error.INVALID_ARGUMENT
    assert result.episodes[0].operation.name == "VALIDATE"
    assert result.episodes[0].severity is Severity.FATAL
    assert result.episodes[0].context.trigger_detail.backend_status == 0


# A bad call in known RX is a fatal invariant; uncertain hardware first runs recovery.
@pytest.mark.parametrize("uncertain", [False, True])
def test_invalid_state_handling(owner, uncertain):
    radio, _, _ = owner
    radio.initialize()
    if uncertain:
        radio.backend.profile = None  # Explicitly lost confirmation, no fake hardware mode.
    result = radio.finish_ack()
    assert result.state is (State.RX_SINGLE if uncertain else State.SHUTDOWN)
    assert len(result.episodes) == 1
    assert result.episodes[0].error_code is Error.INVALID_STATE
    assert result.episodes[0].severity is (Severity.ERROR if uncertain else Severity.FATAL)


# Cancellation after profile/SetTx transfer retains uncertainty and never submits a new TX.
@pytest.mark.parametrize("opcode", [0x8C, 0x83])
def test_shutdown_during_transmit_command(owner, opcode):
    radio, io, _ = received(owner)
    radio.prepare_ack(b"ACK")
    io.after_transfer = lambda command: radio.request_shutdown() if command[0] == opcode else None
    result = radio.start_ack(RadioTxAuthorization(1000000))
    assert result.state is State.SHUTDOWN and result.safe_shutdown
    assert result.tx.ack_tx_result is (AckTxResult.TX_UNCONFIRMED if opcode == 0x8C else AckTxResult.UNKNOWN_INTERRUPTED)
    assert result.tx.facts.profile_uncertain is (opcode == 0x8C)
    assert (result.tx.t4_set_tx_attempted_monotonic_us is None) is (opcode == 0x8C)


# Unexpected implementation exceptions propagate to CORE without fabricated RADIO status.
def test_unexpected_exception_not_normalized(owner):
    radio, io, _ = owner
    io.hooks[0x8A] = lambda _: (_ for _ in ()).throw(TypeError("implementation bug"))
    with pytest.raises(TypeError, match="implementation bug"):
        radio.initialize()
    assert radio.shutdown().safe_shutdown


# A new packet resets prior TX timestamps so shutdown cannot report an old ACK again.
def test_new_packet_does_not_inherit_prior_tx(owner):
    radio, io, clock = transmitting(owner)
    signal(io, clock, 1)
    assert radio.finish_ack().tx.ack_tx_result is AckTxResult.TX_DONE
    signal(io, clock, 2)
    assert radio.receive(deadline_monotonic_us=clock.now_monotonic_us()).packet is not None
    assert radio.shutdown().tx is None


# A close overrun is visible even when standby confirmation succeeded before the bound.
def test_shutdown_close_overrun(owner):
    radio, io, clock = owner
    radio.initialize()
    io.close = lambda: clock.advance_elapsed_us(500000)
    result = radio.shutdown()
    assert result.safe_shutdown is False
    assert result.episodes[0].error_code is Error.DEADLINE


# A foreign thread may request shutdown but cannot perform any radio action itself.
def test_cross_thread_shutdown_intent(owner):
    from threading import Thread
    radio, io, _ = owner
    radio.initialize()
    calls = io.calls.copy()
    worker = Thread(target=radio.request_shutdown)
    worker.start()
    worker.join(5)
    assert not worker.is_alive() and io.calls == calls
    result = radio.receive(deadline_monotonic_us=99999)
    assert result.state is State.SHUTDOWN and result.safe_shutdown


# Invalid terminal TX IRQ combinations retain the selected ACK outcome as unconfirmed.
@pytest.mark.parametrize("irq", [0, 2, 3, 0x20, 0x40, 0x201, 0x202, 0xFFFF])
def test_unexpected_tx_irq(owner, irq):
    radio, io, clock = transmitting(owner)
    signal(io, clock, irq)
    result = radio.finish_ack()
    assert result.state is State.RECOVERING
    assert result.tx.ack_tx_result is AckTxResult.TX_UNCONFIRMED
    result = radio.recover()
    assert result.episodes[0].error_code is Error.UNEXPECTED_IRQ
    assert result.episodes[0].severity is Severity.ERROR


# Persistent BUSY remains bounded at every public semantic boundary and never fabricates success.
@pytest.mark.parametrize("operation", ["initialize", "receive", "prepare", "start", "finish", "rearm", "recover", "shutdown"])
def test_busy_semantic_matrix(owner, operation):
    radio, io, clock = owner
    if operation in ("receive", "shutdown"):
        radio.initialize()
        if operation == "receive":
            signal(io, clock)
    elif operation in ("prepare", "start", "rearm"):
        received(owner)
        if operation == "start":
            radio.prepare_ack(b"ACK")
    elif operation == "finish":
        transmitting(owner)
        signal(io, clock, 1)
    elif operation == "recover":
        needs_recovery(owner)
    io.busy_forever = True
    start = clock.now_monotonic_us()
    actions = {
        "initialize": radio.initialize, "receive": lambda: radio.receive(deadline_monotonic_us=start + 1000),
        "prepare": lambda: radio.prepare_ack(b"ACK"), "start": lambda: radio.start_ack(RadioTxAuthorization(start + 1000000)),
        "finish": radio.finish_ack, "rearm": radio.rearm, "recover": radio.recover, "shutdown": radio.shutdown,
    }
    result = actions[operation]()
    assert result.state.name in {"INITIALIZATION_FAILED", "RECOVERING", "RECOVERY_EXHAUSTED", "SHUTDOWN"}
    assert clock.now_monotonic_us() - start <= 2490750
    assert radio.backend.metrics.timeout_count >= 1


# Throwing away a completed diagnostic cannot change the recovered profile or permit extra TX.
def test_discarded_episode_independence(owner):
    radio, io, clock = needs_recovery(owner)
    result = radio.recover()
    assert len(result.episodes) == 1
    del result
    assert radio.state is State.RX_SINGLE and io.status == 0x54
    assert radio.counters.recovery_successes == 1
    signal(io, clock, 2)
    assert radio.receive(deadline_monotonic_us=clock.now_monotonic_us()).packet is not None


# Successful command submission cannot spend beyond the host terminal-confirmation bound.
def test_set_tx_confirmation_overruns_host_bound(owner):
    radio, io, clock = received(owner)
    radio.prepare_ack(b"ACK")
    def delay(command):
        if command[0] == 0x83:
            clock.advance_elapsed_us(249076)
    io.after_transfer = delay
    result = radio.start_ack(RadioTxAuthorization(clock.now_monotonic_us() + 1000000))
    assert result.state is State.RECOVERING
    assert result.tx.ack_tx_result is AckTxResult.TX_UNCONFIRMED


_TX_SETUP = [bytes.fromhex(value) for value in (
    "8000", "8a01", "863641999a", "8b07040100", "8c000800030101", "8e0e02",
    "9320", "9f00", "a000", "080263026300000000", "0d07401424", "0d08ac96",
    "1d08890000", "0d088904", "1d07360000", "0d073600", "17000000",
    "12000000", "02ffff", "83001900",
)]
_TX_TRANSCRIPT = [part for command in _TX_SETUP for part in (command, b"\xc0\x00")]


# Fault every reviewed setup/confirmation transaction, including repeated register opcodes.
@pytest.mark.parametrize("index", range(40))
def test_every_tx_setup_transaction_failure(owner, index):
    radio, io, _ = received(owner)
    radio.prepare_ack(b"ACK")
    original = io.transfer
    issued = []

    def transfer(data, *, deadline_monotonic_us):
        issued.append(data)
        if len(issued) == index + 1:
            fail()()
        return original(data, deadline_monotonic_us=deadline_monotonic_us)

    io.transfer = transfer
    result = radio.start_ack(RadioTxAuthorization(1000000))
    assert issued[:index + 1] == _TX_TRANSCRIPT[:index + 1]
    uncertain = index % 2 == 1
    assert result.tx.ack_tx_result is (AckTxResult.TX_UNCONFIRMED if uncertain else AckTxResult.SET_TX_FAILED)
    assert (result.tx.t4_set_tx_attempted_monotonic_us is not None) is (index >= 38)
    assert result.state is (State.RECOVERING if uncertain else State.RX_SINGLE)
    if uncertain:
        result = radio.recover()
    assert len(result.episodes) == 1 and result.episodes[0].error_code is Error.IO
