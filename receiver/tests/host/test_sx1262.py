"""Command encodings and physical-port faults, without receiver orchestration."""

import errno

import pytest

from cura_receiver.ports.radio import (
    Dio1Edge, Error, Outcome, RadioBackendError, RadioFailure, Stage,
)
from cura_receiver.sx1262 import Sx1262
from tests.support.fakes.os_clock import FakeOsClock
from tests.support.fakes.radio_io import PhysicalPort, Wait


@pytest.fixture
def radio():
    clock = FakeOsClock(monotonic_us=1000)
    io = PhysicalPort(clock)
    waiter = Wait(clock)
    backend = Sx1262(io, clock, waiter)
    backend.open(2000000)
    return backend, io, clock, waiter


# Reviewed Semtech/protocol bytes: the complete receive profile, independent of code.
RX_PROFILE = [
    "8000", "8a01", "863641999a", "8b07040100", "8c000800ff0100",
    "8e0e02", "9320", "9f00", "a000", "080263026300000000",
    "0d07401424", "0d08ac96", "1d08890000", "0d088904",
    "1d07360000", "0d073604", "17000000",
]


def non_status(io):
    return [command.hex() for command in io.commands if command[0] != 0xC0]


def injected(code=Error.IO, *, outcome=Outcome.DEFINITELY_NOT_APPLIED):
    def fail(_):
        raise RadioBackendError(RadioFailure(
            code, Stage.WRITE_COMMAND, outcome, os_errno=errno.EIO if code is Error.IO else None,
        ))
    return fail


# Startup includes the board-specific oscillator/PA setup and complete RX profile.
def test_initialize_literal_transcript(radio):
    backend, io, _, waiter = radio
    backend.initialize(2000000)
    assert non_status(io) == [
        "8000", "17000000", "9601", "1d08d80000", "0d08d81e", "9701000140",
        "070000", "897f", "98d7db", "9d01", "9504070001",
        "8f0000", "17000000",
    ] + RX_PROFILE
    assert backend.profile == "rx"
    assert io.calls[1:3] == [("reset", True, 1000), ("reset", False, 2004)]
    assert waiter.deadlines[0] == 2004
    backend.arm_receive(2000000)
    assert non_status(io)[-3:] == ["12000000", "02ffff", "82000000"]
    assert backend.last_set_rx_issued_us == 2004


# F-001: assertion-call latency cannot consume the minimum physical reset pulse.
@pytest.mark.parametrize("before_assert_us", [1, 1004, 2000])
@pytest.mark.parametrize("after_assert_us", [0, 500])
def test_reset_minimum_starts_after_assertion(radio, monkeypatch, before_assert_us, after_assert_us):
    backend, io, clock, _ = radio
    original = io.set_reset

    def delayed_reset(*, asserted):
        if asserted:
            clock.advance_elapsed_us(before_assert_us)
        original(asserted=asserted)
        if asserted:
            clock.advance_elapsed_us(after_assert_us)

    monkeypatch.setattr(io, "set_reset", delayed_reset)
    backend.reset(100000)
    edges = [call for call in io.calls if call[0] == "reset"]
    assert edges == [
        ("reset", True, 1000 + before_assert_us),
        ("reset", False, 2004 + before_assert_us + after_assert_us),
    ]
    assert edges[1][2] - edges[0][2] >= 1004
    assert backend.initial_reset_status == 0x24


# F-001: an exhausted pulse budget releases RESET, fails uncertain and submits no SPI.
@pytest.mark.parametrize("assertion_delay_us", [2496, 3000, 3500, 4000])
def test_reset_assertion_latency_exhausts_deadline(radio, monkeypatch, assertion_delay_us):
    backend, io, clock, waiter = radio
    original = io.set_reset

    def delayed_reset(*, asserted):
        if asserted:
            clock.advance_elapsed_us(assertion_delay_us)
        original(asserted=asserted)

    monkeypatch.setattr(io, "set_reset", delayed_reset)
    with pytest.raises(RadioBackendError) as caught:
        backend.reset(4500)
    failure = caught.value.failure
    assert (failure.code, failure.stage, failure.outcome) == (
        Error.DEADLINE, Stage.RESET, Outcome.UNCERTAIN,
    )
    assert failure.hardware_touched
    assert io.calls[1:] == [
        ("reset", True, 1000 + assertion_delay_us),
        ("reset", False, max(4500, 1000 + assertion_delay_us)),
    ]
    assert all(bound <= 4500 for bound in waiter.deadlines)
    assert backend.initial_reset_status is None and backend.profile is None
    assert io.commands == []


# A known insufficient reset budget rejects before changing the GPIO level.
@pytest.mark.parametrize("deadline", [1000, 1500, 2004])
def test_reset_rejects_insufficient_budget_before_assertion(radio, deadline):
    backend, io, _, waiter = radio
    with pytest.raises(RadioBackendError) as caught:
        backend.reset(deadline)
    failure = caught.value.failure
    assert (failure.code, failure.stage, failure.outcome) == (
        Error.DEADLINE, Stage.RESET, Outcome.DEFINITELY_NOT_APPLIED,
    )
    assert not failure.hardware_touched
    assert len(io.calls) == 1 and io.commands == [] and waiter.deadlines == []


# Release-call latency is also charged to the enclosing reset deadline.
def test_reset_release_latency_exhausts_deadline(radio, monkeypatch):
    backend, io, clock, _ = radio
    original = io.set_reset

    def delayed_reset(*, asserted):
        if not asserted:
            clock.advance_elapsed_us(3000)
        original(asserted=asserted)

    monkeypatch.setattr(io, "set_reset", delayed_reset)
    with pytest.raises(RadioBackendError) as caught:
        backend.reset(4500)
    failure = caught.value.failure
    assert (failure.code, failure.stage, failure.outcome) == (
        Error.DEADLINE, Stage.RESET, Outcome.UNCERTAIN,
    )
    assert failure.hardware_touched
    assert io.calls[-1] == ("reset", False, 5004) and io.commands == []


# An unexpected wait exception still releases the asserted GPIO and propagates unchanged.
def test_reset_wait_exception_releases_gpio(radio, monkeypatch):
    backend, io, _, waiter = radio
    error = RuntimeError("interrupted wait")

    def interrupted(_):
        raise error

    monkeypatch.setattr(waiter, "wait_until_monotonic_us", interrupted)
    with pytest.raises(RuntimeError) as caught:
        backend.reset(100000)
    assert caught.value is error
    assert io.calls[1:] == [("reset", True, 1000), ("reset", False, 1000)]
    assert io.commands == []


# The transmit profile changes length/IQ and preserves unrelated workaround bits.
def test_transmit_profile_and_exact_buffer(radio):
    backend, io, _, _ = radio
    io.registers[0x0736] = 0xAB
    io.registers[0x0889] = 0x80
    frame = bytes(range(17))
    backend.write_buffer(frame, 100000)
    backend.install_profile(transmit=True, payload_length=17, deadline=100000)
    expected = RX_PROFILE.copy()
    expected[4] = "8c000800110101"
    expected[13] = "0d088984"
    expected[15] = "0d0736ab"
    assert non_status(io) == ["0e00" + frame.hex()] + expected
    assert backend.profile == "tx"
    backend.start_tx(100000)
    assert non_status(io)[-3:] == ["12000000", "02ffff", "83001900"]
    assert backend.set_tx_outcome is Outcome.CONFIRMED_APPLIED
    backend.install_profile(transmit=False, payload_length=255, deadline=100000)
    assert io.registers[0x0736] == 0xAF
    assert io.registers[0x08AC] == 0x96


# ReadBuffer bytes and copy time survive buffer mutation before the status query.
def test_packet_copy_status_and_rtc_workaround(radio):
    backend, io, clock, _ = radio

    def after(data):
        if data[0] == 0x1E:
            io.buffer[:] = b"mutate"
        elif data[0] == 0xC0:
            clock.advance_elapsed_us(1)

    io.after_transfer = after
    assert backend.read_packet(100000) == (b"packet", -200, -12, 1001)
    assert non_status(io) == ["13000000", "1e0700000000000000", "1400000000", "0d090200", "1d09440000", "0d094402"]


# Every profile command failure leaves the profile unconfirmed and stops submission.
@pytest.mark.parametrize("failed_opcode", [0x80, 0x8A, 0x86, 0x8B, 0x8C, 0x8E, 0x93, 0x9F, 0xA0, 0x08, 0x0D, 0x1D, 0x17])
@pytest.mark.parametrize("outcome", [Outcome.DEFINITELY_NOT_APPLIED, Outcome.UNCERTAIN])
def test_profile_failure_stops_at_primitive(radio, failed_opcode, outcome):
    backend, io, _, _ = radio
    io.hooks[failed_opcode] = injected(outcome=outcome)
    with pytest.raises(RadioBackendError) as caught:
        backend.install_profile(transmit=True, payload_length=17, deadline=100000)
    assert caught.value.failure.opcode == failed_opcode
    assert caught.value.failure.outcome is outcome
    assert backend.profile is None
    assert io.commands[-1][0] == failed_opcode
    assert not any(c[0] in (0x82, 0x83) for c in io.commands)


# Failed GetStatus after a written command makes its effect uncertain.
def test_failed_confirmation_is_uncertain(radio):
    backend, io, _, _ = radio
    io.hooks[0xC0] = injected()
    with pytest.raises(RadioBackendError) as caught:
        backend.standby(100000)
    assert caught.value.failure.outcome is Outcome.UNCERTAIN
    assert io.commands == [b"\x80\x00", b"\xc0\x00"]


# Raw status failure classes preserve the byte and never accept reserved shapes.
@pytest.mark.parametrize("status,code", [
    (0x26, Error.COMMAND_STATUS), (0x28, Error.COMMAND_STATUS), (0x2A, Error.COMMAND_STATUS),
    (0xFF, Error.MALFORMED_RESPONSE), (0, Error.MALFORMED_RESPONSE),
    (0x2E, Error.MALFORMED_RESPONSE), (0xA4, Error.MALFORMED_RESPONSE),
])
def test_status_validation(radio, status, code):
    backend, io, _, _ = radio
    io.statuses.append(status)
    with pytest.raises(RadioBackendError) as caught:
        backend.standby(100000)
    assert (caught.value.failure.code, caught.value.failure.chip_status) == (code, status)


# Repeated zero/ones after reset classify unresponsive hardware without a fake errno.
@pytest.mark.parametrize("statuses", [(0, 0), (255, 255), (0, 255)])
def test_unresponsive_reset(radio, statuses):
    backend, io, _, _ = radio
    io.statuses.extend(statuses)
    with pytest.raises(RadioBackendError) as caught:
        backend.reset(100000)
    assert caught.value.failure.hardware_missing
    assert caught.value.failure.os_errno is None
    assert caught.value.failure.chip_status == statuses[-1]


# The captured Pi reset status precedes host commands; TCXO startup errors clear before RX.
@pytest.mark.parametrize("status", [0x24, 0x2A])
@pytest.mark.parametrize("errors", [0, 0x20])
def test_tcxo_initial_status_and_error_clear(radio, status, errors):
    backend, io, _, _ = radio
    io.statuses.append(status)
    def startup_errors(_):
        io.errors = errors
        del io.hooks[0x17]
    io.hooks[0x17] = startup_errors
    backend.initialize(2000000)
    backend.arm_receive(2000000)
    assert backend.initial_reset_status == status
    assert [c.hex() for c in io.commands[:5]] == ["c000", "8000", "c000", "17000000", "c000"]
    assert io.commands.index(b"\x07\x00\x00") < io.commands.index(b"\x89\x7f")
    assert io.errors == 0 and io.status == 0x54 and backend.profile == "rx"


# Every device-error bit other than XOSC startup is rejected before oscillator configuration.
@pytest.mark.parametrize("bit", [n for n in range(16) if n != 5])
@pytest.mark.parametrize("startup", [0, 0x20])
def test_unexpected_startup_device_error_bits(radio, bit, startup):
    backend, io, _, _ = radio
    mask = (1 << bit) | startup
    io.statuses.append(0x2A)
    io.hooks[0x17] = lambda _: setattr(io, "errors", mask)
    with pytest.raises(RadioBackendError) as caught:
        backend.initialize(2000000)
    assert caught.value.failure.code is Error.DEVICE_ERROR
    assert caught.value.failure.device_errors == mask
    assert not any(c[0] in (0x97, 0x82, 0x83) for c in io.commands)


# A fresh standby failure cannot inherit the initial-status exception.
@pytest.mark.parametrize("status", [0x26, 0x28, 0x2A])
def test_post_reset_standby_confirmation_failure(radio, status):
    backend, io, _, _ = radio
    io.statuses.extend([0x2A, status])
    with pytest.raises(RadioBackendError) as caught:
        backend.initialize(2000000)
    assert caught.value.failure.code is Error.COMMAND_STATUS
    assert caught.value.failure.chip_status == status
    assert caught.value.failure.outcome is Outcome.UNCERTAIN
    assert [c.hex() for c in io.commands] == ["c000", "8000", "c000"]


# Initial status still must have a valid structure and the actual reset standby mode.
@pytest.mark.parametrize("status,code", [
    (0x34, Error.COMMAND_STATUS), (0x54, Error.COMMAND_STATUS),
    (0x64, Error.COMMAND_STATUS), (0x2E, Error.MALFORMED_RESPONSE),
    (0xA4, Error.MALFORMED_RESPONSE),
])
def test_initial_status_structure_and_mode(radio, status, code):
    backend, io, _, _ = radio
    io.statuses.append(status)
    with pytest.raises(RadioBackendError) as caught:
        backend.initialize(2000000)
    assert caught.value.failure.code is code
    assert caught.value.failure.chip_status == status
    assert io.commands == [b"\xc0\x00"]


# Even the expected power-on oscillator bit is a failure if it returns after calibration.
@pytest.mark.parametrize("errors", [0x20, 0x40])
def test_device_errors_after_calibration_are_fatal(radio, errors):
    backend, io, _, _ = radio
    io.hooks[0x89] = lambda _: setattr(io, "errors", errors)
    with pytest.raises(RadioBackendError) as caught:
        backend.initialize(2000000)
    assert caught.value.failure.code is Error.DEVICE_ERROR
    assert caught.value.failure.device_errors == errors
    assert backend.profile is None
    assert not any(c[0] in (0x82, 0x83) for c in io.commands)


# BUSY uses the rate-conservative 100 ms bound, clipped to the enclosing deadline.
@pytest.mark.parametrize("enclosing,expected", [(200000, 100630), (1100, 1100)])
@pytest.mark.parametrize("after", [False, True])
def test_busy_timeout_and_metrics(radio, enclosing, expected, after):
    backend, io, clock, _ = radio
    io.busy_forever = True
    with pytest.raises(RadioBackendError) as caught:
        backend.wait_busy(enclosing, opcode=0x83, after=after)
    assert clock.now_monotonic_us() == expected
    assert caught.value.failure.code is Error.BUSY_TIMEOUT
    assert caught.value.failure.outcome is (Outcome.UNCERTAIN if after else Outcome.DEFINITELY_NOT_APPLIED)
    assert backend.metrics.total_us == expected - 1000
    assert (backend.metrics.count, backend.metrics.timeout_count, backend.metrics.last_timeout_opcode) == (1, 1, 0x83)
    assert io.commands == []


# A BUSY release on the inclusive boundary succeeds without an extra poll interval.
def test_busy_release_at_bound(radio):
    backend, io, clock, _ = radio
    io.busy_until = 1100
    backend.wait_busy(1100)
    assert clock.now_monotonic_us() == 1100
    assert backend.metrics.timeout_count == 0


# Immediate completion is confirmed only using a fresh appropriate terminal IRQ.
@pytest.mark.parametrize("transmit,irq,status", [
    (False, 2, 0x24), (False, 0x20, 0x24), (False, 0x200, 0x26),
    (True, 1, 0x24), (True, 1, 0x2C), (True, 0x200, 0x24), (True, 0x200, 0x26),
])
def test_immediate_completion(radio, transmit, irq, status):
    backend, io, _, _ = radio
    backend.install_profile(transmit=transmit, payload_length=17, deadline=100000)
    io.irq = 1 if not transmit else 2  # Must be cleared before the new command.
    io.edges.append(Dio1Edge(999000, 1))

    def complete(data):
        if data[0] == (0x83 if transmit else 0x82):
            assert io.irq == 0
            assert not io.edges
            io.status = status
            io.irq = irq
            io.edges.append(Dio1Edge(1000000, 2))

    io.after_transfer = complete
    (backend.start_tx if transmit else backend.arm_receive)(100000)
    assert backend.wait_edge(deadline_monotonic_us=100000) == Dio1Edge(1000000, 2)
    assert backend.wait_edge(deadline_monotonic_us=100000) is None


# Event reads preserve the captured timeout bytes before any command interpretation or mutation.
def test_immutable_timeout_observation(radio):
    from dataclasses import FrozenInstanceError
    backend, io, _, _ = radio
    io.status, io.irq, io.errors = 0x26, 0x200, 0
    event = backend.observe_event(100000)
    assert (event.irq_status, event.chip_status, event.device_errors) == (0x200, 0x26, 0)
    assert [c.hex() for c in io.commands] == ["12000000", "17000000", "c000"]
    io.status, io.irq, io.errors = 0x2A, 0, 0x40
    assert backend.validate_event(event, transmit=True)
    assert backend.validate_event(event, transmit=False)
    with pytest.raises(FrozenInstanceError):
        event.irq_status = 0


# Immediate completion must have a fresh, real, timely edge retained for the owner.
@pytest.mark.parametrize("transmit", [False, True])
@pytest.mark.parametrize("edge_kind,code", [
    ("missing", Error.DEADLINE), ("stale", Error.UNEXPECTED_IRQ),
    ("future", Error.MALFORMED_RESPONSE), ("late", Error.DEADLINE),
])
def test_immediate_completion_edge_rejection(radio, transmit, edge_kind, code):
    backend, io, clock, _ = radio
    backend.install_profile(transmit=transmit, payload_length=17, deadline=100000)
    def complete(data):
        if data[0] == (0x83 if transmit else 0x82):
            io.status, io.irq = 0x26, 0x200
            timestamps = {"stale": 999999, "future": 1001000, "late": 100000001}
            if edge_kind != "missing":
                io.edges.append(Dio1Edge(timestamps[edge_kind], 1))
    io.after_transfer = complete
    with pytest.raises(RadioBackendError) as caught:
        (backend.start_tx if transmit else backend.arm_receive)(100000)
    assert caught.value.failure.code is code
    assert caught.value.failure.outcome is Outcome.UNCERTAIN
    assert backend.wait_edge(deadline_monotonic_us=100000) is None
    if transmit:
        assert backend.set_tx_outcome is Outcome.UNCERTAIN


# The timestamp boundary remains inclusive when immediate completion is serviced later.
@pytest.mark.parametrize("transmit", [False, True])
def test_immediate_completion_at_deadline(radio, transmit):
    backend, io, clock, _ = radio
    backend.install_profile(transmit=transmit, payload_length=17, deadline=100000)
    def complete(data):
        if data[0] == (0x83 if transmit else 0x82):
            io.status, io.irq = 0x26, 0x200
            io.edges.append(Dio1Edge(100000000, 1))
    io.after_transfer = complete
    original_wait = io.wait_edge
    def wait(*, deadline_monotonic_us):
        if io.irq == 0x200:
            clock.advance_elapsed_us(100000)
        return original_wait(deadline_monotonic_us=deadline_monotonic_us)
    io.wait_edge = wait
    (backend.start_tx if transmit else backend.arm_receive)(100000)
    assert backend.wait_edge(deadline_monotonic_us=100000) == Dio1Edge(100000000, 1)


# Contradictory terminal facts cannot prove an immediately completed SetRx/SetTx.
@pytest.mark.parametrize("transmit", [False, True])
@pytest.mark.parametrize("status,irq,errors,code", [
    (0x26, 0, 0, Error.COMMAND_STATUS), (0x26, 0x201, 0, Error.COMMAND_STATUS),
    (0x26, 2, 0, Error.COMMAND_STATUS), (0x36, 0x200, 0, Error.COMMAND_STATUS),
    (0x28, 0x200, 0, Error.COMMAND_STATUS), (0x2A, 0x200, 0, Error.COMMAND_STATUS),
    (0x2C, 0x200, 0, Error.COMMAND_STATUS), (0x26, 0x200, 0x20, Error.DEVICE_ERROR),
    (0xFF, 0x200, 0, Error.MALFORMED_RESPONSE),
])
def test_immediate_completion_conflicting_evidence(radio, transmit, status, irq, errors, code):
    backend, io, _, _ = radio
    backend.install_profile(transmit=transmit, payload_length=17, deadline=100000)
    def complete(data):
        if data[0] == (0x83 if transmit else 0x82):
            io.status, io.irq, io.errors = status, irq, errors
            io.edges.append(Dio1Edge(1000000, 1))
    io.after_transfer = complete
    with pytest.raises(RadioBackendError) as caught:
        (backend.start_tx if transmit else backend.arm_receive)(100000)
    assert caught.value.failure.code is code
    assert caught.value.failure.outcome is Outcome.UNCERTAIN


# SetTx's physical-port refusal differs from uncertainty after crossing SPI.
@pytest.mark.parametrize("outcome", [Outcome.DEFINITELY_NOT_APPLIED, Outcome.UNCERTAIN])
def test_set_tx_failure_certainty(radio, outcome):
    backend, io, _, _ = radio
    backend.install_profile(transmit=True, payload_length=17, deadline=100000)
    io.hooks[0x83] = injected(outcome=outcome)
    with pytest.raises(RadioBackendError):
        backend.start_tx(100000)
    assert backend.set_tx_outcome is outcome
    assert backend.last_set_tx_issued_us == 1000


# Safe standby disables every IRQ route and confirms mode after clearing evidence.
def test_safe_standby_transcript(radio):
    backend, io, _, _ = radio
    backend.safe_standby(100000)
    assert non_status(io) == ["8000", "080000000000000000", "12000000", "02ffff"]
    assert io.commands[-1] == b"\xc0\x00"
    assert backend.profile is None


# Startup and cleanup expose the unresolved catalogue pairs recorded as D-026.
@pytest.mark.parametrize("operation", ["initial_set_rx", "cleanup"])
def test_unresolved_irq_during_startup_or_cleanup(radio, operation):
    backend, io, _, _ = radio
    backend.initialize(100000)
    io.dio1 = lambda: True
    with pytest.raises(RadioBackendError) as caught:
        (backend.arm_receive if operation == "initial_set_rx" else backend.safe_standby)(100000)
    assert caught.value.failure.code is Error.UNEXPECTED_IRQ
    assert caught.value.failure.stage is Stage.CLEAR_IRQ
    assert caught.value.failure.irq_status == 0
    assert not any(command[0] in (0x82, 0x83) for command in io.commands)


# Event resynchronization requires timely low DIO1 both before and after the physical port call.
@pytest.mark.parametrize("phase", ["before", "after"])
@pytest.mark.parametrize("fault", ["high", "late"])
def test_event_resynchronization_requires_quiet_dio1(radio, monkeypatch, phase, fault):
    backend, io, clock, _ = radio
    backend.initialize(100000)
    readings = 0

    def dio1():
        nonlocal readings
        readings += 1
        if readings == (1 if phase == "before" else 2):
            if fault == "late":
                clock.advance_elapsed_us(100000)
            else:
                return True
        return False

    monkeypatch.setattr(io, "dio1", dio1)
    with pytest.raises(RadioBackendError) as caught:
        backend.arm_receive(100000, resynchronize=True)
    assert caught.value.failure.code is (Error.DEADLINE if fault == "late" else Error.UNEXPECTED_IRQ)
    assert caught.value.failure.stage is Stage.CLEAR_IRQ
    assert sum(call[0] == "resynchronize_events" for call in io.calls) == (phase == "after")
    assert not any(command[0] == 0x82 for command in io.commands)


# Soft resynchronization records/clears device faults and restores all RX settings.
def test_soft_restore_complete_transcript(radio):
    backend, io, _, _ = radio
    io.errors = 0x40
    io.irq = 0x200
    backend.soft_restore(100000)
    assert non_status(io) == ["8000", "12000000", "17000000", "070000", "02ffff"] + RX_PROFILE + ["12000000", "02ffff", "82000000"]
    assert backend.profile == "rx"
    assert io.irq == io.errors == 0


# Device faults outside the documented POR TCXO startup bit remain exact evidence.
def test_initial_device_error_preserved(radio):
    backend, io, _, _ = radio
    io.hooks[0x17] = lambda _: setattr(io, "errors", 0x140)
    with pytest.raises(RadioBackendError) as caught:
        backend.initialize(100000)
    assert caught.value.failure.code is Error.DEVICE_ERROR
    assert caught.value.failure.device_errors == 0x140
    assert not any(command[0] == 0x82 for command in io.commands)


# Failed confirmation of SetTx remains uncertain even when its SPI call succeeded.
def test_set_tx_confirmation_failure(radio):
    backend, io, _, _ = radio
    backend.install_profile(transmit=True, payload_length=17, deadline=100000)

    def after(data):
        if data[0] == 0x83:
            io.hooks[0xC0] = injected()

    io.after_transfer = after
    with pytest.raises(RadioBackendError):
        backend.start_tx(100000)
    assert backend.set_tx_outcome is Outcome.UNCERTAIN
    assert backend.last_set_tx_issued_us == 1000


# Foreign-thread calls cannot even invalidate profile/timestamp facts before rejection.
def test_backend_owner_rejects_before_mutation(radio):
    from threading import Thread
    backend, io, _, _ = radio
    backend.initialize(100000)
    backend.arm_receive(100000)
    original = (backend.profile, backend.last_set_rx_issued_us, io.commands.copy())
    failures = []

    def foreign():
        for call in (lambda: backend.standby(100000), lambda: backend.arm_receive(100000),
                     lambda: backend.install_profile(transmit=True, payload_length=3, deadline=100000)):
            try:
                call()
            except RuntimeError as error:
                failures.append(error)

    thread = Thread(target=foreign)
    thread.start()
    thread.join(5)
    assert not thread.is_alive() and len(failures) == 3
    assert (backend.profile, backend.last_set_rx_issued_us, io.commands) == original
