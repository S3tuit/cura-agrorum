"""Independent primitive owner model: reviewed examples precede generated histories."""

from hypothesis import given, settings, strategies as st
import pytest

from cura_receiver.radio import Radio
from cura_receiver.ports.radio import Dio1Edge, Error, Outcome, RadioBackendError, RadioFailure, RadioTxAuthorization, Stage
from cura_receiver.sx1262 import Sx1262
from tests.support.fakes.os_clock import FakeOsClock
from tests.support.fakes.radio_io import PhysicalPort, Wait


class Model:
    """Only primitive labels/counters; no production algorithm or encoder calls."""

    def __init__(self):
        self.state = "INITIALIZING"
        self.prepared = False
        self.attempts = self.successes = self.failures = 0
        self.tx = None

    def choices(self):
        return {
            "INITIALIZING": ["init", "shutdown"],
            "RX_SINGLE": ["packet", "crc", "unexpected", "rx_fault", "shutdown"],
            "RX_EVENT_PENDING": ["tx", "tx_definite", "tx_uncertain", "shutdown"] if self.prepared else ["prepare", "rearm", "shutdown"],
            "TX_ACTIVE": ["done", "timeout", "missing", "shutdown"],
            "RECOVERING": ["soft", "hard", "exhaust", "shutdown"],
        }.get(self.state, ["terminal"])

    def apply(self, action):
        diagnostics = 0
        if action == "init":
            self.state = "RX_SINGLE"
        elif action == "packet":
            self.state, self.prepared, self.tx = "RX_EVENT_PENDING", False, None
        elif action in ("crc", "unexpected"):
            self.tx = None
            diagnostics = int(action == "unexpected")
        elif action == "prepare":
            self.prepared = True
        elif action == "rearm":
            self.state = "RX_SINGLE"
        elif action == "tx":
            self.state, self.tx = "TX_ACTIVE", None
        elif action == "tx_definite":
            self.state, self.tx = "RX_SINGLE", "SET_TX_FAILED"
            diagnostics = 1
        elif action in ("rx_fault", "tx_uncertain", "missing"):
            self.state = "RECOVERING"
            self.attempts += 1
            self.tx = None if action == "rx_fault" else "TX_UNCONFIRMED"
        elif action in ("soft", "hard"):
            self.state = "RX_SINGLE"
            self.successes += 1
            diagnostics = 1
        elif action == "exhaust":
            self.state = "RECOVERY_EXHAUSTED"
            self.failures += 1
            diagnostics = 1
        elif action in ("done", "timeout"):
            self.state = "RX_SINGLE"
            self.tx = "TX_DONE" if action == "done" else "TX_TIMEOUT"
        elif action == "shutdown":
            if self.state == "RECOVERING":
                self.failures += 1
                diagnostics = 1
            if self.state == "TX_ACTIVE":
                self.tx = "UNKNOWN_INTERRUPTED"
            elif self.state == "RX_EVENT_PENDING" and self.prepared:
                self.tx = "SET_TX_FAILED"
            self.state = "SHUTDOWN"
        return diagnostics


# Reviewed uncertainty path retains its result after recovery and counts one episode.
def test_model_reviewed_uncertainty():
    model = Model()
    for action in ("init", "packet", "prepare", "tx_uncertain"):
        assert model.apply(action) == 0
    assert (model.state, model.tx, model.attempts) == ("RECOVERING", "TX_UNCONFIRMED", 1)
    assert model.apply("hard") == 1
    assert (model.state, model.tx, model.successes, model.failures) == ("RX_SINGLE", "TX_UNCONFIRMED", 1, 0)


# Reviewed interruption is an unsuccessful recovery; terminal states never reinitialize.
def test_model_reviewed_shutdown():
    model = Model()
    for action in ("init", "rx_fault"):
        model.apply(action)
    assert model.apply("shutdown") == 1
    assert (model.state, model.attempts, model.successes, model.failures) == ("SHUTDOWN", 1, 0, 1)
    assert model.choices() == ["terminal"]


def run_action(radio, io, clock, action):
    def irq(bits):
        io.irq, io.status = bits, 0x24
        io.edges.append(Dio1Edge(clock.now_monotonic_us() * 1000, 1))

    def fault(opcode, *, once, uncertain=False):
        def raise_failure(_):
            if once:
                del io.hooks[opcode]
            raise RadioBackendError(RadioFailure(Error.IO, Stage.WRITE_COMMAND,
                Outcome.UNCERTAIN if uncertain else Outcome.DEFINITELY_NOT_APPLIED, os_errno=5))
        io.hooks[opcode] = raise_failure

    if action == "init":
        return radio.initialize()
    if action in ("packet", "crc", "unexpected", "rx_fault"):
        irq(2 if action in ("packet", "rx_fault") else 0x42 if action == "crc" else 0x8000)
        if action == "rx_fault":
            fault(0x13, once=True)
        return radio.receive(deadline_monotonic_us=clock.now_monotonic_us() + 100)
    if action == "prepare":
        return radio.prepare_ack(b"reviewed bytes")
    if action == "rearm":
        return radio.rearm()
    if action in ("tx", "tx_definite", "tx_uncertain"):
        if action != "tx":
            fault(0x8C, once=True, uncertain=action == "tx_uncertain")
        return radio.start_ack(RadioTxAuthorization(clock.now_monotonic_us() + 500000))
    if action in ("done", "timeout", "missing"):
        clock.advance_elapsed_us(249075)
        if action != "missing":
            irq(1 if action == "done" else 0x200)
            if action == "timeout":
                io.status = 0x26  # Captured timeout status; model decisions stay independent.
        return radio.finish_ack()
    if action in ("soft", "hard", "exhaust"):
        if action != "soft":
            fault(0x80, once=action == "hard")
        return radio.recover()
    if action == "shutdown":
        return radio.shutdown()
    calls = io.calls.copy()
    with pytest.raises(RuntimeError):
        radio.initialize()
    assert io.calls == calls
    return None


# Generated valid/faulted histories compare states, terminal facts, episodes and counters.
@settings(max_examples=100, deadline=None, derandomize=True)
@given(st.data())
def test_radio_histories(data):
    clock = FakeOsClock(monotonic_us=1000)
    io = PhysicalPort(clock)
    radio = Radio(Sx1262(io, clock, Wait(clock)))
    model = Model()
    for _ in range(30):
        action = data.draw(st.sampled_from(model.choices()))
        expected_episodes = model.apply(action)
        result = run_action(radio, io, clock, action)
        assert radio.state.name == model.state
        assert (radio.counters.recovery_attempts, radio.counters.recovery_successes, radio.counters.recovery_failures) == (model.attempts, model.successes, model.failures)
        if result is not None:
            assert len(result.episodes) == expected_episodes
            assert (result.tx.ack_tx_result.name if result.tx else None) == model.tx
        if model.state == "RX_SINGLE":
            assert io.status == 0x54
            assert (io.registers[0x0740], io.registers[0x0741]) == (0x14, 0x24)
            assert io.registers[0x08AC] == 0x96 and io.registers[0x0736] & 4
            assert result.t6_set_rx_issued_monotonic_us is not None
