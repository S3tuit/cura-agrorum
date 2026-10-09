"""Temporary instrumentation preserves real radio recovery and manual arming."""

import json
import threading
import time
from types import SimpleNamespace

import pytest

from cura_receiver import radio_investigation as investigation
from cura_receiver.ports.radio import Dio1Edge
from cura_receiver.radio import Radio, State
from cura_receiver.sx1262 import Sx1262
from tests.support.fakes.os_clock import FakeOsClock
from tests.support.fakes.radio_io import PhysicalPort, Wait


def eventually(predicate):
    deadline = time.monotonic() + 3
    while not predicate():
        assert time.monotonic() < deadline
        time.sleep(.01)


class Scope:
    def __init__(self):
        self.state = "Stop"
        self.exports = []
        self.entered = threading.Event()
        self.release_export = threading.Event()
        self.release_export.set()
        self.failure = None

    def query(self, command):
        if command == "*IDN?":
            return "Siglent Technologies,SDS804X HD,test,1"
        assert command == ":TRIG:STAT?"
        return self.state

    def ready(self):
        return self.state == "Ready"

    def export(self, directory):
        self.exports.append(directory)
        self.entered.set()
        assert self.release_export.wait(3)
        (directory / "partial.bin").write_bytes(b"retained")
        if self.failure:
            raise self.failure

    def close(self):
        self.release_export.set()


class Marker:
    def __init__(self, scope, io):
        self.scope, self.io = scope, io
        self.writes = []
        self.released = False
        self.fail_high = False

    def set(self, high):
        self.writes.append((high, threading.get_ident(), list(self.io.commands)))
        if high and self.fail_high:
            raise OSError("marker unavailable")
        if high:
            self.scope.state = "Stop"

    def release(self):
        self.released = True


@pytest.fixture
def setup(tmp_path, monkeypatch):
    clock = FakeOsClock(monotonic_us=10000)
    io = PhysicalPort(clock)
    backend = Sx1262(io, clock, Wait(clock))
    scope = Scope()
    marker = Marker(scope, io)
    session = investigation.Investigation(backend, bytes(16), tmp_path, marker=marker, scope=scope)
    monkeypatch.setattr(investigation, "_session", session)
    radio = Radio(backend)
    assert radio.initialize().state is State.RX_SINGLE
    yield radio, io, clock, scope, marker, session
    session.close()


def receive(setup, irq=0, sequence=1):
    radio, io, clock, *_ = setup
    clock.advance_elapsed_us(100)
    io.irq, io.status = irq, 0x24
    io.edges.append(Dio1Edge(clock.now_monotonic_us() * 1000, sequence))
    return radio.receive(deadline_monotonic_us=clock.now_monotonic_us() + 500000)


def arm(setup):
    scope, session = setup[3], setup[5]
    scope.state = "Ready"  # operator presses Single
    session.wake.set()
    eventually(lambda: session.armed)


def incident(session, sequence=1):
    files = list(session.root.glob(f"captures/*-dio1-{sequence}-*/incident.json"))
    return json.loads(files[0].read_text()) if files else None


def test_zero_irq_marked_before_standby_and_exported_after_recovery(setup):
    radio, io, clock, scope, marker, session = setup
    arm(setup)
    scope.release_export.clear()
    result = receive(setup)
    assert result.state is State.RX_SINGLE
    assert result.episodes[0].error_code.name == "UNEXPECTED_IRQ"
    high = next(write for write in marker.writes if write[0])
    assert high[1] == threading.get_ident()
    assert [command[0] for command in high[2]][-3:] == [0x12, 0x17, 0xC0]
    assert scope.entered.wait(2)
    record = next(iter(session.records.values()))
    assert record["original_irq_read"]["rx_hex"] == "00000000"
    assert record["last_irq_clear"]["mask"] == 0xffff
    assert record["last_irq_clear"]["outcome"] == "CONFIRMED_APPLIED"
    assert record["last_set_rx_issued_monotonic_us"] < record["handler_started_monotonic_us"]
    assert record["pins"]["dio1"] is False
    assert len([write for write in marker.writes if write[0]]) == 1
    scope.release_export.set()
    eventually(lambda: incident(session) and incident(session)["capture_status"] == "SAVED")
    assert marker.writes[-1][0] is False
    assert scope.state == "Stop"
    assert not session.armed


def test_stopped_scope_retains_details_without_trigger_or_rearm(setup):
    result = receive(setup)
    session = setup[5]
    eventually(lambda: incident(session) is not None)
    assert result.state is State.RX_SINGLE
    assert incident(session)["capture_status"] == "SKIPPED_SCOPE_NOT_READY"
    assert not any(write[0] for write in setup[4].writes)
    assert setup[3].exports == []


def test_only_one_capture_until_operator_rearms_and_ids_are_late_bound(setup):
    arm(setup)
    radio, io, clock, scope, marker, session = setup
    scope.release_export.clear()
    first = receive(setup)
    assert scope.entered.wait(2)
    second = receive(setup, sequence=2)
    assert second.state is State.RX_SINGLE
    assert len([write for write in marker.writes if write[0]]) == 1
    investigation.investigate("occurrence", packet=first.receive_event, sequence=19)
    investigation.investigate("diagnostic", episode=first.episodes[0],
        diagnostic=SimpleNamespace(diagnostic_sequence=23), admission=SimpleNamespace(name="RESERVED"))
    scope.release_export.set()
    eventually(lambda: incident(session) and incident(session)["capture_status"] == "SAVED")
    assert incident(session)["occurrence_sequence"] == 19
    assert incident(session)["diagnostic_sequence"] == 23
    assert incident(session)["diagnostic_admission"] == "RESERVED"
    assert len(scope.exports) == 1
    arm(setup)
    receive(setup, sequence=3)
    eventually(lambda: incident(session, 3) and incident(session, 3)["capture_status"] == "SAVED")
    assert len(scope.exports) == 2


def test_failed_export_retains_partial_capture_and_lowers_gpio(setup):
    arm(setup)
    scope, marker, session = setup[3:]
    scope.failure = TimeoutError("lost scope connection")
    assert receive(setup).state is State.RX_SINGLE
    eventually(lambda: incident(session) and incident(session)["capture_status"] == "FAILED")
    assert scope.exports[0].joinpath("partial.bin").read_bytes() == b"retained"
    assert marker.writes[-1][0] is False
    assert session.error is not None and not session.armed


def test_marker_failure_does_not_replace_unexpected_irq_or_recovery(setup):
    arm(setup)
    setup[4].fail_high = True
    result = receive(setup)
    assert result.state is State.RX_SINGLE
    assert result.episodes[0].error_code.name == "UNEXPECTED_IRQ"
    eventually(lambda: incident(setup[5]) is not None)
    assert incident(setup[5])["capture_status"] == "MARKER_FAILED"
    assert setup[3].exports == []


def test_shutdown_before_recovery_result_retains_context_and_releases_marker(setup):
    arm(setup)
    clock, marker, session = setup[2], setup[4], setup[5]
    session.observe("zero_irq", edge=Dio1Edge(clock.now_monotonic_us() * 1000, 1),
        started=clock.now_monotonic_us(),
        event=SimpleNamespace(irq_status=0, chip_status=0x54, device_errors=0),
        episode=SimpleNamespace(_trigger=SimpleNamespace(at_us=clock.now_monotonic_us())))
    session.close()  # teardown can occur before an episode/diagnostic is returned
    assert marker.released
    assert marker.writes[-1][0] is False
    assert incident(session)["capture_status"] == "INCOMPLETE_SHUTDOWN"
    assert incident(session)["diagnostic_sequence"] is None


@pytest.mark.parametrize("irq", [2, 0x20, 0x200])
def test_other_irq_classes_do_not_raise_marker(setup, irq):
    arm(setup)
    receive(setup, irq=irq)
    assert not any(write[0] for write in setup[4].writes)
    assert setup[5].records == {}
