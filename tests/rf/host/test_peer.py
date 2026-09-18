"""Run the actual peer/production owner/backend over the existing physical fake."""
from collections import deque
import importlib.util
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest

from cura_receiver.ports.radio import Dio1Edge
from cura_receiver.radio import Radio, State
from cura_receiver.sx1262 import Sx1262
from tests.support.fakes.os_clock import FakeOsClock
from tests.support.fakes.radio_io import PhysicalPort

module = importlib.util.spec_from_file_location("component_radio_peer", Path(__file__).resolve().parents[3] /
                                             "receiver/test_apps/radio_peer/peer.py")
peer = importlib.util.module_from_spec(module)
module.loader.exec_module(peer)


class Clock(FakeOsClock):
    def wait_until_monotonic_us(self, deadline):
        self.advance_elapsed_us(max(0, deadline - self.now_monotonic_us()))


class Air(PhysicalPort):
    def __init__(self, clock):
        super().__init__(clock)
        self.incoming = deque()
        self.tx_done = None
        self.sequence = 0
        self.lose_done = False

    def transfer(self, data, *, deadline_monotonic_us):
        result = super().transfer(data, deadline_monotonic_us=deadline_monotonic_us)
        if data[0] == 0x83:
            # Independent reviewed PHY durations by actual buffer size.
            duration = {1: 25856, 4: 30976, 23: 61696}[len(self.buffer)]
            self.tx_done = self.clock.now_monotonic_us() + duration
        return result

    def wait_edge(self, *, deadline_monotonic_us):
        candidate = self.tx_done if self.status == 0x64 and not self.lose_done else None
        incoming = False
        if self.status == 0x54 and self.incoming:
            candidate, incoming = self.incoming[0][0], True
        if candidate is None or candidate > deadline_monotonic_us:
            self.clock.wait_until_monotonic_us(deadline_monotonic_us)
            return None
        self.clock.wait_until_monotonic_us(candidate)
        self.status = 0x24
        self.irq = 2 if incoming else 1
        if incoming:
            _, frame = self.incoming.popleft()
            self.buffer[:] = frame
        else:
            self.tx_done = None
        self.sequence += 1
        return Dio1Edge(candidate * 1000, self.sequence)


def rig(case):
    clock = Clock(monotonic_us=10000)
    io = Air(clock)
    backend = Sx1262(io, clock, clock)
    radio = None if case == "RF-006.invalid" else Radio(backend)
    if radio:
        assert radio.initialize().state is State.RX_SINGLE
    else:
        end = backend.deadline(2000000)
        backend.open(end); backend.initialize(end); backend.arm_receive(end)
    return clock, io, backend, radio


@pytest.mark.parametrize("case,uplinks,downlinks", [
    ("RF-001.exchange", [peer.A], [peer.B]),
    ("RF-003.silence", [peer.A], []),
    ("RF-006.invalid", [peer.A], [b"\0", b"\xde\xad\xbe\xef", peer.X]),
    ("RF-008.silence", [peer.A, peer.U2], []),
    ("RF-008.exchange", [peer.A, peer.U2], [peer.B, peer.D2]),
    ("RF-009.untouched", [peer.A], []),
    ("RF-009.initialized", [peer.A], []),
    ("RF-010.wake", [peer.A, peer.U2], [peer.B, peer.D2]),
    ("RF-012.disconnected", [peer.A], []),
    ("RF-013.absent", [], []),
])
def test_real_peer_sequences(case, uplinks, downlinks):
    clock, io, backend, radio = rig(case)
    start = clock.now_monotonic_us()
    io.incoming.extend((start + 100000 + i * 6000000, frame) for i, frame in enumerate(uplinks))
    result = peer.execute(case, backend, radio, threading.Event(), start)
    written = [cmd[2:] for cmd in io.commands if cmd[0] == 0x0e]
    assert written == downlinks
    assert len([cmd for cmd in io.commands if cmd[0] == 0x83]) == len(downlinks)
    assert len(result["packets"]) == len(uplinks)
    assert backend.profile == "rx"
    assert io.registers[0x0736] & 4 and io.registers[0x08ac] == 0x96
    assert clock.now_monotonic_us() - start < 45_000_000
    if radio:
        assert radio.shutdown().safe_shutdown is True
    else:
        backend.safe_standby(backend.deadline(500000)); backend.close()


@pytest.mark.parametrize("fault", ["missing", "wrong", "extra", "lost_done", "stop"])
def test_peer_failures_do_not_become_silent_passes(fault):
    clock, io, backend, radio = rig("RF-001.exchange")
    start = clock.now_monotonic_us()
    stop = threading.Event()
    if fault != "missing":
        io.incoming.append((start + 100000, b"wrong" if fault == "wrong" else peer.A))
    if fault == "extra":
        io.incoming.append((start + 1000000, peer.A))
    if fault == "lost_done":
        io.lose_done = True
    if fault == "stop":
        stop.set()
    with pytest.raises(RuntimeError):
        peer.execute("RF-001.exchange", backend, radio, stop, start)
    assert sum(cmd[0] == 0x83 for cmd in io.commands) <= 1
    assert radio.shutdown().safe_shutdown is True


def test_missed_local_burst_target_stops_before_tx():
    clock, io, backend, _ = rig("RF-006.invalid")
    clock.advance_elapsed_us(500000)
    with pytest.raises(RuntimeError, match="missed local target"):
        peer.lower_burst(backend, {"edge_timestamp_ns": 0}, threading.Event())
    assert not any(cmd[0] == 0x83 for cmd in io.commands)


@pytest.mark.parametrize("version,method", [("3.6", True), ("3.8", False), ("4.0", True)])
def test_dependency_preflight_rejects_incompatible_spi_before_device_access(monkeypatch, version, method):
    modules = {"gpiod": SimpleNamespace(__version__="2.2.0"), "spidev": SimpleNamespace(
        __version__=version, SpiDev=SimpleNamespace(open_path=(lambda _: None) if method else None))}
    monkeypatch.setattr(peer.importlib, "import_module", modules.__getitem__)
    with pytest.raises(RuntimeError):
        peer.dependencies()


def test_trace_admission_guard_prevents_unarmed_or_extra_settx(monkeypatch):
    clock = Clock(monotonic_us=10000)
    calls = []
    monkeypatch.setattr(peer.LinuxRadioIo, "transfer", lambda self, data, **kwargs: calls.append(data) or bytes(len(data)))
    io = peer.TraceIo(clock, 1)
    with pytest.raises(RuntimeError, match="not armed"):
        io.transfer(b"\x83\0\0\1", deadline_monotonic_us=50000)
    assert calls == []
    io.transmit_deadline = 20000
    io.transfer(b"\x83\0\0\1", deadline_monotonic_us=50000)
    with pytest.raises(RuntimeError, match="ceiling"):
        io.transfer(b"\x83\0\0\1", deadline_monotonic_us=50000)
    assert len(calls) == io.attempts == 1
