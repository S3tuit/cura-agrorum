"""F-004: cancel actual RF-006 sessions during production backend preparation."""
import json
from pathlib import Path
import queue
import runpy
import threading
from types import SimpleNamespace

import pytest

support = runpy.run_path(str(Path(__file__).with_name("test_peer.py")))
peer = support["peer"]


# STOP/EOF/signals must prevent a pending SetTx and preserve earlier TX facts.
@pytest.mark.parametrize("source", ["STOP", "EOF", "signal"])
@pytest.mark.parametrize("stage", ["buffer", "profile", "stale_irq"])
@pytest.mark.parametrize("packet_index", [1, 2])
def test_cancel_during_burst_preparation_keeps_cleanup(source, stage, packet_index, monkeypatch, capsys):
    clock = support["Clock"](monotonic_us=10000)
    air = support["Air"](clock)
    signals, context = {}, {}
    pending = queue.Queue()
    pending.put(f"GO {'a' * 32} RF-006.invalid\n")
    watchers = []

    class Input:
        def readline(self, *_):
            current = threading.current_thread()
            if current is not threading.main_thread():
                watchers.append(current)
            return pending.get(timeout=3)

    stream = Input()
    monkeypatch.setattr(peer.sys, "stdin", stream)
    monkeypatch.setattr(peer.select, "select", lambda *_: ([stream], [], []))
    monkeypatch.setattr(peer.signal, "signal", signals.__setitem__)
    monkeypatch.setattr(peer, "LinuxOsClock", lambda: clock)
    # Exercise TraceIo and Sx1262; only the actual physical Linux port is fake.
    for name in ("open", "close", "busy", "dio1", "set_reset", "wait_edge", "resynchronize_events"):
        def forward(self, *args, _name=name, **kwargs):
            return getattr(air, _name)(*args, **kwargs)
        monkeypatch.setattr(peer.LinuxRadioIo, name, forward)
    prepared = 0
    cancelled = False

    def transfer(self, data, **kwargs):
        nonlocal prepared, cancelled
        result = air.transfer(data, **kwargs)
        if data[0] == 0x0e:
            prepared += 1
        opcode = {"buffer": 0x0e, "profile": 0x8e, "stale_irq": 0x02}[stage]
        if not cancelled and prepared == packet_index and data[0] == opcode:
            cancelled = True
            if source == "signal":
                signals[peer.signal.SIGTERM](peer.signal.SIGTERM, None)
            else:
                pending.put("STOP\n" if source == "STOP" else "")
            assert context["stop"].wait(timeout=2), "control cancellation was not processed"
        return result

    monkeypatch.setattr(peer.LinuxRadioIo, "transfer", transfer)

    def execute(case, backend, radio, stop, started):
        assert radio is None
        context["stop"] = stop
        air.incoming.append((started + 100000, peer.A))
        return peer.execute(case, backend, radio, stop, started)

    args = SimpleNamespace(run="a" * 32, case="RF-006.invalid")
    try:
        status = peer.run_session(args, {}, "source", 3, execute)
    finally:
        pending.put("")
        for watcher in watchers:
            watcher.join(timeout=2)
            assert not watcher.is_alive()
    records = [json.loads(line) for line in capsys.readouterr().out.splitlines()]
    assert [record["kind"] for record in records] == ["ready", "armed", "complete"]
    complete = records[-1]
    assert status == 1 and cancelled
    assert complete["attempts"] == packet_index - 1
    assert sum(command[0] == 0x83 for command in air.commands) == packet_index - 1
    assert "cancelled before SetTx" in complete["failure"]
    assert complete["cleanup"]["safe_shutdown"] is True
    assert air.status == 0x24 and air.irq == 0
    assert air.calls.count(("close",)) == 1
    assert complete["trace"][-1]["operation"] == "close"
    assert complete["trace"][-1]["result"] is None
