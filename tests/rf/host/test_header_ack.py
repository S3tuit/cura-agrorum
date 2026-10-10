"""Interrupted ACK sender over real backend commands and a physical-port fake."""
from pathlib import Path
import runpy
import threading

import pytest

from test_apps.radio_peer.header_ack import send

support = runpy.run_path(str(Path(__file__).with_name("test_peer.py")))


@pytest.mark.parametrize("valid_after", [False, True])
def test_two_interrupted_acks_rearm_and_reserve_completion_margin(valid_after):
    clock, io, backend, _ = support["rig"]("component.invalid_downlinks")
    origin = clock.now_monotonic_us()
    records = send(backend, {"edge_timestamp_ns": origin * 1000},
                   [bytes(23)] * (3 if valid_after else 2), threading.Event(), interrupt_count=2)
    assert [r["interrupted"] for r in records] == [True, True] + ([False] if valid_after else [])
    for record in records[:2]:
        assert record["abort_before"] - record["set_tx"] == 18_000
        assert record["irq"] == record["device_errors"] == 0
    assert records[1]["set_tx"] - records[0]["abort_after"] >= 60_000
    if valid_after:
        assert records[2]["tx_done"] < origin + 400_000
    assert backend.profile == "rx"
    assert len([c for c in io.commands if c[0] == 0x83]) == len(records)


def test_missed_target_never_starts_an_ack():
    clock, io, backend, _ = support["rig"]("component.invalid_downlinks")
    packet = {"edge_timestamp_ns": clock.now_monotonic_us() * 1000}
    clock.advance_elapsed_us(120_000)
    with pytest.raises(RuntimeError, match="missed HeaderErr"):
        send(backend, packet, [bytes(23)] * 2, threading.Event(), interrupt_count=2)
    assert not any(c[0] == 0x83 for c in io.commands)


def test_cancelled_burst_never_starts_an_ack():
    clock, io, backend, _ = support["rig"]("component.invalid_downlinks")
    stop = threading.Event()
    stop.set()
    with pytest.raises(RuntimeError, match="cancelled"):
        send(backend, {"edge_timestamp_ns": clock.now_monotonic_us() * 1000},
             [bytes(23)] * 2, stop, interrupt_count=2)
    assert not any(c[0] == 0x83 for c in io.commands)
