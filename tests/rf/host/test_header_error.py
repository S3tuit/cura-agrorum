"""Real owner/peer handling and independent RF evidence rejection."""
from copy import deepcopy
import threading

import pytest

from .test_peer import rig, peer
from .test_verifier import record, profile_trace
from verify import A, U, verify_case

CASE = "component.header_error_rearm"


@pytest.mark.parametrize("stimulus", [True, False])
def test_peer_requires_header_between_complete_packets(stimulus):
    clock, io, backend, radio = rig(CASE)
    start = clock.now_monotonic_us()
    io.incoming.extend([(start + 100000, peer.A), (start + 24500000, peer.U2)])
    original = io.wait_edge
    injected = False
    def wait_edge(*, deadline_monotonic_us):
        nonlocal injected
        at = start + 12300000
        if stimulus and not injected and clock.now_monotonic_us() <= at <= deadline_monotonic_us:
            clock.wait_until_monotonic_us(at)
            io.status, io.irq = 0x52, 0x20
            io.sequence += 1
            injected = True
            from cura_receiver.ports.radio import Dio1Edge
            return Dio1Edge(at * 1000, io.sequence)
        return original(deadline_monotonic_us=deadline_monotonic_us)
    io.wait_edge = wait_edge
    if not stimulus:
        with pytest.raises(RuntimeError, match="missing real HeaderErr"):
            peer.execute(CASE, backend, radio, threading.Event(), start)
        return
    result = peer.execute(CASE, backend, radio, threading.Event(), start)
    assert len(result["handled"]) == 1 and len(result["packets"]) == 2
    assert result["counters_before"] == result["counters_after"]
    assert not any(c[0] == 0x83 for c in io.commands)


def header_record():
    events, capture = deepcopy(record())
    for v in events:
        if "case" in v: v["case"] = CASE
        if "previous_case" in v: v["previous_case"] = CASE
    first = deepcopy(events[3])
    cut = dict(first, before=12200190, after=12220000, deadline=14200190,
               set_tx=12200200, tx_done=0, done=False)
    final = dict(first, payload=U, before=24400100, after=24502900,
                 deadline=26400100, set_tx=24400200, tx_done=24502856)
    events[3:5] = [first, cut, final]
    trace_index = next(i for i, v in enumerate(events) if v.get("operation") == "start_tx")
    events[trace_index+1:trace_index+1] = [
        dict(kind="trace", operation="start_tx", before=cut["set_tx"], after=cut["set_tx"]+100, result=2),
        dict(kind="trace", operation="start_tx", before=final["set_tx"], after=final["set_tx"]+100, result=2),
        dict(kind="trace", operation="header_abort", argument=18000, result=2, before=cut["set_tx"],
             after=cut["after"], tx_hal_after=cut["set_tx"]+5253,
             abort_hal_before=cut["set_tx"]+18005, irq=0, device_errors=0),
    ]
    capture["case"], capture["attempts"] = CASE, 0
    packets = [dict(frame=A, irq_status=2, device_errors=0, edge_timestamp_ns=1000000000,
                    t2_packet_copied_monotonic_us=1000100),
               dict(frame=U, irq_status=2, device_errors=0, edge_timestamp_ns=3000000000,
                    t2_packet_copied_monotonic_us=3000100)]
    counters = dict(recovery_attempts=0, recovery_successes=0, recovery_failures=0,
                    recovery_attempts_by_reason=[0]*8, header_errors=0, crc_errors=0)
    handled = dict(state="RX_SINGLE", episodes=[], t6_set_rx_issued_monotonic_us=2000000,
                   receive_event=dict(disposition="HANDLED_NO_PACKET", frame=None, irq_status=32,
                                      device_errors=0, edge_timestamp_ns=1900000000,
                                      t1_handler_started_monotonic_us=1900100))
    capture["outcome"] = dict(packets=packets, transmissions=[], handled=[handled],
                              counters_before=counters, counters_after=deepcopy(counters))
    def spi(tx, reply="a222", before=1900200):
        return dict(operation="spi", tx=tx, result=reply, before=before, after=before)
    restoration = profile_trace(False)
    for v in restoration: v.update(before=1999999, after=1999999)
    restoration[-1].update(before=2000001, after=2000002)
    capture["trace"] = (profile_trace(False) + profile_trace(False) + [
        spi("12000000", "d2d20020"), spi("17000000", "d2d20000"), spi("c000", "d252"),
        spi("8000"), spi("c000", "d224"), spi("020020"),
    ] + restoration + [spi("c000", "d254", 2000003)] + profile_trace(False) +
        [spi(c) for c in ("8000", "080000000000000000", "12000000", "02ffff", "c000")] +
        [dict(operation="close", result=None)])
    return events, capture


def verify(events, capture):
    return verify_case(CASE, events, capture, "a"*32, {"c6_dut": "cc8da2fc0224"}, "image")


def test_reviewed_header_record():
    assert verify(*header_record())["c6_attempts"] == 3


@pytest.mark.parametrize("fault", ["irq", "status", "device", "standby", "clear", "profile", "setrx",
                                  "disposition", "recovery", "counter", "final_packet", "extra_tx",
                                  "missing_result", "abort", "pacing", "reset"])
def test_header_verifier_rejects_missing_or_contradictory_evidence(fault):
    events, capture = header_record()
    spi = capture["trace"]
    header = next(v for v in spi if v.get("result") == "d2d20020")
    i = spi.index(header)
    if fault == "irq": header["result"] = "d2d20000"
    elif fault == "status": spi[i+2]["result"] = "d224"
    elif fault == "device": spi[i+1]["result"] = "d2d20001"
    elif fault == "standby": spi[i+4]["result"] = "d252"
    elif fault == "clear": spi[i+5]["tx"] = "02ffff"
    elif fault == "profile":
        spi.pop(next(j for j in range(i+6, len(spi)) if spi[j].get("tx") == "9f00"))
    elif fault == "setrx": capture["outcome"]["handled"][0]["t6_set_rx_issued_monotonic_us"] = 0
    elif fault == "disposition": capture["outcome"]["handled"][0]["receive_event"]["disposition"] = "FAILED"
    elif fault == "recovery": capture["outcome"]["handled"][0]["episodes"] = [{}]
    elif fault == "counter": capture["outcome"]["counters_after"]["header_errors"] = 1
    elif fault == "final_packet": capture["outcome"]["packets"].pop()
    elif fault == "extra_tx": spi.insert(0, dict(operation="spi", tx="83001900", result="a222"))
    elif fault == "missing_result": capture["outcome"]["handled"] = []
    elif fault == "abort": next(v for v in events if v.get("operation") == "header_abort")["abort_hal_before"] += 1000
    elif fault == "pacing": next(v for v in events if v.get("payload") == U)["set_tx"] -= 1
    elif fault == "reset": spi.append(dict(operation="reset", asserted=True, result=None, before=2000000))
    with pytest.raises((ValueError, IndexError)):
        verify(events, capture)
