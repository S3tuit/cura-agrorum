"""Reviewed minimal record and mutations that would otherwise hide RF failures."""
from copy import deepcopy
import pytest
from verify import A, B, verify_case, verify_pi_profiles


def profile_trace(transmit, length=255):
    commands = ["8000", "8a01", "863641999a", "8b07040100",
                f"8c000800{length:02x}01{int(transmit):02x}", "8e0e02", "9320", "9f00", "a000",
                "080263026300000000", "0d07401424", "0d08ac96", "0d088904",
                "0d073600" if transmit else "0d073604", "83001900" if transmit else "82000000"]
    return [dict(operation="spi", tx=command, result="a222") for command in commands]


def complete_trace():
    return (profile_trace(False) + [dict(operation="spi", tx="0e00"+B, result="a222")] +
            profile_trace(True, 23) + profile_trace(False) +
            [dict(operation="spi", tx=command, result="a222") for command in
             ("8000", "080000000000000000", "12000000", "02ffff", "c000")] +
            [dict(operation="close", result=None)])


def record():
    identity = dict(run="a"*32, case="RF-001.exchange", boot=11, phase=0)
    boot = dict(kind="boot", dut="cc8da2fc0224", elf="image", boot=11)
    tx = dict(kind="result", tx=True, payload=A, before=100, after=102900, deadline=2000100,
              error=0, operation=0, diagnostic="", started=True, done=True, set_tx=200, tx_done=102856)
    rx = dict(kind="result", tx=False, payload=B, before=103000, after=400100, deadline=3103000,
              error=0, operation=0, diagnostic="", rx_outcome=1, rx_done=400000, rssi=-100, snr=20)
    events = [boot, dict(kind="begin", **identity),
              dict(kind="command", boot=11, bytes=58, elapsed_us=8000), tx, rx,
              dict(kind="trace", operation="initialize", before=100, after=150, result=1),
              dict(kind="trace", operation="start_tx", before=200, after=250, result=2),
              dict(kind="trace", operation="set_sleep_cold", before=500000, after=500020, result=2),
              dict(kind="end", **identity, failed=0, cleanup_error=0),
              dict(kind="boot", dut=boot["dut"], elf="image", boot=12, reset=8, previous_boot=11,
                   previous_run=identity["run"], previous_case=identity["case"])]
    peer = dict(kind="complete", run=identity["run"], case=identity["case"], failure=None,
                cleanup=dict(safe_shutdown=True), attempts=1, layer="Radio/Sx1262/LinuxRadioIo",
                outcome=dict(packets=[dict(frame=A, irq_status=2, device_errors=0,
                             edge_timestamp_ns=102856000, t2_packet_copied_monotonic_us=103000)],
                             transmissions=[dict(tx=dict(ack_tx_result="TX_DONE", t5_tx_done_monotonic_us=461696),
                                                 t6_set_rx_issued_monotonic_us=470000)]),
                trace=complete_trace())
    return events, peer


def check(events, peer):
    return verify_case("RF-001.exchange", events, peer, "a"*32, {"c6_dut": "cc8da2fc0224"}, "image")


def test_reviewed_nominal_record():
    assert check(*record())["status"] == "PASS"


@pytest.mark.parametrize("fault", ["payload", "clock", "timing", "reset", "failed", "cleanup", "extra_tx",
                                  "peer_packet", "peer_cleanup", "peer_extra", "restoration", "missing_irq",
                                  "command_missing", "command_late", "command_rejected"])
def test_independent_verifier_rejects_hidden_failures(fault):
    events, peer = deepcopy(record())
    if fault == "payload": events[4]["payload"] = "00"
    elif fault == "clock": events[3]["set_tx"] = 0
    elif fault == "timing": events[3]["tx_done"] = 102000
    elif fault == "reset": events[-1]["reset"] = 3
    elif fault == "failed": events[-2]["failed"] = 1
    elif fault == "cleanup": events[-2]["cleanup_error"] = 0x20004
    elif fault == "extra_tx": events.insert(-2, dict(events[6]))
    elif fault == "command_missing": events.pop(2)
    elif fault == "command_late": events[2]["elapsed_us"] = 2000001
    elif fault == "command_rejected": events.append(dict(kind="reject"))
    elif fault == "peer_packet": peer["outcome"]["packets"][0]["frame"] = "00"
    elif fault == "peer_cleanup": peer["cleanup"]["safe_shutdown"] = False
    elif fault == "peer_extra": peer["trace"].append(dict(operation="spi", tx="83001900"))
    elif fault == "restoration": peer["outcome"]["transmissions"][0]["t6_set_rx_issued_monotonic_us"] = None
    elif fault == "missing_irq": peer["outcome"]["packets"][0]["irq_status"] = 0
    with pytest.raises(ValueError):
        check(events, peer)


@pytest.mark.parametrize("fault", ["frequency", "modulation", "sync", "boost", "iq_workaround", "iq_packet",
                                  "rx_missing", "irq_disable", "irq_clear", "standby", "close"])
def test_raw_profile_and_cleanup_corruption_cannot_hide_behind_reported_rx_state(fault):
    trace = complete_trace()
    prefix = dict(frequency="86", modulation="8b", sync="0d0740", boost="0d08ac",
                  iq_workaround="0d0736", iq_packet="8c", rx_missing="82",
                  irq_disable="080000", irq_clear="02ffff", standby="c000")
    if fault == "close":
        trace.pop()
    else:
        index = next(i for i in range(len(trace)-1, -1, -1) if trace[i].get("tx", "").startswith(prefix[fault]))
        if fault == "standby": trace[index]["result"] = "a252"
        elif fault == "iq_workaround": trace[index]["tx"] = "0d073600"
        elif fault == "iq_packet": trace[index]["tx"] = "8c000800ff0101"
        else: trace.pop(index)
    with pytest.raises(ValueError):
        verify_pi_profiles(trace, [B])
