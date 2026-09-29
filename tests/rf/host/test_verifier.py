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


def before_mode(trace, transmit):
    prefix = "83" if transmit else "82"
    return next(i for i in range(len(trace)-1, -1, -1)
                if trace[i].get("tx", "").startswith(prefix))


def spi(command):
    return dict(operation="spi", tx=command, result="a222")


# Each later write changes the effective profile despite the earlier correct one.
@pytest.mark.parametrize("transmit", [False, True], ids=["rx", "tx"])
@pytest.mark.parametrize("overwrite", [
    "8a00", "863641999b", "8b08040100", "8e1602", "8e0e04", "9300", "9f01", "a001",
    "080001000100000000", "0d07405678", "0d074134", "0d073f001434", "0d08ac94",
    "0d088900", "0d08880000", "packet_iq", "packet_length", "iq_workaround",
])
def test_later_profile_overwrite_is_rejected(transmit, overwrite):
    trace = complete_trace()
    if overwrite == "packet_iq":
        overwrite = f"8c000800{23 if transmit else 255:02x}01{int(not transmit):02x}"
    elif overwrite == "packet_length":
        overwrite = f"8c00080001{1:02x}{int(transmit):02x}"
    elif overwrite == "iq_workaround":
        overwrite = "0d07350404" if transmit else "0d07350400"
    trace.insert(before_mode(trace, transmit), spi(overwrite))
    with pytest.raises(ValueError):
        verify_pi_profiles(trace, [B])


def test_later_power_overwrite_fails_the_complete_case():
    events, peer = record()
    peer["trace"].insert(before_mode(peer["trace"], True), spi("8e1602"))
    with pytest.raises(ValueError):
        check(events, peer)


# Retained values from an earlier RX/TX profile cannot fill a missing fresh write.
@pytest.mark.parametrize("prefix", ["8a", "86", "8b", "8c", "8e", "93", "9f", "a0", "080263",
                                  "0d0740", "0d08ac", "0d0889", "0d0736"])
def test_every_receive_operation_requires_a_fresh_complete_profile(prefix):
    trace = complete_trace()
    index = next(i for i in range(len(trace)-1, -1, -1)
                 if trace[i].get("tx", "").startswith(prefix))
    trace.pop(index)
    with pytest.raises(ValueError):
        verify_pi_profiles(trace, [B])


@pytest.mark.parametrize("transmit", [False, True], ids=["rx", "tx"])
@pytest.mark.parametrize("interrupt", ["reset", "sleep", "packet_type", "unknown_command"])
def test_invalidated_profile_cannot_be_reused(transmit, interrupt):
    trace = complete_trace()
    events = {
        "reset": [dict(operation="reset", asserted=True, result=None),
                  dict(operation="reset", asserted=False, result=None)],
        "sleep": [spi("8400")],
        "packet_type": [spi("8a00"), spi("8a01")],
        "unknown_command": [spi("ffff")],
    }[interrupt]
    index = before_mode(trace, transmit)
    trace[index:index] = events
    with pytest.raises(ValueError):
        verify_pi_profiles(trace, [B])


@pytest.mark.parametrize("fault", ["asserted", "failed", "unrecorded", "malformed"])
def test_reset_must_succeed_and_be_released(fault):
    trace = complete_trace()
    reset = dict(operation="reset", asserted=fault == "asserted", result=None)
    if fault == "failed": reset["error"] = "OSError"
    elif fault == "unrecorded": reset.pop("result")
    elif fault == "malformed": reset["asserted"] = "false"
    trace.insert(0, reset)
    with pytest.raises(ValueError):
        verify_pi_profiles(trace, [B])


@pytest.mark.parametrize("command", ["8e0e", "8e0e0200", "8b070401", "0d0740", "0d07", "0dffff1424"])
def test_malformed_profile_write_is_rejected(command):
    trace = complete_trace()
    trace.insert(before_mode(trace, True), spi(command))
    with pytest.raises(ValueError):
        verify_pi_profiles(trace, [B])


@pytest.mark.parametrize("transmit", [False, True], ids=["rx", "tx"])
def test_byte_addressed_sync_writes_can_be_split(transmit):
    trace = complete_trace()
    end = before_mode(trace, transmit)
    index = next(i for i in range(end-1, -1, -1) if trace[i].get("tx") == "0d07401424")
    trace[index:index+1] = [spi("0d074124"), spi("0d074014")]
    verify_pi_profiles(trace, [B])


@pytest.mark.parametrize("transmit", [False, True], ids=["rx", "tx"])
def test_restored_register_and_command_values_are_accepted(transmit):
    trace = complete_trace()
    correct_iq = "0d0736a0" if transmit else "0d0736a4"
    trace[before_mode(trace, transmit):before_mode(trace, transmit)] = [
        spi("8e1602"), spi("8e0e02"), spi("0d074134"), spi("0d074124"),
        spi("0d088900"), spi("0d088984"), spi(correct_iq), spi(correct_iq),
    ]
    verify_pi_profiles(trace, [B])


@pytest.mark.parametrize("parameters,workaround", [("8b07040100", "0d088904"),
                                                    ("8c000800170101", "0d073600")])
def test_dependent_workaround_must_follow_latest_parameters(parameters, workaround):
    trace = complete_trace()
    index = before_mode(trace, True)
    trace.insert(index, spi(parameters))
    with pytest.raises(ValueError):
        verify_pi_profiles(trace, [B])
    trace.insert(index + 1, spi(workaround))
    verify_pi_profiles(trace, [B])


def test_register_read_cannot_hide_an_overwritten_sync_byte():
    trace = complete_trace()
    index = before_mode(trace, True)
    trace[index:index] = [spi("0d074134"), dict(operation="spi", tx="1d0740000000", result="002200001424")]
    with pytest.raises(ValueError):
        verify_pi_profiles(trace, [B])


def test_independent_profile_fields_can_be_reordered():
    trace = []
    for transmit, length in ((False, 255), (True, 23), (False, 255)):
        profile = profile_trace(transmit, length)
        # Packet type precedes LoRa parameters; dependent workarounds follow them.
        trace.extend(profile[:2] + list(reversed(profile[2:10])) + profile[10:])
    trace.extend(complete_trace()[-6:])
    verify_pi_profiles(trace, [B])


@pytest.mark.parametrize("interrupt", ["reset", "sleep", "packet_type"])
def test_complete_reinstallation_after_invalidation_is_accepted(interrupt):
    trace = complete_trace()
    events = {
        "reset": [dict(operation="reset", asserted=True, result=None),
                  dict(operation="reset", asserted=False, result=None)],
        "sleep": [spi("8400")],
        "packet_type": [spi("8a00")],
    }[interrupt]
    index = before_mode(trace, True)
    trace[index:index] = events + profile_trace(True, 23)[:-1]
    verify_pi_profiles(trace, [B])
