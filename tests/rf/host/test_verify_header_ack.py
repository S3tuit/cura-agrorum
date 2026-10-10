"""Independent transcripts and falsified evidence for the node HeaderErr cases."""
import json
from pathlib import Path
import runpy
import struct

from cryptography.hazmat.primitives.ciphers.aead import AESCCM
import pytest

from verify_ack import verify_ack_case, verify_ack_transmissions
from verify_header_ack import node_observations, verify_header_ack_rf

base = runpy.run_path(str(Path(__file__).with_name("test_verify_ack.py")))
KEY, NODE = base["KEY"], base["NODE"]


def uart_bytes(groups):
    lines = []
    for group in groups:
        lines.extend("RF_NODE_PHY " + json.dumps(e) for e in group)
        lines.extend([f"RF_NODE_PHY_END count={len(group)} overflow=0", "RF_NODE_SLEEP duration_us=10000000"])
    return ("\n".join(lines) + "\n").encode()


def transcript(retry):
    packets, dump = base["accepted_transcript"]()
    first = 1_220_656
    done = 1_102_656
    calls = [dict(tx_started=True, tx_done=True, set_tx_at_us=1_000_000, tx_done_at_us=done,
                  outcome="ACK_TIMEOUT" if retry else "ACK_RECEIVED",
                  ack_rx_done_at_us=None if retry else done + 321_696)]
    if retry:
        packets.insert(2, dict(frame=packets[1]["frame"], at=packets[1]["at"] + 502_656))
        calls.append(dict(tx_started=True, tx_done=True, set_tx_at_us=done + 400_000,
                          tx_done_at_us=done + 502_656, outcome="ACK_RECEIVED",
                          ack_rx_done_at_us=done + 714_352))
        # Observation wake reports two current attempts plus one backlog attempt.
        body = list(struct.unpack("<IHHHhhhIHBBHHBBH", bytes.fromhex(dump["logs"]["pending.log"][0]["reading_body"])))
        body[10], body[13] = 2, 3
        raw = struct.pack("<IHHHhhhIHBBHHBBH", *body)
        header = bytes.fromhex(packets[-1]["frame"])[:14]
        nonce = struct.pack("<8sIB", NODE, 103, 1)
        packets[-1]["frame"] = (header + AESCCM(KEY, tag_length=8).encrypt(nonce, raw, header)).hex()
        dump["logs"]["pending.log"][0]["reading_body"] = raw.hex()
    dump["logs"]["delivery.log"][3].update(tx_calls=calls, attempt_count=len(calls))
    dump["logs"]["diagnostic.log"] = [dict(error_domain=4, error_code=15, operation=17,
        context_schema=1, cycle_sample_id=41, message_id=101,
        context=struct.pack("<BBQHH", 1, int(not retry), first, 2, 0).hex())]

    def observation(index, opcode, before, value=0, irq=0):
        return dict(index=index, opcode=opcode, value=value, before_us=before,
                    after_us=before + 1, irq_at_us=irq, result=0)

    group = []
    def add(opcode, before, value=0, irq=0):
        group.append(observation(len(group), opcode, before, value, irq))
    add(0x83, 1_000_000)
    add(0x12, done + 1, 1, done)
    add(0x82, done + 10)
    add(0x12, first + 1, 0x20, first)
    add(0x82, first + 10)
    add(0x12, first + 80_001, 0x20, first + 80_000)
    add(0x82, first + 80_010)
    if retry:
        add(0x83, calls[1]["set_tx_at_us"])
        add(0x12, calls[1]["tx_done_at_us"] + 1, 1, calls[1]["tx_done_at_us"])
        add(0x82, calls[1]["tx_done_at_us"] + 10)
    add(0x12, calls[-1]["ack_rx_done_at_us"] + 1, 2, calls[-1]["ack_rx_done_at_us"])
    groups = [[observation(0, 0x83, 1)], group, [observation(0, 0x83, 1)]]

    origin = packets[1]["at"]
    targets = [packets[0]["at"] + 150_000, origin + 100_000, origin + 180_000,
               packets[2]["at"] + 150_000 if retry else origin + 260_000,
               packets[-2]["at"] + 150_000, packets[-1]["at"] + 150_000]
    transmissions, trace = [], []
    for i, (message, status) in enumerate(((100, 1), (101, 0), (101, 0), (101, 0), (102, 0), (103, 1))):
        header = struct.pack("<BB8sI", 32, status + 3, NODE, message)
        nonce = struct.pack("<8sIB", NODE, message, status + 3)
        frame = header + AESCCM(KEY, tag_length=8).encrypt(nonce, bytes([status]), header)
        start = targets[i]
        cut = i in (1, 2)
        t = dict(frame=frame.hex(), target=start, set_tx=start, interrupted=cut)
        if cut:
            t.update(abort_before=start + 18_000, abort_after=start + 18_100, irq=0, device_errors=0)
        else:
            t["tx_done"] = start + 61_696
        transmissions.append(t)
        trace.extend([dict(operation="spi", tx="0e00" + frame.hex(), before=start - 1000, after=start - 900),
                      dict(operation="spi", tx="83001900", before=start, after=start + 10)])
        if cut:
            for j, (cmd, reply) in enumerate((("8000", "d200"), ("c000", "d224"),
                                            ("12000000", "d2d20000"), ("17000000", "d2d20000"))):
                trace.append(dict(operation="spi", tx=cmd, result=reply,
                                  before=start + 18_000 + j * 10, after=start + 18_010 + j * 10))
    return dict(packets=packets, transmissions=transmissions), trace, dump, groups


def retime_control_ack(dump, groups, ack_time):
    dump["logs"]["delivery.log"][3]["tx_calls"][-1]["ack_rx_done_at_us"] = ack_time
    ack = next(e for e in groups[1] if e["opcode"] == 0x12 and e["value"] == 2)
    # Leave room for the TX_DONE read and SetRx at the zero-delta boundary.
    ack.update(irq_at_us=ack_time, before_us=ack_time + 20, after_us=ack_time + 21)
    groups[1].sort(key=lambda e: e["before_us"])
    for index, event in enumerate(groups[1]):
        event["index"] = index


@pytest.mark.parametrize("retry", [False, True])
@pytest.mark.parametrize("header_irq", [0x20, 0x22])
def test_complete_independent_header_ack_evidence(retry, header_irq):
    outcome, trace, dump, groups = transcript(retry)
    groups[1][3]["value"] = groups[1][5]["value"] = header_irq
    case = "node.current.header_error_" + ("retry" if retry else "rearm")
    verify_ack_case(case, NODE, KEY, outcome["packets"], dump)
    verify_ack_transmissions(case, NODE, KEY, outcome, trace, 6)
    verify_header_ack_rf(case, outcome, trace, dump, uart_bytes(groups))


@pytest.mark.parametrize("point", ["before_set_tx", "during_tx", "before_tx_done", "at_tx_done"])
def test_retry_control_ack_must_follow_its_own_tx_done(point):
    outcome, trace, dump, groups = transcript(True)
    call = dump["logs"]["delivery.log"][3]["tx_calls"][-1]
    ack_time = {
        "before_set_tx": call["set_tx_at_us"] - 100_000,
        "during_tx": call["set_tx_at_us"] + 1,
        "before_tx_done": call["tx_done_at_us"] - 1,
        "at_tx_done": call["tx_done_at_us"],
    }[point]
    retime_control_ack(dump, groups, ack_time)
    uart = uart_bytes(groups)
    # Matching saved/raw timestamps and a valid trace still need attempt binding.
    node_observations(uart)
    case = "node.current.header_error_retry"
    verify_ack_case(case, NODE, KEY, outcome["packets"], dump)
    verify_ack_transmissions(case, NODE, KEY, outcome, trace, 6)
    with pytest.raises(ValueError, match="valid control ACK outside unchanged window"):
        verify_header_ack_rf(case, outcome, trace, dump, uart)


@pytest.mark.parametrize("retry", [False, True])
@pytest.mark.parametrize("deadline_offset", [-1, 0, 1])
def test_control_ack_window_deadline_is_inclusive(retry, deadline_offset):
    outcome, trace, dump, groups = transcript(retry)
    call = dump["logs"]["delivery.log"][3]["tx_calls"][-1]
    window_us = 300_000 if retry else 400_000
    retime_control_ack(dump, groups, call["tx_done_at_us"] + window_us + deadline_offset)
    uart = uart_bytes(groups)
    node_observations(uart)
    case = "node.current.header_error_" + ("retry" if retry else "rearm")
    if deadline_offset > 0:
        with pytest.raises(ValueError, match="valid control ACK outside unchanged window"):
            verify_header_ack_rf(case, outcome, trace, dump, uart)
    else:
        verify_header_ack_rf(case, outcome, trace, dump, uart)


@pytest.mark.parametrize("damage", ["no_irq", "payload_crc", "mixed", "first_time", "wrong_attempt",
    "wrong_count", "ack_flag", "no_rearm", "overflow", "missing_end", "missing_wake", "reordered",
    "cutoff", "raw_cutoff", "bad_status", "raw_irq", "missing_spi", "late_start", "ack_time", "wrong_wake",
    "node_tx_done", "node_set_tx", "missing_node_tx", "rx_done_only", "rx_done_crc", "header_timeout"])
def test_missing_or_conflicting_evidence_fails(damage):
    outcome, trace, dump, groups = transcript(False)
    record = dump["logs"]["diagnostic.log"][0]
    context = bytearray.fromhex(record["context"])
    if damage == "no_irq": groups[1][3]["value"] = 0
    if damage == "payload_crc": groups[1][3]["value"] = 0x40
    if damage == "mixed": groups[1][3]["value"] = 0x60
    if damage == "rx_done_only": groups[1][3]["value"] = 0x02
    if damage == "rx_done_crc": groups[1][3]["value"] = 0x62
    if damage == "header_timeout": groups[1][3]["value"] = 0x220
    if damage == "first_time": context[2] ^= 1
    if damage == "wrong_attempt": context[0] = 2
    if damage == "wrong_count": context[10] = 1
    if damage == "ack_flag": context[1] = 0
    record["context"] = context.hex()
    if damage == "no_rearm": groups[1][4]["opcode"] = 0x83
    if damage == "reordered": groups[1][3], groups[1][4] = groups[1][4], groups[1][3]
    if damage == "cutoff": outcome["transmissions"][1]["abort_before"] += 5000
    if damage == "raw_cutoff": next(e for e in trace if e["tx"] == "8000")["before"] += 5000
    if damage == "bad_status": next(e for e in trace if e["tx"] == "c000")["result"] = "d228"
    if damage == "raw_irq": next(e for e in trace if e["tx"] == "12000000")["result"] = "d2d20001"
    if damage == "missing_spi": trace[:] = [e for e in trace if e["tx"] != "8000"]
    if damage == "late_start": outcome["transmissions"][1]["set_tx"] += 15_000
    if damage == "ack_time": dump["logs"]["delivery.log"][3]["tx_calls"][0]["ack_rx_done_at_us"] += 1
    if damage == "node_tx_done": groups[1][1]["irq_at_us"] -= 1
    if damage == "node_set_tx": dump["logs"]["delivery.log"][3]["tx_calls"][0]["set_tx_at_us"] -= 1_001
    if damage == "missing_node_tx": groups[1][0]["opcode"] = 0x82
    if damage == "wrong_wake": groups[0], groups[1] = groups[1], groups[0]
    uart = uart_bytes(groups)
    if damage == "overflow": uart = uart.replace(b"overflow=0", b"overflow=1", 1)
    if damage == "missing_end": uart = b"\n".join(l for l in uart.splitlines() if not l.startswith(b"RF_NODE_PHY_END"))
    if damage == "missing_wake": uart = uart_bytes(groups[:2])
    with pytest.raises(ValueError):
        verify_header_ack_rf("node.current.header_error_rearm", outcome, trace, dump, uart)


def test_reservations_include_all_interrupted_and_control_transmissions():
    from spec import node_episode
    for action in ("header_error_rearm", "header_error_retry"):
        plan = node_episode("node.current." + action, 10)
        assert plan["pi_max_packets"] == 6
        assert plan["charges"]["pi_us"] == 6 * 67_866
        assert plan["wakes"] == 3 and plan["lease_seconds"] == 160
