"""Independent acceptance transcript and deliberately missing/wrong evidence."""
from copy import deepcopy
import struct

from cryptography.hazmat.primitives.ciphers.aead import AESCCM
import pytest

from verify_ack import verify_ack_case, verify_ack_transmissions

KEY = bytes(range(16))
NODE = bytes.fromhex("0102030405060708")


def reading(sample, previous_attempts, previous_accepted, flags, reset):
    return struct.pack("<IHHHhhhIHBBHHBBH", sample, 10, 0, 0, 0, 0, 0, 0, 0,
                       reset, 1 if previous_attempts else 0, 100 if previous_attempts else 0,
                       200 if flags & 512 else 0, previous_attempts, previous_accepted, flags)


def accepted_transcript():
    bodies = [reading(40, 0, 0, 0, 1), reading(41, 1, 0, 257, 8),
              reading(42, 2, 2, 769, 8)]
    packets, deliveries = [], []
    # Setup RETRY_LATER; accepted current; accepted backlog; observation RETRY_LATER.
    for message, sample, cycle, domain, result, body in (
        (100, 40, 40, 1, 2, bodies[0]), (101, 41, 41, 1, 1, bodies[1]),
        (102, 40, 41, 2, 1, bodies[0]), (103, 42, 42, 1, 2, bodies[2])):
        header = struct.pack("<BB8sI", 32, domain, NODE, message)
        nonce = struct.pack("<8sIB", NODE, message, domain)
        frame = header + AESCCM(KEY, tag_length=8).encrypt(nonce, body, header)
        packets.append(dict(frame=frame.hex(), at=1_000_000 + (cycle - 40) * 901_000_000))
        identity = dict(cycle_sample_id=cycle, sample_id=sample, message_id=message, domain=domain)
        deliveries.extend([dict(type=4, **identity, start_offset_ms=10),
                           dict(type=5, **identity, final_result=result, attempt_count=1)])
    logs = {"pending.log": [dict(type=1, sample_id=42, reading_body=bodies[2].hex())],
            "quarantine.log": None, "diagnostic.log": None, "delivery.log": deliveries}
    return packets, dict(logs=logs)


def test_real_packets_require_matching_node_storage():
    packets, dump = accepted_transcript()
    result = verify_ack_case("RF-019.current.accepted", NODE, KEY, packets, dump)
    assert result["status"] == "PASS"
    assert result["host_deadline_assertion"] == "separate prerequisite"


@pytest.mark.parametrize("damage", [None, "wrong_ack", "missing_spi", "extra_tx"])
def test_downlinks_match_actual_spi_and_selected_status(damage):
    packets, _ = accepted_transcript()
    transmissions, trace = [], []
    for message, status in ((100, 1), (101, 0), (102, 0), (103, 1)):
        domain = status + 3
        header = struct.pack("<BB8sI", 32, domain, NODE, message)
        nonce = struct.pack("<8sIB", NODE, message, domain)
        frame = header + AESCCM(KEY, tag_length=8).encrypt(nonce, bytes([status]), header)
        transmissions.append(dict(frame=frame.hex()))
        trace.extend([dict(operation="spi", tx="0e00" + frame.hex()), dict(operation="spi", tx="83000100")])
    if damage == "wrong_ack":
        transmissions[0]["frame"] = transmissions[1]["frame"]
    elif damage == "missing_spi":
        trace.pop(0)
    elif damage == "extra_tx":
        trace.append(dict(operation="spi", tx="83000100"))
    args = ("RF-019.current.accepted", NODE, KEY, dict(packets=packets, transmissions=transmissions), trace, 4)
    if damage:
        with pytest.raises(ValueError, match="ACK bytes"):
            verify_ack_transmissions(*args)
    else:
        verify_ack_transmissions(*args)


@pytest.mark.parametrize("damage", ["missing_finish", "wrong_result", "wrong_count", "extra_pending",
                                     "no_pending_file", "diagnostic", "quarantine", "missing_wake", "bad_tag"])
def test_incomplete_or_conflicting_evidence_cannot_pass(damage):
    packets, dump = accepted_transcript()
    logs = dump["logs"]
    if damage == "missing_finish":
        logs["delivery.log"].pop()
    elif damage == "wrong_result":
        logs["delivery.log"][3]["final_result"] = 2
    elif damage == "wrong_count":
        logs["delivery.log"][3]["attempt_count"] = 2
    elif damage == "extra_pending":
        logs["pending.log"].append(deepcopy(logs["pending.log"][0]))
    elif damage == "no_pending_file":
        logs["pending.log"] = None
    elif damage == "diagnostic":
        logs["diagnostic.log"] = [{"error_domain": 4, "error_code": 7}]
    elif damage == "quarantine":
        logs["quarantine.log"] = [dict(sample_id=41, reading_body="00" * 32)]
    elif damage == "missing_wake":
        packets.pop()
    else:
        frame = bytes.fromhex(packets[0]["frame"])
        packets[0]["frame"] = (frame[:-1] + bytes([frame[-1] ^ 1])).hex()
    with pytest.raises(ValueError):
        verify_ack_case("RF-019.current.accepted", NODE, KEY, packets, dump)
