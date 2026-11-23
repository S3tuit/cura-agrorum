"""Exercise write suppression and verification with independently framed replies."""
import ctypes
from pathlib import Path
import subprocess

import pytest

ROOT = Path(__file__).resolve().parent
EXCHANGE = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p,
                            ctypes.POINTER(ctypes.c_uint8),
                            ctypes.POINTER(ctypes.c_uint8), ctypes.c_size_t)


def frame(payload):
    data = bytes.fromhex(payload)
    remainder = 65535
    for byte in data:
        remainder ^= byte
        for _ in range(8):
            remainder = (remainder >> 1) ^ (40961 if remainder & 1 else 0)
    return data + remainder.to_bytes(2, "little")


ONE = frame("01 03 02 00 01")
TWO = frame("02 03 02 00 02")
ACK = frame("01 06 07 d0 00 02")


@pytest.fixture(scope="module")
def library(tmp_path_factory):
    output = tmp_path_factory.mktemp("commission") / "commission.so"
    subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC",
                    str(ROOT / "main/commission.c"), "-o", str(output)], check=True)
    lib = ctypes.CDLL(str(output))
    lib.sen0600_assign_two.argtypes = [EXCHANGE, ctypes.c_void_p]
    lib.sen0600_assign_two.restype = ctypes.c_int
    return lib


def run(library, replies):
    requests = []
    pending = list(replies)
    errors = []

    @EXCHANGE
    def exchange(context, request, response, capacity):
        packet = bytes(request[:8])
        requests.append(packet)
        if not pending:
            errors.append("unexpected transaction")
            return -1
        answer = pending.pop(0)
        if isinstance(answer, int):
            return answer
        if len(answer) > capacity:
            errors.append("response overflow")
            return -1
        for i, value in enumerate(answer):
            response[i] = value
        return len(answer)

    result = library.sen0600_assign_two(exchange, None)
    assert not errors and not pending
    # Check every transmitted frame and forbid broadcast/calibration/baud writes.
    assert all(packet == frame(packet[:6].hex()) for packet in requests)
    assert all(packet[0] in (1, 2) and packet[2:4] == b"\x07\xd0" for packet in requests)
    writes = [packet for packet in requests if packet[1] == 6]
    assert all(packet == ACK for packet in writes)
    assert len(writes) <= 1
    return result, requests, writes


@pytest.mark.parametrize("ack", [ACK, frame("02 06 07 d0 00 02"), b""])
def test_assign_and_verify_new_and_old_address(library, ack):
    result, requests, writes = run(library, [b"", ONE, ack, TWO, b""])
    assert result == 0 and len(writes) == 1
    assert [p[0] for p in requests] == [2, 1, 1, 2, 1]


@pytest.mark.parametrize("replies,result", [([TWO, b""], 1), ([b"", b""], 2),
    ([TWO, ONE], 3), ([b"", ONE[:-1] + bytes([ONE[-1] ^ 1])], 4),
    ([b"", frame("01 83 02")], 4), ([b"", frame("01 03 02 00 02")], 4),
    ([-1, ONE], 4), ([frame("01 03 02 00 01"), ONE], 4)])
def test_no_write_on_existing_address_conflict_or_bad_traffic(library, replies, result):
    actual, _, writes = run(library, replies)
    assert actual == result and not writes


@pytest.mark.parametrize("ack", [-1, frame("01 86 02"), frame("01 06 07 d1 00 02"),
                                ACK[:-1], ACK[:-1] + bytes([ACK[-1] ^ 1])])
def test_uncertain_write_is_never_retried(library, ack):
    result, _, writes = run(library, [b"", ONE, ack])
    assert result == 5 and len(writes) == 1


@pytest.mark.parametrize("after", [[b"", ONE], [TWO, ONE], [b"", b""], [TWO, -1]])
def test_readback_failure_is_not_success_or_retried(library, after):
    result, _, writes = run(library, [b"", ONE, ACK, *after])
    assert result == 6 and len(writes) == 1


def test_crc_reference():
    # DFRobot's published read-measurements request is an external CRC anchor.
    assert frame("01 03 00 00 00 02") == bytes.fromhex("01 03 00 00 00 02 c4 0b")
