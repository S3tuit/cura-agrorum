import struct

from cryptography.hazmat.primitives.ciphers.aead import AESCCM
import pytest

from test_apps.radio_peer.ack_cases import AckCase, CASES, episode
from cura_receiver.protocol_v2_lora_crypto import open_frame

KEY = bytes(range(16))
NODE = bytes.fromhex("0102030405060708")


def body(sample, *, first=False):
    return struct.pack("<IHHHhhhIHBBHHBBH", sample, 10, 0, 0, 0, 0, 0, 0, 0,
                       1 if first else 8, 0 if first else 1, 0 if first else 100,
                       0, 0 if first else 1, 0, 0 if first else 257)


def frame(message, payload, domain=1):
    header = struct.pack("<BB8sI", 0x20, domain, NODE, message)
    nonce = struct.pack("<8sIB", NODE, message, domain)
    return header + AESCCM(KEY, tag_length=8).encrypt(nonce, payload, header)


def seeds(policy):
    for i in range(policy.target_index):
        replies, _ = policy.receive(frame(100 + i, body(40 + i, first=i == 0)), 1_000_000 + i * 901_000_000)
        ack = open_frame(KEY, replies[0])
        assert ack.header.domain == 4 and ack.plaintext_body == b"\x01"


@pytest.mark.parametrize("case", CASES)
def test_real_authenticated_case_plans(case):
    policy = AckCase(case, NODE, KEY)
    seeds(policy)
    index = policy.target_index
    at = 1_000_000 + index * 901_000_000
    target = frame(100 + index, body(40 + index))
    replies, observation = policy.receive(target, at)
    action = policy.plan["action"]
    if policy.plan["scope"] == "backlog":
        assert open_frame(KEY, replies[0]).plaintext_body == b"\0"
        replies, _ = policy.receive(frame(101 + index, body(40 + index - 1), 2), at + 500_000)
    if action in ("accepted", "retry_later", "unsupported", "malformed"):
        expected = {"accepted": 0, "retry_later": 1, "unsupported": 2, "malformed": 3}[action]
        ack = open_frame(KEY, replies[0])
        assert (ack.header.domain, ack.plaintext_body) == (3 + expected, bytes([expected]))
    elif action == "invalid_auth":
        from cura_receiver.protocol_v2_lora_crypto import AuthenticationError
        with pytest.raises(AuthenticationError):
            open_frame(KEY, replies[0])
        assert open_frame(KEY, replies[1]).plaintext_body == b"\0"
    elif action == "wrong_message":
        assert open_frame(KEY, replies[0]).header.message_id == 0xffffffff
        assert policy.receive(target, at + 800_000)[0] == []
    else:
        ack = open_frame(KEY, replies[0])
        assert (ack.header.domain, ack.plaintext_body) == (4, b"\0")
    # A new current wake is the metrics observation and cannot drain evidence.
    replies, _ = policy.receive(frame(110, body(41 + index)), at + 901_000_000)
    assert open_frame(KEY, replies[0]).plaintext_body == b"\x01"
    assert not policy.complete(at + 935_000_000)
    assert policy.complete(at + 936_000_000)


def test_changed_retry_cannot_reuse_message_id():
    policy = AckCase("RF-019.current.accepted", NODE, KEY)
    policy.receive(frame(100, body(40, first=True)), 1_000_000)
    with pytest.raises(ValueError, match="changed frame"):
        policy.receive(frame(100, body(41, first=True)), 1_100_000)


@pytest.mark.parametrize("when,reading", [
    (2_000_000, body(41)),  # Unscheduled reset/shortened sleep.
    (902_000_000, body(42)),  # Unobserved wake.
    (902_000_000, body(41, first=True)),  # RTC/deep-sleep continuity missing.
])
def test_invalid_wake_sequence_fails(when, reading):
    policy = AckCase("RF-019.current.accepted", NODE, KEY)
    seeds(policy)
    with pytest.raises(ValueError):
        policy.receive(frame(101, reading), when)


def test_current_rejection_never_allows_backlog():
    policy = AckCase("RF-019.current.unsupported", NODE, KEY)
    seeds(policy)
    policy.receive(frame(101, body(41)), 902_000_000)
    with pytest.raises(ValueError, match="backlog after"):
        policy.receive(frame(102, body(40, first=True), 2), 902_500_000)


def test_case_selection_and_bounds_are_explicit():
    with pytest.raises(ValueError, match="unknown"):
        episode("RF-019")
    assert episode("RF-019.current.accepted")["wakes"] == 3
    assert episode("RF-019.backlog.accepted")["wakes"] == 4


@pytest.mark.parametrize("elapsed,valid", [
    (854_999_999, False), (855_000_000, True), (896_733_577, True),
    (945_000_000, True), (945_000_001, False),
])
def test_operator_wake_tolerance(elapsed, valid):
    policy = AckCase("RF-019.current.accepted", NODE, KEY)
    seeds(policy)
    if valid:
        policy.receive(frame(101, body(41)), 1_000_000 + elapsed)
    else:
        with pytest.raises(ValueError, match="scheduled wake outside"):
            policy.receive(frame(101, body(41)), 1_000_000 + elapsed)
