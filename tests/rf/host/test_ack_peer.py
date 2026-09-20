"""Real ACK peer over production Radio/Sx1262 and the existing physical fake."""
from pathlib import Path
import runpy
import struct
import threading

from cryptography.hazmat.primitives.ciphers.aead import AESCCM
import pytest

from cura_receiver.radio import Radio, State
from cura_receiver.sx1262 import Sx1262
from test_apps.radio_peer.ack_cases import AckCase
from test_apps.radio_peer.ack_peer import execute

support = runpy.run_path(str(Path(__file__).with_name("test_peer.py")))


@pytest.mark.parametrize("sleep_seconds", [900, 10])
@pytest.mark.parametrize("action,downlinks", [("accepted", 4), ("invalid_auth", 5)])
def test_production_radio_peer_completes_real_scheduled_sequence(action, downlinks, sleep_seconds):
    clock = support["Clock"](monotonic_us=10000)
    io = support["Air"](clock)
    backend = Sx1262(io, clock, clock)
    radio = None if action == "invalid_auth" else Radio(backend)
    if radio:
        assert radio.initialize().state is State.RX_SINGLE
    else:
        deadline = backend.deadline(2_000_000)
        backend.open(deadline)
        backend.initialize(deadline)
        backend.arm_receive(deadline)
    key, node = bytes(range(16)), bytes.fromhex("0102030405060708")
    policy = AckCase("RF-019.current." + action, node, key, sleep_seconds)
    interval = (sleep_seconds + 1) * 1_000_000
    if sleep_seconds == 10:
        original_complete = policy.complete
        def observed_complete(now):
            if now >= 2 * interval + 2_000_000 and not policy.sleep_entered.is_set():
                policy.observe_sleep(f"SLEEP {'a'*32} {policy.plan['case']} 3\n", 'a'*32, policy.plan['case'])
            return original_complete(now)
        policy.complete = observed_complete
    bodies = [struct.pack("<IHHHhhhIHBBHHBBH", 40 + i, 10, 0, 0, 0, 0, 0, 0, 0,
                         1 if i == 0 else 8, 0 if i == 0 else 1, 0 if i == 0 else 100,
                         0, 0 if i == 0 else 1, 0, 0 if i == 0 else 257)
              for i in range(3)]
    for message, domain, payload, at in (
        (100, 1, bodies[0], 1_000_000), (101, 1, bodies[1], 1_000_000 + interval),
        (102, 2, bodies[0], 1_500_000 + interval), (103, 1, bodies[2], 1_000_000 + 2 * interval)):
        header = struct.pack("<BB8sI", 32, domain, node, message)
        nonce = struct.pack("<8sIB", node, message, domain)
        frame = header + AESCCM(key, tag_length=8).encrypt(nonce, payload, header)
        io.incoming.append((at, frame))
    outcome = execute(policy, backend, radio, threading.Event(), clock.now_monotonic_us())
    assert len(outcome["packets"]) == 4
    assert len(outcome["transmissions"]) == downlinks
    assert len([c for c in io.commands if c[0] == 0x83]) == downlinks
    assert backend.profile == "rx"
    assert outcome["observation_end"] >= (1_838_000_000 if sleep_seconds == 900 else 2 * interval + 2_000_000)
    assert outcome["final_sleep_observed"] == (sleep_seconds == 10)
    if radio:
        assert radio.shutdown().safe_shutdown is True
    else:
        backend.safe_standby(backend.deadline(500_000))
        backend.close()
