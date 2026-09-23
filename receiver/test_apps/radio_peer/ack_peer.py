"""RF-019 controlled authenticated peer; never the production receiver service."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import pwd
import re
import sys

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "receiver"))
sys.path.insert(0, str(REPO / "protocol/protocol-v2-lora/python"))

from cura_protocol_v2_lora.receiver_group import load_receiver_group
from cura_receiver.application import authentication_keys
from cura_receiver.platform.linux_radio import LinuxRadioIo
from cura_receiver.ports.radio import RadioTxAuthorization
from cura_receiver.radio import State
from cura_receiver.sx1262 import IRQ_TX_DONE
from test_apps.radio_peer.ack_cases import AckCase, CASES, episode
from test_apps.radio_peer import peer


class AckTraceIo(peer.TraceIo):
    def wait_edge(self, *, deadline_monotonic_us):
        # Sleeping between ordinary wakes produces no RF edge. Keep real edges
        # and failures without storing thousands of identical passive polls.
        before = self.clock.now_monotonic_us()
        try:
            edge = LinuxRadioIo.wait_edge(self, deadline_monotonic_us=deadline_monotonic_us)
        except BaseException as error:
            self.trace.append(dict(operation="edge", before=before, error=str(error),
                                   after=self.clock.now_monotonic_us()))
            raise
        if edge is not None:
            peer.require(len(self.trace) < 8192, "peer trace overflow")
            self.trace.append(dict(operation="edge", before=before, result=edge,
                                   after=self.clock.now_monotonic_us()))
        return edge


def burst(backend, packet, frames, stop):
    records = []
    origin = packet["edge_timestamp_ns"] // 1000
    for frame, delay in zip(frames, (150_000, 350_000)):
        peer.scheduled(backend.clock, origin + delay)
        peer.require(not stop.is_set(), "peer cancelled before ACK")
        deadline = backend.deadline(500_000)
        backend.write_buffer(frame, deadline)
        backend.install_profile(transmit=True, payload_length=23, deadline=deadline)
        backend.account_stale_irqs(deadline)
        backend.start_tx(deadline)
        issued = backend.last_set_tx_issued_us
        edge = backend.wait_edge(deadline_monotonic_us=issued + 250_000)
        peer.require(edge is not None and issued <= edge.monotonic_us <= issued + 250_000,
                     "unconfirmed ACK TX")
        end = backend.deadline(500_000)
        event = backend.observe_event(end)
        backend.validate_event(event, transmit=True)
        peer.require(event.irq_status == IRQ_TX_DONE, "ACK TX_DONE missing")
        backend.standby(end)
        backend.clear_irq(event.irq_status, end)
        records.append(dict(frame=frame.hex(), target=origin + delay,
                            set_tx=issued, tx_done=edge.monotonic_us))
    end = backend.deadline(500_000)
    backend.install_profile(transmit=False, payload_length=255, deadline=end)
    backend.arm_receive(end)
    return records


def execute(policy, backend, radio, stop, started):
    deadline = started + policy.plan["lease_seconds"] * 1_000_000 - 500_000
    packets, transmissions = [], []
    while backend.clock.now_monotonic_us() < deadline:
        peer.require(not stop.is_set(), "peer cancelled or lease expired")
        if policy.complete(backend.clock.now_monotonic_us()):
            return dict(packets=packets, transmissions=transmissions,
                        observation=policy.observation(),
                        final_sleep_observed=policy.sleep_entered.is_set(),
                        observation_end=backend.clock.now_monotonic_us())
        if radio is not None:
            result = peer.healthy(radio.receive(deadline_monotonic_us=deadline),
                                  State.RX_SINGLE, State.RX_EVENT_PENDING)
            packet = result.receive_event
            if packet is None:
                continue
            peer.require(packet.usable_for_ingress, "unusable RF packet")
            frame, at = packet.frame, packet.received_at_monotonic_us
        else:
            packet = peer.lower_receive(backend, deadline)
            if packet is None:
                continue
            frame, at = packet["frame"], packet["edge_timestamp_ns"] // 1000
        replies, _ = policy.receive(frame, at)
        packets.append(dict(frame=frame.hex(), at=at))
        if radio is not None:
            if replies:
                peer.require(len(replies) == 1, "burst requires explicit lower-layer case")
                peer.scheduled(backend.clock, at + 150_000)
                peer.require(not stop.is_set(), "peer cancelled before ACK")
                sequence = len(packets)
                peer.healthy(radio.prepare_ack(replies[0], occurrence_sequence=sequence), State.RX_EVENT_PENDING)
                peer.healthy(radio.start_ack(RadioTxAuthorization(
                    backend.clock.now_monotonic_us() + 500_000, occurrence_sequence=sequence)), State.TX_ACTIVE)
                result = peer.healthy(radio.finish_ack(), State.RX_SINGLE, State.RX_EVENT_PENDING)
                peer.require(result.tx is not None and result.tx.ack_tx_result.name == "TX_DONE", "unconfirmed ACK")
                transmissions.append(dict(frame=replies[0].hex(), result=result))
            else:
                peer.healthy(radio.rearm(), State.RX_SINGLE, State.RX_EVENT_PENDING)
        else:
            transmissions.extend(burst(backend, packet, replies, stop))
    raise TimeoutError("production wake sequence incomplete before finite lease")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=CASES, required=True)
    parser.add_argument("--run", required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--fixture", type=Path, required=True)
    parser.add_argument("--group", type=Path, required=True)
    parser.add_argument("--node-id", required=True)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--sleep-seconds", type=int, choices=(10, 900), default=900)
    args = parser.parse_args()
    peer.require(re.fullmatch("[0-9a-f]{32}", args.run), "invalid run")
    peer.require(re.fullmatch("[0-9a-f]{16}", args.node_id), "invalid node ID")
    seal = peer.verify_sources(args.manifest)
    manifest = json.loads(args.manifest.read_text())
    for name in ("ack_peer.py", "ack_cases.py"):
        peer.require(f"receiver/test_apps/radio_peer/{name}" in manifest["files"], "missing ACK peer source")
    fixture = json.loads(args.fixture.read_text())
    peer.require(os.geteuid() != 0 and pwd.getpwuid(os.geteuid()).pw_name == fixture["pi_user"], "wrong peer UID")
    peer.require(fixture["receiver_service_stopped"] is True and fixture["exclusive_radios"] is True
                 and fixture["pi_fixture"] == "radio_nominal" and fixture["rtc_shunts_open"] is True,
                 "unconfirmed nominal peer fixture")
    peer.require(Path("/proc/device-tree/model").read_bytes().startswith(b"Raspberry Pi"), "not a Pi")
    peer.require(Path("/proc/device-tree/serial-number").read_bytes().rstrip(b"\0").decode() == fixture["pi_board_id"], "wrong Pi")
    group = load_receiver_group(args.group)
    node_id = bytes.fromhex(args.node_id)
    keys = authentication_keys(group)
    peer.require(node_id in keys, "node not active in isolated group")
    dependencies = peer.dependencies()
    if args.check:
        print(json.dumps(dict(kind="preflight", source=seal, group_id=group.group_id.hex(),
                              node_id=args.node_id, uid=os.geteuid(), dependencies=dependencies)))
        return 0
    policy = AckCase(args.case, node_id, keys[node_id], args.sleep_seconds)
    return peer.run_session(args, fixture, seal, policy.plan["pi_max_packets"],
        lambda case, backend, radio, stop, started: execute(policy, backend, radio, stop, started),
        lease_seconds=policy.plan["lease_seconds"], raw=args.case.endswith(".invalid_auth"), io_class=AckTraceIo,
        control=(lambda line: policy.observe_sleep(line, args.run, args.case)) if args.sleep_seconds == 10 else None)


if __name__ == "__main__":
    raise SystemExit(main())
