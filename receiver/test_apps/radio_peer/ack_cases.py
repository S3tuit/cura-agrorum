"""RF-019's finite controlled-peer cases; no hardware or production policy edits."""
from dataclasses import asdict

from cura_receiver.generated import protocol_v2_lora_generated as wire
from cura_receiver.protocol_v2_lora_crypto import open_frame, seal_frame

STATUSES = {"accepted": 0, "retry_later": 1, "unsupported": 2, "malformed": 3}
CASES = tuple(f"RF-019.{scope}.{status}" for scope in ("current", "backlog")
              for status in STATUSES) + (
    "RF-019.current.invalid_auth", "RF-019.current.wrong_message",
    "RF-019.current.domain_status",
)


def episode(case, sleep_seconds=900):
    if sleep_seconds not in (10, 900):
        raise ValueError("unreviewed RF sleep duration")
    if case not in CASES:
        raise ValueError("unknown RF-019 case")
    scope, action = case.split(".")[1:]
    seeds = 2 if scope == "backlog" else 1
    wakes = seeds + 2  # Setup, target, then next-wake metrics observation.
    backlogs = (2 if action != "retry_later" else 1) if scope == "backlog" else (
        1 if action in ("accepted", "invalid_auth") else 0)
    replies = seeds + 1 + (2 if action == "invalid_auth" else 1) + backlogs
    return dict(case=case, scope=scope, action=action, seed_wakes=seeds,
                wakes=wakes, sleep_seconds=sleep_seconds,
                lease_seconds=(wakes - 1) * 945 + 60 if sleep_seconds == 900 else wakes * 50 + 10,
                c6_max_packets=70 * wakes, pi_max_packets=replies)


class AckCase:
    """One disposable-storage episode, preserving identity/counters across wakes.

    Setup wakes receive RETRY_LATER, creating real pending readings. The target
    wake tests the selected outcome. The observation wake receives RETRY_LATER
    to preserve remaining pending evidence while transmitting previous metrics.
    """
    def __init__(self, case, node_id, key, sleep_seconds=900):
        import threading
        self.sleep_entered = threading.Event()
        self.plan = episode(case, sleep_seconds)
        self.node_id, self.key = node_id, key
        self.currents = []
        self.frames = {}
        self.replies = {}
        self.attempts = {}
        self.packet_count = 0
        self.backlog_seen = []
        self.observation_started_at = None
        self.first_received_at = None

    @property
    def target_index(self):
        return self.plan["seed_wakes"]

    def ack(self, message_id, status, *, domain=None):
        header = wire.ClearHeader(0x20, status + 3 if domain is None else domain,
                                  self.node_id, message_id)
        return seal_frame(self.key, header, bytes([status]))

    def receive(self, frame, received_at):
        if self.packet_count >= self.plan["c6_max_packets"]:
            raise ValueError("uplink episode ceiling exceeded")
        opened = open_frame(self.key, frame)
        h = opened.header
        if len(frame) != 54 or h.control != 0x20 or h.node_id != self.node_id or h.domain not in (1, 2):
            raise ValueError("unexpected authenticated uplink")
        body = wire.decode_reading(opened.plaintext_body)
        previous = self.frames.get(h.message_id)
        if previous is not None and previous != frame:
            raise ValueError("message ID reused for changed frame")
        new_message = previous is None
        if new_message and self.frames and h.message_id <= max(self.frames):
            raise ValueError("new message counter did not advance")
        if h.domain == 1 and new_message:
            if self.currents:
                last = self.currents[-1]
                if body.sample_id != last["body"]["sample_id"] + 1:
                    raise ValueError("unexpected reset/missing sample between scheduled wakes")
                if body.reset_reason != 8 or not body.flags & 1 or not body.flags & 256:
                    raise ValueError("missing genuine deep-sleep wake/previous metrics")
                # Accelerated intervals include the unchanged 30s retry budget
                # and acquisition/finalization overhead; not a cadence proof.
                lower, upper = ((855_000_000, 945_000_000) if self.plan["sleep_seconds"] == 900
                                else (9_500_000, 45_500_000))
                if not lower <= received_at - last["at"] <= upper:
                    raise ValueError("scheduled wake outside observation bound")
            if len(self.currents) >= self.plan["wakes"]:
                raise ValueError("extra autonomous wake")
            self.currents.append(dict(message_id=h.message_id, body=asdict(body),
                                      bytes=opened.plaintext_body.hex(), at=received_at))
        if not self.currents:
            raise ValueError("backlog before current")
        index = len(self.currents) - 1
        current = self.currents[-1]
        if h.domain == 1 and h.message_id != current["message_id"]:
            raise ValueError("retry of an abandoned current message")
        if h.domain == 2:
            if index != self.target_index:
                raise ValueError("backlog after a wake-stopping ACK")
            if self.plan["scope"] == "current" and self.plan["action"] not in ("accepted", "invalid_auth"):
                raise ValueError("backlog after rejected/unaccepted current")
            if self.plan["scope"] == "backlog" and self.plan["action"] == "retry_later" and self.backlog_seen:
                raise ValueError("backlog continued after RETRY_LATER")
            seeds = self.currents[:self.target_index]
            seed = next((s for s in seeds if s["body"]["sample_id"] == body.sample_id), None)
            if seed is None or seed["bytes"] != opened.plaintext_body.hex():
                raise ValueError("backlog body differs from actual setup reading")
            if new_message:
                expected = [s["body"]["sample_id"] for s in reversed(seeds)]
                if (len(self.backlog_seen) >= len(expected) or
                        self.backlog_seen != expected[:len(self.backlog_seen)] or
                        body.sample_id != expected[len(self.backlog_seen)]):
                    raise ValueError("backlog not newest first")
                self.backlog_seen.append(body.sample_id)
        self.frames[h.message_id] = frame
        self.attempts[h.message_id] = self.attempts.get(h.message_id, 0) + 1
        self.packet_count += 1
        if self.first_received_at is None:
            self.first_received_at = received_at
        if index == self.target_index + 1:
            self.observation_started_at = current["at"]
        if h.message_id in self.replies:
            if self.replies[h.message_id]["action"] not in ("wrong_message", "domain_status"):
                raise ValueError("retry after terminal ACK; stop, no automatic resend")
            return [], self.observation()
        if index != self.target_index:
            action = "retry_later"
        elif self.plan["scope"] == "current":
            action = self.plan["action"] if h.domain == 1 else "accepted"
        else:
            action = self.plan["action"] if h.domain == 2 and len(self.backlog_seen) == 1 else "accepted"
        if action in STATUSES:
            replies = [self.ack(h.message_id, STATUSES[action])]
        elif action == "invalid_auth":
            valid = self.ack(h.message_id, 0)
            replies = [valid[:-1] + bytes([valid[-1] ^ 1]), valid]
        elif action == "wrong_message":
            if h.message_id == 0xffffffff:
                raise ValueError("no reserved wrong-message test value")
            replies = [self.ack(0xffffffff, 0)]
        else:
            # Domain 4 with status 0: authenticated but semantically invalid.
            # Never later encrypt a different body for this nonce.
            replies = [self.ack(h.message_id, 0, domain=4)]
        self.replies[h.message_id] = dict(action=action, frames=[r.hex() for r in replies])
        return replies, self.observation()

    def observation(self):
        return dict(currents=self.currents, attempts=self.attempts,
                    replies=self.replies, backlog_seen=self.backlog_seen,
                    packet_count=self.packet_count)

    def complete(self, now):
        if self.plan["sleep_seconds"] == 10:
            return self.observation_started_at is not None and self.sleep_entered.is_set()
        # Observe the rest of the final wake, including any forbidden retry or
        # backlog. Do not infer node shutdown from radio silence.
        return self.observation_started_at is not None and now >= self.observation_started_at + 35_000_000

    def observe_sleep(self, line, run, case):
        if (self.plan["sleep_seconds"] != 10 or self.sleep_entered.is_set() or
                line != f"SLEEP {run} {case} {self.plan['wakes']}\n" or
                len(self.currents) != self.plan["wakes"]):
            raise ValueError("invalid or premature final sleep notification")
        self.sleep_entered.set()
