"""Receipts for RF evidence already retained by the owning capture/parser."""
import re
import threading
import time


class Finalization:
    def __init__(self, select, send, *, clock=time.monotonic):
        self.select, self.send, self.clock = select, send, clock
        self.scope = self.nonce = self.action = None
        self.parked = threading.Event()
        self.failure = None
        self._tail = b""
        self._sent_at = None

    def feed(self, data):
        if self.failure is not None:
            raise self.failure
        try:
            lines = (self._tail + data).split(b"\n")
            self._tail = lines.pop()
            if len(self._tail) > 4096:
                raise ValueError("oversized RF console line")
            for line in lines:
                line = line.rstrip(b"\r")
                if line.startswith(b"RF_FINALIZE_FAILED "):
                    raise RuntimeError("RF finalization failed; image parked until reset")
                if line.startswith(b"RF_PARKED "):
                    if self.action != "STOP" or line != b"RF_PARKED " + self.nonce:
                        raise ValueError("unbound RF park acknowledgement")
                    self.parked.set()
                elif line.startswith(b"RF_FINALIZE "):
                    match = re.fullmatch(rb"RF_FINALIZE ([0-9a-f]{8})", line)
                    if not match:
                        raise ValueError("invalid RF finalization greeting")
                    choice = self.select()
                    if choice is None:
                        continue  # The component parser has not retained END/SLEEP yet.
                    scope, action = choice
                    if action not in {"SLEEP", "STOP"}:
                        raise ValueError("invalid RF finalization action")
                    if scope != self.scope:
                        if self.scope is not None and (self.action != "SLEEP" or scope != self.scope + 1):
                            raise ValueError("unexpected RF finalization progression")
                        self.scope, self.nonce, self.action = scope, match[1], action
                        self._sent_at = None
                    if match[1] != self.nonce or action != self.action:
                        raise ValueError("RF finalization identity/action changed")
                    now = self.clock()
                    if self._sent_at is None or now - self._sent_at >= 0.5:
                        # Retry only upon a fresh greeting. Selectors, resets and
                        # other actions never pass through this replay path.
                        self.send(b"RF_FINALIZE_ACK " + self.nonce + b" " + action.encode() + b"\n")
                        self._sent_at = now
        except BaseException as error:
            self.failure = error
            raise
