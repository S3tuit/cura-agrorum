"""Recorded calls and scripted results for the existing ChronyControl port."""

from collections import deque


class FakeChronyControl:
    def __init__(self):
        self.tracking_results = deque()
        self.step_results = deque()
        self.calls = []

    def read_tracking(self, *, deadline_monotonic_us):
        self.calls.append(("tracking", deadline_monotonic_us))
        result = self.tracking_results.popleft()
        return result() if callable(result) else result

    def apply_pending_correction_by_step(self, *, deadline_monotonic_us):
        self.calls.append(("step", deadline_monotonic_us))
        result = self.step_results.popleft()
        return result() if callable(result) else result
