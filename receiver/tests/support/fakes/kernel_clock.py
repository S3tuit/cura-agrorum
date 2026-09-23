"""Scripted results for the production KernelClock port, without time policy."""

from collections import deque


class FakeKernelClock:
    def __init__(self):
        self.results = deque()
        self.deadlines = []

    def sample(self, *, deadline_monotonic_us):
        self.deadlines.append(deadline_monotonic_us)
        result = self.results.popleft()
        return result() if callable(result) else result
