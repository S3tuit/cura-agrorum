"""Scripted calls at the production RTC port, with no device IO or trust policy."""

from collections import deque


class FakeDs3231Control:
    def __init__(self):
        self.read_results = deque()
        self.write_results = deque()
        self.calls = []

    def read_time(self, *, deadline_monotonic_us):
        self.calls.append(("read", deadline_monotonic_us))
        value = self.read_results.popleft()
        return value() if callable(value) else value

    def write_time(self, *, rtc_utc_s, deadline_monotonic_us):
        self.calls.append(("write", rtc_utc_s, deadline_monotonic_us))
        value = self.write_results.popleft()
        return value() if callable(value) else value
