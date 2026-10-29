"""Commit coverage and its timer, owned solely by the persistence thread."""

from .elapsed_duration import checked_monotonic_deadline


class CheckpointMaintenance:
    def __init__(self, *, clock, interval_us=5_000_000):
        if type(interval_us) is not int or interval_us < 1:
            raise ValueError("checkpoint interval must be a positive integer")
        self._clock = clock
        self.interval_us = interval_us
        self.pending = False
        self.deadline_monotonic_us = None

    def mark_possible_work(self):
        """First possible COMMIT or unknown connection coverage arms the timer."""
        if not self.pending:
            deadline = checked_monotonic_deadline(
                self._clock.now_monotonic_us(), self.interval_us
            )
            self.pending = True
            self.deadline_monotonic_us = deadline

    def due(self, now):
        return self.pending and now >= self.deadline_monotonic_us

    def complete(self):
        self.pending = False
        self.deadline_monotonic_us = None

    def partial(self):
        self.pending = True
        self.deadline_monotonic_us = checked_monotonic_deadline(
            self._clock.now_monotonic_us(), self.interval_us
        )
