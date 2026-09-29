"""Main-thread stop intent, independent of notification and cleanup."""

from .elapsed_duration import checked_monotonic_deadline


class StopIntent:
    def __init__(self, clock, shutdown_budget_us):
        checked_monotonic_deadline(0, shutdown_budget_us)
        self._clock = clock
        self._budget = shutdown_budget_us
        self._request = None

    def request(self):
        """Record intent only; the clock must be safe in a Python signal handler.

        Requests belong to the main thread. A handler can interrupt another
        request: capture the candidate before reading retained state, then keep
        the earliest pair. A nested later request cannot extend the deadline.
        """
        if self._request is not None:
            return
        now = self._clock.now_monotonic_us()
        candidate = (now, checked_monotonic_deadline(now, self._budget))
        retained = self._request
        if retained is None or now < retained[0]:
            self._request = candidate

    def is_requested(self):
        return self._request is not None

    @property
    def requested_at_monotonic_us(self):
        request = self._request
        return None if request is None else request[0]

    @property
    def deadline_monotonic_us(self):
        request = self._request
        return None if request is None else request[1]
