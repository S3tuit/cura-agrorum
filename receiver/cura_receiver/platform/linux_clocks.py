"""Linux implementations of the receiver clock capabilities."""

from __future__ import annotations

import time


_NANOSECONDS_PER_MICROSECOND = 1_000
_UINT64_MAX = (1 << 64) - 1
_INT64_MIN = -(1 << 63)
_INT64_MAX = (1 << 63) - 1


def _clock_microseconds(
    clock_id: int,
    *,
    clock_name: str,
    minimum_us: int,
    maximum_us: int,
) -> int:
    raw_ns = time.clock_gettime_ns(clock_id)
    if type(raw_ns) is not int:
        raise TypeError(f"{clock_name} returned a non-integer nanosecond value")

    value_us = raw_ns // _NANOSECONDS_PER_MICROSECOND
    if not minimum_us <= value_us <= maximum_us:
        raise OverflowError(f"{clock_name} microseconds are outside the contract")
    return value_us


class LinuxOsClock:
    """Read the two named POSIX clocks required by the receiver."""

    def now_monotonic_us(self) -> int:
        """Read `CLOCK_MONOTONIC` as canonical unsigned microseconds."""

        return _clock_microseconds(
            time.CLOCK_MONOTONIC,
            clock_name="CLOCK_MONOTONIC",
            minimum_us=0,
            maximum_us=_UINT64_MAX,
        )

    def now_realtime_us(self) -> int:
        """Read `CLOCK_REALTIME` as canonical signed UTC microseconds."""

        return _clock_microseconds(
            time.CLOCK_REALTIME,
            clock_name="CLOCK_REALTIME",
            minimum_us=_INT64_MIN,
            maximum_us=_INT64_MAX,
        )

    def wait_until_monotonic_us(self, deadline_monotonic_us: int) -> None:
        """Polling/reset delay; recheck the absolute bound after an early wake."""
        if type(deadline_monotonic_us) is not int or not 0 <= deadline_monotonic_us <= _UINT64_MAX:
            raise ValueError("invalid monotonic wait deadline")
        while (remaining := deadline_monotonic_us - self.now_monotonic_us()) > 0:
            time.sleep(min(remaining, 1_000_000) / 1_000_000)
