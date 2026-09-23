from __future__ import annotations

from collections.abc import Callable

import pytest

from cura_receiver.platform import linux_clocks
from cura_receiver.platform.linux_clocks import LinuxOsClock
from cura_receiver.ports.clocks import MonotonicClock, RealtimeClock


UINT64_MAX = (1 << 64) - 1
INT64_MIN = -(1 << 63)
INT64_MAX = (1 << 63) - 1


# Uses the named POSIX clocks and discards only sub-microsecond precision.
def test_linux_clock_selects_exact_sources_and_converts_to_microseconds(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    reads = {
        linux_clocks.time.CLOCK_MONOTONIC: 1_234_567,
        linux_clocks.time.CLOCK_REALTIME: -1,
    }
    observed_clock_ids: list[int] = []

    def read_clock(clock_id: int) -> int:
        observed_clock_ids.append(clock_id)
        return reads[clock_id]

    monkeypatch.setattr(linux_clocks.time, "clock_gettime_ns", read_clock)
    clock = LinuxOsClock()

    assert clock.now_monotonic_us() == 1_234
    assert clock.now_realtime_us() == -1
    assert observed_clock_ids == [
        linux_clocks.time.CLOCK_MONOTONIC,
        linux_clocks.time.CLOCK_REALTIME,
    ]


# Exposes separate monotonic and realtime capabilities from one Linux adapter.
def test_linux_clock_implements_both_narrow_clock_protocols() -> None:
    clock = LinuxOsClock()

    assert isinstance(clock, MonotonicClock)
    assert isinstance(clock, RealtimeClock)


# Accepts exact scalar endpoints after integer-microsecond conversion.
@pytest.mark.parametrize(
    ("method_name", "raw_ns", "expected_us"),
    (
        ("now_monotonic_us", 0, 0),
        ("now_monotonic_us", UINT64_MAX * 1_000 + 999, UINT64_MAX),
        ("now_realtime_us", INT64_MIN * 1_000, INT64_MIN),
        ("now_realtime_us", INT64_MAX * 1_000 + 999, INT64_MAX),
    ),
)
def test_linux_clock_accepts_exact_scalar_boundaries(
    monkeypatch: pytest.MonkeyPatch,
    method_name: str,
    raw_ns: int,
    expected_us: int,
) -> None:
    monkeypatch.setattr(
        linux_clocks.time,
        "clock_gettime_ns",
        lambda _clock_id: raw_ns,
    )

    method = getattr(LinuxOsClock(), method_name)
    assert method() == expected_us


# Rejects values one microsecond outside each clock's scalar contract.
@pytest.mark.parametrize(
    ("method_name", "raw_ns"),
    (
        ("now_monotonic_us", -1),
        ("now_monotonic_us", (UINT64_MAX + 1) * 1_000),
        ("now_realtime_us", (INT64_MIN - 1) * 1_000),
        ("now_realtime_us", (INT64_MAX + 1) * 1_000),
    ),
)
def test_linux_clock_rejects_values_outside_scalar_boundaries(
    monkeypatch: pytest.MonkeyPatch,
    method_name: str,
    raw_ns: int,
) -> None:
    monkeypatch.setattr(
        linux_clocks.time,
        "clock_gettime_ns",
        lambda _clock_id: raw_ns,
    )

    method: Callable[[], int] = getattr(LinuxOsClock(), method_name)
    with pytest.raises(OverflowError, match="outside the contract"):
        method()


# Rejects a non-integer OS result rather than silently coercing it.
def test_linux_clock_rejects_non_integer_os_values(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        linux_clocks.time,
        "clock_gettime_ns",
        lambda _clock_id: 1.5,
    )

    with pytest.raises(TypeError, match="non-integer"):
        LinuxOsClock().now_monotonic_us()
