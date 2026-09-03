from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor

import pytest

from cura_receiver.ports.clocks import MonotonicClock, RealtimeClock
from tests.support.fakes.os_clock import FakeOsClock


UINT64_MAX = (1 << 64) - 1
INT64_MIN = -(1 << 63)
INT64_MAX = (1 << 63) - 1


# Leaves both clock values unchanged no matter how often they are read.
def test_fake_clock_reads_never_advance_automatically() -> None:
    clock = FakeOsClock(monotonic_us=12, realtime_us=34)

    assert [clock.now_monotonic_us() for _ in range(3)] == [12, 12, 12]
    assert [clock.now_realtime_us() for _ in range(3)] == [34, 34, 34]


# Advances both clocks together only when elapsed time is explicitly requested.
def test_fake_clock_advances_elapsed_time_explicitly() -> None:
    clock = FakeOsClock(monotonic_us=100, realtime_us=1_000)

    clock.advance_elapsed_us(250)

    assert clock.now_monotonic_us() == 350
    assert clock.now_realtime_us() == 1_250


# Steps realtime in either direction without changing monotonic deadlines.
def test_fake_clock_steps_realtime_independently() -> None:
    clock = FakeOsClock(monotonic_us=100, realtime_us=1_000)

    clock.step_realtime_us(500)
    assert clock.now_monotonic_us() == 100
    assert clock.now_realtime_us() == 1_500

    clock.step_realtime_us(-2_000)
    assert clock.now_monotonic_us() == 100
    assert clock.now_realtime_us() == -500


# Prevents explicit elapsed advancement from moving monotonic time backward.
def test_fake_clock_rejects_negative_elapsed_advancement() -> None:
    clock = FakeOsClock(monotonic_us=100, realtime_us=1_000)

    with pytest.raises(ValueError, match="backward"):
        clock.advance_elapsed_us(-1)

    assert clock.now_monotonic_us() == 100
    assert clock.now_realtime_us() == 1_000


# Accepts the exact monotonic and realtime scalar endpoints.
@pytest.mark.parametrize(
    ("monotonic_us", "realtime_us"),
    (
        (0, INT64_MIN),
        (UINT64_MAX, INT64_MAX),
    ),
)
def test_fake_clock_accepts_exact_scalar_boundaries(
    monotonic_us: int,
    realtime_us: int,
) -> None:
    clock = FakeOsClock(monotonic_us=monotonic_us, realtime_us=realtime_us)

    assert clock.now_monotonic_us() == monotonic_us
    assert clock.now_realtime_us() == realtime_us


# Rejects initial values outside either clock's scalar contract.
@pytest.mark.parametrize(
    ("monotonic_us", "realtime_us"),
    (
        (-1, 0),
        (UINT64_MAX + 1, 0),
        (0, INT64_MIN - 1),
        (0, INT64_MAX + 1),
    ),
)
def test_fake_clock_rejects_initial_values_outside_scalar_boundaries(
    monotonic_us: int,
    realtime_us: int,
) -> None:
    with pytest.raises(OverflowError, match="clock scalar range"):
        FakeOsClock(monotonic_us=monotonic_us, realtime_us=realtime_us)


# Rejects bools and non-integers instead of accepting Python's implicit coercions.
@pytest.mark.parametrize("invalid", (True, 1.5))
def test_fake_clock_rejects_non_integer_inputs(invalid: object) -> None:
    with pytest.raises(TypeError, match="must be an integer"):
        FakeOsClock(monotonic_us=invalid)  # type: ignore[arg-type]

    clock = FakeOsClock()
    with pytest.raises(TypeError, match="must be an integer"):
        clock.advance_elapsed_us(invalid)  # type: ignore[arg-type]
    with pytest.raises(TypeError, match="must be an integer"):
        clock.step_realtime_us(invalid)  # type: ignore[arg-type]


# Leaves both values unchanged when an elapsed advance would overflow either one.
def test_fake_clock_elapsed_overflow_is_atomic() -> None:
    clock = FakeOsClock(monotonic_us=10, realtime_us=INT64_MAX)

    with pytest.raises(OverflowError, match="realtime_us"):
        clock.advance_elapsed_us(1)

    assert clock.now_monotonic_us() == 10
    assert clock.now_realtime_us() == INT64_MAX


# Serializes concurrent advances so no elapsed update is lost.
def test_fake_clock_serializes_concurrent_advances() -> None:
    clock = FakeOsClock()

    with ThreadPoolExecutor(max_workers=8) as executor:
        list(executor.map(clock.advance_elapsed_us, [1] * 1_000))

    assert clock.now_monotonic_us() == 1_000
    assert clock.now_realtime_us() == 1_000


# Satisfies both production clock capabilities structurally.
def test_fake_clock_implements_both_narrow_clock_protocols() -> None:
    clock = FakeOsClock()

    assert isinstance(clock, MonotonicClock)
    assert isinstance(clock, RealtimeClock)
