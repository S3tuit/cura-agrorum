import ctypes

import pytest

from cura_receiver.platform.linux_kernel_clock import (
    LinuxKernelClock,
    Timex,
    kernel_metadata_usable,
)
from cura_receiver.ports.kernel_clock import KernelSampleStatus as S
from tests.support.fakes.os_clock import FakeOsClock


def adapter(*, bits=0x2040, result=5, seconds=123, fraction=123456789, elapsed=10):
    clock = FakeOsClock(monotonic_us=100)
    kernel = LinuxKernelClock(clock)

    def syscall(pointer):
        tx = ctypes.cast(pointer, ctypes.POINTER(Timex)).contents
        assert tx.modes == 0
        tx.status, tx.time.tv_sec, tx.time.tv_usec = bits, seconds, fraction
        clock.advance_elapsed_us(elapsed)
        ctypes.set_errno(5)
        return result

    kernel._adjtimex = syscall
    return kernel, clock


# The native struct's LP64 layout matches Linux's time, metadata and reserved-field offsets.
def test_native_layout():
    assert ctypes.sizeof(Timex) == 208
    assert (
        Timex.status.offset == 40
        and Timex.time.offset == 72
        and Timex.tai.offset == 160
    )


# Both documented kernel states and resolutions yield exact bounded UTC, including negative epochs.
@pytest.mark.parametrize(
    "bits,result,fraction,seconds,expected",
    [
        (0x2040, 5, 123456789, 123, 123123456),
        (0x40, 5, 123456, 123, 123123456),
        (0x2000, 0, 999999999, -1, -1),
        (0, 0, 0, 0, 0),
    ],
)
def test_readonly_sampling(bits, result, fraction, seconds, expected):
    kernel, _ = adapter(bits=bits, result=result, fraction=fraction, seconds=seconds)
    value = kernel.sample(deadline_monotonic_us=200)
    assert value.status is S.OK and value.sampled_utc_us == expected
    assert (
        value.operation_started_at_monotonic_us,
        value.operation_finished_at_monotonic_us,
    ) == (100, 110)


# Every non-whitelisted status bit and every unexpected return-state pair suppresses trust.
def test_kernel_interference_matrix():
    for bit in range(32):
        assert kernel_metadata_usable(5, 0x2040 | (1 << bit)) == (bit in (6, 13))
    for result in range(6):
        assert kernel_metadata_usable(result, 0x2040) == (result == 5)
        assert kernel_metadata_usable(result, 0) == (result == 0)


# Entry expiry avoids the syscall; late actual returns discard UTC and retain the measured bracket.
def test_deadlines():
    kernel, clock = adapter(elapsed=100)
    assert kernel.sample(deadline_monotonic_us=100).status is S.DEADLINE_EXCEEDED
    assert clock.now_monotonic_us() == 100
    value = kernel.sample(deadline_monotonic_us=200)
    assert value.status is S.DEADLINE_EXCEEDED and value.sampled_utc_us is None
    assert value.operation_finished_at_monotonic_us == 200


# Syscall failures, malformed values and interference retain their distinct operational outcomes.
@pytest.mark.parametrize(
    "changes,expected",
    [
        ({"result": -1}, S.IO_ERROR),
        ({"result": 6}, S.INVALID_RESPONSE),
        ({"fraction": -1}, S.INVALID_RESPONSE),
        ({"fraction": 1_000_000_000}, S.INVALID_RESPONSE),
        ({"seconds": 1 << 62}, S.INVALID_RESPONSE),
        ({"bits": 0x2041}, S.CLOCK_INTERFERENCE),
    ],
)
def test_failed_samples(changes, expected):
    kernel, _ = adapter(**changes)
    value = kernel.sample(deadline_monotonic_us=200)
    assert value.status is expected and value.sampled_utc_us is None
