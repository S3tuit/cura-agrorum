"""LP64 libc adjtimex(modes=0); never a system-clock writer."""

import ctypes
import platform
import sys

from ..elapsed_duration import checked_utc_us
from ..ports.clocks import MonotonicClock
from ..ports.kernel_clock import KernelClockResult, KernelSampleStatus as Status
from ..time_diagnostics import integer


class Timeval(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_usec", ctypes.c_long)]


class Timex(ctypes.Structure):
    _fields_ = (
        [("modes", ctypes.c_uint)]
        + [(name, ctypes.c_long) for name in ("offset", "freq", "maxerror", "esterror")]
        + [("status", ctypes.c_int)]
        + [(name, ctypes.c_long) for name in ("constant", "precision", "tolerance")]
        + [("time", Timeval)]
        + [(name, ctypes.c_long) for name in ("tick", "ppsfreq", "jitter")]
        + [("shift", ctypes.c_int)]
        + [
            (name, ctypes.c_long)
            for name in ("stabil", "jitcnt", "calcnt", "errcnt", "stbcnt")
        ]
        + [("tai", ctypes.c_int), ("reserved", ctypes.c_int * 11)]
    )


def validate_native_abi():
    if (
        sys.platform != "linux"
        or sys.byteorder != "little"
        or platform.machine() not in ("x86_64", "aarch64")
        or ctypes.sizeof(ctypes.c_long) != 8
        or ctypes.sizeof(Timex) != 208
        or Timex.time.offset != 72
        or Timex.status.offset != 40
    ):
        raise ValueError("unsupported native time ABI")


def kernel_metadata_usable(result, status_bits):
    if status_bits & ~(0x40 | 0x2000):
        return False
    return result == (5 if status_bits & 0x40 else 0)


class LinuxKernelClock:
    def __init__(self, clock: MonotonicClock):
        validate_native_abi()
        self.clock = clock
        self._libc = ctypes.CDLL(None, use_errno=True)
        self._adjtimex = self._libc.adjtimex
        self._adjtimex.argtypes = [ctypes.POINTER(Timex)]
        self._adjtimex.restype = ctypes.c_int

    def sample(self, *, deadline_monotonic_us):
        integer(deadline_monotonic_us)
        start = self.clock.now_monotonic_us()
        if start >= deadline_monotonic_us:
            return KernelClockResult(Status.DEADLINE_EXCEEDED, start, start)
        tx = Timex()
        result = self._adjtimex(ctypes.byref(tx))
        os_errno = ctypes.get_errno() if result < 0 else None
        finish = self.clock.now_monotonic_us()
        metadata = dict(
            adjtimex_return=result if 0 <= result <= 5 else None,
            kernel_status_bits=tx.status if 0 <= tx.status <= 0xFFFFFFFF else None,
            os_errno=os_errno or None,
        )
        status, utc = Status.OK, None
        if finish >= deadline_monotonic_us:
            status = Status.DEADLINE_EXCEEDED
        elif result < 0:
            status = Status.IO_ERROR
        elif result > 5 or tx.status < 0 or tx.modes != 0:
            status = Status.INVALID_RESPONSE
        elif not kernel_metadata_usable(result, tx.status):
            status = Status.CLOCK_INTERFERENCE
        else:
            scale = 1000 if tx.status & 0x2000 else 1
            if not 0 <= tx.time.tv_usec < 1_000_000 * scale:
                status = Status.INVALID_RESPONSE
            else:
                try:
                    utc = checked_utc_us(
                        tx.time.tv_sec * 1_000_000 + tx.time.tv_usec // scale
                    )
                except OverflowError:
                    status = Status.INVALID_RESPONSE
        return KernelClockResult(status, start, finish, utc, **metadata)
