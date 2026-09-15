"""One read-only system UTC and kernel-metadata observation."""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Protocol

from ..elapsed_duration import checked_monotonic_elapsed
from ..time_diagnostics import integer


class KernelSampleStatus(Enum):
    OK = auto()
    IO_ERROR = auto()
    DEADLINE_EXCEEDED = auto()
    INVALID_RESPONSE = auto()
    CLOCK_INTERFERENCE = auto()


@dataclass(frozen=True, slots=True)
class KernelClockResult:
    status: KernelSampleStatus
    operation_started_at_monotonic_us: int
    operation_finished_at_monotonic_us: int
    sampled_utc_us: int | None = None
    adjtimex_return: int | None = None
    kernel_status_bits: int | None = None
    os_errno: int | None = None

    def __post_init__(self):
        if type(self.status) is not KernelSampleStatus:
            raise TypeError("invalid kernel result status")
        checked_monotonic_elapsed(
            self.operation_started_at_monotonic_us,
            self.operation_finished_at_monotonic_us,
        )
        if (self.sampled_utc_us is not None) != (self.status is KernelSampleStatus.OK):
            raise ValueError("UTC is present exactly on OK")
        if self.sampled_utc_us is not None:
            integer(self.sampled_utc_us, -(1 << 63), (1 << 63) - 1)
        if self.adjtimex_return is not None:
            integer(self.adjtimex_return, 0, 5)
        if self.kernel_status_bits is not None:
            integer(self.kernel_status_bits, 0, (1 << 32) - 1)
        if self.os_errno is not None:
            integer(self.os_errno, 1, (1 << 31) - 1)
            if self.status is KernelSampleStatus.OK:
                raise ValueError("successful kernel read has no errno")


class KernelClock(Protocol):
    def sample(self, *, deadline_monotonic_us: int) -> KernelClockResult: ...
