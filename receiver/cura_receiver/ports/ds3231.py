"""RTC device effects and failure reasons are independent runtime values."""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Protocol

from ..elapsed_duration import checked_monotonic_elapsed
from ..time_diagnostics import integer

RTC_MIN_UTC_S = 946684800
RTC_MAX_UTC_S = 4102444799


class Ds3231ReadStatus(Enum):
    OK = auto()
    MISSING = auto()
    INVALID = auto()
    IO_ERROR = auto()
    DEADLINE_EXCEEDED = auto()


class Ds3231WriteDisposition(Enum):
    COMPLETED = auto()
    NOT_APPLIED = auto()
    OUTCOME_UNKNOWN = auto()


class Ds3231Failure(Enum):
    NONE = auto()
    MISSING = auto()
    IO_ERROR = auto()
    DEADLINE_EXCEEDED = auto()


@dataclass(frozen=True, slots=True)
class Ds3231ReadResult:
    status: Ds3231ReadStatus
    operation_started_at_monotonic_us: int
    operation_finished_at_monotonic_us: int
    rtc_utc_s: int | None = None
    os_errno: int | None = None

    def __post_init__(self):
        if type(self.status) is not Ds3231ReadStatus:
            raise TypeError("invalid RTC read status")
        checked_monotonic_elapsed(
            self.operation_started_at_monotonic_us,
            self.operation_finished_at_monotonic_us,
        )
        if (self.rtc_utc_s is not None) != (self.status is Ds3231ReadStatus.OK):
            raise ValueError("RTC seconds present exactly on OK")
        if self.rtc_utc_s is not None:
            integer(self.rtc_utc_s, RTC_MIN_UTC_S, RTC_MAX_UTC_S)
        _errno(self.os_errno, self.status is Ds3231ReadStatus.OK)


def _errno(value, success):
    if value is not None:
        integer(value, 1, (1 << 31) - 1)
        if success:
            raise ValueError("successful RTC operation has no errno")


@dataclass(frozen=True, slots=True)
class Ds3231WriteResult:
    disposition: Ds3231WriteDisposition
    failure: Ds3231Failure
    operation_started_at_monotonic_us: int
    operation_finished_at_monotonic_us: int
    os_errno: int | None = None

    def __post_init__(self):
        if (
            type(self.disposition) is not Ds3231WriteDisposition
            or type(self.failure) is not Ds3231Failure
        ):
            raise TypeError("invalid RTC write result enum")
        checked_monotonic_elapsed(
            self.operation_started_at_monotonic_us,
            self.operation_finished_at_monotonic_us,
        )
        completed = self.disposition is Ds3231WriteDisposition.COMPLETED
        if completed != (self.failure is Ds3231Failure.NONE):
            raise ValueError("write disposition and failure disagree")
        _errno(self.os_errno, completed)


class Ds3231Control(Protocol):
    def read_time(self, *, deadline_monotonic_us: int) -> Ds3231ReadResult: ...
    def write_time(
        self, *, rtc_utc_s: int, deadline_monotonic_us: int
    ) -> Ds3231WriteResult: ...
