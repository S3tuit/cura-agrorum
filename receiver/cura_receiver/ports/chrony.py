"""Fixed local chrony operations; process-local results never enter SQLite."""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Protocol

from ..elapsed_duration import checked_monotonic_elapsed
from ..time_diagnostics import integer
from ..time_policy import NetworkEvidence


class ChronyQueryStatus(Enum):
    OK = auto()
    UNAVAILABLE = auto()
    DEADLINE_EXCEEDED = auto()
    INVALID_RESPONSE = auto()


class ChronyStepDisposition(Enum):
    SUBMITTED = auto()
    NOT_SUBMITTED = auto()
    OUTCOME_UNKNOWN = auto()


@dataclass(frozen=True, slots=True)
class ChronyTrackingResult:
    status: ChronyQueryStatus
    sample_started_at_monotonic_us: int
    sample_finished_at_monotonic_us: int
    source_selected: bool = False
    synchronized: bool = False
    remaining_correction_us: int = 0
    root_distance_us: int = 0
    estimated_skew_ppb: int = 0

    def __post_init__(self):
        if type(self.status) is not ChronyQueryStatus:
            raise TypeError("invalid chrony query status")
        checked_monotonic_elapsed(
            self.sample_started_at_monotonic_us, self.sample_finished_at_monotonic_us
        )
        if (
            type(self.source_selected) is not bool
            or type(self.synchronized) is not bool
        ):
            raise TypeError("chrony source flags must be Boolean")
        integer(self.remaining_correction_us, -(1 << 63), (1 << 63) - 1)
        integer(self.root_distance_us)
        integer(self.estimated_skew_ppb)
        if self.synchronized and not self.source_selected:
            raise ValueError("synchronized source must be selected")
        if self.status is not ChronyQueryStatus.OK and any(
            (
                self.source_selected,
                self.synchronized,
                self.remaining_correction_us,
                self.root_distance_us,
                self.estimated_skew_ppb,
            )
        ):
            raise ValueError("failed tracking result carries no normalized evidence")

    def evidence(self):
        if self.status is not ChronyQueryStatus.OK:
            return None
        return NetworkEvidence(
            self.sample_started_at_monotonic_us,
            self.sample_finished_at_monotonic_us,
            self.source_selected,
            self.synchronized,
            self.remaining_correction_us,
            self.root_distance_us,
            self.estimated_skew_ppb,
        )


@dataclass(frozen=True, slots=True)
class ChronyStepResult:
    disposition: ChronyStepDisposition
    operation_started_at_monotonic_us: int
    operation_finished_at_monotonic_us: int

    def __post_init__(self):
        if type(self.disposition) is not ChronyStepDisposition:
            raise TypeError("invalid chrony step disposition")
        checked_monotonic_elapsed(
            self.operation_started_at_monotonic_us,
            self.operation_finished_at_monotonic_us,
        )


class ChronyControl(Protocol):
    def read_tracking(self, *, deadline_monotonic_us: int) -> ChronyTrackingResult: ...
    def apply_pending_correction_by_step(
        self, *, deadline_monotonic_us: int
    ) -> ChronyStepResult: ...
