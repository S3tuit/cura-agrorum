"""Production SX1262 SPI/GPIO boundary; no protocol, queue or airtime policy."""

from dataclasses import dataclass
from typing import Protocol

from ..generated.receiver_enums_generated import (
    RadioCommandOutcome as Outcome,
    RadioDiagnosticErrorCode as Error,
    RadioFailureStage as Stage,
)


def integer(value, minimum=0, maximum=(1 << 64) - 1):
    if type(value) is not int:
        raise TypeError("radio values require exact integers")
    if not minimum <= value <= maximum:
        raise ValueError("radio value outside its interface range")
    return value


@dataclass(frozen=True, slots=True)
class RadioConfiguration:
    spi_device: str = "/dev/spidev0.0"
    gpio_chip: str = "/dev/gpiochip0"
    reset_line: int = 22
    dio1_line: int = 23
    busy_line: int = 24
    spi_speed_hz: int = 1_000_000

    def __post_init__(self):
        for path in (self.spi_device, self.gpio_chip):
            if type(path) is not str or not path.startswith("/") or "\0" in path:
                raise ValueError("radio device path must be absolute")
        lines = (self.reset_line, self.dio1_line, self.busy_line)
        for line in lines:
            integer(line, 0, (1 << 32) - 1)
        if len(set(lines)) != 3:
            raise ValueError("radio GPIO lines must be distinct")
        if self.spi_speed_hz != 1_000_000 or type(self.spi_speed_hz) is not int:
            raise ValueError("pilot radio SPI speed is 1 MHz")


@dataclass(frozen=True, slots=True)
class RadioFailure:
    code: Error
    stage: Stage
    outcome: Outcome = Outcome.NOT_APPLICABLE
    opcode: int = 0
    os_errno: int | None = None
    chip_status: int | None = None
    irq_status: int | None = None
    device_errors: int | None = None
    hardware_touched: bool = False
    hardware_missing: bool = False

    def __post_init__(self):
        for value, enum in ((self.code, Error), (self.stage, Stage), (self.outcome, Outcome)):
            if type(value) is not enum:
                raise TypeError("undefined radio failure enum")
        if self.code is Error.NONE or self.stage is Stage.NONE:
            raise ValueError("failure requires a code and primitive stage")
        integer(self.opcode, 0, 255)
        if self.os_errno is not None:
            integer(self.os_errno, 1, (1 << 31) - 1)
        for value, maximum in (
            (self.chip_status, 255), (self.irq_status, 65535), (self.device_errors, 65535)
        ):
            if value is not None:
                integer(value, 0, maximum)
        if type(self.hardware_touched) is not bool or type(self.hardware_missing) is not bool:
            raise TypeError("hardware facts require Boolean values")
        if self.hardware_missing and self.code is not Error.IO:
            raise ValueError("unavailable hardware uses the IO trigger")


class RadioBackendError(Exception):
    """An expected normalized failure, separate from implementation exceptions."""

    def __init__(self, failure: RadioFailure):
        if type(failure) is not RadioFailure:
            raise TypeError("expected immutable radio failure evidence")
        self.failure = failure
        super().__init__(failure.code.name)


@dataclass(frozen=True, slots=True)
class RadioLifecycleFailure:
    primary_failure: RadioFailure | None
    release_failures: tuple[RadioFailure, ...]

    def __post_init__(self):
        if self.primary_failure is not None and type(self.primary_failure) is not RadioFailure:
            raise TypeError("invalid acquisition failure")
        if type(self.release_failures) is not tuple or any(
            type(failure) is not RadioFailure for failure in self.release_failures
        ):
            raise TypeError("release failures require an immutable tuple")
        if len(self.release_failures) > 2 or (self.primary_failure is None and not self.release_failures):
            raise ValueError("invalid resource lifecycle failure count")


class RadioLifecycleError(RadioBackendError):
    """Complete lifecycle evidence; failure is only the primary projection."""

    def __init__(self, lifecycle: RadioLifecycleFailure):
        if type(lifecycle) is not RadioLifecycleFailure:
            raise TypeError("expected immutable lifecycle failure evidence")
        self.lifecycle = lifecycle
        super().__init__(lifecycle.primary_failure or lifecycle.release_failures[0])


@dataclass(frozen=True, slots=True)
class Dio1Edge:
    timestamp_ns: int
    sequence: int

    def __post_init__(self):
        integer(self.timestamp_ns)
        integer(self.sequence, 1)

    @property
    def monotonic_us(self):
        return self.timestamp_ns // 1000


@dataclass(frozen=True, slots=True)
class BusyMetrics:
    total_us: int = 0
    maximum_us: int = 0
    count: int = 0
    timeout_count: int = 0
    last_timeout_opcode: int | None = None

    def __post_init__(self):
        integer(self.total_us)
        integer(self.maximum_us, 0, self.total_us)
        integer(self.count, 0, (1 << 32) - 1)
        integer(self.timeout_count, 0, self.count)
        if (self.last_timeout_opcode is None) != (self.timeout_count == 0):
            raise ValueError("BUSY timeout opcode presence disagrees with count")
        if self.last_timeout_opcode is not None:
            integer(self.last_timeout_opcode, 0, 255)


@dataclass(frozen=True, slots=True)
class RadioTxAuthorization:
    """Caller-owned allowance has been consumed before TX-profile installation.

    This is a submission deadline and diagnostic correlation, not a grant or
    proof of durable admission. The caller owns grant validation/settlement.
    """

    submission_deadline_monotonic_us: int
    occurrence_sequence: int | None = None
    airtime_bucket_expiration_utc_us: int | None = None

    def __post_init__(self):
        integer(self.submission_deadline_monotonic_us)
        if self.occurrence_sequence is not None:
            integer(self.occurrence_sequence, 1)
        if self.airtime_bucket_expiration_utc_us is not None:
            integer(self.airtime_bucket_expiration_utc_us, -(1 << 63), (1 << 63) - 1)


@dataclass(frozen=True, slots=True)
class RadioTxFacts:
    """Facts used by the caller's existing conservative airtime policy."""

    set_tx_outcome: Outcome = Outcome.NOT_APPLICABLE
    profile_uncertain: bool = False

    def __post_init__(self):
        if type(self.set_tx_outcome) is not Outcome or type(self.profile_uncertain) is not bool:
            raise TypeError("invalid TX command facts")


class RadioWait(Protocol):
    def wait_until_monotonic_us(self, deadline_monotonic_us: int) -> None:
        """Wait until the specified monotonic point; never supplies radio policy."""
        ...


class RadioIo(Protocol):
    """One owner's physical effects. Expected failures raise RadioBackendError.

    Each SPI transfer keeps NSS asserted for the complete byte string. No
    primitive retries an uncertain transfer. Queued DIO1 edges preserve kernel
    nanoseconds and line sequence numbers; timestamp ordering/loss is checked
    by the adapter. wait_edge returns an already queued edge even if its time
    is beyond the supplied bound, so the radio owner can classify it exactly.
    Failed open releases acquired resources before raising RadioLifecycleError;
    close reports all release failures in the same immutable lifecycle value.
    Resource close attempts all acquired resources once and is idempotent.
    """

    def open(self, configuration: RadioConfiguration, *, deadline_monotonic_us: int) -> None: ...
    def transfer(self, data: bytes, *, deadline_monotonic_us: int) -> bytes: ...
    def busy(self) -> bool: ...
    def dio1(self) -> bool: ...
    def set_reset(self, *, asserted: bool) -> None: ...
    def wait_edge(self, *, deadline_monotonic_us: int) -> Dio1Edge | None: ...
    def resynchronize_events(self, *, deadline_monotonic_us: int) -> None:
        """In confirmed standby after IRQ clearing, drain stale events and
        establish a trusted sequence baseline within the enclosing deadline.
        Never called after SetRx; malformed evidence cannot establish trust.
        """
        ...
    def close(self) -> None: ...
