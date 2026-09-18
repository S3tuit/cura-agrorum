"""Boot-scoped provisional RTC-to-system copy, independent of receiver storage."""

from enum import Enum
import os
from pathlib import Path

from .elapsed_duration import checked_monotonic_deadline
from .platform.linux_boot_identity import read_linux_boot_id
from .ports.ds3231 import Ds3231ReadStatus as R
from .ports.kernel_clock import KernelSampleStatus as K


class BootstrapOutcome(Enum):
    COPIED = 'COPIED'
    ALREADY_ATTEMPTED = 'ALREADY_ATTEMPTED'
    ALREADY_SYNCHRONIZED = 'ALREADY_SYNCHRONIZED'
    MISSING = 'MISSING'
    INVALID = 'INVALID'
    IO_ERROR = 'IO_ERROR'
    DEADLINE_EXCEEDED = 'DEADLINE_EXCEEDED'
    KERNEL_UNAVAILABLE = 'KERNEL_UNAVAILABLE'


def copy_rtc_once(*, clock, kernel, rtc, claim_attempt, set_system_utc,
                  attempts=3, total_budget_us=5_000_000):
    """claim_attempt atomically prevents another copy, including after a crash."""
    deadline = checked_monotonic_deadline(clock.now_monotonic_us(), total_budget_us)
    if not claim_attempt():
        return BootstrapOutcome.ALREADY_ATTEMPTED
    sample = kernel.sample(deadline_monotonic_us=min(deadline,
        checked_monotonic_deadline(clock.now_monotonic_us(), 250_000)))
    if sample.status is not K.OK or sample.kernel_status_bits is None:
        return BootstrapOutcome.KERNEL_UNAVAILABLE
    if not sample.kernel_status_bits & 0x40:  # Linux STA_UNSYNC is clear.
        return BootstrapOutcome.ALREADY_SYNCHRONIZED
    outcome = BootstrapOutcome.DEADLINE_EXCEEDED
    for _ in range(attempts):
        if clock.now_monotonic_us() >= deadline:
            return BootstrapOutcome.DEADLINE_EXCEEDED
        result = rtc.read_time(deadline_monotonic_us=deadline)
        if clock.now_monotonic_us() >= deadline:
            return BootstrapOutcome.DEADLINE_EXCEEDED
        if result.status is R.OK:
            try:
                set_system_utc(result.rtc_utc_s)
            except OSError:
                return BootstrapOutcome.IO_ERROR
            return BootstrapOutcome.COPIED
        if result.status is R.MISSING:
            return BootstrapOutcome.MISSING
        if result.status is R.INVALID:
            return BootstrapOutcome.INVALID
        outcome = (BootstrapOutcome.DEADLINE_EXCEEDED if result.status is R.DEADLINE_EXCEEDED
                   else BootstrapOutcome.IO_ERROR)
    return outcome


def claim_boot_attempt(directory=Path('/run/cura-agrorum-rtc-bootstrap')):
    """The root-owned systemd RuntimeDirectory survives unit restarts, not reboot."""
    boot = read_linux_boot_id().hex()
    try:
        fd = os.open(directory / boot, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC | os.O_NOFOLLOW, 0o600)
    except FileExistsError:
        return False
    os.close(fd)
    return True


def main():
    import argparse
    from .platform.linux_clocks import LinuxOsClock, set_bootstrap_utc_seconds
    from .platform.linux_kernel_clock import LinuxKernelClock
    from .platform.linux_ds3231 import LinuxDs3231Control
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rtc-kernel-bound-us', type=int, required=True)
    args = parser.parse_args()
    clock = LinuxOsClock()
    rtc = LinuxDs3231Control(clock, kernel_operation_bound_us=args.rtc_kernel_bound_us)
    outcome = copy_rtc_once(clock=clock, kernel=LinuxKernelClock(clock), rtc=rtc,
        claim_attempt=claim_boot_attempt,
        set_system_utc=set_bootstrap_utc_seconds)
    print(outcome.value)
    # Completion, not successful UTC copying, permits the other services to start.
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
