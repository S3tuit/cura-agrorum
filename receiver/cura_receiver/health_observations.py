"""One-shot RTC evidence, independent of runtime time state and provenance."""

from .generated import receiver_enums_generated as E
from .ports.ds3231 import Ds3231ReadResult


def read_rtc_for_health(runtime_time, rtc, *, deadline=None):
    start = runtime_time.clock.now_monotonic_us()
    bound = runtime_time.deadline(runtime_time.settings.rtc_read_budget_us, cap=deadline)
    owner = runtime_time.state_owner
    # A pending durable RTC mutation still owns the device. Observation must not
    # bypass that boundary or reopen a closed resource during terminal cleanup.
    if rtc is None or bound <= start or (owner is not None and owner.pending is not None):
        return E.RtcReadStatus.NOT_ATTEMPTED, None, start, start, None
    try:
        result = rtc.read_time(deadline_monotonic_us=bound)
    except OSError as error:
        return E.RtcReadStatus.IO_ERROR, error.errno, start, runtime_time.clock.now_monotonic_us(), None
    if type(result) is not Ds3231ReadResult:
        raise TypeError('RTC health observation requires a typed read result')
    return (E.RtcReadStatus[result.status.name], result.os_errno,
            result.operation_started_at_monotonic_us,
            result.operation_finished_at_monotonic_us, result.rtc_utc_s)
