"""Compare observations without promoting them into operational state.

Contradictions take precedence over unavailable comparisons. Consistency only
describes agreement: an invalid RTC can agree with software reporting INVALID.
Failed access cannot establish physical absence or a mode mismatch.
"""

from .generated import receiver_enums_generated as E


def assess_health(snapshot, rtc_health, rtc_read_status):
    radio = _radio_consistency(snapshot)
    rtc = E.HealthConsistency.INDETERMINATE
    if rtc_read_status is E.RtcReadStatus.OK:
        rtc = _agreement(rtc_health is E.RtcHealth.PRESENT)
    elif rtc_read_status is E.RtcReadStatus.INVALID:
        rtc = _agreement(rtc_health is E.RtcHealth.INVALID)
    results = (radio, rtc)
    if E.HealthConsistency.INCONSISTENT in results:
        return E.HealthConsistency.INCONSISTENT
    if E.HealthConsistency.INDETERMINATE in results:
        return E.HealthConsistency.INDETERMINATE
    return E.HealthConsistency.CONSISTENT


def _agreement(matches):
    return E.HealthConsistency.CONSISTENT if matches else E.HealthConsistency.INCONSISTENT


def _radio_consistency(snapshot):
    if snapshot.capture_status is not E.RadioCaptureStatus.COMPLETE:
        return E.HealthConsistency.INDETERMINATE
    if snapshot.software_state is E.RadioState.HARDWARE_MISSING:
        return E.HealthConsistency.INCONSISTENT
    # A packet can complete between GPIO/status/IRQ reads. Its handling belongs
    # to the radio owner; telemetry must not mistake single-RX standby for a fault.
    if (snapshot.event_sequence is not None or snapshot.dio1_before
            or snapshot.dio1_after or snapshot.gpio_event_pending or snapshot.irq_status or snapshot.busy):
        return E.HealthConsistency.INDETERMINATE
    expected = {
        E.RadioState.RX_SINGLE: (E.RadioChipMode.RX,),
        E.RadioState.TX_ACTIVE: (E.RadioChipMode.TX,),
        E.RadioState.SHUTDOWN: (E.RadioChipMode.STBY_RC, E.RadioChipMode.STBY_XOSC),
    }.get(snapshot.software_state)
    if expected is None:
        return E.HealthConsistency.INDETERMINATE
    return _agreement(snapshot.chip_mode in expected and snapshot.device_errors == 0
                      and (snapshot.chip_status >> 1) & 7 not in (3, 4, 5))
