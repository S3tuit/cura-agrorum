"""Reviewed immutable control inputs, shared by state and control transaction tests."""

from dataclasses import replace
from cura_receiver.generated.receiver_entities_generated import (
    CommunicatorStateV2,
    AirtimeSnapshotV1,
    AirtimeEntryV2,
)
from cura_receiver.generated.receiver_enums_generated import (
    RtcHealth,
    SystemTimeQuality,
)


def state(**changes):
    return replace(
        CommunicatorStateV2(
            generation=1,
            last_observed_system_time_quality=SystemTimeQuality.NETWORK_SYNCED,
            last_observed_rtc_health=RtcHealth.PRESENT,
            rtc_provenance=None,
            rolling_window_us=3_600_000_000,
            tx_airtime_budget_us=36_000_000,
            entry_charge_us=2_000_000,
            airtime_snapshot=AirtimeSnapshotV1(0, 0),
            entries=(AirtimeEntryV2(0),) * 18,
        ),
        **changes,
    )


def synthetic():
    # Full fallback snapshot at the startup correlation, conservatively converted.
    hold = (3_600_250_000 * 1_003_700 + 999_999) // 1_000_000
    lifetime = (hold * 1_000_000 + 996_299) // 996_300
    return state(entries=(AirtimeEntryV2(lifetime),) * 18)
