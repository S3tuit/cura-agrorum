"""Reviewed immutable control inputs, shared by state and control transaction tests."""

from dataclasses import replace
from cura_receiver.generated.receiver_entities_generated import (
    CommunicatorStateV1,
    TxAirtimeBucketV1,
)
from cura_receiver.generated.receiver_enums_generated import (
    RtcHealth,
    SystemTimeQuality,
)


def state(**changes):
    return replace(
        CommunicatorStateV1(
            generation=1,
            last_observed_system_time_quality=SystemTimeQuality.NETWORK_SYNCED,
            last_observed_rtc_health=RtcHealth.PRESENT,
            rtc_provenance=None,
            rolling_window_us=3_600_000_000,
            tx_airtime_budget_us=36_000_000,
            bucket_width_us=60_000_000,
            bucket_charge_limit_us=8_000_000,
            bucket_expiration_guard_us=1_000_000,
            airtime_snapshot_utc_us=0,
            buckets=(TxAirtimeBucketV1(0, 0),) * 62,
        ),
        **changes,
    )


def synthetic():
    return state(
        buckets=tuple(
            TxAirtimeBucketV1(charge, expiration)
            for charge, expiration in (
                (4_000_000, 3_421_000_000),
                (8_000_000, 3_481_000_000),
                (8_000_000, 3_541_000_000),
                (8_000_000, 3_601_000_000),
                (8_000_000, 3_661_000_000),
            )
        )
        + (TxAirtimeBucketV1(0, 0),) * 57
    )
