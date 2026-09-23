from dataclasses import replace

import pytest

from cura_receiver.clock_correlation import (
    AnalysisInstance,
    ClockCorrelation,
    CorrelatedUtc,
)
from cura_receiver.generated.receiver_entities_generated import ClockObservationV1
from cura_receiver.generated.receiver_enums_generated import (
    RtcHealth as Health,
    SystemTimeQuality as Quality,
)

INSTANCE = AnalysisInstance(1, b"i" * 16, b"b" * 16, 100)


def observation(
    sequence, monotonic, utc, *, step=False, instance=INSTANCE, generation=None
):
    return ClockObservationV1(
        instance.receiver_instance_id,
        sequence,
        sequence if generation is None else generation,
        monotonic,
        utc,
        step,
        Quality.UNTRUSTED if utc is None else Quality.NETWORK_SYNCED,
        Health.PRESENT,
    )


# Open segments use the latest preceding observation, while initial/ordinary gaps may backfill.
@pytest.mark.parametrize(
    "event,utc,sequence",
    [
        (100, 900, 0),
        (199, 999, 0),
        (200, 1000, 0),
        (299, 1099, 0),
        (300, 1100, 1),
        (350, 1150, 1),
        (400, 1900, 3),
        (499, 1999, 3),
        (500, 2000, 3),
        (600, 2100, 3),
    ],
)
def test_correlation_segments(event, utc, sequence):
    history = [
        observation(0, 200, 1000),
        observation(1, 300, 1100),
        observation(2, 400, None),
        observation(3, 500, 2000),
    ]
    correlation = ClockCorrelation([INSTANCE], reversed(history))
    assert correlation.correlate(INSTANCE.receiver_instance_id, event) == CorrelatedUtc(
        utc, INSTANCE.receiver_instance_id, sequence
    )


# A step gap survives intervening ordinary untrusted observations and ends exactly at the next trusted one.
@pytest.mark.parametrize(
    "event,expected",
    [
        (199, 999),
        (200, None),
        (249, None),
        (250, None),
        (299, None),
        (300, 2000),
        (301, 2001),
    ],
)
def test_permanent_step_gap(event, expected):
    correlation = ClockCorrelation(
        [INSTANCE],
        [
            observation(0, 100, 900),
            observation(1, 200, None, step=True),
            observation(2, 250, None),
            observation(3, 300, 2000),
        ],
    )
    result = correlation.correlate(INSTANCE.receiver_instance_id, event)
    assert (None if result is None else result.utc_us) == expected


# A later trusted sample cannot reach backward across a step even to an event before that step.
def test_no_backfill_across_future_step():
    correlation = ClockCorrelation(
        [INSTANCE], [observation(0, 200, None, step=True), observation(1, 300, 2000)]
    )
    assert correlation.correlate(INSTANCE.receiver_instance_id, 100) is None
    assert correlation.correlate(INSTANCE.receiver_instance_id, 199) is None


# Sequence order resolves equal-microsecond boundaries before events, including an empty step gap.
def test_same_microsecond_order():
    boundary = observation(1, 200, None, step=True)
    trusted = observation(0, 200, 1000)
    blocked = ClockCorrelation([INSTANCE], [boundary, trusted])
    assert blocked.correlate(INSTANCE.receiver_instance_id, 200) is None
    resumed = ClockCorrelation(
        [INSTANCE], [trusted, boundary, observation(2, 200, 2000)]
    )
    assert resumed.correlate(INSTANCE.receiver_instance_id, 200) == CorrelatedUtc(
        2000, INSTANCE.receiver_instance_id, 2
    )


# Periodic observations can reuse a generation, and the latest sequence wins a tied sample time.
def test_periodic_generation_and_latest_sequence():
    correlation = ClockCorrelation(
        [INSTANCE],
        [
            observation(2, 200, 2000, generation=0),
            observation(0, 200, 1000, generation=0),
        ],
    )
    assert (
        correlation.correlate(
            INSTANCE.receiver_instance_id, 200
        ).clock_observation_sequence
        == 2
    )


# Receiver restarts fence correlation even on one boot; different boots' monotonic values are never ordered together.
@pytest.mark.parametrize("boot,start", [(b"b" * 16, 300), (b"c" * 16, 0)])
def test_process_start_and_boot_fences(boot, start):
    new = AnalysisInstance(2, b"j" * 16, boot, start)
    correlation = ClockCorrelation(
        [INSTANCE, new], [observation(0, start + 100, 2000, instance=new)]
    )
    assert correlation.correlate(INSTANCE.receiver_instance_id, 150) is None
    assert correlation.correlate(new.receiver_instance_id, start) == CorrelatedUtc(
        1900, new.receiver_instance_id, 0
    )
    if start:
        assert correlation.correlate(new.receiver_instance_id, start - 1) is None


# Missing observations, unresolved event instances and events before startup have no derived UTC.
def test_unresolved_events():
    correlation = ClockCorrelation([INSTANCE], [])
    assert correlation.correlate(INSTANCE.receiver_instance_id, 100) is None
    assert correlation.correlate(b"z" * 16, 100) is None
    assert correlation.correlate(INSTANCE.receiver_instance_id, 99) is None


# Canonical absence is None; a trusted UTC zero is retained and all input records remain unchanged.
def test_utc_zero_and_input_immutability():
    record = observation(0, 200, 0)
    before = (INSTANCE, record)
    correlation = ClockCorrelation([INSTANCE], [record])
    assert correlation.correlate(INSTANCE.receiver_instance_id, 200).utc_us == 0
    assert correlation.correlate(INSTANCE.receiver_instance_id, 199).utc_us == -1
    assert before == (INSTANCE, record)


# Correlation cannot manufacture a representable UTC by wrapping signed arithmetic.
@pytest.mark.parametrize("utc,event", [((1 << 63) - 1, 201), (-(1 << 63), 199)])
def test_utc_overflow_has_no_assignment(utc, event):
    correlation = ClockCorrelation([INSTANCE], [observation(0, 200, utc)])
    assert correlation.correlate(INSTANCE.receiver_instance_id, event) is None


# Malformed stored observations are rejected as history errors, never silently skipped to gain trust.
@pytest.mark.parametrize(
    "records",
    [
        [observation(0, 99, 1000)],
        [replace(observation(0, 200, 1000), receiver_instance_id=b"z" * 16)],
        [observation(0, 200, 1000), observation(0, 300, 2000)],
        [observation(1, 200, 1000), observation(0, 300, 2000)],
        [
            observation(0, 200, 1000, generation=1),
            observation(1, 300, 2000, generation=0),
        ],
        [replace(observation(0, 200, None), sampled_at_utc_us=0)],
        [replace(observation(0, 200, 1000), sampled_at_utc_us=None)],
        [observation(0, 200, 1000, step=True)],
        [replace(observation(0, 200, 1000), step_discontinuity_boundary=1)],
        [replace(observation(0, 200, 1000), system_time_quality=2)],
        [
            replace(
                observation(0, 200, 1000),
                system_time_quality=Quality.RTC_HOLDOVER,
                rtc_health=Health.MISSING,
            )
        ],
        [observation(1 << 63, 200, 1000)],
    ],
)
def test_invalid_history(records):
    with pytest.raises((ValueError, TypeError, OverflowError)):
        ClockCorrelation([INSTANCE], records)


# Duplicate instance identities or ordinal assignments cannot supply ambiguous provenance.
@pytest.mark.parametrize(
    "extra",
    [
        INSTANCE,
        replace(INSTANCE, instance_ordinal=2),
        replace(INSTANCE, receiver_instance_id=b"j" * 16),
    ],
)
def test_duplicate_instances(extra):
    with pytest.raises(ValueError):
        ClockCorrelation([INSTANCE, extra], [])
