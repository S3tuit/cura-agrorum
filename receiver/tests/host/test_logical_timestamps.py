from dataclasses import asdict, replace

import pytest
from hypothesis import given, settings, strategies as st

from cura_receiver.clock_correlation import AnalysisInstance, ClockCorrelation
from cura_receiver.generated import protocol_v2_lora_generated as protocol
from cura_receiver.generated.receiver_entities_generated import (
    ClockObservationV1,
    MessageProfileRowV1,
    MessageProfilingV1,
    ReadingMessageRowV1,
)
from cura_receiver.generated.receiver_enums_generated import (
    AckSelection,
    AckTxResult,
    PersistenceClassification as Classification,
    ProcessingResult,
    RtcHealth,
    SystemTimeQuality,
)
from cura_receiver.logical_timestamps import (
    ReadingTimestamp,
    TimestampSource as Source,
    analyze_reading_timestamps,
    direct_anchor_midpoint_us,
)
from tests.support.builders.protocol_ingress import authenticated_frame

NODE = b"n" * 8
INSTANCE = AnalysisInstance(1, b"i" * 16, b"b" * 16, 0)


def reading(
    sample,
    *,
    node=NODE,
    message=None,
    run_ms=1000,
    flags=257,
    awake_ms=40_000,
    canonical=True,
):
    value = protocol.Reading(
        sample,
        run_ms,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        8 if flags & 1 else 1,
        0,
        awake_ms if flags & 256 else 0,
        0,
        0,
        0,
        flags,
    )
    return ReadingMessageRowV1(
        node_id=node,
        message_id=sample if message is None else message,
        reading_body=protocol.encode_reading(value),
        is_canonical_for_sample=canonical,
        first_receiver_instance_id=INSTANCE.receiver_instance_id,
        first_occurrence_sequence=sample,
        **asdict(value),
    )


def profile(
    reading,
    *,
    received=15_551_328,
    sequence=None,
    instance=INSTANCE,
    domain=1,
    classification=Classification.FIRST_SEEN,
    processing=ProcessingResult.ACCEPTED,
):
    frame = authenticated_frame(
        node_id=reading.node_id,
        message_id=reading.message_id,
        domain=domain,
        body=reading.reading_body,
    )
    accepted = processing is ProcessingResult.ACCEPTED
    ack = authenticated_frame(
        node_id=reading.node_id,
        message_id=reading.message_id,
        domain=3 if accepted else 4,
        body=b"\x00" if accepted else b"\x01",
    )
    value = MessageProfilingV1(
        receiver_instance_id=instance.receiver_instance_id,
        occurrence_sequence=reading.sample_id if sequence is None else sequence,
        received_at_monotonic_us=received,
        received_frame_length=len(frame),
        received_frame=frame + bytes(255 - len(frame)),
        claimed_control=32,
        claimed_domain=domain,
        claimed_node_id=reading.node_id,
        claimed_message_id=reading.message_id,
        header_authenticated=True,
        decoded_sample_id=reading.sample_id,
        rssi_dbm_x2=-100,
        snr_db_x4=20,
        irq_status=2,
        device_errors=0,
        processing_result=processing,
        ack_selected=AckSelection.ACCEPTED if accepted else AckSelection.RETRY_LATER,
        ack_tx_result=AckTxResult.SUPPRESSED_AIRTIME_BUDGET,
        ack_frame=ack,
        busy_wait_total_us=0,
        busy_wait_max_us=0,
        busy_wait_count=0,
        busy_timeout_count=0,
        last_busy_timeout_opcode=None,
        t1_handler_started_monotonic_us=received,
        t2_packet_copied_monotonic_us=received,
        t3_authentication_completed_monotonic_us=received,
        t4_set_tx_attempted_monotonic_us=None,
        t5_tx_done_monotonic_us=None,
        t6_set_rx_issued_monotonic_us=received,
    )
    return MessageProfileRowV1(value, classification)


def clocks(*, instances=(INSTANCE,), observations=None):
    if observations is None:
        observations = [
            ClockObservationV1(
                INSTANCE.receiver_instance_id,
                0,
                0,
                0,
                0,
                False,
                SystemTimeQuality.NETWORK_SYNCED,
                RtcHealth.PRESENT,
            )
        ]
    return ClockCorrelation(instances, observations)


# The scalar direct-anchor interval permits equality; fixed pilot airtime has adjacent integral run_ms limits.
def test_direct_interval_exact_boundary():
    assert direct_anchor_midpoint_us(100_000_000, 30_000_000) == 70_000_000
    assert direct_anchor_midpoint_us(100_000_000, 29_999_999) == 70_000_000
    assert direct_anchor_midpoint_us(100_000_000, 30_000_001) is None
    good, bad = reading(1, run_ms=29897), reading(2, run_ms=29898)
    output = analyze_reading_timestamps(
        [good, bad], [profile(good), profile(bad)], clocks()
    )
    assert output[(NODE, 1)].timestamp_source is Source.DIRECT
    assert output[(NODE, 2)].timestamp_source is Source.EXTRAPOLATED


# Direct estimates use RX_DONE and retain the source clock observation, including UTC zero output.
def test_direct_anchor_provenance_and_rx_done():
    row = reading(100)
    occurrence = profile(row)
    occurrence = replace(
        occurrence,
        profile=replace(
            occurrence.profile,
            t1_handler_started_monotonic_us=99_000_000,
            t2_packet_copied_monotonic_us=99_000_000,
            t3_authentication_completed_monotonic_us=99_000_000,
            t6_set_rx_issued_monotonic_us=99_000_000,
        ),
    )
    assert analyze_reading_timestamps([row], [occurrence], clocks())[
        (NODE, 100)
    ] == ReadingTimestamp(0, Source.DIRECT, 100, INSTANCE.receiver_instance_id, 0)


# All three non-conflict persisted classifications can supply a direct current-reading anchor.
@pytest.mark.parametrize(
    "classification",
    [
        Classification.FIRST_SEEN,
        Classification.RETRANSMISSION,
        Classification.DUPLICATE_SAME_CONTENT,
    ],
)
def test_anchor_classifications(classification):
    row = reading(100)
    assert (
        analyze_reading_timestamps(
            [row], [profile(row, classification=classification)], clocks()
        )[(NODE, 100)].timestamp_source
        is Source.DIRECT
    )


# Conflict occurrences and authenticated frames without successful queue admission cannot anchor a sample.
@pytest.mark.parametrize(
    "classification,processing,domain",
    [
        (Classification.DUPLICATE_CONFLICT, ProcessingResult.ACCEPTED, 1),
        (Classification.MESSAGE_ID_CONFLICT, ProcessingResult.ACCEPTED, 1),
        (Classification.NOT_APPLICABLE, ProcessingResult.ACCEPTED, 1),
        (Classification.NOT_APPLICABLE, ProcessingResult.RETRY_LATER_QUEUE_FULL, 1),
        (Classification.FIRST_SEEN, ProcessingResult.ACCEPTED, 2),
    ],
)
def test_ineligible_occurrences(classification, processing, domain):
    row = reading(100)
    assert (
        analyze_reading_timestamps(
            [row],
            [
                profile(
                    row,
                    classification=classification,
                    processing=processing,
                    domain=domain,
                )
            ],
            clocks(),
        )
        == {}
    )


# The earliest eligible current occurrence wins regardless of supplied order or a later smaller UTC estimate.
def test_earliest_eligible_occurrence():
    row = reading(100)
    first = profile(row, received=15_551_328, sequence=1)
    later = profile(
        row,
        received=20_551_328,
        sequence=2,
        classification=Classification.RETRANSMISSION,
    )
    output = analyze_reading_timestamps([row], [later, first], clocks())
    assert output[(NODE, 100)].timestamp_utc_us == 0


# Across a reboot, durable instance order selects the first occurrence without comparing monotonic or UTC clocks.
def test_earliest_occurrence_across_boots():
    row = reading(100)
    new = AnalysisInstance(2, b"j" * 16, b"c" * 16, 0)
    observations = [
        ClockObservationV1(
            i.receiver_instance_id,
            0,
            0,
            0,
            utc,
            False,
            SystemTimeQuality.NETWORK_SYNCED,
            RtcHealth.PRESENT,
        )
        for i, utc in ((INSTANCE, 100_000_000), (new, 0))
    ]
    output = analyze_reading_timestamps(
        [row],
        [profile(row, instance=new, sequence=0), profile(row, sequence=99)],
        clocks(instances=[INSTANCE, new], observations=observations),
    )
    assert output[(NODE, 100)].timestamp_utc_us == 100_000_000
    assert (
        output[(NODE, 100)].clock_observation_receiver_instance_id
        == INSTANCE.receiver_instance_id
    )


# An earliest step-gap occurrence is ineligible, so a later same-message occurrence can anchor instead.
def test_step_gap_occurrence_cannot_anchor():
    row = reading(100)
    observations = [
        ClockObservationV1(
            INSTANCE.receiver_instance_id,
            0,
            0,
            0,
            None,
            True,
            SystemTimeQuality.UNTRUSTED,
            RtcHealth.PRESENT,
        ),
        ClockObservationV1(
            INSTANCE.receiver_instance_id,
            1,
            1,
            20_000_000,
            20_000_000,
            False,
            SystemTimeQuality.NETWORK_SYNCED,
            RtcHealth.PRESENT,
        ),
    ]
    output = analyze_reading_timestamps(
        [row],
        [
            profile(row, sequence=0),
            profile(
                row,
                sequence=1,
                received=25_551_328,
                classification=Classification.RETRANSMISSION,
            ),
        ],
        clocks(observations=observations),
    )
    assert output[(NODE, 100)].timestamp_utc_us == 10_000_000
    assert output[(NODE, 100)].clock_observation_sequence == 1


# A later current message with the canonical body can anchor a sample first received as backlog.
def test_current_duplicate_of_backlog():
    backlog = reading(100, message=10)
    current = replace(backlog, message_id=11, is_canonical_for_sample=False)
    output = analyze_reading_timestamps(
        [backlog, current],
        [
            profile(backlog, sequence=0, domain=2),
            profile(
                current,
                sequence=1,
                classification=Classification.DUPLICATE_SAME_CONTENT,
            ),
        ],
        clocks(),
    )
    assert output[(NODE, 100)].timestamp_source is Source.DIRECT


# Both directions use the newer sample's previous awake duration and add no separate ACK time.
@pytest.mark.parametrize("anchor", [100, 102])
def test_bidirectional_extrapolation(anchor):
    rows = [reading(100), reading(101, awake_ms=10_000), reading(102, awake_ms=20_000)]
    selected = rows[0] if anchor == 100 else rows[2]
    output = analyze_reading_timestamps(rows, [profile(selected)], clocks())
    expected = (
        {100: 0, 101: 910_000_000, 102: 1_830_000_000}
        if anchor == 100
        else {100: -1_830_000_000, 101: -920_000_000, 102: 0}
    )
    assert {
        sample: value.timestamp_utc_us for (_, sample), value in output.items()
    } == expected
    assert all(value.anchor_sample_id == anchor for value in output.values())
    assert all(
        value.clock_observation_receiver_instance_id == INSTANCE.receiver_instance_id
        for value in output.values()
    )


# Missing continuity flags break the incoming edge in both directions while leaving the sample usable from a newer anchor.
@pytest.mark.parametrize("flags", [0, 1, 256])
def test_broken_continuity_flags(flags):
    rows = [reading(100), reading(101, flags=flags), reading(102)]
    backward = analyze_reading_timestamps(rows, [profile(rows[2])], clocks())
    assert (NODE, 100) not in backward
    assert backward[(NODE, 101)].timestamp_source is Source.EXTRAPOLATED
    forward = analyze_reading_timestamps(rows, [profile(rows[0])], clocks())
    assert set(forward) == {(NODE, 100)}


# Sample gaps, identity changes and counter wrap cannot form a continuity edge.
@pytest.mark.parametrize(
    "other", [reading(102), reading(101, node=b"x" * 8), reading(0)]
)
def test_identity_and_sample_fences(other):
    start = reading(100)
    output = analyze_reading_timestamps([start, other], [profile(start)], clocks())
    assert set(output) == {(NODE, 100)}
    maximum = reading((1 << 32) - 1)
    assert set(
        analyze_reading_timestamps([maximum, reading(0)], [profile(maximum)], clocks())
    ) == {(NODE, (1 << 32) - 1)}


# Competing equally distant anchors use the newer sample, reproducing the reviewed eight-second disagreement.
def test_competing_anchor_tie():
    rows = [reading(i) for i in (100, 101, 102)]
    output = analyze_reading_timestamps(
        rows, [profile(rows[0]), profile(rows[2], received=1_903_551_328)], clocks()
    )
    assert output[(NODE, 100)].timestamp_utc_us == 0
    assert output[(NODE, 102)].timestamp_utc_us == 1_888_000_000
    assert output[(NODE, 101)] == ReadingTimestamp(
        948_000_000, Source.EXTRAPOLATED, 102, INSTANCE.receiver_instance_id, 0
    )


# Fewer valid hops beat recency, and direct samples always retain their own anchors.
def test_nearest_anchor_and_direct_priority():
    rows = [reading(i) for i in range(100, 105)]
    output = analyze_reading_timestamps(
        rows, [profile(rows[0]), profile(rows[4], received=4_000_000_000)], clocks()
    )
    assert output[(NODE, 101)].anchor_sample_id == 100
    assert output[(NODE, 102)].anchor_sample_id == 104
    assert output[(NODE, 103)].anchor_sample_id == 104
    assert output[(NODE, 104)].timestamp_source is Source.DIRECT


# Newer closer or direct evidence never replaces an already materialized estimate or mutates its mapping.
def test_materialized_output_preserved():
    rows = [reading(i) for i in (100, 101, 102)]
    initial = analyze_reading_timestamps(rows, [profile(rows[0])], clocks())
    previous = dict(initial)
    output = analyze_reading_timestamps(
        rows,
        [
            profile(row, received=2_000_000_000 + i * 1_000_000)
            for i, row in enumerate(rows)
        ],
        clocks(),
        materialized=initial,
    )
    assert output == previous
    assert initial == previous
    assert output[(NODE, 101)] is initial[(NODE, 101)]


# Duplicate keys and unresolved accepted joins reject inconsistent history rather than choosing a replacement anchor.
def test_invalid_reading_history():
    row = reading(100)
    occurrence = profile(row)
    with pytest.raises(ValueError):
        analyze_reading_timestamps([row, row], [], clocks())
    with pytest.raises(ValueError):
        analyze_reading_timestamps([row], [occurrence, occurrence], clocks())
    with pytest.raises(ValueError):
        analyze_reading_timestamps([], [occurrence], clocks())
    with pytest.raises(ValueError):
        analyze_reading_timestamps(
            [row], [occurrence], clocks(instances=[], observations=[])
        )


# Checked direct and extrapolated timestamps fail without wrapping or inventing another eligible source.
def test_logical_timestamp_overflow():
    with pytest.raises(OverflowError):
        direct_anchor_midpoint_us(-(1 << 63), 1_000_000)
    rows = [reading(100), reading(101)]
    observations = [
        ClockObservationV1(
            INSTANCE.receiver_instance_id,
            0,
            0,
            0,
            (1 << 63) - 2,
            False,
            SystemTimeQuality.NETWORK_SYNCED,
            RtcHealth.PRESENT,
        )
    ]
    output = analyze_reading_timestamps(
        rows, [profile(rows[0], received=0)], clocks(observations=observations)
    )
    assert (NODE, 100) in output
    assert (NODE, 101) not in output


def reference_logical_chain(samples, anchors):
    """Walk each possible anchor path independently, using only primitive input facts."""
    result = {}
    for target in samples:
        candidates = []
        for anchor, utc in anchors.items():
            low, high = sorted((target, anchor))
            valid = True
            elapsed = 0
            for sample in range(low + 1, high + 1):
                if sample not in samples or sample - 1 not in samples:
                    valid = False
                    break
                flags, awake = samples[sample]
                if not (flags & 1 and flags & 256):
                    valid = False
                    break
                elapsed += 900_000_000 + 1000 * awake
            if valid:
                value = utc + elapsed if target >= anchor else utc - elapsed
                candidates.append((abs(target - anchor), -anchor, value))
        if candidates:
            _, negative_anchor, utc = min(candidates)
            result[target] = (utc, -negative_anchor)
    return result


# Reviewed graph walking reproduces the competing-anchor tie and refuses a broken incoming edge.
def test_reference_logical_choices():
    samples = {100: (257, 40_000), 101: (257, 40_000), 102: (257, 40_000)}
    assert reference_logical_chain(samples, {100: 0, 102: 1_888_000_000})[101] == (
        948_000_000,
        102,
    )
    samples[101] = (256, 40_000)
    assert reference_logical_chain(samples, {100: 0}) == {100: (0, 100)}


# Generated chains compare nearest reachable anchor selection against an independent per-anchor graph walk.
@settings(max_examples=100, derandomize=True, deadline=None)
@given(
    facts=st.lists(
        st.tuples(
            st.sampled_from([0, 1, 256, 257, 769]),
            st.integers(0, 65535),
            st.booleans(),
            st.integers(0, 1000),
        ),
        min_size=1,
        max_size=20,
    )
)
def test_logical_chain_properties(facts):
    rows, profiles, primitives, anchors = [], [], {}, {}
    for index, (flags, awake, anchored, offset) in enumerate(facts):
        sample = 100 + index
        awake = awake if flags & 256 else 0
        row = reading(sample, flags=flags, awake_ms=awake)
        rows.append(row)
        primitives[sample] = (flags, awake)
        if anchored:
            utc = index * 1_000_000_000 + offset
            anchors[sample] = utc
            profiles.append(profile(row, received=15_551_328 + utc))
    expected = reference_logical_chain(primitives, anchors)
    output = analyze_reading_timestamps(reversed(rows), reversed(profiles), clocks())
    assert {
        sample: (value.timestamp_utc_us, value.anchor_sample_id)
        for (_, sample), value in output.items()
    } == expected
    for (_, sample), value in output.items():
        assert value.timestamp_source is (
            Source.DIRECT if sample in anchors else Source.EXTRAPOLATED
        )
