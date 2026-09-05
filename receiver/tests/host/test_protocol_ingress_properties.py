from __future__ import annotations

import struct

from cryptography.hazmat.primitives.ciphers.aead import AESCCM
from hypothesis import given, settings, strategies as st

from cura_receiver.generated.receiver_enums_generated import (
    PersistenceAdmissionState,
)
from cura_receiver.persist_queue import PersistQueue, PersistenceAdmissionSnapshot
from cura_receiver.persist_queue_entities import PROFILE_ONLY_V1_SPEC
from cura_receiver.protocol_ingress import ProtocolIngress
from tests.support.builders.protocol_ingress import (
    REVIEWED_CURRENT_FRAME,
    REVIEWED_NODE_ID,
    REVIEWED_NODE_KEY,
    REVIEWED_READING_BODY,
    ingress_packet,
)
from tests.support.fakes.os_clock import FakeOsClock
from tests.support.models.protocol_ingress import classify_protocol_ingress


UNKNOWN_NODE_ID = bytes.fromhex("1112131415161718")
HEADER_STRUCT = struct.Struct("<BB8sI")
READING_STRUCT = struct.Struct("<IHHHhhhIHBBHHBBH")


def _boundary_unsigned(bits: int) -> st.SearchStrategy[int]:
    maximum = (1 << bits) - 1
    return st.one_of(
        st.sampled_from((0, 1, maximum - 1, maximum)),
        st.integers(min_value=0, max_value=maximum),
    )


def _boundary_i16() -> st.SearchStrategy[int]:
    return st.one_of(
        st.sampled_from((-(1 << 15), -(1 << 15) + 1, -1, 0, 1, (1 << 15) - 2, (1 << 15) - 1)),
        st.integers(min_value=-(1 << 15), max_value=(1 << 15) - 1),
    )


U8 = _boundary_unsigned(8)
U16 = _boundary_unsigned(16)
U32 = _boundary_unsigned(32)
I16 = _boundary_i16()


@st.composite
def valid_reading_bodies(draw: st.DrawFn) -> bytes:
    flags = 0

    deep_sleep = draw(st.booleans())
    if deep_sleep:
        reset_reason = 8
        flags |= 1 << 0
    else:
        reset_reason = draw(U8.filter(lambda value: value != 8))

    sensor_fields: list[int] = []
    for flag, strategy in (
        (1 << 1, U16),
        (1 << 2, U16),
        (1 << 3, I16),
        (1 << 4, I16),
        (1 << 5, I16),
        (1 << 6, U32),
        (1 << 7, U16),
    ):
        valid = draw(st.booleans())
        sensor_fields.append(draw(strategy) if valid else 0)
        if valid:
            flags |= flag

    previous_metrics_valid = draw(st.booleans())
    if previous_metrics_valid:
        flags |= 1 << 8
        previous_current_tx_attempts = draw(U8)
        previous_awake_ms = draw(U16)
        previous_cycle_tx_attempts = draw(U8)
        previous_cycle_accepted_readings = draw(U8)
        previous_accepted = draw(st.booleans())
        if previous_accepted:
            flags |= 1 << 9
            previous_current_delivery_ms = draw(U16)
        else:
            previous_current_delivery_ms = 0
    else:
        previous_current_tx_attempts = 0
        previous_awake_ms = 0
        previous_current_delivery_ms = 0
        previous_cycle_tx_attempts = 0
        previous_cycle_accepted_readings = 0

    return READING_STRUCT.pack(
        draw(U32),
        draw(U16),
        *sensor_fields,
        reset_reason,
        previous_current_tx_attempts,
        previous_awake_ms,
        previous_current_delivery_ms,
        previous_cycle_tx_attempts,
        previous_cycle_accepted_readings,
        flags,
    )


def _seal_independently(
    *,
    key: bytes,
    control: int,
    domain: int,
    node_id: bytes,
    message_id: int,
    body: bytes,
) -> bytes:
    header = HEADER_STRUCT.pack(control, domain, node_id, message_id)
    nonce = node_id + struct.pack("<I", message_id) + bytes((domain,))
    return header + AESCCM(key, tag_length=8).encrypt(nonce, body, header)


@st.composite
def ingress_cases(draw: st.DrawFn) -> tuple[bytes, str]:
    admission_result = draw(
        st.sampled_from(("RESERVED", "PERSISTENCE_UNAVAILABLE", "QUEUE_FULL"))
    )
    if draw(st.integers(min_value=0, max_value=3)) == 0:
        frame = draw(st.binary(min_size=0, max_size=255))
        return frame, admission_result

    control = draw(U8)
    domain = draw(U8)
    node_id = draw(st.sampled_from((REVIEWED_NODE_ID, UNKNOWN_NODE_ID)))
    message_id = draw(U32)
    body = draw(
        st.one_of(
            valid_reading_bodies(),
            st.binary(min_size=1, max_size=33),
        )
    )
    correct_key = draw(st.booleans())
    key = REVIEWED_NODE_KEY if correct_key else bytes(reversed(REVIEWED_NODE_KEY))
    frame = _seal_independently(
        key=key,
        control=control,
        domain=domain,
        node_id=node_id,
        message_id=message_id,
        body=body,
    )
    if draw(st.booleans()):
        index = draw(st.integers(min_value=0, max_value=len(frame) - 1))
        changed = bytearray(frame)
        changed[index] ^= draw(st.integers(min_value=1, max_value=255))
        frame = bytes(changed)
    return frame, admission_result


def _queue_for(admission_result: str) -> PersistQueue:
    state = (
        PersistenceAdmissionState.UNAVAILABLE_IO
        if admission_result == "PERSISTENCE_UNAVAILABLE"
        else PersistenceAdmissionState.AVAILABLE
    )
    queue = PersistQueue(capacity_entities=1)
    queue.publish_admission_state(
        PersistenceAdmissionSnapshot(
            generation=1,
            state=state,
            changed_at_monotonic_us=1,
        )
    )
    if admission_result == "QUEUE_FULL":
        occupied = queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
        assert occupied.reservation is not None
        occupied.reservation.publish(object())
    return queue


def _actual(frame: bytes, admission_result: str):
    queue = _queue_for(admission_result)
    ingress = ProtocolIngress(
        queue=queue,
        monotonic_clock=FakeOsClock(monotonic_us=20, realtime_us=0),
        auth_node_keys={REVIEWED_NODE_ID: REVIEWED_NODE_KEY},
    )
    return ingress.begin(ingress_packet(frame=frame))


# Establishes the independent model boundary with the reviewed accepted vector.
def test_reference_model_matches_the_reviewed_current_reading() -> None:
    expected = classify_protocol_ingress(
        frame=REVIEWED_CURRENT_FRAME,
        auth_node_keys={REVIEWED_NODE_ID: REVIEWED_NODE_KEY},
        admission_result="RESERVED",
    )

    assert expected.processing_result == "ACCEPTED"
    assert expected.ack_selected == "ACCEPTED"
    assert expected.header_authenticated
    assert expected.decoded_sample_id == 0x55667788
    assert expected.candidate == (
        REVIEWED_NODE_ID,
        0x11223344,
        1,
        0x55667788,
        REVIEWED_READING_BODY,
    )
    assert expected.admission_kind == "MEASUREMENT_PROFILE"
    assert expected.admission_result == "RESERVED"


# Establishes silent, permanent-rejection and retry model boundaries explicitly.
def test_reference_model_reviewed_nonaccepted_boundaries() -> None:
    unknown = classify_protocol_ingress(
        frame=bytes(22),
        auth_node_keys={REVIEWED_NODE_ID: REVIEWED_NODE_KEY},
        admission_result="RESERVED",
    )
    unsupported_frame = _seal_independently(
        key=REVIEWED_NODE_KEY,
        control=0x30,
        domain=1,
        node_id=REVIEWED_NODE_ID,
        message_id=0x11223344,
        body=REVIEWED_READING_BODY,
    )
    unsupported = classify_protocol_ingress(
        frame=unsupported_frame,
        auth_node_keys={REVIEWED_NODE_ID: REVIEWED_NODE_KEY},
        admission_result="RESERVED",
    )
    retry = classify_protocol_ingress(
        frame=REVIEWED_CURRENT_FRAME,
        auth_node_keys={REVIEWED_NODE_ID: REVIEWED_NODE_KEY},
        admission_result="QUEUE_FULL",
    )

    assert (unknown.processing_result, unknown.ack_selected) == (
        "REJECTED_MALFORMED_LENGTH",
        "NONE",
    )
    assert (unsupported.processing_result, unsupported.ack_selected) == (
        "REJECTED_UNSUPPORTED_CONTROL",
        "REJECTED_UNSUPPORTED",
    )
    assert (retry.processing_result, retry.ack_selected) == (
        "RETRY_LATER_QUEUE_FULL",
        "RETRY_LATER",
    )


# Compares valid and invalid integer/header/body/frame boundaries with an independent model.
@settings(max_examples=600, deadline=None)
@given(case=ingress_cases())
def test_ingress_properties_match_independent_validation_order_model(
    case: tuple[bytes, str],
) -> None:
    frame, admission_result = case
    expected = classify_protocol_ingress(
        frame=frame,
        auth_node_keys={REVIEWED_NODE_ID: REVIEWED_NODE_KEY},
        admission_result=admission_result,
    )
    decision = _actual(frame, admission_result)
    actual = decision.pre_tx_profile

    assert actual.processing_result.name == expected.processing_result
    assert actual.ack_selected.name == expected.ack_selected
    assert actual.ack_frame == expected.ack_frame
    assert actual.claimed_control == expected.claimed_control
    assert actual.claimed_domain == expected.claimed_domain
    assert actual.claimed_node_id == expected.claimed_node_id
    assert actual.claimed_message_id == expected.claimed_message_id
    assert actual.header_authenticated is expected.header_authenticated
    assert actual.decoded_sample_id == expected.decoded_sample_id

    if expected.candidate is None:
        assert decision.candidate is None
    else:
        assert decision.candidate is not None
        assert (
            decision.candidate.node_id,
            decision.candidate.message_id,
            decision.candidate.domain,
            decision.candidate.sample_id,
            decision.candidate.reading_body,
        ) == expected.candidate

    if expected.admission_result is None:
        assert decision.admission is None
    else:
        assert decision.admission is not None
        assert decision.admission.entity_kind.name == expected.admission_kind
        assert decision.admission.result.name == expected.admission_result
