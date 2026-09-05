from __future__ import annotations

from collections.abc import Callable

import pytest
from cryptography.hazmat.primitives.ciphers.aead import AESCCM

from cura_receiver.generated import protocol_v2_lora_generated as protocol
from cura_receiver.generated.receiver_enums_generated import (
    AckSelection,
    AckTxResult,
    AdmissionResult,
    PersistenceAdmissionState,
    ProcessingResult,
)
from cura_receiver.persist_queue import (
    PersistQueue,
    PersistenceAdmissionSnapshot,
)
from cura_receiver.persist_queue_entities import PROFILE_ONLY_V1_SPEC, ProfileOnlyUnitV1
from cura_receiver.protocol_ingress import (
    ProtocolIngress,
    ProtocolIngressTerminalV1,
)
from tests.support.builders.protocol_ingress import (
    REVIEWED_NODE_ID,
    REVIEWED_NODE_KEY,
    REVIEWED_REJECTED_MALFORMED_ACK,
    REVIEWED_REJECTED_UNSUPPORTED_ACK,
    REVIEWED_READING_BODY,
    authenticated_frame,
    ingress_packet,
)
from tests.support.fakes.os_clock import FakeOsClock


UNKNOWN_NODE_ID = bytes.fromhex("1112131415161718")


def _queue(
    *,
    state: PersistenceAdmissionState = PersistenceAdmissionState.AVAILABLE,
    full: bool = False,
) -> PersistQueue:
    queue = PersistQueue(capacity_entities=1)
    queue.publish_admission_state(
        PersistenceAdmissionSnapshot(
            generation=1,
            state=state,
            changed_at_monotonic_us=1,
        )
    )
    if full:
        result = queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
        assert result.reservation is not None
        result.reservation.publish(object())
    return queue


def _ingress(queue: PersistQueue | None = None) -> ProtocolIngress:
    return ProtocolIngress(
        queue=_queue() if queue is None else queue,
        monotonic_clock=FakeOsClock(monotonic_us=20, realtime_us=0),
        auth_node_keys={REVIEWED_NODE_ID: REVIEWED_NODE_KEY},
    )


def _direct_authenticated_frame(
    *,
    key: bytes = REVIEWED_NODE_KEY,
    control: int = protocol.CONTROL,
    domain: int = protocol.Domain.CURRENT_READING_UPLINK,
    node_id: bytes = REVIEWED_NODE_ID,
    body: bytes = REVIEWED_READING_BODY,
) -> bytes:
    header = protocol.ClearHeader(
        control=control,
        domain=domain,
        node_id=node_id,
        message_id=0x11223344,
    )
    encoded_header = protocol.encode_clear_header(header)
    return encoded_header + AESCCM(
        key,
        tag_length=protocol.TAG_SIZE,
    ).encrypt(protocol.build_nonce(header), body, encoded_header)


def _changed_byte(frame: bytes, index: int) -> bytes:
    changed = bytearray(frame)
    changed[index] ^= 1
    return bytes(changed)


def _raw_body(*, offset: int, encoded_value: bytes, flags: int = 0) -> bytes:
    body = bytearray(protocol.READING_BODY_SIZE)
    body[offset : offset + len(encoded_value)] = encoded_value
    body[30:32] = flags.to_bytes(2, "little")
    return bytes(body)


# Resolves combined faults by the normative validation order, never a later stage.
@pytest.mark.parametrize(
    ("make_frame", "expected_result", "expected_ack"),
    (
        (
            lambda: _direct_authenticated_frame(control=0x30, body=bytes(33)),
            ProcessingResult.REJECTED_MALFORMED_LENGTH,
            AckSelection.NONE,
        ),
        (
            lambda: _direct_authenticated_frame(
                control=0x30,
                node_id=UNKNOWN_NODE_ID,
            ),
            ProcessingResult.UNKNOWN_NODE,
            AckSelection.NONE,
        ),
        (
            lambda: _direct_authenticated_frame(
                key=bytes.fromhex("fff9a1a0f386692e01028082be92330e"),
                control=0x30,
            ),
            ProcessingResult.AUTHENTICATION_FAILED,
            AckSelection.NONE,
        ),
        (
            lambda: _direct_authenticated_frame(
                control=0x30,
                domain=protocol.Domain.ACK_ACCEPTED_DOWNLINK,
                body=bytes(31),
            ),
            ProcessingResult.REJECTED_UNSUPPORTED_CONTROL,
            AckSelection.REJECTED_UNSUPPORTED,
        ),
        (
            lambda: _direct_authenticated_frame(
                domain=protocol.Domain.ACK_ACCEPTED_DOWNLINK,
                body=bytes(31),
            ),
            ProcessingResult.WRONG_DIRECTION,
            AckSelection.NONE,
        ),
        (
            lambda: _direct_authenticated_frame(domain=0, body=bytes(1)),
            ProcessingResult.REJECTED_UNSUPPORTED_DOMAIN,
            AckSelection.REJECTED_UNSUPPORTED,
        ),
        (
            lambda: _direct_authenticated_frame(
                body=_raw_body(offset=6, encoded_value=b"\x01\x00")
            ),
            ProcessingResult.REJECTED_MALFORMED_BODY,
            AckSelection.REJECTED_MALFORMED,
        ),
    ),
)
def test_combined_faults_stop_at_the_first_applicable_validation_stage(
    make_frame: Callable[[], bytes],
    expected_result: ProcessingResult,
    expected_ack: AckSelection,
) -> None:
    decision = _ingress().begin(ingress_packet(frame=make_frame()))

    assert decision.pre_tx_profile.processing_result is expected_result
    assert decision.pre_tx_profile.ack_selected is expected_ack


# Prevents every untrusted AAD region from selecting an ACK or pre-TX admission.
@pytest.mark.parametrize("aad_index", (0, 1, 2, 10))
def test_tampered_authenticated_data_never_reaches_ack_or_queue_admission(
    aad_index: int,
) -> None:
    ingress = _ingress()
    auth_map_before = dict(ingress._auth_node_keys)

    decision = ingress.begin(
        ingress_packet(
            frame=_changed_byte(
                authenticated_frame(),
                aad_index,
            )
        )
    )

    assert decision.pre_tx_profile.processing_result in {
        ProcessingResult.UNKNOWN_NODE,
        ProcessingResult.AUTHENTICATION_FAILED,
    }
    assert decision.pre_tx_profile.ack_selected is AckSelection.NONE
    assert decision.pre_tx_profile.ack_frame is None
    assert decision.pre_tx_profile.decoded_sample_id is None
    assert decision.candidate is None
    assert decision.admission is None
    assert dict(ingress._auth_node_keys) == auth_map_before
    assert ingress._queue.snapshot().reserved_entities == 0
    assert ingress._queue.snapshot().published_entities == 0


# Rejects short and over-maximum clear-frame encodings before node or crypto use.
@pytest.mark.parametrize(
    "frame",
    (b"", bytes(1), bytes(9), bytes(13), bytes(22), bytes(55), bytes(255)),
)
def test_unusable_frame_lengths_remain_silent_and_unadmitted(frame: bytes) -> None:
    decision = _ingress().begin(ingress_packet(frame=frame))

    assert decision.pre_tx_profile.processing_result is (
        ProcessingResult.REJECTED_MALFORMED_LENGTH
    )
    assert decision.pre_tx_profile.ack_selected is AckSelection.NONE
    assert not decision.pre_tx_profile.header_authenticated
    assert decision.candidate is None
    assert decision.admission is None


# Gives wrong keys, modified ciphertext and modified tags the same silent outcome.
@pytest.mark.parametrize(
    "frame",
    (
        _direct_authenticated_frame(
            key=bytes.fromhex("c1f9a1a0f386692e01028082be92330e")
        ),
        _changed_byte(authenticated_frame(), protocol.CLEAR_HEADER_SIZE),
        _changed_byte(authenticated_frame(), protocol.READING_FRAME_SIZE - 1),
    ),
)
def test_authentication_failures_are_silent_without_candidate_reservation(
    frame: bytes,
) -> None:
    ingress = _ingress()

    decision = ingress.begin(ingress_packet(frame=frame))

    assert decision.pre_tx_profile.processing_result is (
        ProcessingResult.AUTHENTICATION_FAILED
    )
    assert decision.pre_tx_profile.ack_selected is AckSelection.NONE
    assert not decision.pre_tx_profile.header_authenticated
    assert decision.pre_tx_profile.decoded_sample_id is None
    assert decision.candidate is None
    assert decision.admission is None
    assert ingress._queue.snapshot().reserved_entities == 0


MALFORMED_BODY_CASES = (
    ("soil_0_without_valid", _raw_body(offset=6, encoded_value=b"\x01\x00")),
    ("soil_1_without_valid", _raw_body(offset=8, encoded_value=b"\x01\x00")),
    ("soil_temp_0_without_valid", _raw_body(offset=10, encoded_value=b"\x01\x00")),
    ("soil_temp_1_without_valid", _raw_body(offset=12, encoded_value=b"\x01\x00")),
    ("enclosure_temp_without_valid", _raw_body(offset=14, encoded_value=b"\x01\x00")),
    ("pressure_without_valid", _raw_body(offset=16, encoded_value=b"\x01\x00\x00\x00")),
    ("humidity_without_valid", _raw_body(offset=20, encoded_value=b"\x01\x00")),
    ("deep_sleep_flag_without_reason", _raw_body(offset=0, encoded_value=b"", flags=1)),
    ("deep_sleep_reason_without_flag", _raw_body(offset=22, encoded_value=b"\x08")),
    ("previous_current_attempts_without_metrics", _raw_body(offset=23, encoded_value=b"\x01")),
    ("previous_awake_without_metrics", _raw_body(offset=24, encoded_value=b"\x01\x00")),
    ("previous_delivery_without_metrics", _raw_body(offset=26, encoded_value=b"\x01\x00")),
    ("previous_cycle_attempts_without_metrics", _raw_body(offset=28, encoded_value=b"\x01")),
    ("previous_accepted_count_without_metrics", _raw_body(offset=29, encoded_value=b"\x01")),
    (
        "previous_accepted_flag_without_metrics",
        _raw_body(offset=0, encoded_value=b"", flags=1 << 9),
    ),
    (
        "delivery_without_accepted_flag",
        _raw_body(offset=26, encoded_value=b"\x01\x00", flags=1 << 8),
    ),
) + tuple(
    (
        f"reserved_flag_{bit}",
        _raw_body(offset=0, encoded_value=b"", flags=1 << bit),
    )
    for bit in range(10, 16)
)


# Exercises every constrained reading field and flag as authenticated malformed input.
@pytest.mark.parametrize(
    ("case_name", "body"),
    MALFORMED_BODY_CASES,
    ids=tuple(case[0] for case in MALFORMED_BODY_CASES),
)
def test_every_structural_reading_constraint_selects_malformed_profile(
    case_name: str,
    body: bytes,
) -> None:
    del case_name
    decision = _ingress().begin(
        ingress_packet(frame=authenticated_frame(body=body))
    )

    assert decision.pre_tx_profile.processing_result is (
        ProcessingResult.REJECTED_MALFORMED_BODY
    )
    assert decision.pre_tx_profile.header_authenticated
    assert decision.pre_tx_profile.decoded_sample_id is None
    assert decision.pre_tx_profile.ack_selected is AckSelection.REJECTED_MALFORMED
    assert decision.pre_tx_profile.ack_frame == REVIEWED_REJECTED_MALFORMED_ACK
    assert decision.candidate is None
    assert decision.admission is not None
    assert decision.admission.result is AdmissionResult.RESERVED


# Rejects every authenticated non-reading body length with the exact malformed ACK.
@pytest.mark.parametrize("body_length", (1, 2, 31))
def test_authenticated_reading_body_length_is_exact(body_length: int) -> None:
    decision = _ingress().begin(
        ingress_packet(frame=authenticated_frame(body=bytes(body_length)))
    )

    assert decision.pre_tx_profile.processing_result is (
        ProcessingResult.REJECTED_MALFORMED_LENGTH
    )
    assert decision.pre_tx_profile.header_authenticated
    assert decision.pre_tx_profile.ack_selected is AckSelection.REJECTED_MALFORMED
    assert decision.pre_tx_profile.ack_frame == REVIEWED_REJECTED_MALFORMED_ACK
    assert decision.pre_tx_profile.decoded_sample_id is None


# Keeps every downlink ACK domain silent under all persistence admission conditions.
@pytest.mark.parametrize("domain", tuple(protocol.ACK_STATUS_BY_DOMAIN))
@pytest.mark.parametrize(
    ("state", "full", "expected_admission"),
    (
        (PersistenceAdmissionState.AVAILABLE, False, AdmissionResult.RESERVED),
        (PersistenceAdmissionState.AVAILABLE, True, AdmissionResult.QUEUE_FULL),
    )
    + tuple(
        (state, False, AdmissionResult.PERSISTENCE_UNAVAILABLE)
        for state in PersistenceAdmissionState
        if state is not PersistenceAdmissionState.AVAILABLE
    ),
)
def test_wrong_direction_is_silent_under_every_queue_state_with_at_most_one_profile_attempt(
    domain: protocol.Domain,
    state: PersistenceAdmissionState,
    full: bool,
    expected_admission: AdmissionResult,
) -> None:
    queue = _queue(state=state, full=full)
    ingress = _ingress(queue)
    body = protocol.encode_ack(
        protocol.Ack(status=protocol.ACK_STATUS_BY_DOMAIN[domain].value)
    )
    decision = ingress.begin(
        ingress_packet(frame=authenticated_frame(domain=domain, body=body))
    )
    occupied_before = queue.snapshot().published_entities

    assert decision.pre_tx_profile.processing_result is ProcessingResult.WRONG_DIRECTION
    assert decision.pre_tx_profile.ack_selected is AckSelection.NONE
    assert decision.admission is None

    finalized = ingress.finalize(
        decision,
        ProtocolIngressTerminalV1(
            ack_tx_result=AckTxResult.NOT_APPLICABLE,
            t4_set_tx_attempted_monotonic_us=None,
            t5_tx_done_monotonic_us=None,
            t6_set_rx_issued_monotonic_us=21,
        ),
    )

    assert finalized.admission is not None
    assert finalized.admission.result is expected_admission
    assert decision.pre_tx_profile.processing_result is ProcessingResult.WRONG_DIRECTION
    assert decision.pre_tx_profile.ack_selected is AckSelection.NONE
    assert queue.snapshot().published_entities == occupied_before + int(
        expected_admission is AdmissionResult.RESERVED
    )


# Selects the exact permanent rejection for unsupported authenticated control values.
@pytest.mark.parametrize("control", (0x00, 0x10, 0x21, 0x2F, 0x30, 0xFF))
def test_unsupported_authenticated_control_selects_complete_profile(
    control: int,
) -> None:
    ingress = _ingress()
    decision = ingress.begin(
        ingress_packet(frame=authenticated_frame(control=control))
    )

    assert decision.pre_tx_profile.processing_result is (
        ProcessingResult.REJECTED_UNSUPPORTED_CONTROL
    )
    assert decision.pre_tx_profile.ack_selected is AckSelection.REJECTED_UNSUPPORTED
    assert decision.pre_tx_profile.ack_frame == REVIEWED_REJECTED_UNSUPPORTED_ACK
    finalized = ingress.finalize(
        decision,
        ProtocolIngressTerminalV1(
            ack_tx_result=AckTxResult.TX_DONE,
            t4_set_tx_attempted_monotonic_us=21,
            t5_tx_done_monotonic_us=22,
            t6_set_rx_issued_monotonic_us=23,
        ),
    )
    assert isinstance(finalized.published_entity, ProfileOnlyUnitV1)
    profile = finalized.published_entity.profile
    assert profile.processing_result is ProcessingResult.REJECTED_UNSUPPORTED_CONTROL
    assert profile.claimed_control == control
    assert profile.header_authenticated
    assert profile.decoded_sample_id is None
    assert profile.ack_frame == REVIEWED_REJECTED_UNSUPPORTED_ACK


# Selects unsupported before reading parsing for every authenticated unknown domain.
@pytest.mark.parametrize("domain", (0x00, 0x07, 0x7F, 0xFF))
def test_unsupported_authenticated_domain_selects_exact_rejection(domain: int) -> None:
    decision = _ingress().begin(
        ingress_packet(frame=authenticated_frame(domain=domain, body=bytes(1)))
    )

    assert decision.pre_tx_profile.processing_result is (
        ProcessingResult.REJECTED_UNSUPPORTED_DOMAIN
    )
    assert decision.pre_tx_profile.ack_selected is AckSelection.REJECTED_UNSUPPORTED
    assert decision.pre_tx_profile.ack_frame == REVIEWED_REJECTED_UNSUPPORTED_ACK
    assert decision.pre_tx_profile.decoded_sample_id is None
