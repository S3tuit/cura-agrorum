from __future__ import annotations

from collections.abc import Callable

import pytest

from cura_receiver.generated import protocol_v2_lora_generated as protocol
from cura_receiver.generated.receiver_enums_generated import (
    AckSelection,
    PersistenceAdmissionState,
    ProcessingResult,
)
from cura_receiver.persist_queue import (
    PersistQueue,
    PersistenceAdmissionSnapshot,
)
from cura_receiver.protocol_ingress import ProtocolIngress, ProtocolIngressPacketV1
from tests.support.builders.protocol_ingress import (
    REVIEWED_ACCEPTED_ACK as EXPECTED_ACCEPTED_ACK,
    REVIEWED_MESSAGE_ID,
    REVIEWED_NODE_ID as NODE_ID,
    REVIEWED_NODE_KEY as NODE_KEY,
    REVIEWED_READING_BODY as VALID_READING_BODY,
    REVIEWED_SAMPLE_ID,
    authenticated_frame as _frame,
    ingress_packet as _packet,
)
from tests.support.fakes.os_clock import FakeOsClock


def _ingress() -> ProtocolIngress:
    queue = PersistQueue(capacity_entities=4)
    queue.publish_admission_state(
        PersistenceAdmissionSnapshot(
            generation=1,
            state=PersistenceAdmissionState.AVAILABLE,
            changed_at_monotonic_us=1,
        )
    )
    return ProtocolIngress(
        queue=queue,
        monotonic_clock=FakeOsClock(monotonic_us=20, realtime_us=0),
        auth_node_keys={NODE_ID: NODE_KEY},
    )


def _tamper_tag(frame: bytes) -> bytes:
    changed = bytearray(frame)
    changed[-1] ^= 1
    return bytes(changed)


def _malformed_body() -> bytes:
    body = bytearray(VALID_READING_BODY)
    body[6] = 1
    body[30:32] = (0).to_bytes(2, "little")
    return bytes(body)


# Applies the protocol stages in order and exposes the first applicable result.
@pytest.mark.parametrize(
    (
        "make_frame",
        "expected_result",
        "expected_ack",
        "expected_authenticated",
        "expected_authentication_attempt",
    ),
    (
        (
            lambda: bytes(13),
            ProcessingResult.REJECTED_MALFORMED_LENGTH,
            AckSelection.NONE,
            False,
            False,
        ),
        (
            lambda: bytes(55),
            ProcessingResult.REJECTED_MALFORMED_LENGTH,
            AckSelection.NONE,
            False,
            False,
        ),
        (
            lambda: _frame(node_id=b"unknown!"),
            ProcessingResult.UNKNOWN_NODE,
            AckSelection.NONE,
            False,
            False,
        ),
        (
            lambda: _tamper_tag(_frame()),
            ProcessingResult.AUTHENTICATION_FAILED,
            AckSelection.NONE,
            False,
            True,
        ),
        (
            lambda: _frame(control=0x30),
            ProcessingResult.REJECTED_UNSUPPORTED_CONTROL,
            AckSelection.REJECTED_UNSUPPORTED,
            True,
            True,
        ),
        (
            lambda: _frame(
                domain=protocol.Domain.ACK_ACCEPTED_DOWNLINK,
                body=protocol.encode_ack(protocol.Ack(status=0)),
            ),
            ProcessingResult.WRONG_DIRECTION,
            AckSelection.NONE,
            True,
            True,
        ),
        (
            lambda: _frame(domain=0x7F),
            ProcessingResult.REJECTED_UNSUPPORTED_DOMAIN,
            AckSelection.REJECTED_UNSUPPORTED,
            True,
            True,
        ),
        (
            lambda: _frame(body=VALID_READING_BODY[:-1]),
            ProcessingResult.REJECTED_MALFORMED_LENGTH,
            AckSelection.REJECTED_MALFORMED,
            True,
            True,
        ),
        (
            lambda: _frame(body=_malformed_body()),
            ProcessingResult.REJECTED_MALFORMED_BODY,
            AckSelection.REJECTED_MALFORMED,
            True,
            True,
        ),
        (
            _frame,
            ProcessingResult.ACCEPTED,
            AckSelection.ACCEPTED,
            True,
            True,
        ),
    ),
)
def test_validation_order_selects_the_first_terminal_protocol_outcome(
    make_frame: Callable[[], bytes],
    expected_result: ProcessingResult,
    expected_ack: AckSelection,
    expected_authenticated: bool,
    expected_authentication_attempt: bool,
) -> None:
    decision = _ingress().begin(_packet(frame=make_frame()))
    pre_tx = decision.pre_tx_profile

    assert pre_tx.processing_result is expected_result
    assert pre_tx.ack_selected is expected_ack
    assert pre_tx.header_authenticated is expected_authenticated
    assert (pre_tx.ack_frame is not None) == (expected_ack is not AckSelection.NONE)
    assert (
        pre_tx.t3_authentication_completed_monotonic_us is not None
    ) is expected_authentication_attempt


# Reads application identity only after authenticated exact reading decoding succeeds.
def test_sample_identity_appears_only_after_complete_reading_validation() -> None:
    accepted = _ingress().begin(_packet(frame=_frame()))
    malformed = _ingress().begin(
        _packet(frame=_frame(body=_malformed_body()))
    )
    unauthenticated = _ingress().begin(
        _packet(frame=_tamper_tag(_frame()))
    )

    assert accepted.pre_tx_profile.decoded_sample_id == REVIEWED_SAMPLE_ID
    assert accepted.candidate is not None
    assert accepted.candidate.sample_id == REVIEWED_SAMPLE_ID
    assert malformed.pre_tx_profile.decoded_sample_id is None
    assert malformed.candidate is None
    assert unauthenticated.pre_tx_profile.decoded_sample_id is None
    assert unauthenticated.candidate is None


# Builds the reviewed deterministic ACK and keeps transport and sample identities distinct.
def test_accepted_candidate_and_ack_preserve_exact_protocol_identities() -> None:
    decision = _ingress().begin(_packet(frame=_frame()))

    assert decision.pre_tx_profile.ack_frame == EXPECTED_ACCEPTED_ACK
    assert decision.candidate is not None
    assert decision.candidate.node_id == NODE_ID
    assert decision.candidate.message_id == REVIEWED_MESSAGE_ID
    assert decision.candidate.sample_id == REVIEWED_SAMPLE_ID
    assert decision.candidate.domain == protocol.Domain.CURRENT_READING_UPLINK
    assert decision.candidate.reading_body == VALID_READING_BODY
