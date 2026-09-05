from __future__ import annotations

from dataclasses import FrozenInstanceError

import pytest

from cura_receiver.generated import protocol_v2_lora_generated as protocol
from cura_receiver.generated.receiver_enums_generated import (
    AckSelection,
    AdmissionResult,
    PersistenceAdmissionState,
    PersistQueueEntityKind,
    ProcessingResult,
)
from cura_receiver.persist_queue import PersistQueue, PersistenceAdmissionSnapshot
from cura_receiver.protocol_ingress import ProtocolIngress
from tests.support.builders.protocol_ingress import (
    REVIEWED_ACCEPTED_ACK,
    REVIEWED_BACKLOG_FRAME,
    REVIEWED_CURRENT_FRAME,
    REVIEWED_MESSAGE_ID,
    REVIEWED_NODE_ID,
    REVIEWED_NODE_KEY,
    REVIEWED_READING_BODY,
    REVIEWED_SAMPLE_ID,
    ingress_packet,
)
from tests.support.fakes.os_clock import FakeOsClock


def _available_ingress() -> ProtocolIngress:
    queue = PersistQueue(capacity_entities=1)
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
        auth_node_keys={REVIEWED_NODE_ID: REVIEWED_NODE_KEY},
    )


# Processes each reviewed reading domain through the complete production pre-TX path.
@pytest.mark.parametrize(
    ("frame", "domain"),
    (
        (REVIEWED_CURRENT_FRAME, protocol.Domain.CURRENT_READING_UPLINK),
        (REVIEWED_BACKLOG_FRAME, protocol.Domain.BACKLOG_READING_UPLINK),
    ),
)
def test_reviewed_reading_paths_preserve_transport_application_and_profile_facts(
    frame: bytes,
    domain: protocol.Domain,
) -> None:
    decision = _available_ingress().begin(ingress_packet(frame=frame))
    pre_tx = decision.pre_tx_profile

    assert pre_tx.processing_result is ProcessingResult.ACCEPTED
    assert pre_tx.ack_selected is AckSelection.ACCEPTED
    assert pre_tx.ack_frame == REVIEWED_ACCEPTED_ACK
    assert pre_tx.claimed_control == protocol.CONTROL
    assert pre_tx.claimed_domain == domain
    assert pre_tx.claimed_node_id == REVIEWED_NODE_ID
    assert pre_tx.claimed_message_id == REVIEWED_MESSAGE_ID
    assert pre_tx.header_authenticated
    assert pre_tx.decoded_sample_id == REVIEWED_SAMPLE_ID
    assert pre_tx.received_frame_length == protocol.READING_FRAME_SIZE
    assert pre_tx.received_frame[: protocol.READING_FRAME_SIZE] == frame
    assert pre_tx.received_frame[protocol.READING_FRAME_SIZE :] == bytes(
        255 - protocol.READING_FRAME_SIZE
    )
    assert pre_tx.t3_authentication_completed_monotonic_us == 20

    assert decision.candidate is not None
    assert decision.candidate.node_id == REVIEWED_NODE_ID
    assert decision.candidate.message_id == REVIEWED_MESSAGE_ID
    assert decision.candidate.domain == domain
    assert decision.candidate.sample_id == REVIEWED_SAMPLE_ID
    assert decision.candidate.reading_body == REVIEWED_READING_BODY
    assert decision.admission is not None
    assert decision.admission.entity_kind is PersistQueueEntityKind.MEASUREMENT_PROFILE
    assert decision.admission.result is AdmissionResult.RESERVED
    assert not hasattr(pre_tx, "__dict__")
    with pytest.raises(FrozenInstanceError):
        pre_tx.processing_result = ProcessingResult.UNKNOWN_NODE  # type: ignore[misc]


# Limits current-versus-backlog differences to input domain/frame evidence only.
def test_reviewed_domains_select_the_same_deterministic_accepted_ack() -> None:
    current = _available_ingress().begin(
        ingress_packet(frame=REVIEWED_CURRENT_FRAME)
    )
    backlog = _available_ingress().begin(
        ingress_packet(frame=REVIEWED_BACKLOG_FRAME)
    )

    assert current.pre_tx_profile.ack_frame == backlog.pre_tx_profile.ack_frame
    assert current.pre_tx_profile.processing_result is (
        backlog.pre_tx_profile.processing_result
    )
    assert current.candidate is not None
    assert backlog.candidate is not None
    assert current.candidate.message_id == backlog.candidate.message_id
    assert current.candidate.sample_id == backlog.candidate.sample_id
    assert current.candidate.reading_body == backlog.candidate.reading_body
    assert current.candidate.domain != backlog.candidate.domain
    assert current.pre_tx_profile.received_frame != backlog.pre_tx_profile.received_frame
