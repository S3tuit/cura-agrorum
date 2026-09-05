from __future__ import annotations

from cura_receiver.generated import protocol_v2_lora_generated as protocol
from cura_receiver.generated.receiver_enums_generated import (
    AckTxResult,
    PersistenceAdmissionState,
    RadioState,
)
from cura_receiver.persist_queue import (
    PersistQueue,
    PersistQueueBatchDisposition,
    PersistenceAdmissionSnapshot,
)
from cura_receiver.protocol_ingress import (
    ProtocolIngress,
    ProtocolIngressOccurrenceV1,
    ProtocolIngressTerminalV1,
)
from cura_receiver.protocol_v2_lora_crypto import open_frame, seal_frame
from tests.support.builders.protocol_ingress import (
    REVIEWED_ACCEPTED_ACK,
    REVIEWED_CURRENT_FRAME,
    REVIEWED_MESSAGE_ID,
    REVIEWED_NODE_ID,
    REVIEWED_NODE_KEY,
    REVIEWED_READING_BODY,
    authenticated_frame,
    ingress_packet,
)
from tests.support.fakes.os_clock import FakeOsClock


def _context() -> tuple[PersistQueue, ProtocolIngress]:
    queue = PersistQueue(capacity_entities=1)
    queue.publish_admission_state(
        PersistenceAdmissionSnapshot(
            generation=1,
            state=PersistenceAdmissionState.AVAILABLE,
            changed_at_monotonic_us=1,
        )
    )
    ingress = ProtocolIngress(
        queue=queue,
        monotonic_clock=FakeOsClock(monotonic_us=20, realtime_us=0),
        auth_node_keys={REVIEWED_NODE_ID: REVIEWED_NODE_KEY},
    )
    return queue, ingress


def _finish_and_drain(
    queue: PersistQueue,
    ingress: ProtocolIngress,
    decision: ProtocolIngressOccurrenceV1,
) -> None:
    ingress.finalize(
        decision,
        ProtocolIngressTerminalV1(
            ack_tx_result=AckTxResult.SUPPRESSED_AIRTIME_BUDGET,
            t4_set_tx_attempted_monotonic_us=None,
            t5_tx_done_monotonic_us=None,
            t6_set_rx_issued_monotonic_us=21,
            radio_state=RadioState.RX_SINGLE,
        ),
    )
    lease = queue.claim_batch(max_entities=1)
    assert lease is not None
    lease.acknowledge_durable((PersistQueueBatchDisposition.SQLITE_COMMITTED,))


def _reading_body_with_sample(sample_id: int) -> bytes:
    return sample_id.to_bytes(4, "little") + REVIEWED_READING_BODY[4:]


# Reconstructs an identical ACK after unrelated traffic without retaining history.
def test_ack_reconstruction_is_stateless_across_repeated_and_unrelated_messages() -> None:
    queue, ingress = _context()

    first = ingress.begin(
        ingress_packet(frame=REVIEWED_CURRENT_FRAME, occurrence_sequence=1)
    )
    first_ack = first.pre_tx_profile.ack_frame
    _finish_and_drain(queue, ingress, first)

    unrelated = ingress.begin(
        ingress_packet(
            frame=authenticated_frame(
                message_id=0xA0B0C0D0,
                body=_reading_body_with_sample(0x01020304),
            ),
            occurrence_sequence=2,
        )
    )
    assert unrelated.pre_tx_profile.ack_frame != first_ack
    _finish_and_drain(queue, ingress, unrelated)

    repeated = ingress.begin(
        ingress_packet(frame=REVIEWED_CURRENT_FRAME, occurrence_sequence=3)
    )

    assert first_ack == repeated.pre_tx_profile.ack_frame == REVIEWED_ACCEPTED_ACK
    assert set(vars(ingress)) == {
        "_queue",
        "_monotonic_clock",
        "_auth_node_keys",
        "_active_occurrence",
    }
    assert ingress._active_occurrence is repeated
    _finish_and_drain(queue, ingress, repeated)
    assert ingress._active_occurrence is None


# Derives authentication and ACK nonces only from transport identity and domain.
def test_nonce_identity_excludes_deliberately_different_sample_values() -> None:
    queue, ingress = _context()
    message_id = 0xA1B2C3D4
    sample_ids = (0x01020304, 0xF1E2D3C4)
    ack_frames: list[bytes] = []

    for occurrence_sequence, sample_id in enumerate(sample_ids, start=1):
        body = _reading_body_with_sample(sample_id)
        uplink = authenticated_frame(message_id=message_id, body=body)
        uplink_header = protocol.decode_clear_header(
            uplink[: protocol.CLEAR_HEADER_SIZE]
        )

        assert protocol.build_nonce(uplink_header) == (
            REVIEWED_NODE_ID
            + message_id.to_bytes(4, "little")
            + bytes((protocol.Domain.CURRENT_READING_UPLINK,))
        )
        decision = ingress.begin(
            ingress_packet(
                frame=uplink,
                occurrence_sequence=occurrence_sequence,
            )
        )
        assert decision.candidate is not None
        assert decision.candidate.message_id == message_id
        assert decision.candidate.sample_id == sample_id
        assert decision.pre_tx_profile.ack_frame is not None
        ack_frames.append(decision.pre_tx_profile.ack_frame)
        _finish_and_drain(queue, ingress, decision)

    assert ack_frames[0] == ack_frames[1]
    authenticated_ack = open_frame(REVIEWED_NODE_KEY, ack_frames[0])
    assert authenticated_ack.header == protocol.ClearHeader(
        control=protocol.CONTROL,
        domain=protocol.Domain.ACK_ACCEPTED_DOWNLINK,
        node_id=REVIEWED_NODE_ID,
        message_id=message_id,
    )
    assert protocol.build_nonce(authenticated_ack.header) == (
        REVIEWED_NODE_ID
        + message_id.to_bytes(4, "little")
        + bytes((protocol.Domain.ACK_ACCEPTED_DOWNLINK,))
    )
    assert protocol.decode_ack(authenticated_ack.plaintext_body).status == (
        protocol.AckStatus.ACCEPTED
    )


# Changes ACK bytes when transport identity changes even if sample identity does not.
def test_ack_nonce_tracks_message_id_instead_of_sample_id() -> None:
    sample_id = 0x10203040
    first_message_id = 0x01020304
    second_message_id = 0x05060708

    first = seal_frame(
        REVIEWED_NODE_KEY,
        protocol.ClearHeader(
            control=protocol.CONTROL,
            domain=protocol.Domain.ACK_ACCEPTED_DOWNLINK,
            node_id=REVIEWED_NODE_ID,
            message_id=first_message_id,
        ),
        protocol.encode_ack(protocol.Ack(status=protocol.AckStatus.ACCEPTED)),
    )
    second = seal_frame(
        REVIEWED_NODE_KEY,
        protocol.ClearHeader(
            control=protocol.CONTROL,
            domain=protocol.Domain.ACK_ACCEPTED_DOWNLINK,
            node_id=REVIEWED_NODE_ID,
            message_id=second_message_id,
        ),
        protocol.encode_ack(protocol.Ack(status=protocol.AckStatus.ACCEPTED)),
    )

    assert sample_id not in {first_message_id, second_message_id}
    assert first != second
    assert first[10:14] == first_message_id.to_bytes(4, "little")
    assert second[10:14] == second_message_id.to_bytes(4, "little")
    assert sample_id.to_bytes(4, "little") not in {first[10:14], second[10:14]}
