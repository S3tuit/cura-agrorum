"""Authenticated reception evidence; raw bytes never establish message authority."""

from .generated import protocol_v2_lora_generated as protocol
from .generated import receiver_entities_generated as rows
from .generated.receiver_enums_generated import (
    AckTxResult, PacketExceptionReason as Reason,
    PersistenceClassification as Classification, ProcessingResult,
)

_REJECTIONS = {
    ProcessingResult.REJECTED_UNSUPPORTED_CONTROL: Reason.REJECTED_UNSUPPORTED_CONTROL,
    ProcessingResult.REJECTED_UNSUPPORTED_DOMAIN: Reason.REJECTED_UNSUPPORTED_DOMAIN,
    ProcessingResult.REJECTED_MALFORMED_LENGTH: Reason.REJECTED_MALFORMED_LENGTH,
    ProcessingResult.REJECTED_MALFORMED_BODY: Reason.REJECTED_MALFORMED_BODY,
    ProcessingResult.WRONG_DIRECTION: Reason.WRONG_DIRECTION,
}
_ADMITTED_OUTCOMES = frozenset(_REJECTIONS) | {
    ProcessingResult.ACCEPTED, ProcessingResult.RETRY_LATER_QUEUE_FULL,
    ProcessingResult.RETRY_LATER_PERSISTENCE_UNAVAILABLE,
}


def validate_reception(profile, received_frame):
    if type(profile) is not rows.PacketReceptionV1:
        raise TypeError("invalid reception type")
    if type(profile.processing_result) is not ProcessingResult or profile.processing_result not in _ADMITTED_OUTCOMES:
        raise ValueError("only authenticated processing outcomes can be persisted")
    if type(profile.ack_tx_result) is not AckTxResult:
        raise TypeError("invalid ACK TX outcome")
    if type(received_frame) is not bytes or not 22 <= len(received_frame) <= 255:
        raise ValueError("invalid unpadded authenticated frame")
    header = protocol.decode_clear_header(received_frame[:14])
    if (header.node_id, header.message_id, header.domain) != (profile.node_id, profile.message_id, profile.domain):
        raise ValueError("reception identity differs from authenticated frame")
    if type(profile.node_id) is not bytes or len(profile.node_id) != 8:
        raise ValueError("invalid authenticated node identity")
    if type(profile.message_id) is not int or type(profile.domain) is not int:
        raise TypeError("invalid authenticated message identity")
    if profile.processing_result is ProcessingResult.WRONG_DIRECTION:
        if profile.ack_tx_result is not AckTxResult.NOT_APPLICABLE:
            raise ValueError("wrong-direction reception cannot have a response")
    elif profile.ack_tx_result is AckTxResult.NOT_APPLICABLE:
        raise ValueError("response-eligible reception needs a terminal TX outcome")


def reception_exception(profile, received_frame, classification):
    validate_reception(profile, received_frame)
    reason = _REJECTIONS.get(profile.processing_result)
    if classification is Classification.DUPLICATE_CONFLICT:
        reason = Reason.DUPLICATE_CONFLICT
    elif classification is Classification.MESSAGE_ID_CONFLICT:
        reason = Reason.MESSAGE_ID_CONFLICT
    if reason is None:
        return None
    return rows.PacketReceptionExceptionRowV1(
        profile.receiver_instance_id, profile.occurrence_sequence,
        len(received_frame), received_frame, reason,
    )
