"""Independent observable validation-order model for protocol ingress tests."""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Mapping

from cryptography.exceptions import InvalidTag
from cryptography.hazmat.primitives.ciphers.aead import AESCCM


_CONTROL = 0x20
_CLEAR_HEADER_SIZE = 14
_TAG_SIZE = 8
_MIN_FRAME_SIZE = _CLEAR_HEADER_SIZE + _TAG_SIZE
_MAX_FRAME_SIZE = 54
_READING_BODY_SIZE = 32
_READING_DOMAINS = frozenset({1, 2})
_ACK_DOMAINS = frozenset({3, 4, 5, 6})
_READING_STRUCT = struct.Struct("<IHHHhhhIHBBHHBBH")
_ACK_STATUS_BY_SELECTION = {
    "ACCEPTED": (3, 0),
    "RETRY_LATER": (4, 1),
    "REJECTED_UNSUPPORTED": (5, 2),
    "REJECTED_MALFORMED": (6, 3),
}


@dataclass(frozen=True, slots=True)
class ReferenceIngressDecision:
    processing_result: str
    ack_selected: str
    ack_frame: bytes | None
    claimed_control: int | None
    claimed_domain: int | None
    claimed_node_id: bytes | None
    claimed_message_id: int | None
    header_authenticated: bool
    decoded_sample_id: int | None
    candidate: tuple[bytes, int, int, int, bytes] | None
    admission_kind: str | None
    admission_result: str | None


def _reading_is_valid(body: bytes) -> bool:
    (
        _sample_id,
        _run_ms,
        soil_0_mv,
        soil_1_mv,
        soil_temp_0_centi_c,
        soil_temp_1_centi_c,
        enclosure_centi_c,
        enclosure_pressure_pa,
        enclosure_humidity_centi_pct,
        reset_reason,
        previous_current_tx_attempts,
        previous_awake_ms,
        previous_current_delivery_ms,
        previous_cycle_tx_attempts,
        previous_cycle_accepted_readings,
        flags,
    ) = _READING_STRUCT.unpack(body)

    validity_pairs = (
        (1 << 1, soil_0_mv),
        (1 << 2, soil_1_mv),
        (1 << 3, soil_temp_0_centi_c),
        (1 << 4, soil_temp_1_centi_c),
        (1 << 5, enclosure_centi_c),
        (1 << 6, enclosure_pressure_pa),
        (1 << 7, enclosure_humidity_centi_pct),
    )
    if any(not flags & flag and value != 0 for flag, value in validity_pairs):
        return False
    if bool(flags & (1 << 0)) != (reset_reason == 8):
        return False

    previous_metrics = (
        previous_current_tx_attempts,
        previous_awake_ms,
        previous_current_delivery_ms,
        previous_cycle_tx_attempts,
        previous_cycle_accepted_readings,
    )
    if not flags & (1 << 8) and any(previous_metrics):
        return False
    if flags & (1 << 9) and not flags & (1 << 8):
        return False
    if not flags & (1 << 9) and previous_current_delivery_ms != 0:
        return False
    return not flags & 0xFC00


def _build_ack(
    *,
    node_key: bytes,
    node_id: bytes,
    message_id: int,
    ack_selected: str,
) -> bytes:
    ack_domain, status = _ACK_STATUS_BY_SELECTION[ack_selected]
    header = struct.pack("<BB8sI", _CONTROL, ack_domain, node_id, message_id)
    nonce = node_id + struct.pack("<I", message_id) + bytes((ack_domain,))
    return header + AESCCM(node_key, tag_length=_TAG_SIZE).encrypt(
        nonce,
        bytes((status,)),
        header,
    )


def classify_protocol_ingress(
    *,
    frame: bytes,
    auth_node_keys: Mapping[bytes, bytes],
    admission_result: str,
) -> ReferenceIngressDecision:
    """Return the modeled first result and one pre-response admission decision."""

    if admission_result not in {
        "RESERVED",
        "PERSISTENCE_UNAVAILABLE",
        "QUEUE_FULL",
    }:
        raise ValueError("unknown modeled admission result")

    claimed_control = frame[0] if len(frame) >= 1 else None
    claimed_domain = frame[1] if len(frame) >= 2 else None
    claimed_node_id = frame[2:10] if len(frame) >= 10 else None
    claimed_message_id = (
        int.from_bytes(frame[10:14], "little") if len(frame) >= 14 else None
    )
    header_authenticated = False
    decoded_sample_id: int | None = None
    candidate: tuple[bytes, int, int, int, bytes] | None = None
    node_key: bytes | None = None

    if not _MIN_FRAME_SIZE <= len(frame) <= _MAX_FRAME_SIZE:
        processing_result = "REJECTED_MALFORMED_LENGTH"
        ack_selected = "NONE"
    elif claimed_node_id not in auth_node_keys:
        processing_result = "UNKNOWN_NODE"
        ack_selected = "NONE"
    else:
        assert claimed_node_id is not None
        assert claimed_control is not None
        assert claimed_domain is not None
        assert claimed_message_id is not None
        node_key = auth_node_keys[claimed_node_id]
        header = frame[:_CLEAR_HEADER_SIZE]
        nonce = (
            claimed_node_id
            + struct.pack("<I", claimed_message_id)
            + bytes((claimed_domain,))
        )
        try:
            body = AESCCM(node_key, tag_length=_TAG_SIZE).decrypt(
                nonce,
                frame[_CLEAR_HEADER_SIZE:],
                header,
            )
        except InvalidTag:
            processing_result = "AUTHENTICATION_FAILED"
            ack_selected = "NONE"
        else:
            header_authenticated = True
            if claimed_control != _CONTROL:
                processing_result = "REJECTED_UNSUPPORTED_CONTROL"
                ack_selected = "REJECTED_UNSUPPORTED"
            elif claimed_domain in _ACK_DOMAINS:
                processing_result = "WRONG_DIRECTION"
                ack_selected = "NONE"
            elif claimed_domain not in _READING_DOMAINS:
                processing_result = "REJECTED_UNSUPPORTED_DOMAIN"
                ack_selected = "REJECTED_UNSUPPORTED"
            elif len(body) != _READING_BODY_SIZE:
                processing_result = "REJECTED_MALFORMED_LENGTH"
                ack_selected = "REJECTED_MALFORMED"
            elif not _reading_is_valid(body):
                processing_result = "REJECTED_MALFORMED_BODY"
                ack_selected = "REJECTED_MALFORMED"
            else:
                decoded_sample_id = int.from_bytes(body[:4], "little")
                processing_result = "ACCEPTED"
                ack_selected = "ACCEPTED"
                candidate = (
                    claimed_node_id,
                    claimed_message_id,
                    claimed_domain,
                    decoded_sample_id,
                    body,
                )

    admission_kind: str | None = None
    applied_admission: str | None = None
    if ack_selected != "NONE":
        admission_kind = (
            "MEASUREMENT_PROFILE"
            if processing_result == "ACCEPTED"
            else "PROFILE_ONLY"
        )
        applied_admission = admission_result
        if admission_result != "RESERVED":
            processing_result = (
                "RETRY_LATER_PERSISTENCE_UNAVAILABLE"
                if admission_result == "PERSISTENCE_UNAVAILABLE"
                else "RETRY_LATER_QUEUE_FULL"
            )
            ack_selected = "RETRY_LATER"
            candidate = None

    ack_frame = None
    if ack_selected != "NONE":
        assert node_key is not None
        assert claimed_node_id is not None
        assert claimed_message_id is not None
        ack_frame = _build_ack(
            node_key=node_key,
            node_id=claimed_node_id,
            message_id=claimed_message_id,
            ack_selected=ack_selected,
        )

    return ReferenceIngressDecision(
        processing_result=processing_result,
        ack_selected=ack_selected,
        ack_frame=ack_frame,
        claimed_control=claimed_control,
        claimed_domain=claimed_domain,
        claimed_node_id=claimed_node_id,
        claimed_message_id=claimed_message_id,
        header_authenticated=header_authenticated,
        decoded_sample_id=decoded_sample_id,
        candidate=candidate,
        admission_kind=admission_kind,
        admission_result=applied_admission,
    )


__all__ = ["ReferenceIngressDecision", "classify_protocol_ingress"]
