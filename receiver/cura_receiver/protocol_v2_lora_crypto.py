"""Authenticated frame construction for Cura Agrorum LoRa protocol v2.

The clear authenticated header carries transport ``message_id``. Application
``sample_id`` is present only in an authenticated, decoded reading body.

Wire contract: protocol/protocol-v2-lora/README.md, especially "Routine
packet", "CCM nonce", and "Receiver validation".
"""

from __future__ import annotations

from dataclasses import dataclass

from cryptography.exceptions import InvalidTag
from cryptography.hazmat.primitives.ciphers.aead import AESCCM

from .generated import protocol_v2_lora_generated as schema


MIN_BODY_SIZE = schema.ACK_BODY_SIZE
MAX_BODY_SIZE = schema.READING_BODY_SIZE
MIN_FRAME_SIZE = schema.CLEAR_HEADER_SIZE + MIN_BODY_SIZE + schema.TAG_SIZE
MAX_FRAME_SIZE = schema.CLEAR_HEADER_SIZE + MAX_BODY_SIZE + schema.TAG_SIZE


class CryptoError(ValueError):
    """The key, header, body, or frame cannot be processed."""


class AuthenticationError(CryptoError):
    """The frame did not authenticate under the supplied node key."""


@dataclass(frozen=True)
class AuthenticatedFrame:
    """A header and plaintext body trusted only after CCM authentication."""

    header: schema.ClearHeader
    plaintext_body: bytes


def _require_bytes(value: object, name: str) -> bytes:
    if not isinstance(value, bytes):
        raise CryptoError(f"{name} must be bytes")
    return value


def _require_node_key(node_key: object) -> bytes:
    key = _require_bytes(node_key, "node_key")
    if len(key) != schema.KEY_SIZE:
        raise CryptoError(
            f"node_key must be {schema.KEY_SIZE} bytes, got {len(key)}"
        )
    return key


def _require_body_size(size: int) -> None:
    if not MIN_BODY_SIZE <= size <= MAX_BODY_SIZE:
        raise CryptoError(
            f"plaintext body must be {MIN_BODY_SIZE}..{MAX_BODY_SIZE} bytes, "
            f"got {size}"
        )


def seal_frame(
    node_key: bytes,
    header: schema.ClearHeader,
    plaintext_body: bytes,
) -> bytes:
    """Return ``clear header || ciphertext || tag`` for one logical packet."""

    key = _require_node_key(node_key)
    body = _require_bytes(plaintext_body, "plaintext_body")
    _require_body_size(len(body))

    try:
        associated_data = schema.encode_clear_header(header)
        nonce = schema.build_nonce(header)
    except (AttributeError, schema.CodecError) as exc:
        raise CryptoError(f"invalid clear header: {exc}") from exc

    encrypted_body_and_tag = AESCCM(
        key,
        tag_length=schema.TAG_SIZE,
    ).encrypt(nonce, body, associated_data)
    return associated_data + encrypted_body_and_tag


def open_frame(node_key: bytes, frame: bytes) -> AuthenticatedFrame:
    """Authenticate a complete frame and return its trusted decoded contents."""

    key = _require_node_key(node_key)
    encoded_frame = _require_bytes(frame, "frame")
    if not MIN_FRAME_SIZE <= len(encoded_frame) <= MAX_FRAME_SIZE:
        raise CryptoError(
            f"frame must be {MIN_FRAME_SIZE}..{MAX_FRAME_SIZE} bytes, "
            f"got {len(encoded_frame)}"
        )

    associated_data = encoded_frame[: schema.CLEAR_HEADER_SIZE]
    encrypted_body_and_tag = encoded_frame[schema.CLEAR_HEADER_SIZE :]
    try:
        header = schema.decode_clear_header(associated_data)
        nonce = schema.build_nonce(header)
    except schema.CodecError as exc:
        raise CryptoError(f"invalid clear header: {exc}") from exc

    try:
        plaintext_body = AESCCM(
            key,
            tag_length=schema.TAG_SIZE,
        ).decrypt(nonce, encrypted_body_and_tag, associated_data)
    except InvalidTag as exc:
        raise AuthenticationError("frame authentication failed") from exc

    _require_body_size(len(plaintext_body))
    return AuthenticatedFrame(
        header=header,
        plaintext_body=plaintext_body,
    )
