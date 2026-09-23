from __future__ import annotations

from typing import Any

import pytest


def _node_key(vectors: dict[str, Any]) -> bytes:
    return bytes.fromhex(vectors["crypto"]["node_key_hex"])


def _header_without_encoding(vector: dict[str, Any]) -> dict[str, Any]:
    return {
        key: value
        for key, value in vector["clear_header"].items()
        if key != "encoded_hex"
    }


def _assert_golden_frame(
    frame_crypto: Any,
    vectors: dict[str, Any],
    vector: dict[str, Any],
    body_name: str,
) -> None:
    key = _node_key(vectors)
    header = _header_without_encoding(vector)
    body = bytes.fromhex(vector[body_name]["encoded_hex"])
    expected = bytes.fromhex(vector["encrypted"]["frame_hex"])

    assert frame_crypto.seal_frame(key, header, body) == expected
    assert frame_crypto.seal_frame(key, header, body) == expected
    assert frame_crypto.open_frame(key, expected) == (header, body)

    encoded_header = bytes.fromhex(vector["clear_header"]["encoded_hex"])
    ciphertext = bytes.fromhex(vector["encrypted"]["ciphertext_hex"])
    tag = bytes.fromhex(vector["encrypted"]["tag_hex"])
    assert expected == encoded_header + ciphertext + tag


def test_golden_encrypted_reading(
    frame_crypto: Any,
    golden_vectors: dict[str, Any],
    reading_vector: dict[str, Any],
) -> None:
    _assert_golden_frame(
        frame_crypto,
        golden_vectors,
        reading_vector,
        "reading",
    )


def test_golden_encrypted_ack(
    frame_crypto: Any,
    golden_vectors: dict[str, Any],
    ack_vector: dict[str, Any],
) -> None:
    _assert_golden_frame(
        frame_crypto,
        golden_vectors,
        ack_vector,
        "ack",
    )


def test_rejects_wrong_key(
    frame_crypto: Any,
    golden_vectors: dict[str, Any],
    reading_vector: dict[str, Any],
) -> None:
    frame = bytes.fromhex(reading_vector["encrypted"]["frame_hex"])
    wrong_key = bytearray(_node_key(golden_vectors))
    wrong_key[0] ^= 0x80

    with pytest.raises(ValueError):
        frame_crypto.open_frame(bytes(wrong_key), frame)


def test_rejects_every_tampered_reading_frame_byte(
    frame_crypto: Any,
    golden_vectors: dict[str, Any],
    reading_vector: dict[str, Any],
) -> None:
    key = _node_key(golden_vectors)
    frame = bytes.fromhex(reading_vector["encrypted"]["frame_hex"])

    for index in range(len(frame)):
        tampered = bytearray(frame)
        tampered[index] ^= 0x01
        with pytest.raises(ValueError):
            frame_crypto.open_frame(key, bytes(tampered))


@pytest.mark.parametrize("body_size", (0, 33))
def test_rejects_invalid_plaintext_body_size(
    frame_crypto: Any,
    golden_vectors: dict[str, Any],
    reading_vector: dict[str, Any],
    body_size: int,
) -> None:
    with pytest.raises(ValueError):
        frame_crypto.seal_frame(
            _node_key(golden_vectors),
            _header_without_encoding(reading_vector),
            bytes(body_size),
        )


@pytest.mark.parametrize("frame_size", (0, 22, 55))
def test_rejects_invalid_frame_size(
    frame_crypto: Any,
    golden_vectors: dict[str, Any],
    frame_size: int,
) -> None:
    with pytest.raises(ValueError):
        frame_crypto.open_frame(
            _node_key(golden_vectors),
            bytes(frame_size),
        )


@pytest.mark.parametrize("key_size", (0, 15, 17))
def test_rejects_invalid_key_size(
    frame_crypto: Any,
    reading_vector: dict[str, Any],
    key_size: int,
) -> None:
    header = _header_without_encoding(reading_vector)
    body = bytes.fromhex(reading_vector["reading"]["encoded_hex"])
    frame = bytes.fromhex(reading_vector["encrypted"]["frame_hex"])

    with pytest.raises(ValueError):
        frame_crypto.seal_frame(bytes(key_size), header, body)
    with pytest.raises(ValueError):
        frame_crypto.open_frame(bytes(key_size), frame)


def test_domains_separate_current_and_backlog_frames(
    frame_crypto: Any,
    golden_vectors: dict[str, Any],
    reading_vector: dict[str, Any],
) -> None:
    key = _node_key(golden_vectors)
    current_header = _header_without_encoding(reading_vector)
    backlog_header = current_header | {"domain": 2}
    body = bytes.fromhex(reading_vector["reading"]["encoded_hex"])

    current_frame = frame_crypto.seal_frame(key, current_header, body)
    backlog_frame = frame_crypto.seal_frame(key, backlog_header, body)

    assert current_frame != backlog_frame
    assert frame_crypto.open_frame(key, current_frame) == (
        current_header,
        body,
    )
    assert frame_crypto.open_frame(key, backlog_frame) == (
        backlog_header,
        body,
    )
