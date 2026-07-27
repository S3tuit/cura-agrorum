from __future__ import annotations

from typing import Any


def _without_encoded_hex(fields: dict[str, Any]) -> dict[str, Any]:
    return {key: value for key, value in fields.items() if key != "encoded_hex"}


def test_golden_reading(codec: Any, reading_vector: dict[str, Any]) -> None:
    header = reading_vector["clear_header"]
    expected_header = bytes.fromhex(header["encoded_hex"])

    assert codec.encode_header(header) == expected_header
    assert codec.decode_header(expected_header) == _without_encoded_hex(header)
    assert codec.build_nonce(header) == bytes.fromhex(
        reading_vector["nonce_hex"]
    )

    reading = reading_vector["reading"]
    expected_reading = bytes.fromhex(reading["encoded_hex"])

    assert codec.encode_reading(reading) == expected_reading
    assert codec.decode_reading(expected_reading) == _without_encoded_hex(
        reading
    )


def test_golden_ack(codec: Any, ack_vector: dict[str, Any]) -> None:
    header = ack_vector["clear_header"]
    expected_header = bytes.fromhex(header["encoded_hex"])

    assert codec.encode_header(header) == expected_header
    assert codec.decode_header(expected_header) == _without_encoded_hex(header)
    assert codec.build_nonce(header) == bytes.fromhex(ack_vector["nonce_hex"])

    ack = ack_vector["ack"]
    expected_ack = bytes.fromhex(ack["encoded_hex"])

    assert codec.encode_ack(ack) == expected_ack
    assert codec.decode_ack(expected_ack) == _without_encoded_hex(ack)
    assert codec.ack_status_matches_domain(header["domain"], ack["status"])
