from __future__ import annotations

import struct
from typing import Any

import pytest


READING_STRUCT = struct.Struct("<HHHhhhIHBBHHBBH")
READING_FIELDS = (
    "run_ms",
    "soil_0_mv",
    "soil_1_mv",
    "soil_temp_0_centi_c",
    "soil_temp_1_centi_c",
    "enclosure_centi_c",
    "enclosure_pressure_pa",
    "enclosure_humidity_centi_pct",
    "reset_reason",
    "previous_current_tx_attempts",
    "previous_awake_ms",
    "previous_current_delivery_ms",
    "previous_cycle_tx_attempts",
    "previous_cycle_accepted_readings",
    "flags",
)

DEEP_SLEEP_BOOT = 1 << 0
PREVIOUS_CYCLE_METRICS_VALID = 1 << 8
PREVIOUS_CURRENT_ACCEPTED = 1 << 9

READING_DOMAINS = (1, 2)
ACK_DOMAIN_STATUS = {
    3: 0,
    4: 1,
    5: 2,
    6: 3,
}

VALIDITY_CASES = (
    ("soil_0_mv", 1, 1),
    ("soil_1_mv", 2, 1),
    ("soil_temp_0_centi_c", 3, -1),
    ("soil_temp_1_centi_c", 4, -1),
    ("enclosure_centi_c", 5, -1),
    ("enclosure_pressure_pa", 6, 1),
    ("enclosure_humidity_centi_pct", 7, 1),
)

PREVIOUS_METRIC_FIELDS = (
    "previous_current_tx_attempts",
    "previous_awake_ms",
    "previous_current_delivery_ms",
    "previous_cycle_tx_attempts",
    "previous_cycle_accepted_readings",
)


def _fields(reading_vector: dict[str, Any]) -> dict[str, int]:
    return {
        key: value
        for key, value in reading_vector["reading"].items()
        if key != "encoded_hex"
    }


def _pack_reading_without_validation(fields: dict[str, int]) -> bytes:
    return READING_STRUCT.pack(*(fields[name] for name in READING_FIELDS))


def _assert_reading_rejected(codec: Any, fields: dict[str, int]) -> None:
    with pytest.raises(ValueError):
        codec.encode_reading(fields)

    malformed_body = _pack_reading_without_validation(fields)
    with pytest.raises(ValueError):
        codec.decode_reading(malformed_body)


def _cold_previous_cycle(fields: dict[str, int]) -> dict[str, int]:
    fields = fields.copy()
    fields["flags"] &= ~(
        PREVIOUS_CYCLE_METRICS_VALID | PREVIOUS_CURRENT_ACCEPTED
    )
    for field in PREVIOUS_METRIC_FIELDS:
        fields[field] = 0
    return fields


@pytest.mark.parametrize("length", (0, 13, 15))
def test_rejects_invalid_header_length(codec: Any, length: int) -> None:
    with pytest.raises(ValueError):
        codec.decode_header(bytes(length))


@pytest.mark.parametrize("length", (0, 27, 29))
def test_rejects_invalid_reading_length(codec: Any, length: int) -> None:
    with pytest.raises(ValueError):
        codec.decode_reading(bytes(length))


@pytest.mark.parametrize("length", (0, 2))
def test_rejects_invalid_ack_length(codec: Any, length: int) -> None:
    with pytest.raises(ValueError):
        codec.decode_ack(bytes(length))


@pytest.mark.parametrize(
    "control",
    (
        pytest.param(0x00, id="reserved-version-zero"),
        pytest.param(0x10, id="version-one"),
        pytest.param(0x21, id="pilot-flags-nonzero"),
        pytest.param(0x30, id="version-three"),
        pytest.param(0xFF, id="all-bits-set"),
    ),
)
def test_reports_nonpilot_control_as_unsupported(
    codec: Any,
    control: int,
) -> None:
    assert not codec.is_supported_control(control)


def test_reports_unknown_domain_as_neither_reading_nor_ack(
    codec: Any,
) -> None:
    assert not codec.domain_is_reading(77)
    assert not codec.domain_is_ack(77)


@pytest.mark.parametrize("domain", ACK_DOMAIN_STATUS)
def test_reports_ack_domain_as_not_reading(codec: Any, domain: int) -> None:
    assert not codec.domain_is_reading(domain)


@pytest.mark.parametrize("domain", READING_DOMAINS)
def test_reports_reading_domain_as_not_ack(codec: Any, domain: int) -> None:
    assert not codec.domain_is_ack(domain)


@pytest.mark.parametrize(
    ("field", "flag_bit", "nonzero_value"),
    VALIDITY_CASES,
    ids=(case[0] for case in VALIDITY_CASES),
)
def test_rejects_nonzero_measurement_without_valid_flag(
    codec: Any,
    reading_vector: dict[str, Any],
    field: str,
    flag_bit: int,
    nonzero_value: int,
) -> None:
    fields = _fields(reading_vector)
    fields[field] = nonzero_value
    fields["flags"] &= ~(1 << flag_bit)

    _assert_reading_rejected(codec, fields)


def test_rejects_deep_sleep_flag_with_other_reset_reason(
    codec: Any,
    reading_vector: dict[str, Any],
) -> None:
    fields = _fields(reading_vector)
    fields["flags"] |= DEEP_SLEEP_BOOT
    fields["reset_reason"] = 1

    _assert_reading_rejected(codec, fields)


def test_rejects_deep_sleep_reset_reason_without_flag(
    codec: Any,
    reading_vector: dict[str, Any],
) -> None:
    fields = _fields(reading_vector)
    fields["flags"] &= ~DEEP_SLEEP_BOOT
    fields["reset_reason"] = 8

    _assert_reading_rejected(codec, fields)


@pytest.mark.parametrize("field", PREVIOUS_METRIC_FIELDS)
def test_rejects_previous_metric_without_metrics_valid_flag(
    codec: Any,
    reading_vector: dict[str, Any],
    field: str,
) -> None:
    fields = _cold_previous_cycle(_fields(reading_vector))
    fields[field] = 1

    _assert_reading_rejected(codec, fields)


def test_rejects_previous_accepted_without_metrics_valid(
    codec: Any,
    reading_vector: dict[str, Any],
) -> None:
    fields = _cold_previous_cycle(_fields(reading_vector))
    fields["flags"] |= PREVIOUS_CURRENT_ACCEPTED

    _assert_reading_rejected(codec, fields)


def test_rejects_delivery_time_without_previous_accepted(
    codec: Any,
    reading_vector: dict[str, Any],
) -> None:
    fields = _cold_previous_cycle(_fields(reading_vector))
    fields["flags"] |= PREVIOUS_CYCLE_METRICS_VALID
    fields["previous_current_delivery_ms"] = 1

    _assert_reading_rejected(codec, fields)


@pytest.mark.parametrize("reserved_bit", range(10, 16))
def test_rejects_each_reserved_reading_flag(
    codec: Any,
    reading_vector: dict[str, Any],
    reserved_bit: int,
) -> None:
    fields = _fields(reading_vector)
    fields["flags"] |= 1 << reserved_bit

    _assert_reading_rejected(codec, fields)


@pytest.mark.parametrize("status", (4, 255))
def test_rejects_unknown_ack_status(codec: Any, status: int) -> None:
    with pytest.raises(ValueError):
        codec.encode_ack({"status": status})

    with pytest.raises(ValueError):
        codec.decode_ack(bytes((status,)))


@pytest.mark.parametrize(
    ("domain", "status"),
    tuple(
        (domain, status)
        for domain, expected_status in ACK_DOMAIN_STATUS.items()
        for status in (*range(4), 4, 255)
        if status != expected_status
    ),
)
def test_rejects_ack_domain_status_mismatch(
    codec: Any,
    domain: int,
    status: int,
) -> None:
    assert not codec.ack_status_matches_domain(domain, status)


@pytest.mark.parametrize(
    ("domain", "status"),
    tuple(
        (domain, status)
        for domain in READING_DOMAINS
        for status in range(4)
    ),
)
def test_rejects_ack_status_with_reading_domain(
    codec: Any,
    domain: int,
    status: int,
) -> None:
    assert not codec.ack_status_matches_domain(domain, status)


@pytest.mark.parametrize("status", range(4))
def test_rejects_ack_status_with_unknown_domain(
    codec: Any,
    status: int,
) -> None:
    assert not codec.ack_status_matches_domain(77, status)
