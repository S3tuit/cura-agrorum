from __future__ import annotations

from typing import Any

from hypothesis import given, settings
from hypothesis import strategies as st


U8 = st.integers(min_value=0, max_value=(1 << 8) - 1)
U16 = st.integers(min_value=0, max_value=(1 << 16) - 1)
U32 = st.integers(min_value=0, max_value=(1 << 32) - 1)
I16 = st.integers(min_value=-(1 << 15), max_value=(1 << 15) - 1)

DEEP_SLEEP_BOOT = 1 << 0
SOIL_0_VALID = 1 << 1
SOIL_1_VALID = 1 << 2
SOIL_TEMP_0_VALID = 1 << 3
SOIL_TEMP_1_VALID = 1 << 4
ENCLOSURE_TEMP_VALID = 1 << 5
ENCLOSURE_PRESSURE_VALID = 1 << 6
ENCLOSURE_HUMIDITY_VALID = 1 << 7
PREVIOUS_CYCLE_METRICS_VALID = 1 << 8
PREVIOUS_CURRENT_ACCEPTED = 1 << 9


@st.composite
def headers(draw: st.DrawFn) -> dict[str, Any]:
    return {
        "control": draw(U8),
        "domain": draw(U8),
        "node_id_hex": draw(st.binary(min_size=8, max_size=8)).hex(),
        "sample_id": draw(U32),
    }


@st.composite
def valid_readings(draw: st.DrawFn) -> dict[str, int]:
    flags = 0

    deep_sleep_boot = draw(st.booleans())
    if deep_sleep_boot:
        reset_reason = 8
        flags |= DEEP_SLEEP_BOOT
    else:
        reset_reason = draw(U8.filter(lambda value: value != 8))

    soil_0_valid = draw(st.booleans())
    soil_0_mv = draw(U16) if soil_0_valid else 0
    if soil_0_valid:
        flags |= SOIL_0_VALID

    soil_1_valid = draw(st.booleans())
    soil_1_mv = draw(U16) if soil_1_valid else 0
    if soil_1_valid:
        flags |= SOIL_1_VALID

    soil_temp_0_valid = draw(st.booleans())
    soil_temp_0_centi_c = draw(I16) if soil_temp_0_valid else 0
    if soil_temp_0_valid:
        flags |= SOIL_TEMP_0_VALID

    soil_temp_1_valid = draw(st.booleans())
    soil_temp_1_centi_c = draw(I16) if soil_temp_1_valid else 0
    if soil_temp_1_valid:
        flags |= SOIL_TEMP_1_VALID

    enclosure_temp_valid = draw(st.booleans())
    enclosure_centi_c = draw(I16) if enclosure_temp_valid else 0
    if enclosure_temp_valid:
        flags |= ENCLOSURE_TEMP_VALID

    enclosure_pressure_valid = draw(st.booleans())
    enclosure_pressure_pa = draw(U32) if enclosure_pressure_valid else 0
    if enclosure_pressure_valid:
        flags |= ENCLOSURE_PRESSURE_VALID

    enclosure_humidity_valid = draw(st.booleans())
    enclosure_humidity_centi_pct = (
        draw(U16) if enclosure_humidity_valid else 0
    )
    if enclosure_humidity_valid:
        flags |= ENCLOSURE_HUMIDITY_VALID

    previous_metrics_valid = draw(st.booleans())
    if previous_metrics_valid:
        flags |= PREVIOUS_CYCLE_METRICS_VALID
        previous_current_tx_attempts = draw(U8)
        previous_awake_ms = draw(U16)
        previous_cycle_tx_attempts = draw(U8)
        previous_cycle_accepted_readings = draw(U8)

        previous_current_accepted = draw(st.booleans())
        if previous_current_accepted:
            flags |= PREVIOUS_CURRENT_ACCEPTED
            previous_current_delivery_ms = draw(U16)
        else:
            previous_current_delivery_ms = 0
    else:
        previous_current_tx_attempts = 0
        previous_awake_ms = 0
        previous_current_delivery_ms = 0
        previous_cycle_tx_attempts = 0
        previous_cycle_accepted_readings = 0

    return {
        "run_ms": draw(U16),
        "soil_0_mv": soil_0_mv,
        "soil_1_mv": soil_1_mv,
        "soil_temp_0_centi_c": soil_temp_0_centi_c,
        "soil_temp_1_centi_c": soil_temp_1_centi_c,
        "enclosure_centi_c": enclosure_centi_c,
        "enclosure_pressure_pa": enclosure_pressure_pa,
        "enclosure_humidity_centi_pct": enclosure_humidity_centi_pct,
        "reset_reason": reset_reason,
        "previous_current_tx_attempts": previous_current_tx_attempts,
        "previous_awake_ms": previous_awake_ms,
        "previous_current_delivery_ms": previous_current_delivery_ms,
        "previous_cycle_tx_attempts": previous_cycle_tx_attempts,
        "previous_cycle_accepted_readings": (
            previous_cycle_accepted_readings
        ),
        "flags": flags,
    }


@settings(max_examples=500, deadline=None)
@given(header=headers())
def test_header_implementations_agree_and_round_trip(
    codec_pair: tuple[Any, Any],
    header: dict[str, Any],
) -> None:
    python_codec, c_codec = codec_pair

    python_encoded = python_codec.encode_header(header)
    c_encoded = c_codec.encode_header(header)

    assert python_encoded == c_encoded
    assert len(python_encoded) == 14
    assert python_codec.decode_header(python_encoded) == header
    assert c_codec.decode_header(c_encoded) == header

    python_nonce = python_codec.build_nonce(header)
    c_nonce = c_codec.build_nonce(header)

    assert python_nonce == c_nonce
    assert len(python_nonce) == 13


@settings(max_examples=500, deadline=None)
@given(reading=valid_readings())
def test_valid_reading_implementations_agree_and_round_trip(
    codec_pair: tuple[Any, Any],
    reading: dict[str, int],
) -> None:
    python_codec, c_codec = codec_pair

    python_encoded = python_codec.encode_reading(reading)
    c_encoded = c_codec.encode_reading(reading)

    assert python_encoded == c_encoded
    assert len(python_encoded) == 28
    assert python_codec.decode_reading(python_encoded) == reading
    assert c_codec.decode_reading(c_encoded) == reading


@settings(max_examples=1000, deadline=None)
@given(body=st.binary(min_size=28, max_size=28))
def test_arbitrary_reading_body_acceptance_agrees(
    codec_pair: tuple[Any, Any],
    body: bytes,
) -> None:
    python_codec, c_codec = codec_pair

    try:
        python_reading = python_codec.decode_reading(body)
    except ValueError:
        python_result: tuple[str, dict[str, int] | None] = (
            "rejected",
            None,
        )
    else:
        python_result = ("accepted", python_reading)

    try:
        c_reading = c_codec.decode_reading(body)
    except ValueError:
        c_result: tuple[str, dict[str, int] | None] = ("rejected", None)
    else:
        c_result = ("accepted", c_reading)

    assert python_result == c_result

    if python_result[0] == "accepted":
        reading = python_result[1]
        assert reading is not None
        assert python_codec.encode_reading(reading) == body
        assert c_codec.encode_reading(reading) == body
