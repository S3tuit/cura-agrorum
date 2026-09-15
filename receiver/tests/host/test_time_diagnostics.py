from dataclasses import replace
import struct

import pytest

from cura_receiver.generated import receiver_enums_generated as E
from cura_receiver.time_diagnostics import (
    BackendStatus,
    ReceiverTimeEpisodeContextV1,
    TimeFailureEpisode,
    TimeFailureLatch,
    decode_time_context,
    encode_time_context,
    time_diagnostic,
)


def context(**kwargs):
    return replace(
        ReceiverTimeEpisodeContextV1(
            E.TimeComponent.CHRONY, E.TimeFailureStage.QUERY_TRACKING, 9
        ),
        **kwargs
    )


# Literal wire image fixes every offset, absent-field padding and the mandatory duration.
def test_context_exact_encoding():
    expected = (
        bytes.fromhex("00000000 01 02 00000000 00000000 0000")
        + bytes(56)
        + bytes.fromhex("0900000000000000")
    )
    assert len(expected) == 80
    assert encode_time_context(context()) == expected
    assert decode_time_context(expected) == context()


# Every present field, including zero status, survives the prescribed little-endian projection.
def test_context_present_fields_and_status_zero():
    value = context(
        primary_status=BackendStatus(E.TimeBackendStatusKind.ADJTIMEX_RETURN, 0),
        secondary_status=BackendStatus(E.TimeBackendStatusKind.DS3231_READ_STATUS, 4),
        quality=(E.SystemTimeQuality.NETWORK_SYNCED, E.SystemTimeQuality.UNTRUSTED),
        rtc_health=(E.RtcHealth.PRESENT, E.RtcHealth.MISSING),
        flags=255,
        os_errno=5,
        kernel_status_bits=0x2040,
        clock_state_generation=11,
        operation_generation=12,
        related_clock_observation_sequence=13,
        observed_value_us=-14,
        comparison_value_us=15,
        threshold_us=16,
    )
    data = encode_time_context(value)
    assert data[:4] == b"\xff\x0f\x00\x00"
    assert data[6:10] == b"\x06\x00\x03\x04"
    assert data[16:24] == b"\x05\x00\x00\x00\x40\x20\x00\x00"
    assert data[48:56] == bytes.fromhex("f2ffffffffffffff")
    assert decode_time_context(data) == value


# Reserved bits, undefined enums/statuses and nonzero absent fields never decode.
@pytest.mark.parametrize(
    "offset,byte",
    [
        (2, 1),
        (4, 0),
        (4, 255),
        (5, 0),
        (5, 255),
        (6, 1),
        (7, 1),
        (8, 1),
        (10, 1),
        (12, 1),
        (15, 1),
        (16, 1),
        (20, 1),
        (24, 1),
        (32, 1),
        (40, 1),
        (48, 1),
        (56, 1),
        (64, 1),
    ],
)
def test_reject_noncanonical_context(offset, byte):
    data = bytearray(encode_time_context(context()))
    data[offset] = byte
    with pytest.raises((ValueError, TypeError)):
        decode_time_context(bytes(data))


# Status-kind assignments are closed, with zero valid only for the two specified families.
@pytest.mark.parametrize(
    "kind,valid",
    [
        (1, {1, 2, 3, 4}),
        (2, {1, 2, 3}),
        (3, {1, 2, 3, 4, 5}),
        (4, {1, 2, 3}),
        (5, {0, 1, 2, 3}),
        (6, {0, 1, 2, 3, 4, 5}),
    ],
)
def test_status_assignment_matrix(kind, valid):
    for number in range(256):
        if number in valid:
            assert BackendStatus(E.TimeBackendStatusKind(kind), number).value == number
        else:
            with pytest.raises(ValueError):
                BackendStatus(E.TimeBackendStatusKind(kind), number)


# The normative operation/error matrix accepts exactly these pairs and fixes the factory envelope.
@pytest.mark.parametrize(
    "code,operations",
    [
        ("IO", {"INITIALIZE", "READ", "SYNC"}),
        ("DEADLINE", {"READ", "SYNC"}),
        ("INVALID_RESPONSE", {"INITIALIZE", "READ", "SYNC"}),
        ("COMMAND_REJECTED", {"SYNC"}),
        ("OUTCOME_UNKNOWN", {"SYNC"}),
        ("CLOCK_INTERFERENCE", {"READ", "VALIDATE"}),
        ("RTC_READBACK_MISMATCH", {"SYNC"}),
        ("CALCULATION_RANGE", {"VALIDATE", "SYNC"}),
    ],
)
def test_diagnostic_operation_matrix(code, operations):
    for operation in E.DiagnosticOperation:
        if operation.name not in operations:
            with pytest.raises(ValueError):
                TimeFailureEpisode(
                    operation, E.TimeDiagnosticErrorCode[code], 12, context()
                )
            continue
        failure = TimeFailureEpisode(
            operation, E.TimeDiagnosticErrorCode[code], 12, context()
        )
        diagnostic = time_diagnostic(
            failure, receiver_instance_id=bytes(16), diagnostic_sequence=1
        )
        assert diagnostic.severity is E.DiagnosticSeverity.ERROR
        assert diagnostic.error_domain is E.DiagnosticErrorDomain.TIME
        assert diagnostic.context_length == 80 and diagnostic.context_schema == 1
        assert diagnostic.context[80:] == bytes(48)
        assert diagnostic.sampled_at_monotonic_us == 12


# Episode completion keeps the original trigger even if read-back exposes another failure.
def test_episode_freezes_first_trigger():
    first = TimeFailureEpisode(
        E.DiagnosticOperation.SYNC,
        E.TimeDiagnosticErrorCode.OUTCOME_UNKNOWN,
        15,
        context(
            primary_status=BackendStatus(
                E.TimeBackendStatusKind.DS3231_WRITE_DISPOSITION, 3
            )
        ),
    )
    final = first.finish(
        started_at_monotonic_us=10,
        finished_at_monotonic_us=40,
        secondary_status=BackendStatus(E.TimeBackendStatusKind.DS3231_READ_STATUS, 4),
    )
    assert final.error_code is first.error_code
    assert final.sampled_at_monotonic_us == 15
    assert final.context.primary_status == first.context.primary_status
    assert final.context.operation_duration_us == 30
    assert first.context.operation_duration_us == 9
    with pytest.raises(ValueError):
        first.finish(started_at_monotonic_us=40, finished_at_monotonic_us=39)


# Identical consecutive failures suppress emission; success or a changed signature restarts it.
def test_periodic_failure_latch():
    latch = TimeFailureLatch()
    first = TimeFailureEpisode(
        E.DiagnosticOperation.READ, E.TimeDiagnosticErrorCode.DEADLINE, 10, context()
    )
    assert latch.failed(first) is first
    assert latch.failed(replace(first, sampled_at_monotonic_us=11)) is None
    changed = replace(first, error_code=E.TimeDiagnosticErrorCode.IO)
    assert latch.failed(changed) is changed
    latch.succeeded()
    assert latch.failed(changed) is changed


# Invalid scalars, partial transition pairs and undefined flags fail before factory use.
@pytest.mark.parametrize(
    "changes",
    [
        {"flags": 256},
        {"flags": True},
        {"operation_duration_us": -1},
        {"quality": (E.SystemTimeQuality.UNTRUSTED,)},
        {"os_errno": 1 << 31},
        {"kernel_status_bits": 1 << 32},
        {"clock_state_generation": 1 << 64},
        {"observed_value_us": 1 << 63},
    ],
)
def test_context_rejects_invalid_values(changes):
    with pytest.raises((ValueError, TypeError)):
        context(**changes)
