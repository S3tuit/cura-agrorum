from __future__ import annotations

import base64
from dataclasses import fields
from enum import Enum
import json

import pytest

from cura_receiver.generated.receiver_entities_generated import (
    ClockObservationV1,
    DiagnosticV1,
    MessageProfilingV1,
)
from cura_receiver.generated.protocol_v2_lora_generated import Domain
from cura_receiver.generated.receiver_enums_generated import (
    AckSelection,
    AckTxResult,
    DiagnosticErrorDomain,
    DiagnosticOperation,
    DiagnosticSeverity,
    ProcessingResult,
    RadioState,
    RtcHealth,
    SystemTimeQuality,
)
from cura_receiver.persist_queue_entities import (
    CLOCK_OBSERVATION_V1_SPEC,
    DIAGNOSTIC_V1_SPEC,
    MEASUREMENT_PROFILE_V1_SPEC,
    PROFILE_ONLY_V1_SPEC,
    RECEIVER_HEALTH_REQUEST_V1_SPEC,
    AuthenticatedReadingCandidateV1,
    MeasurementProfileUnitV1,
    ProfileOnlyUnitV1,
    ReceiverHealthRequestV1,
)
from cura_receiver.quarantine_evidence import (
    QUARANTINE_EVIDENCE_FORMAT,
    QUARANTINE_EVIDENCE_MAX_BYTES,
    QUARANTINE_EVIDENCE_MAX_DEPTH,
    QUARANTINE_EVIDENCE_MAX_INTEGER_DIGITS,
    QUARANTINE_EVIDENCE_MAX_NODES,
    QUARANTINE_EVIDENCE_MAX_SCALAR_BYTES,
    QUARANTINE_EVIDENCE_MAX_TUPLE_ITEMS,
    QUARANTINE_EVIDENCE_VERSION,
    NeutralBoolV1,
    NeutralBytesV1,
    NeutralEnumV1,
    NeutralIntV1,
    NeutralNoneV1,
    NeutralRecordV1,
    NeutralStringV1,
    NeutralTupleV1,
    QuarantineEvidenceDecodeError,
    QuarantineEvidenceEncodeError,
    decode_quarantine_evidence_v1,
    encode_quarantine_evidence_v1,
    quarantine_evidence_sha256,
)


def _profile() -> MessageProfilingV1:
    return MessageProfilingV1(
        receiver_instance_id=b"r" * 16,
        occurrence_sequence=1,
        received_at_monotonic_us=2,
        received_frame_length=3,
        received_frame=b"abc" + bytes(252),
        claimed_control=32,
        claimed_domain=1,
        claimed_node_id=b"n" * 8,
        claimed_message_id=5,
        header_authenticated=True,
        decoded_sample_id=6,
        rssi_dbm_x2=-140,
        snr_db_x4=12,
        irq_status=7,
        device_errors=0,
        processing_result=ProcessingResult.ACCEPTED,
        ack_selected=AckSelection.ACCEPTED,
        ack_tx_result=AckTxResult.TX_DONE,
        ack_frame=b"a" * 23,
        busy_wait_total_us=8,
        busy_wait_max_us=9,
        busy_wait_count=10,
        busy_timeout_count=0,
        last_busy_timeout_opcode=None,
        t1_handler_started_monotonic_us=11,
        t2_packet_copied_monotonic_us=12,
        t3_authentication_completed_monotonic_us=13,
        t4_set_tx_attempted_monotonic_us=14,
        t5_tx_done_monotonic_us=15,
        t6_set_rx_issued_monotonic_us=16,
    )


def _health_request() -> ReceiverHealthRequestV1:
    return ReceiverHealthRequestV1(
        receiver_instance_id=b"r" * 16,
        health_sequence=1,
        communicator_sampled_at_monotonic_us=2,
        radio_state=RadioState.RX_SINGLE,
        radio_recovery_attempts=3,
        radio_recovery_successes=2,
        radio_recovery_failures=1,
        radio_recovery_attempts_by_reason=(0,) * 8,
        system_time_quality=SystemTimeQuality.NETWORK_SYNCED,
        rtc_health=RtcHealth.PRESENT,
        time_quality_transition_count=4,
        rtc_health_transition_count=5,
        last_time_quality_transition_monotonic_us=6,
        last_rtc_health_transition_monotonic_us=None,
        chrony_step_command_results=(1, 2, 3),
        rtc_write_results=(4, 5, 6),
        rtc_write_readback_verified_count=7,
        rtc_write_trust_invalidated_count=8,
        persist_queue_admission_counts=((0, 0, 0),) * 5,
    )


def _queue_entities() -> tuple[tuple[object, object], ...]:
    profile = _profile()
    return (
        (
            MeasurementProfileUnitV1(
                candidate=AuthenticatedReadingCandidateV1(
                    node_id=b"n" * 8,
                    message_id=1,
                    domain=1,
                    sample_id=2,
                    reading_body=b"b" * 32,
                ),
                profile=profile,
            ),
            MEASUREMENT_PROFILE_V1_SPEC,
        ),
        (ProfileOnlyUnitV1(profile=profile), PROFILE_ONLY_V1_SPEC),
        (_health_request(), RECEIVER_HEALTH_REQUEST_V1_SPEC),
        (
            DiagnosticV1(
                receiver_instance_id=b"r" * 16,
                diagnostic_sequence=1,
                sampled_at_monotonic_us=2,
                severity=DiagnosticSeverity.ERROR,
                error_domain=DiagnosticErrorDomain.RADIO,
                operation=DiagnosticOperation.VALIDATE,
                error_code=3,
                context_schema=0,
                context_length=0,
                context=bytes(128),
            ),
            DIAGNOSTIC_V1_SPEC,
        ),
        (
            ClockObservationV1(
                receiver_instance_id=b"r" * 16,
                observation_sequence=1,
                clock_state_generation=2,
                sampled_at_monotonic_us=3,
                sampled_at_utc_us=None,
                step_discontinuity_boundary=False,
                system_time_quality=SystemTimeQuality.UNTRUSTED,
                rtc_health=RtcHealth.PRESENT,
            ),
            CLOCK_OBSERVATION_V1_SPEC,
        ),
    )


# Every admitted V1 shape has deterministic bytes and decodes without reconstruction.
@pytest.mark.parametrize(("entity", "spec"), _queue_entities())
def test_every_queue_shape_encodes_deterministically_and_decodes_neutrally(
    entity: object,
    spec: object,
) -> None:
    encoded = encode_quarantine_evidence_v1(entity, spec=spec)

    assert encoded == encode_quarantine_evidence_v1(entity, spec=spec)
    assert encoded == json.dumps(
        json.loads(encoded),
        ensure_ascii=True,
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("ascii")
    evidence = decode_quarantine_evidence_v1(encoded)
    assert evidence.format == QUARANTINE_EVIDENCE_FORMAT
    assert evidence.format_version == QUARANTINE_EVIDENCE_VERSION
    assert evidence.entity_kind == spec.kind.value  # type: ignore[attr-defined]
    assert evidence.entity_schema_version == spec.schema_version  # type: ignore[attr-defined]
    assert isinstance(evidence.value, NeutralRecordV1)
    assert not isinstance(evidence.value, type(entity))
    assert quarantine_evidence_sha256(encoded) == __import__("hashlib").sha256(
        encoded
    ).digest()


# Supported scalar tags preserve malformed types, lengths and large integers exactly.
def test_wrong_supported_types_lengths_and_large_integers_remain_exact() -> None:
    large_integer = -(1 << 700)
    candidate = AuthenticatedReadingCandidateV1(
        node_id=b"short",
        message_id=large_integer,
        domain="wrong-type",  # type: ignore[arg-type]
        sample_id=True,
        reading_body=b"",
    )
    unit = MeasurementProfileUnitV1(candidate=candidate, profile=_profile())

    evidence = decode_quarantine_evidence_v1(
        encode_quarantine_evidence_v1(unit, spec=MEASUREMENT_PROFILE_V1_SPEC)
    )
    assert isinstance(evidence.value, NeutralRecordV1)
    candidate_node = evidence.value.fields[0].value
    assert isinstance(candidate_node, NeutralRecordV1)
    candidate_fields = {field.name: field.value for field in candidate_node.fields}
    assert candidate_fields == {
        "node_id": NeutralBytesV1(b"short"),
        "message_id": NeutralIntV1(large_integer),
        "domain": NeutralStringV1("wrong-type"),
        "sample_id": NeutralBoolV1(True),
        "reading_body": NeutralBytesV1(b""),
    }


# Neutral nodes retain runtime category distinctions needed by later analysis.
def test_neutral_tree_keeps_none_enum_tuple_and_record_tags_distinct() -> None:
    evidence = decode_quarantine_evidence_v1(
        encode_quarantine_evidence_v1(
            _health_request(), spec=RECEIVER_HEALTH_REQUEST_V1_SPEC
        )
    )
    assert isinstance(evidence.value, NeutralRecordV1)
    values = {field.name: field.value for field in evidence.value.fields}
    assert values["last_rtc_health_transition_monotonic_us"] == NeutralNoneV1()
    assert values["radio_state"] == NeutralEnumV1(
        class_name="cura_receiver.generated.receiver_enums_generated.RadioState",
        member_name="RX_SINGLE",
        value=2,
    )
    assert values["chrony_step_command_results"] == NeutralTupleV1(
        (NeutralIntV1(1), NeutralIntV1(2), NeutralIntV1(3))
    )


ALLOWED_ENUM_MEMBERS = tuple(
    member
    for enum_type in (
        Domain,
        AckSelection,
        AckTxResult,
        DiagnosticErrorDomain,
        DiagnosticOperation,
        DiagnosticSeverity,
        ProcessingResult,
        RadioState,
        RtcHealth,
        SystemTimeQuality,
    )
    for member in enum_type
)


# Every fixed enum assignment accepted by the encoder is accepted neutrally by
# the decoder.
@pytest.mark.parametrize("member", ALLOWED_ENUM_MEMBERS)
def test_every_allowlisted_enum_assignment_round_trips(member: Enum) -> None:
    evidence = decode_quarantine_evidence_v1(
        encode_quarantine_evidence_v1(
            ProfileOnlyUnitV1(profile=member),  # type: ignore[arg-type]
            spec=PROFILE_ONLY_V1_SPEC,
        )
    )
    assert isinstance(evidence.value, NeutralRecordV1)
    assert len(evidence.value.fields) == 1
    assert evidence.value.fields[0].name == "profile"
    enum_node = evidence.value.fields[0].value
    assert enum_node == NeutralEnumV1(
        class_name=f"{type(member).__module__}.{type(member).__qualname__}",
        member_name=member.name,
        value=member.value,  # type: ignore[arg-type]
    )


# Mutable, unbounded or ambiguous value categories fail the closed encoder grammar.
@pytest.mark.parametrize(
    "unsupported",
    (
        [1, 2],
        {"field": 1},
        1.5,
        bytearray(b"mutable"),
        memoryview(b"mutable-view"),
    ),
)
def test_encoder_rejects_values_outside_the_closed_algebra(
    unsupported: object,
) -> None:
    unit = ProfileOnlyUnitV1(profile=unsupported)  # type: ignore[arg-type]

    with pytest.raises(QuarantineEvidenceEncodeError, match="unsupported"):
        encode_quarantine_evidence_v1(unit, spec=PROFILE_ONLY_V1_SPEC)


# Deliberately corrupted frozen records cannot induce recursive evidence encoding.
def test_encoder_rejects_cycles() -> None:
    unit = ProfileOnlyUnitV1(profile=_profile())
    object.__setattr__(unit, "profile", unit)

    with pytest.raises(QuarantineEvidenceEncodeError, match="cycle"):
        encode_quarantine_evidence_v1(unit, spec=PROFILE_ONLY_V1_SPEC)


# Each scalar and collection bound is enforced before canonical evidence is returned.
@pytest.mark.parametrize(
    "oversized",
    (
        "x" * (QUARANTINE_EVIDENCE_MAX_SCALAR_BYTES + 1),
        b"x" * (QUARANTINE_EVIDENCE_MAX_SCALAR_BYTES + 1),
        10**QUARANTINE_EVIDENCE_MAX_INTEGER_DIGITS,
        tuple(range(QUARANTINE_EVIDENCE_MAX_TUPLE_ITEMS + 1)),
    ),
)
def test_encoder_rejects_scalar_and_tuple_bounds(oversized: object) -> None:
    unit = ProfileOnlyUnitV1(profile=oversized)  # type: ignore[arg-type]

    with pytest.raises(QuarantineEvidenceEncodeError, match="limit"):
        encode_quarantine_evidence_v1(unit, spec=PROFILE_ONLY_V1_SPEC)


# Aggregate depth, node-count and document-size limits fail closed independently.
def test_encoder_rejects_depth_node_and_total_output_bounds(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    nested: object = 0
    for _ in range(40):
        nested = (nested,)
    too_deep = ProfileOnlyUnitV1(profile=nested)  # type: ignore[arg-type]
    with pytest.raises(QuarantineEvidenceEncodeError, match="depth limit"):
        encode_quarantine_evidence_v1(too_deep, spec=PROFILE_ONLY_V1_SPEC)

    many_nodes = tuple((0, 1, 2, 3) for _ in range(1_024))
    too_many = ProfileOnlyUnitV1(profile=many_nodes)  # type: ignore[arg-type]
    with pytest.raises(QuarantineEvidenceEncodeError, match="node limit"):
        encode_quarantine_evidence_v1(too_many, spec=PROFILE_ONLY_V1_SPEC)

    import cura_receiver.quarantine_evidence as evidence_module

    scalar = b"x" * QUARANTINE_EVIDENCE_MAX_SCALAR_BYTES
    unit = ProfileOnlyUnitV1(profile=(scalar,) * 256)  # type: ignore[arg-type]
    base64_calls = 0
    envelope_values: list[object] = []
    real_base64_encode = evidence_module.base64.b64encode
    real_canonical_json = evidence_module._canonical_json

    def counted_base64_encode(value: bytes) -> bytes:
        nonlocal base64_calls
        base64_calls += 1
        return real_base64_encode(value)

    def observed_canonical_json(document: object) -> bytes:
        if type(document) is dict and "entity_kind" in document:
            envelope_values.append(document["value"])
        return real_canonical_json(document)

    monkeypatch.setattr(evidence_module.base64, "b64encode", counted_base64_encode)
    monkeypatch.setattr(evidence_module, "_canonical_json", observed_canonical_json)
    with pytest.raises(QuarantineEvidenceEncodeError, match="byte limit"):
        encode_quarantine_evidence_v1(unit, spec=PROFILE_ONLY_V1_SPEC)
    assert base64_calls < len(unit.profile)  # type: ignore[arg-type]
    assert envelope_values == [None]


def _canonical_mutation(encoded: bytes, mutation: object) -> bytes:
    document = json.loads(encoded)
    mutation(document)
    return json.dumps(
        document,
        ensure_ascii=True,
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("ascii")


def _canonical_evidence_document(value: object) -> bytes:
    return json.dumps(
        {
            "entity_kind": PROFILE_ONLY_V1_SPEC.kind.value,
            "entity_schema_version": PROFILE_ONLY_V1_SPEC.schema_version,
            "format": QUARANTINE_EVIDENCE_FORMAT,
            "format_version": QUARANTINE_EVIDENCE_VERSION,
            "value": value,
        },
        ensure_ascii=True,
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("ascii")


# Enum nodes must identify one exact member assignment from the fixed grammar.
@pytest.mark.parametrize(
    ("member_name", "value"),
    (("NOT_A_RADIO_STATE", "999"), ("RX_SINGLE", "999")),
)
def test_decoder_rejects_unknown_or_contradictory_enum_assignment(
    member_name: str,
    value: str,
) -> None:
    encoded = _canonical_evidence_document(
        {
            "class": (
                "cura_receiver.generated.receiver_enums_generated.RadioState"
            ),
            "member": member_name,
            "tag": "enum",
            "value": value,
        }
    )

    with pytest.raises(QuarantineEvidenceDecodeError, match="assignment"):
        decode_quarantine_evidence_v1(encoded)


# Canonical input still cannot exceed any decoder-side logical resource bound.
def test_decoder_rejects_depth_node_tuple_and_scalar_bounds() -> None:
    too_deep: dict[str, object] = {"tag": "none"}
    for _ in range(QUARANTINE_EVIDENCE_MAX_DEPTH):
        too_deep = {"items": [too_deep], "tag": "tuple"}

    nodes_per_group = 5
    group_count = QUARANTINE_EVIDENCE_MAX_NODES // nodes_per_group + 1
    too_many_nodes = {
        "items": [
            {
                "items": [{"tag": "none"}] * 4,
                "tag": "tuple",
            }
            for _ in range(group_count)
        ],
        "tag": "tuple",
    }
    over_limit_nodes = (
        too_deep,
        too_many_nodes,
        {
            "items": [{"tag": "none"}]
            * (QUARANTINE_EVIDENCE_MAX_TUPLE_ITEMS + 1),
            "tag": "tuple",
        },
        {
            "tag": "str",
            "value": "x" * (QUARANTINE_EVIDENCE_MAX_SCALAR_BYTES + 1),
        },
        {
            "encoding": "base64",
            "tag": "bytes",
            "value": base64.b64encode(
                b"x" * (QUARANTINE_EVIDENCE_MAX_SCALAR_BYTES + 1)
            ).decode("ascii"),
        },
        {
            "tag": "int",
            "value": "1" * (QUARANTINE_EVIDENCE_MAX_INTEGER_DIGITS + 1),
        },
    )

    for node in over_limit_nodes:
        with pytest.raises(QuarantineEvidenceDecodeError, match="limit"):
            decode_quarantine_evidence_v1(_canonical_evidence_document(node))


# The decoder accepts one bounded canonical ASCII JSON document and nothing around it.
def test_decoder_rejects_noncanonical_duplicate_trailing_and_oversized_json() -> None:
    encoded = encode_quarantine_evidence_v1(
        ProfileOnlyUnitV1(profile=_profile()), spec=PROFILE_ONLY_V1_SPEC
    )
    duplicate = encoded.replace(
        b'{"entity_kind":2,',
        b'{"entity_kind":2,"entity_kind":2,',
        1,
    )

    for invalid in (
        b" " + encoded,
        encoded + b"\n",
        duplicate,
        b"\xff",
        b"x" * (QUARANTINE_EVIDENCE_MAX_BYTES + 1),
    ):
        with pytest.raises(QuarantineEvidenceDecodeError):
            decode_quarantine_evidence_v1(invalid)


# Envelope and node schemas reject all unknown versions, tags, labels and fields.
@pytest.mark.parametrize(
    "mutation",
    (
        lambda document: document.__setitem__("unknown", 1),
        lambda document: document.__setitem__("format_version", 2),
        lambda document: document.__setitem__("format_version", 1.0),
        lambda document: document["value"].__setitem__("tag", "mapping"),
        lambda document: document["value"].__setitem__("class", "unknown.Record"),
        lambda document: document["value"]["fields"].append(
            {"name": "unknown", "value": {"tag": "none"}}
        ),
        lambda document: document["value"]["fields"][0]["value"][
            "fields"
        ].reverse(),
        lambda document: document["value"]["fields"][0]["value"]["fields"][
            1
        ].__setitem__("name", "receiver_instance_id"),
    ),
)
def test_decoder_rejects_unknown_envelope_tags_classes_and_fields(
    mutation: object,
) -> None:
    encoded = encode_quarantine_evidence_v1(
        ProfileOnlyUnitV1(profile=_profile()), spec=PROFILE_ONLY_V1_SPEC
    )

    with pytest.raises(QuarantineEvidenceDecodeError):
        decode_quarantine_evidence_v1(_canonical_mutation(encoded, mutation))


# Alternate integer and base64 spellings cannot represent the same evidence bytes.
def test_decoder_rejects_noncanonical_integer_and_base64_nodes() -> None:
    encoded = encode_quarantine_evidence_v1(
        ProfileOnlyUnitV1(profile=_profile()), spec=PROFILE_ONLY_V1_SPEC
    )

    noncanonical_integer = _canonical_mutation(
        encoded,
        lambda document: document["value"]["fields"][0]["value"]["fields"][1][
            "value"
        ].__setitem__("value", "01"),
    )
    noncanonical_base64 = _canonical_mutation(
        encoded,
        lambda document: document["value"]["fields"][0]["value"]["fields"][0][
            "value"
        ].__setitem__("value", "cg"),
    )

    for invalid in (noncanonical_integer, noncanonical_base64):
        with pytest.raises(QuarantineEvidenceDecodeError):
            decode_quarantine_evidence_v1(invalid)


# Neutral decoding has no runtime production-class registry or reconstruction hook.
def test_decoder_module_exposes_no_imported_production_entity_classes() -> None:
    import cura_receiver.quarantine_evidence as evidence_module

    imported_production_classes = {
        value
        for value in vars(evidence_module).values()
        if isinstance(value, type)
        and value.__module__.startswith("cura_receiver")
        and value.__module__ != evidence_module.__name__
    }
    assert imported_production_classes == set()


# Decoded evidence is immutable, compact data rather than an executable object graph.
def test_neutral_nodes_and_document_are_frozen_and_slotted() -> None:
    evidence = decode_quarantine_evidence_v1(
        encode_quarantine_evidence_v1(
            ProfileOnlyUnitV1(profile=_profile()), spec=PROFILE_ONLY_V1_SPEC
        )
    )

    assert not hasattr(evidence, "__dict__")
    assert not hasattr(evidence.value, "__dict__")
    assert [field.name for field in fields(type(evidence))] == [
        "format",
        "format_version",
        "entity_kind",
        "entity_schema_version",
        "value",
    ]
