"""Canonical, non-executable evidence for poisoned ``PersistQueue`` values."""

from __future__ import annotations

import base64
import binascii
from dataclasses import dataclass, fields, is_dataclass
from enum import Enum
import hashlib
import json
import re
from typing import TYPE_CHECKING, TypeAlias

if TYPE_CHECKING:
    from .persist_queue_entities import PersistQueueEntitySpec


QUARANTINE_EVIDENCE_FORMAT = "cura-agrorum-quarantine-evidence"
QUARANTINE_EVIDENCE_VERSION = 1
QUARANTINE_EVIDENCE_MAX_BYTES = 262_144
QUARANTINE_EVIDENCE_MAX_DEPTH = 32
QUARANTINE_EVIDENCE_MAX_NODES = 4_096
QUARANTINE_EVIDENCE_MAX_TUPLE_ITEMS = 1_024
QUARANTINE_EVIDENCE_MAX_SCALAR_BYTES = 65_536
QUARANTINE_EVIDENCE_MAX_INTEGER_DIGITS = 1_024

_SPEC_CLASS = "cura_receiver.persist_queue_entities.PersistQueueEntitySpec"
_KIND_CLASS = (
    "cura_receiver.generated.receiver_enums_generated.PersistQueueEntityKind"
)

_ALLOWED_RECORD_FIELDS: dict[str, tuple[str, ...]] = {
    "cura_receiver.persist_queue_entities.AuthenticatedReadingCandidateV1": (
        "node_id",
        "message_id",
        "domain",
        "sample_id",
        "reading_body",
    ),
    "cura_receiver.persist_queue_entities.MeasurementProfileUnitV1": (
        "candidate",
        "profile",
    ),
    "cura_receiver.persist_queue_entities.ProfileOnlyUnitV1": ("profile",),
    "cura_receiver.persist_queue_entities.ReceiverHealthRequestV1": (
        "receiver_instance_id",
        "health_sequence",
        "communicator_sampled_at_monotonic_us",
        "radio_state",
        "radio_recovery_attempts",
        "radio_recovery_successes",
        "radio_recovery_failures",
        "radio_recovery_attempts_by_reason",
        "system_time_quality",
        "rtc_health",
        "time_quality_transition_count",
        "rtc_health_transition_count",
        "last_time_quality_transition_monotonic_us",
        "last_rtc_health_transition_monotonic_us",
        "chrony_step_command_results",
        "rtc_write_results",
        "rtc_write_readback_verified_count",
        "rtc_write_trust_invalidated_count",
        "persist_queue_admission_counts",
    ),
    "cura_receiver.generated.receiver_entities_generated.MessageProfilingV1": (
        "receiver_instance_id",
        "occurrence_sequence",
        "received_at_monotonic_us",
        "received_frame_length",
        "received_frame",
        "claimed_control",
        "claimed_domain",
        "claimed_node_id",
        "claimed_message_id",
        "header_authenticated",
        "decoded_sample_id",
        "rssi_dbm_x2",
        "snr_db_x4",
        "irq_status",
        "device_errors",
        "processing_result",
        "ack_selected",
        "ack_tx_result",
        "ack_frame",
        "busy_wait_total_us",
        "busy_wait_max_us",
        "busy_wait_count",
        "busy_timeout_count",
        "last_busy_timeout_opcode",
        "t1_handler_started_monotonic_us",
        "t2_packet_copied_monotonic_us",
        "t3_authentication_completed_monotonic_us",
        "t4_set_tx_attempted_monotonic_us",
        "t5_tx_done_monotonic_us",
        "t6_set_rx_issued_monotonic_us",
    ),
    "cura_receiver.generated.receiver_entities_generated.ClockObservationV1": (
        "receiver_instance_id",
        "observation_sequence",
        "clock_state_generation",
        "sampled_at_monotonic_us",
        "sampled_at_utc_us",
        "step_discontinuity_boundary",
        "system_time_quality",
        "rtc_health",
    ),
    "cura_receiver.generated.receiver_entities_generated.DiagnosticV1": (
        "receiver_instance_id",
        "diagnostic_sequence",
        "sampled_at_monotonic_us",
        "severity",
        "error_domain",
        "operation",
        "error_code",
        "context_schema",
        "context_length",
        "context",
    ),
}

_ALLOWED_ENUM_MEMBERS: dict[str, dict[str, int]] = {
    "cura_receiver.generated.protocol_v2_lora_generated.Domain": {
        "CURRENT_READING_UPLINK": 1,
        "BACKLOG_READING_UPLINK": 2,
        "ACK_ACCEPTED_DOWNLINK": 3,
        "ACK_RETRY_LATER_DOWNLINK": 4,
        "ACK_REJECTED_UNSUPPORTED_DOWNLINK": 5,
        "ACK_REJECTED_MALFORMED_DOWNLINK": 6,
    },
    "cura_receiver.generated.receiver_enums_generated.AckSelection": {
        "NONE": 0,
        "ACCEPTED": 3,
        "RETRY_LATER": 4,
        "REJECTED_UNSUPPORTED": 5,
        "REJECTED_MALFORMED": 6,
    },
    "cura_receiver.generated.receiver_enums_generated.AckTxResult": {
        "NOT_APPLICABLE": 1,
        "SUPPRESSED_AIRTIME_BUDGET": 2,
        "SET_TX_FAILED": 3,
        "TX_TIMEOUT": 4,
        "TX_DONE": 5,
        "UNKNOWN_INTERRUPTED": 6,
    },
    "cura_receiver.generated.receiver_enums_generated.DiagnosticErrorDomain": {
        "NONE": 0,
        "RADIO": 1,
        "TIME": 2,
        "PERSISTENCE_CONTROL": 3,
        "CORE": 4,
    },
    "cura_receiver.generated.receiver_enums_generated.DiagnosticOperation": {
        "NONE": 0,
        "INITIALIZE": 1,
        "VALIDATE": 2,
        "READ": 3,
        "WRITE": 4,
        "APPEND": 5,
        "SYNC": 6,
        "RECOVER": 7,
        "ENCODE": 8,
        "DECODE": 9,
        "TRANSMIT": 10,
        "RECEIVE": 11,
        "CLEANUP": 12,
    },
    "cura_receiver.generated.receiver_enums_generated.DiagnosticSeverity": {
        "WARN": 1,
        "ERROR": 2,
        "FATAL": 3,
    },
    "cura_receiver.generated.receiver_enums_generated.ProcessingResult": {
        "RADIO_ERROR": 1,
        "UNKNOWN_NODE": 2,
        "AUTHENTICATION_FAILED": 3,
        "WRONG_DIRECTION": 4,
        "REJECTED_UNSUPPORTED_CONTROL": 5,
        "REJECTED_UNSUPPORTED_DOMAIN": 6,
        "REJECTED_MALFORMED_LENGTH": 7,
        "REJECTED_MALFORMED_BODY": 8,
        "RETRY_LATER_QUEUE_FULL": 9,
        "RETRY_LATER_PERSISTENCE_UNAVAILABLE": 10,
        "ACCEPTED": 11,
    },
    "cura_receiver.generated.receiver_enums_generated.RadioState": {
        "INITIALIZING": 1,
        "RX_SINGLE": 2,
        "RX_EVENT_PENDING": 3,
        "TX_ACTIVE": 4,
        "RECOVERING": 5,
        "SHUTDOWN": 6,
        "INITIALIZATION_FAILED": 7,
        "RECOVERY_EXHAUSTED": 8,
        "HARDWARE_MISSING": 9,
    },
    "cura_receiver.generated.receiver_enums_generated.RtcHealth": {
        "PRESENT": 1,
        "MISSING": 2,
        "INVALID": 3,
    },
    "cura_receiver.generated.receiver_enums_generated.SystemTimeQuality": {
        "UNTRUSTED": 0,
        "RTC_HOLDOVER": 1,
        "NETWORK_SYNCED": 2,
    },
}

_INTEGER_PATTERN = re.compile(r"(?:0|-[1-9][0-9]*|[1-9][0-9]*)\Z")
_ENUM_MEMBER_PATTERN = re.compile(r"[A-Z][A-Z0-9_]*\Z")


class QuarantineEvidenceError(ValueError):
    """Base class for evidence grammar failures."""


class QuarantineEvidenceEncodeError(QuarantineEvidenceError):
    """The runtime value cannot be represented by the bounded evidence grammar."""


class QuarantineEvidenceDecodeError(QuarantineEvidenceError):
    """The supplied bytes are not canonical V1 evidence."""


@dataclass(frozen=True, slots=True)
class NeutralNoneV1:
    """Neutral representation of Python ``None``."""


@dataclass(frozen=True, slots=True)
class NeutralBoolV1:
    value: bool


@dataclass(frozen=True, slots=True)
class NeutralIntV1:
    value: int


@dataclass(frozen=True, slots=True)
class NeutralStringV1:
    value: str


@dataclass(frozen=True, slots=True)
class NeutralBytesV1:
    value: bytes


@dataclass(frozen=True, slots=True)
class NeutralEnumV1:
    class_name: str
    member_name: str
    value: int


@dataclass(frozen=True, slots=True)
class NeutralTupleV1:
    items: tuple[NeutralEvidenceNodeV1, ...]


@dataclass(frozen=True, slots=True)
class NeutralFieldV1:
    name: str
    value: NeutralEvidenceNodeV1


@dataclass(frozen=True, slots=True)
class NeutralRecordV1:
    class_name: str
    fields: tuple[NeutralFieldV1, ...]


NeutralEvidenceNodeV1: TypeAlias = (
    NeutralNoneV1
    | NeutralBoolV1
    | NeutralIntV1
    | NeutralStringV1
    | NeutralBytesV1
    | NeutralEnumV1
    | NeutralTupleV1
    | NeutralRecordV1
)


@dataclass(frozen=True, slots=True)
class QuarantineEvidenceV1:
    format: str
    format_version: int
    entity_kind: int
    entity_schema_version: int
    value: NeutralEvidenceNodeV1


@dataclass(slots=True)
class _Budget:
    nodes: int = 0
    canonical_bytes: int = 0

    def consume(self, depth: int, error_type: type[QuarantineEvidenceError]) -> None:
        if depth > QUARANTINE_EVIDENCE_MAX_DEPTH:
            raise error_type("quarantine evidence depth limit exceeded")
        self.nodes += 1
        if self.nodes > QUARANTINE_EVIDENCE_MAX_NODES:
            raise error_type("quarantine evidence node limit exceeded")

    def consume_canonical_bytes(
        self,
        amount: int,
        error_type: type[QuarantineEvidenceError],
    ) -> None:
        self.canonical_bytes += amount
        if self.canonical_bytes > QUARANTINE_EVIDENCE_MAX_BYTES:
            raise error_type("quarantine evidence canonical byte limit exceeded")


def _class_name(value: object) -> str:
    value_type = type(value)
    return f"{value_type.__module__}.{value_type.__qualname__}"


def _decimal(value: int, error_type: type[QuarantineEvidenceError]) -> str:
    try:
        encoded = str(value)
    except (ValueError, OverflowError) as error:
        raise error_type("quarantine evidence integer digit limit exceeded") from error
    digits = encoded[1:] if encoded.startswith("-") else encoded
    if len(digits) > QUARANTINE_EVIDENCE_MAX_INTEGER_DIGITS:
        raise error_type("quarantine evidence integer digit limit exceeded")
    return encoded


def _decode_decimal(value: object) -> int:
    if type(value) is not str or _INTEGER_PATTERN.fullmatch(value) is None:
        raise QuarantineEvidenceDecodeError(
            "quarantine evidence integer is not canonical"
        )
    digits = value[1:] if value.startswith("-") else value
    if len(digits) > QUARANTINE_EVIDENCE_MAX_INTEGER_DIGITS:
        raise QuarantineEvidenceDecodeError(
            "quarantine evidence integer digit limit exceeded"
        )
    try:
        return int(value)
    except (ValueError, OverflowError) as error:
        raise QuarantineEvidenceDecodeError(
            "quarantine evidence integer is invalid"
        ) from error


def _scalar_utf8_size(value: str) -> int:
    return len(value.encode("utf-8", errors="surrogatepass"))


def _encode_node(
    value: object,
    *,
    depth: int,
    budget: _Budget,
    ancestors: set[int],
) -> dict[str, object]:
    budget.consume(depth, QuarantineEvidenceEncodeError)

    if value is None:
        node: dict[str, object] = {"tag": "none"}
        budget.consume_canonical_bytes(
            len(_canonical_json(node)), QuarantineEvidenceEncodeError
        )
        return node
    if type(value) is bool:
        node = {"tag": "bool", "value": value}
        budget.consume_canonical_bytes(
            len(_canonical_json(node)), QuarantineEvidenceEncodeError
        )
        return node
    if isinstance(value, Enum):
        class_name = _class_name(value)
        allowed_members = _ALLOWED_ENUM_MEMBERS.get(class_name)
        if allowed_members is None:
            raise QuarantineEvidenceEncodeError(
                f"unsupported quarantine evidence enum class {class_name}"
            )
        enum_value = value.value
        if type(enum_value) is not int:
            raise QuarantineEvidenceEncodeError(
                "unsupported quarantine evidence enum value type"
            )
        member_name = value.name
        if (
            type(member_name) is not str
            or _ENUM_MEMBER_PATTERN.fullmatch(member_name) is None
        ):
            raise QuarantineEvidenceEncodeError(
                "unsupported quarantine evidence enum member name"
            )
        if allowed_members.get(member_name) != enum_value:
            raise QuarantineEvidenceEncodeError(
                "unsupported quarantine evidence enum member assignment"
            )
        node = {
            "class": class_name,
            "member": member_name,
            "tag": "enum",
            "value": _decimal(enum_value, QuarantineEvidenceEncodeError),
        }
        budget.consume_canonical_bytes(
            len(_canonical_json(node)), QuarantineEvidenceEncodeError
        )
        return node
    if type(value) is int:
        node = {
            "tag": "int",
            "value": _decimal(value, QuarantineEvidenceEncodeError),
        }
        budget.consume_canonical_bytes(
            len(_canonical_json(node)), QuarantineEvidenceEncodeError
        )
        return node
    if type(value) is str:
        if _scalar_utf8_size(value) > QUARANTINE_EVIDENCE_MAX_SCALAR_BYTES:
            raise QuarantineEvidenceEncodeError(
                "quarantine evidence string byte limit exceeded"
            )
        node = {"tag": "str", "value": value}
        budget.consume_canonical_bytes(
            len(_canonical_json(node)), QuarantineEvidenceEncodeError
        )
        return node
    if type(value) is bytes:
        if len(value) > QUARANTINE_EVIDENCE_MAX_SCALAR_BYTES:
            raise QuarantineEvidenceEncodeError(
                "quarantine evidence bytes limit exceeded"
            )
        node = {
            "encoding": "base64",
            "tag": "bytes",
            "value": base64.b64encode(value).decode("ascii"),
        }
        budget.consume_canonical_bytes(
            len(_canonical_json(node)), QuarantineEvidenceEncodeError
        )
        return node

    if type(value) is tuple:
        if len(value) > QUARANTINE_EVIDENCE_MAX_TUPLE_ITEMS:
            raise QuarantineEvidenceEncodeError(
                "quarantine evidence tuple item limit exceeded"
            )
        identity = id(value)
        if identity in ancestors:
            raise QuarantineEvidenceEncodeError("quarantine evidence cycle detected")
        ancestors.add(identity)
        try:
            budget.consume_canonical_bytes(
                len(_canonical_json({"items": [], "tag": "tuple"}))
                + max(0, len(value) - 1),
                QuarantineEvidenceEncodeError,
            )
            return {
                "items": [
                    _encode_node(
                        item,
                        depth=depth + 1,
                        budget=budget,
                        ancestors=ancestors,
                    )
                    for item in value
                ],
                "tag": "tuple",
            }
        finally:
            ancestors.remove(identity)

    if is_dataclass(value) and not isinstance(value, type):
        class_name = _class_name(value)
        expected_fields = _ALLOWED_RECORD_FIELDS.get(class_name)
        if expected_fields is None:
            raise QuarantineEvidenceEncodeError(
                f"unsupported quarantine evidence record class {class_name}"
            )
        actual_fields = tuple(field.name for field in fields(value))
        if actual_fields != expected_fields:
            raise QuarantineEvidenceEncodeError(
                f"unsupported quarantine evidence record layout {class_name}"
            )
        identity = id(value)
        if identity in ancestors:
            raise QuarantineEvidenceEncodeError("quarantine evidence cycle detected")
        ancestors.add(identity)
        try:
            structural_bytes = len(
                _canonical_json(
                    {
                        "class": class_name,
                        "fields": [],
                        "tag": "record",
                    }
                )
            ) + max(0, len(expected_fields) - 1)
            for field_name in expected_fields:
                structural_bytes += len(
                    _canonical_json({"name": field_name, "value": None})
                ) - len("null")
            budget.consume_canonical_bytes(
                structural_bytes, QuarantineEvidenceEncodeError
            )
            encoded_fields: list[dict[str, object]] = []
            for field_name in expected_fields:
                encoded_fields.append(
                    {
                        "name": field_name,
                        "value": _encode_node(
                            getattr(value, field_name),
                            depth=depth + 1,
                            budget=budget,
                            ancestors=ancestors,
                        ),
                    }
                )
            return {
                "class": class_name,
                "fields": encoded_fields,
                "tag": "record",
            }
        finally:
            ancestors.remove(identity)

    raise QuarantineEvidenceEncodeError(
        f"unsupported quarantine evidence value type {_class_name(value)}"
    )


def _canonical_json(document: object) -> bytes:
    try:
        return json.dumps(
            document,
            ensure_ascii=True,
            allow_nan=False,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("ascii")
    except (TypeError, ValueError, UnicodeError, RecursionError) as error:
        raise QuarantineEvidenceEncodeError(
            "quarantine evidence cannot be encoded as canonical JSON"
        ) from error


def _spec_metadata(spec: PersistQueueEntitySpec) -> tuple[int, int]:
    if _class_name(spec) != _SPEC_CLASS:
        raise QuarantineEvidenceEncodeError("invalid quarantine evidence entity spec")
    kind = getattr(spec, "kind", None)
    if _class_name(kind) != _KIND_CLASS or type(kind.value) is not int:
        raise QuarantineEvidenceEncodeError("invalid quarantine evidence entity kind")
    schema_version = getattr(spec, "schema_version", None)
    if type(schema_version) is not int:
        raise QuarantineEvidenceEncodeError(
            "invalid quarantine evidence entity schema version"
        )
    if not 0 <= kind.value <= 255 or not 0 <= schema_version <= 255:
        raise QuarantineEvidenceEncodeError(
            "quarantine evidence entity metadata is out of range"
        )
    return kind.value, schema_version


def encode_quarantine_evidence_v1(
    entity: object,
    *,
    spec: PersistQueueEntitySpec,
) -> bytes:
    """Encode one admitted logical value as bounded canonical tagged JSON."""

    entity_kind, entity_schema_version = _spec_metadata(spec)
    envelope_without_value = {
        "entity_kind": entity_kind,
        "entity_schema_version": entity_schema_version,
        "format": QUARANTINE_EVIDENCE_FORMAT,
        "format_version": QUARANTINE_EVIDENCE_VERSION,
        "value": None,
    }
    budget = _Budget()
    budget.consume_canonical_bytes(
        len(_canonical_json(envelope_without_value)) - len("null"),
        QuarantineEvidenceEncodeError,
    )
    document = {
        **envelope_without_value,
        "value": _encode_node(
            entity, depth=1, budget=budget, ancestors=set()
        ),
    }
    encoded = _canonical_json(document)
    if len(encoded) > QUARANTINE_EVIDENCE_MAX_BYTES:
        raise QuarantineEvidenceEncodeError(
            "quarantine evidence canonical byte limit exceeded"
        )
    if len(encoded) != budget.canonical_bytes:
        raise QuarantineEvidenceEncodeError(
            "quarantine evidence canonical byte accounting mismatch"
        )
    return encoded


class _DuplicateKeyError(ValueError):
    pass


def _unique_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise _DuplicateKeyError(f"duplicate JSON key {key!r}")
        result[key] = value
    return result


def _reject_json_constant(value: str) -> object:
    raise ValueError(f"invalid JSON constant {value}")


def _require_keys(
    value: object,
    expected: frozenset[str],
    context: str,
) -> dict[str, object]:
    if type(value) is not dict:
        raise QuarantineEvidenceDecodeError(f"{context} must be a JSON object")
    if frozenset(value) != expected:
        raise QuarantineEvidenceDecodeError(f"{context} has unknown or missing keys")
    return value


def _decode_node(
    encoded: object,
    *,
    depth: int,
    budget: _Budget,
) -> NeutralEvidenceNodeV1:
    budget.consume(depth, QuarantineEvidenceDecodeError)
    if type(encoded) is not dict or type(encoded.get("tag")) is not str:
        raise QuarantineEvidenceDecodeError(
            "quarantine evidence node must have a string tag"
        )
    tag = encoded["tag"]

    if tag == "none":
        _require_keys(encoded, frozenset({"tag"}), "none node")
        return NeutralNoneV1()
    if tag == "bool":
        node = _require_keys(encoded, frozenset({"tag", "value"}), "bool node")
        if type(node["value"]) is not bool:
            raise QuarantineEvidenceDecodeError("bool node value must be Boolean")
        return NeutralBoolV1(node["value"])
    if tag == "int":
        node = _require_keys(encoded, frozenset({"tag", "value"}), "int node")
        return NeutralIntV1(_decode_decimal(node["value"]))
    if tag == "str":
        node = _require_keys(encoded, frozenset({"tag", "value"}), "str node")
        value = node["value"]
        if type(value) is not str:
            raise QuarantineEvidenceDecodeError("str node value must be a string")
        if _scalar_utf8_size(value) > QUARANTINE_EVIDENCE_MAX_SCALAR_BYTES:
            raise QuarantineEvidenceDecodeError(
                "quarantine evidence string byte limit exceeded"
            )
        return NeutralStringV1(value)
    if tag == "bytes":
        node = _require_keys(
            encoded,
            frozenset({"encoding", "tag", "value"}),
            "bytes node",
        )
        if node["encoding"] != "base64" or type(node["value"]) is not str:
            raise QuarantineEvidenceDecodeError("bytes node encoding is invalid")
        try:
            base64_text = node["value"].encode("ascii")
            value = base64.b64decode(base64_text, validate=True)
        except (UnicodeEncodeError, binascii.Error, ValueError) as error:
            raise QuarantineEvidenceDecodeError(
                "bytes node base64 is invalid"
            ) from error
        if base64.b64encode(value) != base64_text:
            raise QuarantineEvidenceDecodeError(
                "bytes node base64 is not canonical"
            )
        if len(value) > QUARANTINE_EVIDENCE_MAX_SCALAR_BYTES:
            raise QuarantineEvidenceDecodeError(
                "quarantine evidence bytes limit exceeded"
            )
        return NeutralBytesV1(value)
    if tag == "enum":
        node = _require_keys(
            encoded,
            frozenset({"class", "member", "tag", "value"}),
            "enum node",
        )
        class_name = node["class"]
        member_name = node["member"]
        if type(class_name) is not str:
            raise QuarantineEvidenceDecodeError("enum node class is unknown")
        allowed_members = _ALLOWED_ENUM_MEMBERS.get(class_name)
        if allowed_members is None:
            raise QuarantineEvidenceDecodeError("enum node class is unknown")
        if (
            type(member_name) is not str
            or _ENUM_MEMBER_PATTERN.fullmatch(member_name) is None
        ):
            raise QuarantineEvidenceDecodeError("enum node member is invalid")
        value = _decode_decimal(node["value"])
        if allowed_members.get(member_name) != value:
            raise QuarantineEvidenceDecodeError(
                "enum node member assignment is unknown"
            )
        return NeutralEnumV1(
            class_name=class_name,
            member_name=member_name,
            value=value,
        )
    if tag == "tuple":
        node = _require_keys(
            encoded,
            frozenset({"items", "tag"}),
            "tuple node",
        )
        items = node["items"]
        if type(items) is not list:
            raise QuarantineEvidenceDecodeError("tuple node items must be an array")
        if len(items) > QUARANTINE_EVIDENCE_MAX_TUPLE_ITEMS:
            raise QuarantineEvidenceDecodeError(
                "quarantine evidence tuple item limit exceeded"
            )
        return NeutralTupleV1(
            tuple(
                _decode_node(item, depth=depth + 1, budget=budget)
                for item in items
            )
        )
    if tag == "record":
        node = _require_keys(
            encoded,
            frozenset({"class", "fields", "tag"}),
            "record node",
        )
        class_name = node["class"]
        encoded_fields = node["fields"]
        if type(class_name) is not str:
            raise QuarantineEvidenceDecodeError("record node class must be a string")
        expected_fields = _ALLOWED_RECORD_FIELDS.get(class_name)
        if expected_fields is None:
            raise QuarantineEvidenceDecodeError("record node class is unknown")
        if type(encoded_fields) is not list or len(encoded_fields) != len(
            expected_fields
        ):
            raise QuarantineEvidenceDecodeError("record node fields are invalid")
        neutral_fields: list[NeutralFieldV1] = []
        for expected_name, encoded_field in zip(
            expected_fields, encoded_fields, strict=True
        ):
            field_node = _require_keys(
                encoded_field,
                frozenset({"name", "value"}),
                "record field",
            )
            if field_node["name"] != expected_name:
                raise QuarantineEvidenceDecodeError(
                    "record node fields are unknown, duplicated, or out of order"
                )
            neutral_fields.append(
                NeutralFieldV1(
                    name=expected_name,
                    value=_decode_node(
                        field_node["value"],
                        depth=depth + 1,
                        budget=budget,
                    ),
                )
            )
        return NeutralRecordV1(class_name, tuple(neutral_fields))

    raise QuarantineEvidenceDecodeError(f"unknown quarantine evidence tag {tag!r}")


def decode_quarantine_evidence_v1(encoded: bytes) -> QuarantineEvidenceV1:
    """Validate canonical V1 evidence and return a neutral, non-executable tree."""

    if type(encoded) is not bytes:
        raise QuarantineEvidenceDecodeError("quarantine evidence must be bytes")
    if not encoded or len(encoded) > QUARANTINE_EVIDENCE_MAX_BYTES:
        raise QuarantineEvidenceDecodeError(
            "quarantine evidence canonical byte limit exceeded"
        )
    try:
        text = encoded.decode("ascii")
        document = json.loads(
            text,
            object_pairs_hook=_unique_object,
            parse_constant=_reject_json_constant,
        )
    except (
        UnicodeDecodeError,
        json.JSONDecodeError,
        _DuplicateKeyError,
        ValueError,
        RecursionError,
    ) as error:
        raise QuarantineEvidenceDecodeError(
            "quarantine evidence is not valid canonical JSON"
        ) from error

    try:
        canonical = _canonical_json(document)
    except QuarantineEvidenceEncodeError as error:
        raise QuarantineEvidenceDecodeError(str(error)) from error
    if canonical != encoded:
        raise QuarantineEvidenceDecodeError(
            "quarantine evidence JSON is not canonical"
        )

    envelope = _require_keys(
        document,
        frozenset(
            {
                "entity_kind",
                "entity_schema_version",
                "format",
                "format_version",
                "value",
            }
        ),
        "quarantine evidence envelope",
    )
    if (
        type(envelope["format"]) is not str
        or envelope["format"] != QUARANTINE_EVIDENCE_FORMAT
    ):
        raise QuarantineEvidenceDecodeError("quarantine evidence format is unknown")
    if (
        type(envelope["format_version"]) is not int
        or envelope["format_version"] != QUARANTINE_EVIDENCE_VERSION
    ):
        raise QuarantineEvidenceDecodeError(
            "quarantine evidence format version is unknown"
        )
    entity_kind = envelope["entity_kind"]
    entity_schema_version = envelope["entity_schema_version"]
    if (
        type(entity_kind) is not int
        or not 0 <= entity_kind <= 255
        or type(entity_schema_version) is not int
        or not 0 <= entity_schema_version <= 255
    ):
        raise QuarantineEvidenceDecodeError(
            "quarantine evidence entity metadata is invalid"
        )

    return QuarantineEvidenceV1(
        format=QUARANTINE_EVIDENCE_FORMAT,
        format_version=QUARANTINE_EVIDENCE_VERSION,
        entity_kind=entity_kind,
        entity_schema_version=entity_schema_version,
        value=_decode_node(
            envelope["value"],
            depth=1,
            budget=_Budget(),
        ),
    )


def quarantine_evidence_sha256(encoded: bytes) -> bytes:
    """Return the quarantine-row identity for exact canonical evidence bytes."""

    if type(encoded) is not bytes:
        raise TypeError("quarantine evidence must be bytes")
    return hashlib.sha256(encoded).digest()


__all__ = [
    "QUARANTINE_EVIDENCE_FORMAT",
    "QUARANTINE_EVIDENCE_VERSION",
    "QUARANTINE_EVIDENCE_MAX_BYTES",
    "QUARANTINE_EVIDENCE_MAX_DEPTH",
    "QUARANTINE_EVIDENCE_MAX_NODES",
    "QUARANTINE_EVIDENCE_MAX_TUPLE_ITEMS",
    "QUARANTINE_EVIDENCE_MAX_SCALAR_BYTES",
    "QUARANTINE_EVIDENCE_MAX_INTEGER_DIGITS",
    "QuarantineEvidenceError",
    "QuarantineEvidenceEncodeError",
    "QuarantineEvidenceDecodeError",
    "NeutralNoneV1",
    "NeutralBoolV1",
    "NeutralIntV1",
    "NeutralStringV1",
    "NeutralBytesV1",
    "NeutralEnumV1",
    "NeutralTupleV1",
    "NeutralFieldV1",
    "NeutralRecordV1",
    "NeutralEvidenceNodeV1",
    "QuarantineEvidenceV1",
    "encode_quarantine_evidence_v1",
    "decode_quarantine_evidence_v1",
    "quarantine_evidence_sha256",
]
