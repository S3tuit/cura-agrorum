from __future__ import annotations

from dataclasses import dataclass
import struct
from uuid import UUID

from .generated.ack_v1 import (
    ACK_FORMAT,
    ACK_RECORD_TYPE,
    ACK_SCHEMA_VERSION,
)
from .generated.fault_v1 import (
    FAULT_FORMAT,
    FAULT_RECORD_TYPE,
    FAULT_SCHEMA_VERSION,
    FAULT_SIZE,
    Fault,
    FaultOperation,
    fault_from_tuple,
)
from .generated.reading_v1 import (
    CURA_RECORD_TYPE,
    FILE_SCHEMA_VERSION,
    READING_DS18B20_TEMP_OK,
    READING_ENV280_HUMIDITY_OK,
    READING_ENV280_PRESSURE_OK,
    READING_ENV280_TEMP_OK,
    READING_FORMAT,
    READING_SIZE,
    READING_SOIL_MV_OK,
    Reading,
    reading_from_tuple,
)

# This must match the TCP port used by the field hotspot deployment.
DEFAULT_PORT = 18032

FRAME_HEADER_FORMAT = "!I"
FRAME_HEADER_SIZE = struct.calcsize(FRAME_HEADER_FORMAT)
ENVELOPE_HEADER_FORMAT = "!HH"
ENVELOPE_HEADER_SIZE = struct.calcsize(ENVELOPE_HEADER_FORMAT)
EVENT_HEADER_FORMAT = "!BBH"
EVENT_HEADER_SIZE = struct.calcsize(EVENT_HEADER_FORMAT)
ENVELOPE_VERSION = 1
ACK_STATUS_OK = 0
ACK_STATUS_ERROR = 1

# Keep a hard server-side allocation limit even though body_len is u32.
MAX_FRAME_BODY_SIZE = 64 * 1024

class ProtocolError(ValueError):
  """Raised when a TCP frame is malformed for the selected schema."""


@dataclass(frozen=True)
class Event:
  record_type: int
  schema_version: int
  payload_len: int
  payload: bytes


def parse_frame_header(data: bytes) -> int:
  if len(data) != FRAME_HEADER_SIZE:
    raise ProtocolError(
        f"frame header must be {FRAME_HEADER_SIZE} bytes, got {len(data)}"
    )

  (body_len,) = struct.unpack(FRAME_HEADER_FORMAT, data)
  if body_len < ENVELOPE_HEADER_SIZE:
    raise ProtocolError(
        f"frame body must be at least {ENVELOPE_HEADER_SIZE} bytes, got {body_len}"
    )
  if body_len > MAX_FRAME_BODY_SIZE:
    raise ProtocolError(
        f"frame body length {body_len} exceeds max {MAX_FRAME_BODY_SIZE}"
    )

  return body_len


def parse_frame_body(body: bytes) -> list[Event]:
  if len(body) < ENVELOPE_HEADER_SIZE:
    raise ProtocolError(
        f"frame body must be at least {ENVELOPE_HEADER_SIZE} bytes, got {len(body)}"
    )

  envelope_version, event_count = struct.unpack_from(ENVELOPE_HEADER_FORMAT, body)
  if envelope_version != ENVELOPE_VERSION:
    raise ProtocolError(f"unsupported envelope version: {envelope_version}")
  if event_count == 0:
    raise ProtocolError("envelope must contain at least one event")

  events: list[Event] = []
  offset = ENVELOPE_HEADER_SIZE
  for index in range(event_count):
    if offset + EVENT_HEADER_SIZE > len(body):
      raise ProtocolError(f"event {index + 1}/{event_count} header is truncated")

    record_type, schema_version, payload_len = struct.unpack_from(
        EVENT_HEADER_FORMAT,
        body,
        offset,
    )
    offset += EVENT_HEADER_SIZE

    payload_end = offset + payload_len
    if payload_end > len(body):
      raise ProtocolError(
          f"event {index + 1}/{event_count} payload is truncated: "
          f"expected {payload_len} bytes"
      )

    events.append(
        Event(
            record_type=record_type,
            schema_version=schema_version,
            payload_len=payload_len,
            payload=body[offset:payload_end],
        )
    )
    offset = payload_end

  if offset != len(body):
    raise ProtocolError(
        f"frame body has {len(body) - offset} trailing bytes after {event_count} events"
    )

  return events


def encode_ack(status: int) -> bytes:
  payload = struct.pack(ACK_FORMAT, status)
  event = struct.pack(
      EVENT_HEADER_FORMAT,
      ACK_RECORD_TYPE,
      ACK_SCHEMA_VERSION,
      len(payload),
  ) + payload
  body = struct.pack(ENVELOPE_HEADER_FORMAT, ENVELOPE_VERSION, 1) + event
  return struct.pack(FRAME_HEADER_FORMAT, len(body)) + body


def is_supported_reading_event(event: Event) -> bool:
  return (
      event.record_type == CURA_RECORD_TYPE
      and event.schema_version == FILE_SCHEMA_VERSION
  )


def is_supported_fault_event(event: Event) -> bool:
  return (
      event.record_type == FAULT_RECORD_TYPE
      and event.schema_version == FAULT_SCHEMA_VERSION
  )


def decode_reading(payload: bytes) -> Reading:
  # Each event carries exactly one generated payload. Keeping the decoder exact
  # avoids silently accepting stale schema versions or concatenated records.
  if len(payload) != READING_SIZE:
    raise ProtocolError(
        f"reading payload must be {READING_SIZE} bytes, got {len(payload)}"
    )

  fields = struct.unpack(READING_FORMAT, payload)
  return reading_from_tuple(fields)


def decode_fault(payload: bytes) -> Fault:
  if len(payload) != FAULT_SIZE:
    raise ProtocolError(
        f"fault payload must be {FAULT_SIZE} bytes, got {len(payload)}"
    )

  fields = struct.unpack(FAULT_FORMAT, payload)
  return fault_from_tuple(fields)


def hex_preview(payload: bytes, max_bytes: int = 16) -> str:
  preview = payload[:max_bytes].hex(" ")
  if len(payload) > max_bytes:
    return f"{preview} ..."
  return preview


def format_reading(
    peer: str,
    event: Event,
    reading: Reading,
    index: int = 1,
    total: int = 1,
) -> str:
  prefix = (
      f"client={peer} record_type={event.record_type}"
      f" schema={event.schema_version}"
      f" event={index}/{total}"
      f" node={format_node_uuid_bytes(reading.node_uuid)}"
  )
  return (
      f"{prefix} sample_id={reading.sample_id}"
      f" boot={reading.bootno}"
      f" wake=0x{reading.wake_causes:08x}"
      f" run={reading.run_ms}ms"
      f" soil_mv={_field(reading.flags, READING_SOIL_MV_OK, reading.soil_mv, 'mV')}"
      f" ds18b20={_field(reading.flags, READING_DS18B20_TEMP_OK, _centi(reading.ds18b20_centi_c, 'C'))}"
      f" env280_temp={_field(reading.flags, READING_ENV280_TEMP_OK, _centi(reading.env280_centi_c, 'C'))}"
      f" pressure={_field(reading.flags, READING_ENV280_PRESSURE_OK, reading.env280_pressure_pa, 'Pa')}"
      f" humidity={_field(reading.flags, READING_ENV280_HUMIDITY_OK, _centi(reading.env280_humidity_centi_pct, '%'))}"
      f" flags=0x{reading.flags:02x}"
  )


def format_fault(
    peer: str,
    event: Event,
    fault: Fault,
    index: int = 1,
    total: int = 1,
) -> str:
  prefix = (
      f"client={peer} record_type={event.record_type}"
      f" schema={event.schema_version}"
      f" event={index}/{total}"
      f" node={format_node_uuid_bytes(fault.node_uuid)}"
  )
  return (
      f"{prefix} fault_id={fault.fault_id.hex()}"
      f" sample_id={fault.sample_id}"
      f" boot={fault.bootno}"
      f" operation={_fault_operation(fault.operation)}"
      f" esp_err={fault.esp_err}"
      f" posix_errno={fault.posix_errno}"
  )


def node_uuid_from_bytes(node_uuid: bytes) -> UUID:
  try:
    return UUID(bytes=node_uuid)
  except ValueError as exc:
    raise ProtocolError(f"invalid node UUID bytes: {exc}") from exc


def format_node_uuid_bytes(node_uuid: bytes) -> str:
  return str(node_uuid_from_bytes(node_uuid))


def format_unsupported_event(peer: str, event: Event) -> str:
  return (
      f"client={peer} unsupported event record_type={event.record_type}"
      f" schema={event.schema_version} payload_len={event.payload_len}"
      f" preview={hex_preview(event.payload)}"
  )


def _field(flags: int, flag: int, value: object, suffix: str = "") -> str:
  if (flags & flag) == 0:
    return "missing"
  return f"{value}{suffix}"


def _centi(value: int, unit: str) -> str:
  sign = "-" if value < 0 else ""
  absolute = abs(value)
  return f"{sign}{absolute // 100}.{absolute % 100:02d}{unit}"


def _fault_operation(operation: int) -> str:
  try:
    return FaultOperation(operation).name
  except ValueError:
    return str(operation)
