from __future__ import annotations

from dataclasses import dataclass
import struct
from uuid import UUID

from .generated.node_config_v1 import (
    NODE_CONFIG_FORMAT,
    NODE_CONFIG_RECORD_TYPE,
    NODE_CONFIG_SCHEMA_VERSION,
    NODE_CONFIG_SIZE,
    NodeConfig,
    node_config_from_tuple,
)
from .generated.config_ack_v1 import (
    CONFIG_ACK_FORMAT,
    CONFIG_ACK_RECORD_TYPE,
    CONFIG_ACK_SCHEMA_VERSION,
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

# This must be the same as the port advertised by Avahi (mDNS responder).
DEFAULT_PORT = 18032

FRAME_HEADER_FORMAT = "!I"
FRAME_HEADER_SIZE = struct.calcsize(FRAME_HEADER_FORMAT)
ENVELOPE_HEADER_FORMAT = "!HH"
ENVELOPE_HEADER_SIZE = struct.calcsize(ENVELOPE_HEADER_FORMAT)
EVENT_HEADER_FORMAT = "!BBH"
EVENT_HEADER_SIZE = struct.calcsize(EVENT_HEADER_FORMAT)
ENVELOPE_VERSION = 1

# Keep a hard server-side allocation limit even though body_len is u32.
MAX_FRAME_BODY_SIZE = 64 * 1024

CONFIG_ACK_OK = 0
CONFIG_ACK_ERROR = 1


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


def is_supported_reading_event(event: Event) -> bool:
  return (
      event.record_type == CURA_RECORD_TYPE
      and event.schema_version == FILE_SCHEMA_VERSION
  )


def is_supported_node_config_event(event: Event) -> bool:
  return (
      event.record_type == NODE_CONFIG_RECORD_TYPE
      and event.schema_version == NODE_CONFIG_SCHEMA_VERSION
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


def decode_node_config(payload: bytes) -> NodeConfig:
  if len(payload) != NODE_CONFIG_SIZE:
    raise ProtocolError(
        f"node_config payload must be {NODE_CONFIG_SIZE} bytes, got {len(payload)}"
    )

  fields = struct.unpack(NODE_CONFIG_FORMAT, payload)
  return node_config_from_tuple(fields)


def encode_config_ack(status: int) -> bytes:
  """Encode the only server response frame currently used by firmware.

  Reading-only frames intentionally have no ACK for now. Frames that include
  node_config_t receive this ACK after the server persists the node config.
  """
  if status < 0 or status > 0xFFFFFFFF:
    raise ProtocolError(f"config ack status out of range: {status}")

  payload = struct.pack(CONFIG_ACK_FORMAT, status)
  return encode_single_event_frame(
      CONFIG_ACK_RECORD_TYPE,
      CONFIG_ACK_SCHEMA_VERSION,
      payload,
  )


def encode_single_event_frame(
    record_type: int,
    schema_version: int,
    payload: bytes,
) -> bytes:
  if record_type < 0 or record_type > 0xFF:
    raise ProtocolError(f"record type out of range: {record_type}")
  if schema_version < 0 or schema_version > 0xFF:
    raise ProtocolError(f"schema version out of range: {schema_version}")
  if len(payload) > 0xFFFF:
    raise ProtocolError(f"event payload too large: {len(payload)}")

  body_len = ENVELOPE_HEADER_SIZE + EVENT_HEADER_SIZE + len(payload)
  if body_len > MAX_FRAME_BODY_SIZE:
    raise ProtocolError(
        f"frame body length {body_len} exceeds max {MAX_FRAME_BODY_SIZE}"
    )

  return b"".join(
      (
          struct.pack(FRAME_HEADER_FORMAT, body_len),
          struct.pack(ENVELOPE_HEADER_FORMAT, ENVELOPE_VERSION, 1),
          struct.pack(EVENT_HEADER_FORMAT, record_type, schema_version, len(payload)),
          payload,
      )
  )


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


def format_node_config(peer: str, event: Event, config: NodeConfig) -> str:
  return (
      f"client={peer} record_type={event.record_type}"
      f" schema={event.schema_version}"
      f" node={format_node_uuid(config)}"
      f" soil_sensor={config.soil_sensor_id}"
      f" ds18b20_sensor={config.ds18b20_sensor_id}"
      f" env280_sensor={config.env280_sensor_id}"
      f" dry={config.soil_dry_mv}mV"
      f" wet={config.soil_wet_mv}mV"
  )


def format_node_uuid(config: NodeConfig) -> str:
  return format_node_uuid_bytes(config.node_uuid)


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
