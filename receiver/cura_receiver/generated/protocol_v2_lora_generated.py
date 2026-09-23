# Generated from protocol/protocol-v2-lora/schemas/protocol_v2_lora.json by protocol/protocol-v2-lora/tools/generate.py.
# Schema SHA-256: ece4ec58bad29ddad03be41ccb257ffef960fa11a2df1d9f8650f4eb499de009. Do not edit by hand.
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum, IntFlag
import struct

SCHEMA_SHA256 = "ece4ec58bad29ddad03be41ccb257ffef960fa11a2df1d9f8650f4eb499de009"
PROTOCOL_VERSION = 2
CONTROL = 32
KEY_SIZE = 16
NONCE_SIZE = 13
TAG_SIZE = 8
CLEAR_HEADER_SIZE = 14
READING_BODY_SIZE = 32
ACK_BODY_SIZE = 1
READING_FRAME_SIZE = 54
ACK_FRAME_SIZE = 23

CLEAR_HEADER_STRUCT = struct.Struct("<BB8sI")
READING_BODY_STRUCT = struct.Struct("<IHHHhhhIHBBHHBBH")
ACK_BODY_STRUCT = struct.Struct("<B")
NONCE_STRUCT = struct.Struct("<8sIB")

assert CLEAR_HEADER_STRUCT.size == CLEAR_HEADER_SIZE
assert READING_BODY_STRUCT.size == READING_BODY_SIZE
assert ACK_BODY_STRUCT.size == ACK_BODY_SIZE
assert NONCE_STRUCT.size == NONCE_SIZE


class CodecError(ValueError):
  """The supplied value or byte sequence is not valid protocol v2 data."""


class Domain(IntEnum):
  CURRENT_READING_UPLINK = 1
  BACKLOG_READING_UPLINK = 2
  ACK_ACCEPTED_DOWNLINK = 3
  ACK_RETRY_LATER_DOWNLINK = 4
  ACK_REJECTED_UNSUPPORTED_DOWNLINK = 5
  ACK_REJECTED_MALFORMED_DOWNLINK = 6


class AckStatus(IntEnum):
  ACCEPTED = 0
  RETRY_LATER = 1
  REJECTED_UNSUPPORTED = 2
  REJECTED_MALFORMED = 3


class ResetReason(IntEnum):
  ESP_RST_UNKNOWN = 0
  ESP_RST_POWERON = 1
  ESP_RST_EXT = 2
  ESP_RST_SW = 3
  ESP_RST_PANIC = 4
  ESP_RST_INT_WDT = 5
  ESP_RST_TASK_WDT = 6
  ESP_RST_WDT = 7
  ESP_RST_DEEPSLEEP = 8
  ESP_RST_BROWNOUT = 9
  ESP_RST_SDIO = 10
  ESP_RST_USB = 11
  ESP_RST_JTAG = 12
  ESP_RST_EFUSE = 13
  ESP_RST_PWR_GLITCH = 14
  ESP_RST_CPU_LOCKUP = 15


class ReadingFlag(IntFlag):
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


READING_RESERVED_FLAGS_MASK = 64512

ACK_STATUS_BY_DOMAIN: dict[Domain, AckStatus] = {
    Domain.ACK_ACCEPTED_DOWNLINK: AckStatus.ACCEPTED,
    Domain.ACK_RETRY_LATER_DOWNLINK: AckStatus.RETRY_LATER,
    Domain.ACK_REJECTED_UNSUPPORTED_DOWNLINK: AckStatus.REJECTED_UNSUPPORTED,
    Domain.ACK_REJECTED_MALFORMED_DOWNLINK: AckStatus.REJECTED_MALFORMED,
}


@dataclass(frozen=True)
class ClearHeader:
  control: int
  domain: int
  node_id: bytes
  message_id: int


@dataclass(frozen=True)
class Reading:
  sample_id: int
  run_ms: int
  soil_0_mv: int
  soil_1_mv: int
  soil_temp_0_centi_c: int
  soil_temp_1_centi_c: int
  enclosure_centi_c: int
  enclosure_pressure_pa: int
  enclosure_humidity_centi_pct: int
  reset_reason: int
  previous_current_tx_attempts: int
  previous_awake_ms: int
  previous_current_delivery_ms: int
  previous_cycle_tx_attempts: int
  previous_cycle_accepted_readings: int
  flags: int


@dataclass(frozen=True)
class Ack:
  status: int


def is_supported_control(control: int) -> bool:
  return control == CONTROL


def domain_is_reading(domain: int) -> bool:
  return domain in {
      Domain.CURRENT_READING_UPLINK,
      Domain.BACKLOG_READING_UPLINK,
  }


def domain_is_ack(domain: int) -> bool:
  return domain in {
      Domain.ACK_ACCEPTED_DOWNLINK,
      Domain.ACK_RETRY_LATER_DOWNLINK,
      Domain.ACK_REJECTED_UNSUPPORTED_DOWNLINK,
      Domain.ACK_REJECTED_MALFORMED_DOWNLINK,
  }


def ack_status_matches_domain(domain: int, status: int) -> bool:
  try:
    return ACK_STATUS_BY_DOMAIN[Domain(domain)] == AckStatus(status)
  except (KeyError, ValueError):
    return False


def validate_reading(reading: Reading) -> None:
  if not (reading.flags & ReadingFlag.SOIL_0_VALID) and reading.soil_0_mv != 0:
    raise CodecError("soil_0_mv must be zero when SOIL_0_VALID is clear")
  if not (reading.flags & ReadingFlag.SOIL_1_VALID) and reading.soil_1_mv != 0:
    raise CodecError("soil_1_mv must be zero when SOIL_1_VALID is clear")
  if not (reading.flags & ReadingFlag.SOIL_TEMP_0_VALID) and reading.soil_temp_0_centi_c != 0:
    raise CodecError("soil_temp_0_centi_c must be zero when SOIL_TEMP_0_VALID is clear")
  if not (reading.flags & ReadingFlag.SOIL_TEMP_1_VALID) and reading.soil_temp_1_centi_c != 0:
    raise CodecError("soil_temp_1_centi_c must be zero when SOIL_TEMP_1_VALID is clear")
  if not (reading.flags & ReadingFlag.ENCLOSURE_TEMP_VALID) and reading.enclosure_centi_c != 0:
    raise CodecError("enclosure_centi_c must be zero when ENCLOSURE_TEMP_VALID is clear")
  if not (reading.flags & ReadingFlag.ENCLOSURE_PRESSURE_VALID) and reading.enclosure_pressure_pa != 0:
    raise CodecError("enclosure_pressure_pa must be zero when ENCLOSURE_PRESSURE_VALID is clear")
  if not (reading.flags & ReadingFlag.ENCLOSURE_HUMIDITY_VALID) and reading.enclosure_humidity_centi_pct != 0:
    raise CodecError("enclosure_humidity_centi_pct must be zero when ENCLOSURE_HUMIDITY_VALID is clear")
  if bool(reading.flags & ReadingFlag.DEEP_SLEEP_BOOT) != (reading.reset_reason == 8):
    raise CodecError("DEEP_SLEEP_BOOT does not match reset_reason")
  if not (reading.flags & ReadingFlag.PREVIOUS_CYCLE_METRICS_VALID) and (reading.previous_current_tx_attempts != 0 or reading.previous_awake_ms != 0 or reading.previous_current_delivery_ms != 0 or reading.previous_cycle_tx_attempts != 0 or reading.previous_cycle_accepted_readings != 0):
    raise CodecError("previous-cycle metrics must be zero when PREVIOUS_CYCLE_METRICS_VALID is clear")
  if (reading.flags & ReadingFlag.PREVIOUS_CURRENT_ACCEPTED) and not (reading.flags & ReadingFlag.PREVIOUS_CYCLE_METRICS_VALID):
    raise CodecError("PREVIOUS_CURRENT_ACCEPTED requires PREVIOUS_CYCLE_METRICS_VALID")
  if not (reading.flags & ReadingFlag.PREVIOUS_CURRENT_ACCEPTED) and reading.previous_current_delivery_ms != 0:
    raise CodecError("previous_current_delivery_ms must be zero when PREVIOUS_CURRENT_ACCEPTED is clear")
  if reading.flags & READING_RESERVED_FLAGS_MASK:
    raise CodecError("reserved reading flag bits must be zero")


def encode_clear_header(value: ClearHeader) -> bytes:
  try:
    return CLEAR_HEADER_STRUCT.pack(
        value.control,
        value.domain,
        value.node_id,
        value.message_id,
    )
  except (struct.error, TypeError) as exc:
    raise CodecError(f"invalid clear_header: {exc}") from exc


def decode_clear_header(data: bytes) -> ClearHeader:
  if len(data) != CLEAR_HEADER_STRUCT.size:
    raise CodecError("clear_header must be "
                     f"{CLEAR_HEADER_STRUCT.size} bytes, got {len(data)}")
  values = CLEAR_HEADER_STRUCT.unpack(data)
  result = ClearHeader(
      control=values[0],
      domain=values[1],
      node_id=values[2],
      message_id=values[3],
  )
  return result


def build_nonce(header: ClearHeader) -> bytes:
  try:
    return NONCE_STRUCT.pack(
        header.node_id,
        header.message_id,
        header.domain,
    )
  except (struct.error, TypeError) as exc:
    raise CodecError(f"invalid nonce input: {exc}") from exc


def encode_reading(value: Reading) -> bytes:
  validate_reading(value)
  try:
    return READING_BODY_STRUCT.pack(
        value.sample_id,
        value.run_ms,
        value.soil_0_mv,
        value.soil_1_mv,
        value.soil_temp_0_centi_c,
        value.soil_temp_1_centi_c,
        value.enclosure_centi_c,
        value.enclosure_pressure_pa,
        value.enclosure_humidity_centi_pct,
        value.reset_reason,
        value.previous_current_tx_attempts,
        value.previous_awake_ms,
        value.previous_current_delivery_ms,
        value.previous_cycle_tx_attempts,
        value.previous_cycle_accepted_readings,
        value.flags,
    )
  except (struct.error, TypeError) as exc:
    raise CodecError(f"invalid reading: {exc}") from exc


def decode_reading(data: bytes) -> Reading:
  if len(data) != READING_BODY_STRUCT.size:
    raise CodecError("reading must be "
                     f"{READING_BODY_STRUCT.size} bytes, got {len(data)}")
  values = READING_BODY_STRUCT.unpack(data)
  result = Reading(
      sample_id=values[0],
      run_ms=values[1],
      soil_0_mv=values[2],
      soil_1_mv=values[3],
      soil_temp_0_centi_c=values[4],
      soil_temp_1_centi_c=values[5],
      enclosure_centi_c=values[6],
      enclosure_pressure_pa=values[7],
      enclosure_humidity_centi_pct=values[8],
      reset_reason=values[9],
      previous_current_tx_attempts=values[10],
      previous_awake_ms=values[11],
      previous_current_delivery_ms=values[12],
      previous_cycle_tx_attempts=values[13],
      previous_cycle_accepted_readings=values[14],
      flags=values[15],
  )
  validate_reading(result)
  return result


def validate_ack(ack: Ack) -> None:
  try:
    AckStatus(ack.status)
  except (TypeError, ValueError) as exc:
    raise CodecError(f"unknown ACK status: {ack.status}") from exc


def encode_ack(value: Ack) -> bytes:
  validate_ack(value)
  try:
    return ACK_BODY_STRUCT.pack(
        value.status,
    )
  except (struct.error, TypeError) as exc:
    raise CodecError(f"invalid ack: {exc}") from exc


def decode_ack(data: bytes) -> Ack:
  if len(data) != ACK_BODY_STRUCT.size:
    raise CodecError("ack must be "
                     f"{ACK_BODY_STRUCT.size} bytes, got {len(data)}")
  values = ACK_BODY_STRUCT.unpack(data)
  result = Ack(
      status=values[0],
  )
  validate_ack(result)
  return result

