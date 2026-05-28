# Generated Python module from protocol/schemas/config_ack_v1.json by protocol/tools/generate.py. Do not edit by hand.
from __future__ import annotations

from dataclasses import dataclass
import struct

CONFIG_ACK_SCHEMA_VERSION = 1
CONFIG_ACK_RECORD_TYPE = 3
CONFIG_ACK_FORMAT = "<I"
CONFIG_ACK_SIZE = struct.calcsize(CONFIG_ACK_FORMAT)



@dataclass(frozen=True)
class ConfigAck:
  status: int


def config_ack_from_tuple(values: tuple[object, ...]) -> ConfigAck:
  return ConfigAck(
      status=values[0],
  )
