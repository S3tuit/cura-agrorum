# Generated Python module from protocol/schemas/ack_v1.json by protocol/tools/generate.py. Do not edit by hand.
from __future__ import annotations

from dataclasses import dataclass
import struct

ACK_SCHEMA_VERSION = 1
ACK_RECORD_TYPE = 2
ACK_FORMAT = "<I"
ACK_SIZE = struct.calcsize(ACK_FORMAT)



@dataclass(frozen=True)
class Ack:
  status: int


def ack_from_tuple(values: tuple[object, ...]) -> Ack:
  return Ack(
      status=values[0],
  )
