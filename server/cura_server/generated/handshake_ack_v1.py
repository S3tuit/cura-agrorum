# Generated Python module from protocol/schemas/handshake_ack_v1.json by protocol/tools/generate.py. Do not edit by hand.
from __future__ import annotations

from dataclasses import dataclass
import struct

HANDSHAKE_ACK_SCHEMA_VERSION = 1
HANDSHAKE_ACK_RECORD_TYPE = 3
HANDSHAKE_ACK_FORMAT = "<I"
HANDSHAKE_ACK_SIZE = struct.calcsize(HANDSHAKE_ACK_FORMAT)



@dataclass(frozen=True)
class HandshakeAck:
  status: int


def handshake_ack_from_tuple(values: tuple[object, ...]) -> HandshakeAck:
  return HandshakeAck(
      status=values[0],
  )
