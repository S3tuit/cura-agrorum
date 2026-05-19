from __future__ import annotations

import argparse
import asyncio
import logging
from collections.abc import Sequence

from .protocol import (
    DEFAULT_PORT,
    HEADER_SIZE,
    ProtocolError,
    decode_readings,
    decode_node_config,
    format_reading,
    format_node_config,
    format_node_uuid,
    format_unsupported_frame,
    is_supported_reading_frame,
    is_supported_node_config_frame,
    parse_header,
)

LOGGER = logging.getLogger("cura_server")


async def handle_client(
    reader: asyncio.StreamReader,
    writer: asyncio.StreamWriter,
    logger: logging.Logger = LOGGER,
) -> None:
  peer = _peer_label(writer)
  logger.info("client=%s connected", peer)
  node_uuid: str | None = None

  try:
    while True:
      try:
        header_data = await reader.readexactly(HEADER_SIZE)
      except asyncio.IncompleteReadError as exc:
        if exc.partial:
          logger.warning(
              "client=%s closed with partial header len=%d",
              peer,
              len(exc.partial),
          )
        break

      try:
        header = parse_header(header_data)
        payload = await reader.readexactly(header.payload_len)
      except asyncio.IncompleteReadError as exc:
        logger.warning(
            "client=%s closed with partial payload expected=%d got=%d",
            peer,
            exc.expected,
            len(exc.partial),
        )
        break
      except ProtocolError as exc:
        logger.warning("client=%s malformed header: %s", peer, exc)
        break

      if not is_supported_reading_frame(header):
        if is_supported_node_config_frame(header):
          try:
            node_config = decode_node_config(payload)
          except ProtocolError as exc:
            logger.warning(
                "client=%s malformed node_config payload record_type=%d schema=%d len=%d: %s",
                peer,
                header.record_type,
                header.schema_version,
                header.payload_len,
                exc,
            )
            break

          node_uuid = format_node_uuid(node_config)
          logger.info(format_node_config(peer, header, node_config))
          continue

        logger.warning(format_unsupported_frame(peer, header, payload))
        continue

      if node_uuid is None:
        logger.warning(
            "client=%s reading frame received before node_config; closing",
            peer,
        )
        break

      try:
        readings = decode_readings(payload)
      except ProtocolError as exc:
        logger.warning(
            "client=%s malformed reading payload record_type=%d schema=%d len=%d: %s",
            peer,
            header.record_type,
            header.schema_version,
            header.payload_len,
            exc,
        )
        continue

      total = len(readings)
      for index, reading in enumerate(readings, start=1):
        logger.info(
            format_reading(peer, header, reading, index, total, node_uuid=node_uuid)
        )
  except ConnectionResetError:
    logger.warning("client=%s reset connection", peer)
  finally:
    writer.close()
    try:
      await writer.wait_closed()
    except ConnectionResetError:
      pass
    logger.info("client=%s disconnected", peer)


async def start_server(
    host: str,
    port: int,
    logger: logging.Logger = LOGGER,
) -> asyncio.AbstractServer:
  return await asyncio.start_server(
      lambda reader, writer: handle_client(reader, writer, logger),
      host,
      port,
  )


async def run_server(host: str, port: int, logger: logging.Logger = LOGGER) -> None:
  server = await start_server(host, port, logger)
  addresses = ", ".join(str(sock.getsockname()) for sock in server.sockets or [])
  logger.info("listening on %s", addresses)

  async with server:
    await server.serve_forever()


def main(argv: Sequence[str] | None = None) -> int:
  parser = argparse.ArgumentParser(description="Cura Agrorum TCP gateway server")
  parser.add_argument("--host", default="0.0.0.0", help="address to listen on")
  parser.add_argument(
      "--port",
      type=int,
      default=DEFAULT_PORT,
      help=f"TCP port to listen on, defaults to {DEFAULT_PORT}",
  )
  parser.add_argument(
      "--log-level",
      default="INFO",
      choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
      help="minimum log level",
  )
  args = parser.parse_args(argv)

  logging.basicConfig(
      level=getattr(logging, args.log_level),
      format="%(asctime)s %(levelname)s %(name)s: %(message)s",
  )

  try:
    asyncio.run(run_server(args.host, args.port))
  except KeyboardInterrupt:
    LOGGER.info("stopped")
  return 0


def _peer_label(writer: asyncio.StreamWriter) -> str:
  peer = writer.get_extra_info("peername")
  if isinstance(peer, tuple) and len(peer) >= 2:
    return f"{peer[0]}:{peer[1]}"
  return str(peer)
