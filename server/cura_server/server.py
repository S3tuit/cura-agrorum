from __future__ import annotations

import argparse
import asyncio
import logging
import os
from collections.abc import Sequence
from uuid import UUID

from .db import DEFAULT_DATABASE_URL, Database
from .protocol import (
    CURA_RECORD_TYPE,
    DEFAULT_PORT,
    HEADER_SIZE,
    HANDSHAKE_ACK_ERROR,
    HANDSHAKE_ACK_OK,
    NODE_CONFIG_RECORD_TYPE,
    ProtocolError,
    decode_reading,
    decode_node_config,
    encode_handshake_ack,
    format_reading,
    format_node_config,
    format_node_uuid_bytes,
    format_unsupported_frame,
    is_supported_reading_frame,
    is_supported_node_config_frame,
    node_uuid_from_bytes,
    parse_header,
)

LOGGER = logging.getLogger("cura_server")

# Process-local handshake registry. The node config itself is not cached here
# because Postgres is the durable source of truth for config history.
HANDSHAKED_NODES: set[UUID] = set()

# Sensor nodes should finish a wake-cycle connection quickly. If WiFi drops or
# deep sleep cuts the TCP close short, this timeout prevents orphaned handlers
# from waiting on readexactly() until the OS TCP stack eventually gives up.
CLIENT_READ_TIMEOUT_SECONDS = 5.0


async def handle_client(
    reader: asyncio.StreamReader,
    writer: asyncio.StreamWriter,
    database: Database,
    logger: logging.Logger = LOGGER,
    handshaked_nodes: set[UUID] | None = None,
) -> None:
  peer = _peer_label(writer)
  logger.info("client=%s connected", peer)
  if handshaked_nodes is None:
    handshaked_nodes = HANDSHAKED_NODES

  try:
    while True:
      try:
        header_data = await _read_exactly_with_timeout(reader, HEADER_SIZE)
      except TimeoutError:
        logger.warning("client=%s timed out waiting for header", peer)
        break
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
      except ProtocolError as exc:
        logger.warning("client=%s malformed header: %s", peer, exc)
        break

      try:
        payload = await _read_exactly_with_timeout(reader, header.payload_len)
      except TimeoutError:
        logger.warning(
            "client=%s timed out waiting for payload expected=%d",
            peer,
            header.payload_len,
        )
        break
      except asyncio.IncompleteReadError as exc:
        logger.warning(
            "client=%s closed with partial payload expected=%d got=%d",
            peer,
            exc.expected,
            len(exc.partial),
        )
        break

      if header.record_type == NODE_CONFIG_RECORD_TYPE:
        if not is_supported_node_config_frame(header):
          logger.warning(format_unsupported_frame(peer, header, payload))
          await _send_handshake_ack(writer, HANDSHAKE_ACK_ERROR)
          break

        try:
          node_config = decode_node_config(payload)
          node_uuid = node_uuid_from_bytes(node_config.node_uuid)
        except ProtocolError as exc:
          logger.warning(
              "client=%s malformed node_config payload record_type=%d schema=%d len=%d: %s",
              peer,
              header.record_type,
              header.schema_version,
              header.payload_len,
              exc,
          )
          await _send_handshake_ack(writer, HANDSHAKE_ACK_ERROR)
          break

        try:
          await database.ingest_node_config_v1(node_uuid, node_config)
        except Exception:
          logger.exception(
              "client=%s node_config persistence failed node=%s",
              peer,
              format_node_uuid_bytes(node_config.node_uuid),
          )
          await _send_handshake_ack(writer, HANDSHAKE_ACK_ERROR)
          break

        # Handshake means the server durably accepted the node identity/config.
        # The process-local set is still useful while firmware caches a session,
        # but Postgres is now the source of truth for config history.
        handshaked_nodes.add(node_uuid)
        logger.info(format_node_config(peer, header, node_config))
        await _send_handshake_ack(writer, HANDSHAKE_ACK_OK)
        continue

      if header.record_type == CURA_RECORD_TYPE:
        if not is_supported_reading_frame(header):
          logger.warning(format_unsupported_frame(peer, header, payload))
          continue

        try:
          reading = decode_reading(payload)
          node_uuid = node_uuid_from_bytes(reading.node_uuid)
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

        # Nodes cache a successful handshake across deep sleep, while this
        # in-memory registry is lost on server restart. Accept the reading
        # because Postgres has the durable config history, but keep the missing
        # process-local handshake visible in the logs.
        if node_uuid not in handshaked_nodes:
          logger.warning(
              "client=%s reading from node=%s without handshake in current process",
              peer,
              format_node_uuid_bytes(reading.node_uuid),
          )

        try:
          await database.insert_reading_v1(node_uuid, reading)
        except Exception:
          logger.exception(
              "client=%s reading persistence failed node=%s sample_id=%d",
              peer,
              format_node_uuid_bytes(reading.node_uuid),
              reading.sample_id,
          )
          break

        # Echo after persistence so the log line means the reading made it to
        # Postgres. Reading duplicates are accepted through the same code path:
        # (node_uuid, sample_id) is the durable idempotency key.
        logger.info(format_reading(peer, header, reading))
        continue

      logger.warning(format_unsupported_frame(peer, header, payload))
      continue
  except ConnectionResetError:
    logger.warning("client=%s reset connection", peer)
  finally:
    writer.close()
    try:
      await writer.wait_closed()
    except ConnectionResetError:
      pass
    logger.info("client=%s disconnected", peer)


def _peer_label(writer: asyncio.StreamWriter) -> str:
  peer = writer.get_extra_info("peername")
  if isinstance(peer, tuple) and len(peer) >= 2:
    return f"{peer[0]}:{peer[1]}"
  return str(peer)


async def _send_handshake_ack(
    writer: asyncio.StreamWriter,
    status: int,
) -> None:
  writer.write(encode_handshake_ack(status))
  await writer.drain()


async def _read_exactly_with_timeout(
    reader: asyncio.StreamReader,
    size: int,
) -> bytes:
  return await asyncio.wait_for(
      reader.readexactly(size),
      timeout=CLIENT_READ_TIMEOUT_SECONDS,
  )


async def start_server(
    host: str,
    port: int,
    database: Database,
    logger: logging.Logger = LOGGER,
    handshaked_nodes: set[UUID] | None = None,
) -> asyncio.AbstractServer:
  if handshaked_nodes is None:
    handshaked_nodes = HANDSHAKED_NODES

  return await asyncio.start_server(
      lambda reader, writer: handle_client(
          reader,
          writer,
          database,
          logger,
          handshaked_nodes,
      ),
      host,
      port,
  )


async def run_server(
    host: str,
    port: int,
    database_url: str,
    logger: logging.Logger = LOGGER,
) -> None:
  database = Database(database_url)
  try:
    await database.open()
    logger.info("connected to postgres")

    server = await start_server(host, port, database, logger)
    addresses = ", ".join(str(sock.getsockname()) for sock in server.sockets or [])
    logger.info("listening on %s", addresses)

    async with server:
      await server.serve_forever()
  finally:
    await database.close()


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
  parser.add_argument(
      "--database-url",
      default=os.environ.get("DATABASE_URL", DEFAULT_DATABASE_URL),
      help="Postgres connection URL",
  )
  args = parser.parse_args(argv)

  logging.basicConfig(
      level=getattr(logging, args.log_level),
      format="%(asctime)s %(levelname)s %(name)s: %(message)s",
  )

  try:
    asyncio.run(run_server(args.host, args.port, args.database_url))
  except KeyboardInterrupt:
    LOGGER.info("stopped")
  return 0
