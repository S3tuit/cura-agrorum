from __future__ import annotations

import argparse
import asyncio
import logging
import os
from collections.abc import Sequence
from uuid import UUID

from .db import DEFAULT_DATABASE_URL, Database
from .protocol import (
    CONFIG_ACK_ERROR,
    CONFIG_ACK_OK,
    CURA_RECORD_TYPE,
    DEFAULT_PORT,
    Event,
    FRAME_HEADER_SIZE,
    NODE_CONFIG_RECORD_TYPE,
    ProtocolError,
    decode_reading,
    decode_node_config,
    encode_config_ack,
    format_reading,
    format_node_config,
    format_node_uuid_bytes,
    format_unsupported_event,
    is_supported_reading_event,
    is_supported_node_config_event,
    node_uuid_from_bytes,
    parse_frame_body,
    parse_frame_header,
)

LOGGER = logging.getLogger("cura_server")

# Process-local config registry. The node config itself is not cached here
# because Postgres is the durable source of truth for config history.
CONFIGURED_NODES: set[UUID] = set()

# Sensor nodes should finish a wake-cycle connection quickly. If WiFi drops or
# deep sleep cuts the TCP close short, this timeout prevents orphaned handlers
# from waiting on readexactly() until the OS TCP stack eventually gives up.
CLIENT_READ_TIMEOUT_SECONDS = 5.0


async def handle_client(
    reader: asyncio.StreamReader,
    writer: asyncio.StreamWriter,
    database: Database,
    logger: logging.Logger = LOGGER,
    configured_nodes: set[UUID] | None = None,
) -> None:
  peer = _peer_label(writer)
  logger.info("client=%s connected", peer)
  if configured_nodes is None:
    configured_nodes = CONFIGURED_NODES

  try:
    while True:
      try:
        frame_header_data = await _read_exactly_with_timeout(reader, FRAME_HEADER_SIZE)
      except TimeoutError:
        logger.warning("client=%s timed out waiting for frame header", peer)
        break
      except asyncio.IncompleteReadError as exc:
        if exc.partial:
          logger.warning(
              "client=%s closed with partial frame header len=%d",
              peer,
              len(exc.partial),
          )
        break

      try:
        body_len = parse_frame_header(frame_header_data)
      except ProtocolError as exc:
        logger.warning("client=%s malformed frame header: %s", peer, exc)
        break

      try:
        body = await _read_exactly_with_timeout(reader, body_len)
      except TimeoutError:
        logger.warning(
            "client=%s timed out waiting for frame body expected=%d",
            peer,
            body_len,
        )
        break
      except asyncio.IncompleteReadError as exc:
        logger.warning(
            "client=%s closed with partial frame body expected=%d got=%d",
            peer,
            exc.expected,
            len(exc.partial),
        )
        break

      try:
        events = parse_frame_body(body)
      except ProtocolError as exc:
        logger.warning("client=%s malformed frame body: %s", peer, exc)
        break

      if any(event.record_type == NODE_CONFIG_RECORD_TYPE for event in events):
        close_connection = await _handle_config_batch(
            writer,
            peer,
            events,
            database,
            logger,
            configured_nodes,
        )
      else:
        close_connection = await _handle_reading_batch(
            peer,
            events,
            database,
            logger,
            configured_nodes,
        )

      if close_connection:
        break
  except ConnectionResetError:
    logger.warning("client=%s reset connection", peer)
  finally:
    writer.close()
    try:
      await writer.wait_closed()
    except ConnectionResetError:
      pass
    logger.info("client=%s disconnected", peer)


async def _handle_config_batch(
    writer: asyncio.StreamWriter,
    peer: str,
    events: Sequence[Event],
    database: Database,
    logger: logging.Logger,
    configured_nodes: set[UUID],
) -> bool:
  """Handle a frame whose first event must be node_config_t.

  Business rule: CONFIG_ACK only means the node_config_t was persisted. Same
  frame readings are ingested independently after the ACK, so a bad reading
  cannot make the firmware retry a config the server already accepted.
  """
  if not events or events[0].record_type != NODE_CONFIG_RECORD_TYPE:
    logger.warning("client=%s config batch must start with node_config", peer)
    await _send_config_ack(writer, CONFIG_ACK_ERROR)
    return True

  if (
      sum(1 for event in events if event.record_type == NODE_CONFIG_RECORD_TYPE)
      != 1
  ):
    logger.warning("client=%s config batch contains multiple node_config events", peer)
    await _send_config_ack(writer, CONFIG_ACK_ERROR)
    return True

  config_event = events[0]
  if not is_supported_node_config_event(config_event):
    logger.warning(format_unsupported_event(peer, config_event))
    await _send_config_ack(writer, CONFIG_ACK_ERROR)
    return True

  try:
    node_config = decode_node_config(config_event.payload)
    node_uuid = node_uuid_from_bytes(node_config.node_uuid)
  except ProtocolError as exc:
    logger.warning(
        "client=%s malformed node_config payload record_type=%d schema=%d len=%d: %s",
        peer,
        config_event.record_type,
        config_event.schema_version,
        config_event.payload_len,
        exc,
    )
    await _send_config_ack(writer, CONFIG_ACK_ERROR)
    return True

  try:
    await database.ingest_node_config_v1(node_uuid, node_config)
  except Exception:
    logger.exception(
        "client=%s node_config persistence failed node=%s",
        peer,
        format_node_uuid_bytes(node_config.node_uuid),
    )
    await _send_config_ack(writer, CONFIG_ACK_ERROR)
    return True

  configured_nodes.add(node_uuid)
  await _send_config_ack(writer, CONFIG_ACK_OK)
  logger.info(format_node_config(peer, config_event, node_config))

  total_events = len(events)
  for event_index, event in enumerate(events[1:], start=2):
    close_connection = await _process_reading_event(
        peer,
        event,
        event_index,
        total_events,
        database,
        logger,
        configured_nodes,
        expected_node_uuid=node_uuid,
        expected_node_uuid_label=format_node_uuid_bytes(node_config.node_uuid),
    )
    if close_connection:
      return True

  return False


async def _handle_reading_batch(
    peer: str,
    events: Sequence[Event],
    database: Database,
    logger: logging.Logger,
    configured_nodes: set[UUID],
) -> bool:
  total_events = len(events)
  for event_index, event in enumerate(events, start=1):
    close_connection = await _process_reading_event(
        peer,
        event,
        event_index,
        total_events,
        database,
        logger,
        configured_nodes,
    )
    if close_connection:
      return True

  return False


async def _process_reading_event(
    peer: str,
    event: Event,
    event_index: int,
    total_events: int,
    database: Database,
    logger: logging.Logger,
    configured_nodes: set[UUID],
    expected_node_uuid: UUID | None = None,
    expected_node_uuid_label: str | None = None,
) -> bool:
  """Persist one reading event, returning True only when the client should close.

  Reading events are best-effort ingest. Malformed or mismatched readings are
  logged and dropped; database failures close the connection because the server
  cannot make progress on the remaining events with confidence.
  """
  if event.record_type != CURA_RECORD_TYPE:
    logger.warning(format_unsupported_event(peer, event))
    return False

  if not is_supported_reading_event(event):
    logger.warning(format_unsupported_event(peer, event))
    return False

  try:
    reading = decode_reading(event.payload)
    node_uuid = node_uuid_from_bytes(reading.node_uuid)
  except ProtocolError as exc:
    logger.warning(
        "client=%s malformed reading payload record_type=%d schema=%d len=%d: %s",
        peer,
        event.record_type,
        event.schema_version,
        event.payload_len,
        exc,
    )
    return False

  if expected_node_uuid is not None and node_uuid != expected_node_uuid:
    logger.warning(
        "client=%s config batch node mismatch config=%s reading=%s sample_id=%d",
        peer,
        expected_node_uuid_label or str(expected_node_uuid),
        format_node_uuid_bytes(reading.node_uuid),
        reading.sample_id,
    )
    return False

  # Nodes cache a successful config ACK across deep sleep, while this
  # in-memory registry is lost on server restart. Accept the reading because
  # Postgres has the durable config history, but keep the missing
  # process-local config visible in the logs.
  if node_uuid not in configured_nodes:
    logger.warning(
        "client=%s reading from node=%s without config in current process",
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
    return True

  # Echo after persistence so the log line means the reading made it to
  # Postgres. Reading duplicates are accepted through the same code path:
  # (node_uuid, sample_id) is the durable idempotency key.
  logger.info(format_reading(peer, event, reading, event_index, total_events))
  return False


def _peer_label(writer: asyncio.StreamWriter) -> str:
  peer = writer.get_extra_info("peername")
  if isinstance(peer, tuple) and len(peer) >= 2:
    return f"{peer[0]}:{peer[1]}"
  return str(peer)


async def _send_config_ack(
    writer: asyncio.StreamWriter,
    status: int,
) -> None:
  writer.write(encode_config_ack(status))
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
    configured_nodes: set[UUID] | None = None,
) -> asyncio.AbstractServer:
  if configured_nodes is None:
    configured_nodes = CONFIGURED_NODES

  return await asyncio.start_server(
      lambda reader, writer: handle_client(
          reader,
          writer,
          database,
          logger,
          configured_nodes,
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
