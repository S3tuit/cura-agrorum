from __future__ import annotations

from uuid import UUID

from psycopg_pool import AsyncConnectionPool

from .generated.fault_v1 import Fault
from .generated.reading_v1 import (
    READING_DS18B20_TEMP_OK,
    READING_ENV280_HUMIDITY_OK,
    READING_ENV280_PRESSURE_OK,
    READING_ENV280_TEMP_OK,
    READING_SOIL_MV_OK,
    Reading,
)

DEFAULT_DATABASE_URL = "postgresql://cura:cura_dev_password@localhost:55432/cura_agrorum"
DATABASE_CONNECT_TIMEOUT_SECONDS = 10.0


class Database:
  def __init__(self, database_url: str) -> None:
    self._pool = AsyncConnectionPool(
        database_url,
        min_size=1,
        max_size=4,
        open=False,
    )

  async def open(self) -> None:
    await self._pool.open()
    await self._pool.wait(timeout=DATABASE_CONNECT_TIMEOUT_SECONDS)

  async def close(self) -> None:
    await self._pool.close()

  async def load_configured_nodes(self) -> set[UUID]:
    async with self._pool.connection() as conn:
      return await _load_configured_nodes(conn)

  async def insert_reading_v1(self, node_uuid: UUID, reading: Reading) -> bool:
    async with self._pool.connection() as conn:
      return await _insert_reading_v1(conn, node_uuid, reading)

  async def insert_fault_v1(self, node_uuid: UUID, fault: Fault) -> bool:
    async with self._pool.connection() as conn:
      return await _insert_fault_v1(conn, node_uuid, fault)


async def _load_configured_nodes(conn) -> set[UUID]:
  cursor = await conn.execute(
      """
      SELECT DISTINCT node_uuid
      FROM node_configuration
      WHERE valid_until IS NULL
      """
  )
  rows = await cursor.fetchall()
  return {
      node_uuid if isinstance(node_uuid, UUID) else UUID(str(node_uuid))
      for (node_uuid,) in rows
  }


async def _insert_reading_v1(conn, node_uuid: UUID, reading: Reading) -> bool:
  # The table stores semantic nullable sensor values. The wire flags remain
  # a protocol detail that is resolved before the raw insert.
  cursor = await conn.execute(
      """
      INSERT INTO sensor_reading (
        node_uuid,
        sample_id,
        bootno,
        wake_causes,
        run_ms,
        soil_mv,
        ds18b20_centi_c,
        env280_centi_c,
        env280_pressure_pa,
        env280_humidity_centi_pct
      ) VALUES (
        %s, %s, %s, %s, %s, %s, %s, %s, %s, %s
      )
      ON CONFLICT (node_uuid, sample_id) DO NOTHING
      RETURNING 1
      """,
      (
          node_uuid,
          reading.sample_id,
          reading.bootno,
          reading.wake_causes,
          reading.run_ms,
          _valid_field(reading.flags, READING_SOIL_MV_OK, reading.soil_mv),
          _valid_field(
              reading.flags,
              READING_DS18B20_TEMP_OK,
              reading.ds18b20_centi_c,
          ),
          _valid_field(
              reading.flags,
              READING_ENV280_TEMP_OK,
              reading.env280_centi_c,
          ),
          _valid_field(
              reading.flags,
              READING_ENV280_PRESSURE_OK,
              reading.env280_pressure_pa,
          ),
          _valid_field(
              reading.flags,
              READING_ENV280_HUMIDITY_OK,
              reading.env280_humidity_centi_pct,
          ),
      ),
  )
  row = await cursor.fetchone()
  return row is not None


async def _insert_fault_v1(conn, node_uuid: UUID, fault: Fault) -> bool:
  cursor = await conn.execute(
      """
      INSERT INTO node_fault (
        node_uuid,
        fault_id,
        sample_id,
        bootno,
        operation,
        esp_err,
        posix_errno
      ) VALUES (
        %s, %s, %s, %s, %s, %s, %s
      )
      ON CONFLICT (node_uuid, fault_id) DO NOTHING
      RETURNING 1
      """,
      (
          node_uuid,
          fault.fault_id,
          fault.sample_id,
          fault.bootno,
          fault.operation,
          fault.esp_err,
          fault.posix_errno,
      ),
  )
  row = await cursor.fetchone()
  return row is not None


def _valid_field(flags: int, flag: int, value: int) -> int | None:
  if (flags & flag) == 0:
    return None
  return value
