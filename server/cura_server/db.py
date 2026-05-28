from __future__ import annotations

from uuid import UUID

from psycopg_pool import AsyncConnectionPool

from .generated.node_config_v1 import NodeConfig
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

  async def ingest_node_config_v1(
      self,
      node_uuid: UUID,
      config: NodeConfig,
  ) -> bool:
    async with self._pool.connection() as conn:
      return await _ingest_node_config_v1(conn, node_uuid, config)

  async def insert_reading_v1(self, node_uuid: UUID, reading: Reading) -> bool:
    async with self._pool.connection() as conn:
      return await _insert_reading_v1(conn, node_uuid, reading)


async def _ingest_node_config_v1(conn, node_uuid: UUID, config: NodeConfig) -> bool:
  cursor = await conn.execute(
      """
      SELECT ingest_node_config_v1(%s, %s, %s, %s, %s, %s)
      """,
      (
          node_uuid,
          config.soil_sensor_id,
          config.ds18b20_sensor_id,
          config.env280_sensor_id,
          config.soil_dry_mv,
          config.soil_wet_mv,
      ),
  )
  row = await cursor.fetchone()
  return bool(row[0]) if row is not None else False


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


def _valid_field(flags: int, flag: int, value: int) -> int | None:
  if (flags & flag) == 0:
    return None
  return value
