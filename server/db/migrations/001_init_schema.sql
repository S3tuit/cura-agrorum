BEGIN;

CREATE TABLE node_configuration (
  node_uuid uuid NOT NULL,
  valid_from timestamptz NOT NULL DEFAULT now(),
  valid_until timestamptz,

  soil_sensor_id integer NOT NULL,
  ds18b20_sensor_id integer NOT NULL,
  env280_sensor_id integer NOT NULL,
  soil_dry_mv integer NOT NULL,
  soil_wet_mv integer NOT NULL,

  PRIMARY KEY (node_uuid, valid_from),

  CHECK (valid_until IS NULL OR valid_until > valid_from),
  CHECK (soil_dry_mv > soil_wet_mv)
);

-- At most one configuration can be active for a node. Historical rows are
-- closed by setting valid_until before inserting the next active config.
CREATE UNIQUE INDEX node_configuration_one_active
  ON node_configuration (node_uuid)
  WHERE valid_until IS NULL;

CREATE FUNCTION ingest_node_config_v1(
  p_node_uuid uuid,
  p_soil_sensor_id integer,
  p_ds18b20_sensor_id integer,
  p_env280_sensor_id integer,
  p_soil_dry_mv integer,
  p_soil_wet_mv integer
) RETURNS boolean
LANGUAGE plpgsql
AS $$
DECLARE
  v_now timestamptz;
  v_active node_configuration%ROWTYPE;
BEGIN
  -- A node may reconnect while a previous config batch is still being handled.
  -- Serialize config ingestion per node so the active-row history never races
  -- the partial unique index on (node_uuid) WHERE valid_until IS NULL.
  PERFORM pg_advisory_xact_lock(hashtextextended(p_node_uuid::text, 0));

  SELECT *
    INTO v_active
    FROM node_configuration
   WHERE node_uuid = p_node_uuid
     AND valid_until IS NULL
   FOR UPDATE;

  v_now := clock_timestamp();

  -- First config batch for this node: create the initial active config.
  IF NOT FOUND THEN
    INSERT INTO node_configuration (
      node_uuid,
      valid_from,
      soil_sensor_id,
      ds18b20_sensor_id,
      env280_sensor_id,
      soil_dry_mv,
      soil_wet_mv
    ) VALUES (
      p_node_uuid,
      v_now,
      p_soil_sensor_id,
      p_ds18b20_sensor_id,
      p_env280_sensor_id,
      p_soil_dry_mv,
      p_soil_wet_mv
    );

    RETURN true;
  END IF;

  -- Repeated config batches are expected after deep sleep. If the active config
  -- has not changed, keep the existing validity window instead of adding
  -- duplicate history rows.
  IF v_active.soil_sensor_id = p_soil_sensor_id
      AND v_active.ds18b20_sensor_id = p_ds18b20_sensor_id
      AND v_active.env280_sensor_id = p_env280_sensor_id
      AND v_active.soil_dry_mv = p_soil_dry_mv
      AND v_active.soil_wet_mv = p_soil_wet_mv THEN
    RETURN false;
  END IF;

  -- Calibration or sensor identity changed. Close the old active row and open
  -- the new one at the same timestamp so validity ranges are contiguous.
  -- If two changes land in the same clock tick, nudge the new boundary forward
  -- so the primary key and validity check remain true.
  IF v_now <= v_active.valid_from THEN
    v_now := v_active.valid_from + interval '1 microsecond';
  END IF;

  UPDATE node_configuration
     SET valid_until = v_now
   WHERE node_uuid = v_active.node_uuid
     AND valid_from = v_active.valid_from;

  INSERT INTO node_configuration (
    node_uuid,
    valid_from,
    soil_sensor_id,
    ds18b20_sensor_id,
    env280_sensor_id,
    soil_dry_mv,
    soil_wet_mv
  ) VALUES (
    p_node_uuid,
    v_now,
    p_soil_sensor_id,
    p_ds18b20_sensor_id,
    p_env280_sensor_id,
    p_soil_dry_mv,
    p_soil_wet_mv
  );

  RETURN true;
END;
$$;

-- When a sensor fails the reading, its values must be NULL.
CREATE TABLE sensor_reading (
  node_uuid uuid NOT NULL,
  sample_id bigint NOT NULL,
  received_at timestamptz NOT NULL DEFAULT now(),

  bootno bigint NOT NULL,
  wake_causes bigint NOT NULL,
  run_ms integer NOT NULL,
  soil_mv integer,
  ds18b20_centi_c smallint,
  env280_centi_c smallint,
  env280_pressure_pa bigint,
  env280_humidity_centi_pct integer,

  PRIMARY KEY (node_uuid, sample_id)
);

COMMIT;
