BEGIN;

CREATE TABLE node_configuration (
  node_uuid uuid NOT NULL,
  valid_from timestamptz NOT NULL DEFAULT now(),
  valid_until timestamptz,

  soil_sensor_id integer NOT NULL,
  ds18b20_sensor_id integer NOT NULL,
  env280_sensor_id integer NOT NULL,

  PRIMARY KEY (node_uuid, valid_from),

  CHECK (valid_until IS NULL OR valid_until > valid_from)
);

-- At most one configuration can be active for a node. Historical rows are
-- closed by setting valid_until before inserting the next active config.
CREATE UNIQUE INDEX node_configuration_one_active
  ON node_configuration (node_uuid)
  WHERE valid_until IS NULL;

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

CREATE TABLE node_fault (
  node_uuid uuid NOT NULL,
  fault_id bytea NOT NULL,
  received_at timestamptz NOT NULL DEFAULT now(),

  sample_id bigint NOT NULL,
  bootno bigint NOT NULL,
  operation integer NOT NULL,
  esp_err integer NOT NULL,
  posix_errno integer NOT NULL,

  PRIMARY KEY (node_uuid, fault_id),

  CHECK (octet_length(fault_id) = 8),
  CHECK (operation > 0),
  CHECK (posix_errno >= 0)
);

COMMIT;
