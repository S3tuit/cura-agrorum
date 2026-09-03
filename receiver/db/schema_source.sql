-- Handwritten receiver database schema source.
--
-- receiver/tools/generate.py appends generated enum catalogues and entity
-- tables to this source to produce schema.sql. The initializer owns the
-- surrounding transaction, sets PRAGMA application_id, and inserts the one
-- metadata row only after it has verified the exact schema.sql fingerprint
-- expected by the application.

CREATE TABLE database_metadata (
    singleton_id INTEGER PRIMARY KEY CHECK (singleton_id = 1),
    group_id BLOB NOT NULL CHECK (length(group_id) = 8),
    database_schema_version INTEGER NOT NULL
        CHECK (database_schema_version BETWEEN 1 AND 4294967295),
    database_schema_fingerprint BLOB NOT NULL
        CHECK (length(database_schema_fingerprint) = 32)
) STRICT;

CREATE TRIGGER database_metadata_no_reinsert
BEFORE INSERT ON database_metadata
WHEN EXISTS (SELECT 1 FROM database_metadata)
BEGIN
    SELECT RAISE(ABORT, 'database_metadata is immutable');
END;

CREATE TRIGGER database_metadata_no_update
BEFORE UPDATE ON database_metadata
BEGIN
    SELECT RAISE(ABORT, 'database_metadata is immutable');
END;

CREATE TRIGGER database_metadata_no_delete
BEFORE DELETE ON database_metadata
BEGIN
    SELECT RAISE(ABORT, 'database_metadata is immutable');
END;

CREATE TABLE receiver_instances (
    instance_ordinal INTEGER PRIMARY KEY,
    receiver_instance_id BLOB NOT NULL UNIQUE
        CHECK (length(receiver_instance_id) = 16),
    linux_boot_id BLOB NOT NULL
        CHECK (length(linux_boot_id) = 16),
    started_at_monotonic_us INTEGER NOT NULL
        CHECK (started_at_monotonic_us >= 0),
    clean_stopped_at_monotonic_us INTEGER,
    clean_stop_state_generation INTEGER
        CHECK (
            clean_stop_state_generation IS NULL
            OR clean_stop_state_generation >= 0
        ),
    CHECK (
        (clean_stopped_at_monotonic_us IS NULL
         AND clean_stop_state_generation IS NULL)
        OR
        (clean_stopped_at_monotonic_us IS NOT NULL
         AND clean_stop_state_generation IS NOT NULL
         AND clean_stopped_at_monotonic_us >= started_at_monotonic_us)
    )
) STRICT;

CREATE TRIGGER receiver_instances_start_open
BEFORE INSERT ON receiver_instances
WHEN NEW.clean_stopped_at_monotonic_us IS NOT NULL
     OR NEW.clean_stop_state_generation IS NOT NULL
BEGIN
    SELECT RAISE(ABORT, 'receiver_instances must start without a clean-stop marker');
END;

CREATE TRIGGER receiver_instances_no_replace
BEFORE INSERT ON receiver_instances
WHEN EXISTS (
    SELECT 1
    FROM receiver_instances
    WHERE instance_ordinal IS NEW.instance_ordinal
       OR receiver_instance_id IS NEW.receiver_instance_id
)
BEGIN
    SELECT RAISE(ABORT, 'receiver_instances is append-only');
END;

CREATE TRIGGER receiver_instances_monotonic_ordinal
AFTER INSERT ON receiver_instances
WHEN NEW.instance_ordinal != (SELECT count(*) FROM receiver_instances)
BEGIN
    SELECT RAISE(ABORT, 'receiver_instances ordinal must be database ordered');
END;

CREATE TRIGGER receiver_instances_controlled_update
BEFORE UPDATE ON receiver_instances
WHEN NOT (
    OLD.clean_stopped_at_monotonic_us IS NULL
    AND OLD.clean_stop_state_generation IS NULL
    AND NEW.instance_ordinal IS OLD.instance_ordinal
    AND NEW.receiver_instance_id IS OLD.receiver_instance_id
    AND NEW.linux_boot_id IS OLD.linux_boot_id
    AND NEW.started_at_monotonic_us IS OLD.started_at_monotonic_us
    AND NEW.clean_stopped_at_monotonic_us IS NOT NULL
    AND NEW.clean_stop_state_generation IS NOT NULL
)
BEGIN
    SELECT RAISE(ABORT, 'receiver_instances permits only one complete clean-stop transition');
END;

CREATE TRIGGER receiver_instances_no_delete
BEFORE DELETE ON receiver_instances
BEGIN
    SELECT RAISE(ABORT, 'receiver_instances is append-only');
END;

CREATE TABLE quarantined_communicator_states (
    quarantined_state_id INTEGER PRIMARY KEY,
    observed_singleton_id ANY,
    observed_state_format_version ANY,
    observed_generation ANY,
    observed_state_blob ANY,
    observed_state_sha256 ANY,
    calculated_blob_sha256 BLOB
        CHECK (
            calculated_blob_sha256 IS NULL
            OR length(calculated_blob_sha256) = 32
        ),
    preserved_by_receiver_instance_id BLOB NOT NULL
        REFERENCES receiver_instances(receiver_instance_id)
        CHECK (length(preserved_by_receiver_instance_id) = 16),
    preserved_at_monotonic_us INTEGER NOT NULL
        CHECK (preserved_at_monotonic_us >= 0),
    database_schema_version INTEGER NOT NULL
        CHECK (database_schema_version > 0)
) STRICT;

CREATE TRIGGER quarantined_communicator_states_no_replace
BEFORE INSERT ON quarantined_communicator_states
WHEN EXISTS (
    SELECT 1
    FROM quarantined_communicator_states
    WHERE quarantined_state_id IS NEW.quarantined_state_id
)
BEGIN
    SELECT RAISE(ABORT, 'quarantined_communicator_states is append-only');
END;

CREATE TRIGGER quarantined_communicator_states_no_update
BEFORE UPDATE ON quarantined_communicator_states
BEGIN
    SELECT RAISE(ABORT, 'quarantined_communicator_states is append-only');
END;

CREATE TRIGGER quarantined_communicator_states_no_delete
BEFORE DELETE ON quarantined_communicator_states
BEGIN
    SELECT RAISE(ABORT, 'quarantined_communicator_states is append-only');
END;
