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
