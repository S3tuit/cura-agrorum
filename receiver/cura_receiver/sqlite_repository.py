"""Concrete row operations; callers own transaction composition and decisions."""

from __future__ import annotations

import sqlite3
from dataclasses import dataclass

from .generated import receiver_entities_generated as rows

SqliteRow = tuple[object, ...]
# The protected format length is u16. Larger blobs cannot be valid envelopes.
COMMUNICATOR_STATE_MAX_BLOB_BYTES = 65535
COMMUNICATOR_STATE_STORAGE_CLASSES = ("integer", "integer", "integer", "blob", "blob")


@dataclass(frozen=True, slots=True)
class CommunicatorStateRow:
    """Immutable validation input; rejected evidence remains inside SQLite.

    Values of unexpected storage classes are absent. row_id identifies the
    original row only inside the transaction that reads and archives it.
    """

    row_id: int
    storage_classes: tuple[str, ...]
    values: SqliteRow


RECEIVER_INSTANCE_COLUMNS = (
    "instance_ordinal",
    "receiver_instance_id",
    "linux_boot_id",
    "started_at_monotonic_us",
    "clean_stopped_at_monotonic_us",
    "clean_stop_state_generation",
)
QUARANTINED_COMMUNICATOR_STATE_COLUMNS = (
    "quarantined_state_id",
    "observed_singleton_id",
    "observed_state_format_version",
    "observed_generation",
    "observed_state_blob",
    "observed_state_sha256",
    "calculated_blob_sha256",
    "preserved_by_receiver_instance_id",
    "preserved_at_monotonic_us",
    "database_schema_version",
)
# One inventory of the row access required by startup and storage recovery.
# Metadata is validated separately as the version/fingerprint authority.
REQUIRED_TABLE_PROJECTIONS = (
    ("airtime_commissioning", ("request",)),
    (rows.CLOCK_OBSERVATION_V1_TABLE, rows.CLOCK_OBSERVATION_V1_COLUMNS),
    (rows.DIAGNOSTIC_V1_TABLE, rows.DIAGNOSTIC_V1_COLUMNS),
    (rows.RECEIVER_HEALTH_V1_TABLE, rows.RECEIVER_HEALTH_V1_COLUMNS),
    (rows.RADIO_OBSERVATION_V1_TABLE, rows.RADIO_OBSERVATION_V1_COLUMNS),
    (rows.PACKET_RECEPTION_ROW_V1_TABLE, rows.PACKET_RECEPTION_ROW_V1_COLUMNS),
    (rows.READING_MESSAGE_ROW_V2_TABLE, rows.READING_MESSAGE_ROW_V2_COLUMNS),
    (rows.RECEIVED_MESSAGE_ROW_V1_TABLE, rows.RECEIVED_MESSAGE_ROW_V1_COLUMNS),
    (rows.PACKET_RECEPTION_EXCEPTION_ROW_V1_TABLE, rows.PACKET_RECEPTION_EXCEPTION_ROW_V1_COLUMNS),
    (rows.QUARANTINED_ENTITY_ROW_V1_TABLE, rows.QUARANTINED_ENTITY_ROW_V1_COLUMNS),
    ("receiver_instances", RECEIVER_INSTANCE_COLUMNS),
    (rows.COMMUNICATOR_STATE_V2_TABLE, ("rowid", *rows.COMMUNICATOR_STATE_V2_COLUMNS)),
    ("quarantined_communicator_states", QUARANTINED_COMMUNICATOR_STATE_COLUMNS),
)

def insert_statement(table: str, columns: tuple[str, ...]) -> str:
    return f"INSERT INTO {table} ({', '.join(columns)}) VALUES ({', '.join('?' for _ in columns)})"


INSTANCE_START_SQL = insert_statement("receiver_instances", (
    "receiver_instance_id", "linux_boot_id", "started_at_monotonic_us"))
INSTANCE_STOP_SQL = (
    "UPDATE receiver_instances SET clean_stopped_at_monotonic_us = ?, "
    "clean_stop_state_generation = ? WHERE receiver_instance_id = ?")
STATE_INSERT_SQL = "INSERT INTO communicator_state VALUES (?, ?, ?, ?, ?)"
STATE_DELETE_SQL = "DELETE FROM communicator_state"
COMMISSIONING_DELETE_SQL = "DELETE FROM airtime_commissioning"
STATE_ARCHIVE_SQL = (
    "INSERT INTO quarantined_communicator_states "
    "(observed_singleton_id, observed_state_format_version, observed_generation, "
    "observed_state_blob, observed_state_sha256, calculated_blob_sha256, "
    "preserved_by_receiver_instance_id, preserved_at_monotonic_us, database_schema_version) "
    "SELECT singleton_id, state_format_version, generation, state_blob, state_sha256, "
    "?, ?, ?, ? FROM communicator_state WHERE rowid = ?")


def required_write_operations():
    for table, columns in REQUIRED_TABLE_PROJECTIONS:
        if table in ("airtime_commissioning", "receiver_instances", "communicator_state",
                     "quarantined_communicator_states"):
            continue
        yield insert_statement(table, columns), (None,) * len(columns)
    for sql, count in ((INSTANCE_START_SQL, 3), (INSTANCE_STOP_SQL, 3),
                       (STATE_INSERT_SQL, 5), (STATE_DELETE_SQL, 0),
                       (COMMISSIONING_DELETE_SQL, 0), (STATE_ARCHIVE_SQL, 5)):
        yield sql, (None,) * count

_BOOLEAN_COLUMNS = frozenset(
    {
        "dio1_before", "dio1_after", "busy", "gpio_event_pending",
        "step_discontinuity_boundary",
        "is_canonical",
    }
)


def _identity(value: bytes, size: int) -> bytes:
    if type(value) is not bytes or len(value) != size:
        raise ValueError(f"identity must contain exactly {size} bytes")
    return value


def _unsigned(value: int, bits: int) -> int:
    if type(value) is not int or not 0 <= value <= (1 << bits) - 1:
        raise ValueError(f"key must be an integer in the unsigned {bits}-bit range")
    return value


def validate_sqlite_parameters(columns: tuple[str, ...], parameters: SqliteRow) -> None:
    for column, value in zip(columns, parameters, strict=True):
        if value is None or type(value) is bytes:
            continue
        if column in _BOOLEAN_COLUMNS:
            if type(value) is not bool:
                raise TypeError(f"{column} must be a bool")
        elif type(value) is not int:
            raise TypeError(f"{column} must have a canonical SQLite scalar type")
        if not -(1 << 63) <= value <= (1 << 63) - 1:
            raise OverflowError(f"{column} exceeds SQLite's signed integer range")


class SqliteRepository:
    """Use only on the thread that owns the validated SQLite connection.

    Inserts require an explicit open transaction, never commit, and never
    suppress a conflict. Lookups return complete tuples in the generated column
    order. They neither decode nor compare stored content or decide replay.
    """

    def __init__(self, connection: sqlite3.Connection) -> None:
        self._connection = connection

    def _insert(
        self, table: str, columns: tuple[str, ...], parameters: SqliteRow
    ) -> None:
        validate_sqlite_parameters(columns, parameters)
        if not self._connection.in_transaction:
            raise RuntimeError("repository inserts require a caller-owned transaction")
        self._connection.execute(
            insert_statement(table, columns),
            parameters,
        )

    def _find(
        self, table: str, columns: tuple[str, ...], where: str, keys: SqliteRow
    ) -> SqliteRow | None:
        row = self._connection.execute(
            f"SELECT {', '.join(columns)} FROM {table} WHERE {where}",
            keys,
        ).fetchone()
        return None if row is None else tuple(row)

    def insert_clock_observation(self, entity: rows.ClockObservationV1) -> None:
        self._insert(
            rows.CLOCK_OBSERVATION_V1_TABLE,
            rows.CLOCK_OBSERVATION_V1_COLUMNS,
            rows.clock_observation_v1_parameters(entity),
        )

    def find_clock_observation(
        self, receiver_instance_id: bytes, sequence: int
    ) -> SqliteRow | None:
        return self._find(
            rows.CLOCK_OBSERVATION_V1_TABLE,
            rows.CLOCK_OBSERVATION_V1_COLUMNS,
            "receiver_instance_id = ? AND observation_sequence = ?",
            (_identity(receiver_instance_id, 16), _unsigned(sequence, 63)),
        )

    def insert_diagnostic(self, entity: rows.DiagnosticV1) -> None:
        self._insert(
            rows.DIAGNOSTIC_V1_TABLE,
            rows.DIAGNOSTIC_V1_COLUMNS,
            rows.diagnostic_v1_parameters(entity),
        )

    def find_diagnostic(
        self, receiver_instance_id: bytes, sequence: int
    ) -> SqliteRow | None:
        return self._find(
            rows.DIAGNOSTIC_V1_TABLE,
            rows.DIAGNOSTIC_V1_COLUMNS,
            "receiver_instance_id = ? AND diagnostic_sequence = ?",
            (_identity(receiver_instance_id, 16), _unsigned(sequence, 63)),
        )

    def insert_quarantined_entity(self, entity: rows.QuarantinedEntityRowV1) -> None:
        self._insert(
            rows.QUARANTINED_ENTITY_ROW_V1_TABLE,
            rows.QUARANTINED_ENTITY_ROW_V1_COLUMNS,
            rows.quarantined_entity_row_v1_parameters(entity),
        )

    def find_quarantined_entity(self, quarantine_id: bytes) -> SqliteRow | None:
        return self._find(
            rows.QUARANTINED_ENTITY_ROW_V1_TABLE,
            rows.QUARANTINED_ENTITY_ROW_V1_COLUMNS,
            "quarantine_id = ?",
            (_identity(quarantine_id, 32),),
        )

    def insert_radio_observation(self, entity: rows.RadioObservationV1) -> None:
        self._insert(rows.RADIO_OBSERVATION_V1_TABLE, rows.RADIO_OBSERVATION_V1_COLUMNS,
                     rows.radio_observation_v1_parameters(entity))

    def find_radio_observation(self, receiver_instance_id: bytes, sequence: int) -> SqliteRow | None:
        return self._find(rows.RADIO_OBSERVATION_V1_TABLE, rows.RADIO_OBSERVATION_V1_COLUMNS,
                          "receiver_instance_id = ? AND observation_sequence = ?",
                          (_identity(receiver_instance_id, 16), _unsigned(sequence, 63)))

    def insert_receiver_health(self, entity: rows.ReceiverHealthV1) -> None:
        self._insert(
            rows.RECEIVER_HEALTH_V1_TABLE,
            rows.RECEIVER_HEALTH_V1_COLUMNS,
            rows.receiver_health_v1_parameters(entity),
        )

    def find_receiver_health(
        self, receiver_instance_id: bytes, sequence: int
    ) -> SqliteRow | None:
        return self._find(
            rows.RECEIVER_HEALTH_V1_TABLE,
            rows.RECEIVER_HEALTH_V1_COLUMNS,
            "receiver_instance_id = ? AND health_sequence = ?",
            (_identity(receiver_instance_id, 16), _unsigned(sequence, 63)),
        )

    def insert_packet_reception(self, entity: rows.PacketReceptionRowV1) -> None:
        self._insert(
            rows.PACKET_RECEPTION_ROW_V1_TABLE,
            rows.PACKET_RECEPTION_ROW_V1_COLUMNS,
            rows.packet_reception_row_v1_parameters(entity),
        )

    def find_packet_reception(
        self, receiver_instance_id: bytes, sequence: int
    ) -> SqliteRow | None:
        return self._find(
            rows.PACKET_RECEPTION_ROW_V1_TABLE,
            rows.PACKET_RECEPTION_ROW_V1_COLUMNS,
            "receiver_instance_id = ? AND occurrence_sequence = ?",
            (_identity(receiver_instance_id, 16), _unsigned(sequence, 63)),
        )

    def insert_reading_message(self, entity: rows.ReadingMessageRowV2) -> None:
        self._insert(
            rows.READING_MESSAGE_ROW_V2_TABLE,
            rows.READING_MESSAGE_ROW_V2_COLUMNS,
            rows.reading_message_row_v2_parameters(entity),
        )

    def find_reading_message(self, node_id: bytes, message_id: int) -> SqliteRow | None:
        return self._find(
            rows.READING_MESSAGE_ROW_V2_TABLE,
            rows.READING_MESSAGE_ROW_V2_COLUMNS,
            "node_id = ? AND message_id = ?",
            (_identity(node_id, 8), _unsigned(message_id, 32)),
        )

    def find_canonical_sample(self, node_id: bytes, sample_id: int) -> SqliteRow | None:
        return self._find(
            rows.READING_MESSAGE_ROW_V2_TABLE,
            rows.READING_MESSAGE_ROW_V2_COLUMNS,
            "node_id = ? AND sample_id = ? AND is_canonical = 1",
            (_identity(node_id, 8), _unsigned(sample_id, 32)),
        )

    def insert_received_message(self, entity: rows.ReceivedMessageRowV1) -> None:
        self._insert(rows.RECEIVED_MESSAGE_ROW_V1_TABLE, rows.RECEIVED_MESSAGE_ROW_V1_COLUMNS,
                     rows.received_message_row_v1_parameters(entity))

    def find_received_message(self, node_id: bytes, message_id: int) -> SqliteRow | None:
        return self._find(rows.RECEIVED_MESSAGE_ROW_V1_TABLE, rows.RECEIVED_MESSAGE_ROW_V1_COLUMNS,
                          "node_id = ? AND message_id = ?",
                          (_identity(node_id, 8), _unsigned(message_id, 32)))

    def insert_packet_reception_exception(self, entity: rows.PacketReceptionExceptionRowV1) -> None:
        self._insert(rows.PACKET_RECEPTION_EXCEPTION_ROW_V1_TABLE,
                     rows.PACKET_RECEPTION_EXCEPTION_ROW_V1_COLUMNS,
                     rows.packet_reception_exception_row_v1_parameters(entity))

    def find_packet_reception_exception(self, receiver_instance_id: bytes, sequence: int) -> SqliteRow | None:
        return self._find(rows.PACKET_RECEPTION_EXCEPTION_ROW_V1_TABLE,
                          rows.PACKET_RECEPTION_EXCEPTION_ROW_V1_COLUMNS,
                          "receiver_instance_id = ? AND occurrence_sequence = ?",
                          (_identity(receiver_instance_id, 16), _unsigned(sequence, 63)))

    def find_receiver_instance(self, receiver_instance_id: bytes) -> SqliteRow | None:
        return self._find(
            "receiver_instances",
            RECEIVER_INSTANCE_COLUMNS,
            "receiver_instance_id = ?",
            (_identity(receiver_instance_id, 16),),
        )

    def find_previous_receiver_instance(
        self, instance_ordinal: int
    ) -> SqliteRow | None:
        return self._find(
            "receiver_instances",
            RECEIVER_INSTANCE_COLUMNS,
            "instance_ordinal < ? ORDER BY instance_ordinal DESC LIMIT 1",
            (_unsigned(instance_ordinal, 63),),
        )

    def read_airtime_commissioning(self):
        """Read at most two bounded values; rejected TEXT never reaches Python."""
        from .airtime_commissioning import (
            AIRTIME_COMMISSIONING_TOKEN, AirtimeCommissioningState as State,
        )
        values = self._connection.execute(
            "SELECT CASE WHEN typeof(request) = 'blob' AND length(request) = ? "
            "THEN request END FROM airtime_commissioning LIMIT 2",
            (len(AIRTIME_COMMISSIONING_TOKEN),),
        ).fetchall()
        if not values:
            return State.ABSENT
        return State.PENDING if values == [(AIRTIME_COMMISSIONING_TOKEN,)] else State.INVALID

    def _communicator_state_rows(self, *, bounded: bool):
        columns = rows.COMMUNICATOR_STATE_V2_COLUMNS
        classes = ", ".join(f"typeof({column})" for column in columns)
        guards = {
            "state_blob": f" AND length(state_blob) <= {COMMUNICATOR_STATE_MAX_BLOB_BYTES}",
            "state_sha256": " AND length(state_sha256) = 32",
        }
        values = ", ".join(
            f"CASE WHEN typeof({column}) = '{expected}'{guards.get(column, '')} THEN {column} END"
            for column, expected in zip(columns, COMMUNICATOR_STATE_STORAGE_CLASSES, strict=True)
        )
        limit = " LIMIT 2" if bounded else ""
        cursor = self._connection.execute(
            f"SELECT rowid, {classes}, {values} FROM {rows.COMMUNICATOR_STATE_V2_TABLE} ORDER BY rowid{limit}"
        )
        for row in cursor:
            yield CommunicatorStateRow(row[0], tuple(row[1:6]), tuple(row[6:]))

    def read_communicator_state_rows(self) -> tuple[CommunicatorStateRow, ...]:
        """Bound readiness materialization; two rows suffice to reject nonsingletons.

        Type/length guards run in SQLite before blobs reach Python or hashing.
        This prefix is never an archival inventory.
        """
        return tuple(self._communicator_state_rows(bounded=True))

    def iter_communicator_state_archive_rows(self):
        """Visit every original row within the caller's recovery transaction.

        INSERT SELECT preserves raw values, including guarded-out oversized blobs.
        Only bounded values enter Python for optional evidence digests.
        """
        if not self._connection.in_transaction:
            raise RuntimeError("state archival requires a caller-owned transaction")
        return self._communicator_state_rows(bounded=False)
