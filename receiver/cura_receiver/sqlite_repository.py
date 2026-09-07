"""Concrete row operations; callers own transaction composition and decisions."""

from __future__ import annotations

import sqlite3

from .generated import receiver_entities_generated as rows

SqliteRow = tuple[object, ...]
RECEIVER_INSTANCE_COLUMNS = (
    "instance_ordinal",
    "receiver_instance_id",
    "linux_boot_id",
    "started_at_monotonic_us",
    "clean_stopped_at_monotonic_us",
    "clean_stop_state_generation",
)
_BOOLEAN_COLUMNS = frozenset(
    {
        "header_authenticated",
        "step_discontinuity_boundary",
        "is_canonical_for_sample",
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
            f"INSERT INTO {table} ({', '.join(columns)}) VALUES ({', '.join('?' for _ in columns)})",
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

    def insert_message_profile(self, entity: rows.MessageProfileRowV1) -> None:
        self._insert(
            rows.MESSAGE_PROFILE_ROW_V1_TABLE,
            rows.MESSAGE_PROFILE_ROW_V1_COLUMNS,
            rows.message_profile_row_v1_parameters(entity),
        )

    def find_message_profile(
        self, receiver_instance_id: bytes, sequence: int
    ) -> SqliteRow | None:
        return self._find(
            rows.MESSAGE_PROFILE_ROW_V1_TABLE,
            rows.MESSAGE_PROFILE_ROW_V1_COLUMNS,
            "receiver_instance_id = ? AND occurrence_sequence = ?",
            (_identity(receiver_instance_id, 16), _unsigned(sequence, 63)),
        )

    def insert_reading_message(self, entity: rows.ReadingMessageRowV1) -> None:
        self._insert(
            rows.READING_MESSAGE_ROW_V1_TABLE,
            rows.READING_MESSAGE_ROW_V1_COLUMNS,
            rows.reading_message_row_v1_parameters(entity),
        )

    def find_reading_message(self, node_id: bytes, message_id: int) -> SqliteRow | None:
        return self._find(
            rows.READING_MESSAGE_ROW_V1_TABLE,
            rows.READING_MESSAGE_ROW_V1_COLUMNS,
            "node_id = ? AND message_id = ?",
            (_identity(node_id, 8), _unsigned(message_id, 32)),
        )

    def find_canonical_sample(self, node_id: bytes, sample_id: int) -> SqliteRow | None:
        return self._find(
            rows.READING_MESSAGE_ROW_V1_TABLE,
            rows.READING_MESSAGE_ROW_V1_COLUMNS,
            "node_id = ? AND sample_id = ? AND is_canonical_for_sample = 1",
            (_identity(node_id, 8), _unsigned(sample_id, 32)),
        )

    def find_first_profile_frame(
        self, node_id: bytes, message_id: int
    ) -> SqliteRow | None:
        """Return the stored (length, fixed-capacity bytes), without normalization."""
        row = self._connection.execute(
            "SELECT p.received_frame_length, p.received_frame "
            "FROM reading_messages AS r JOIN message_profiles AS p "
            "ON p.receiver_instance_id = r.first_receiver_instance_id "
            "AND p.occurrence_sequence = r.first_occurrence_sequence "
            "WHERE r.node_id = ? AND r.message_id = ?",
            (_identity(node_id, 8), _unsigned(message_id, 32)),
        ).fetchone()
        return None if row is None else tuple(row)

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

    def read_communicator_state_rows(self) -> tuple[SqliteRow, ...]:
        """Retain every raw envelope row so later validation cannot miss defects."""
        cursor = self._connection.execute(
            f"SELECT {', '.join(rows.COMMUNICATOR_STATE_V1_COLUMNS)} FROM {rows.COMMUNICATOR_STATE_V1_TABLE}"
        )
        return tuple(tuple(row) for row in cursor.fetchall())
