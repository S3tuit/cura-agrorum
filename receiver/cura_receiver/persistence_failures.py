"""Closed ordinary-storage classification with explicit item-defect candidates."""

import sqlite3
from dataclasses import dataclass

from .generated.receiver_enums_generated import (
    DiagnosticOperation,
)
from .generated.receiver_enums_generated import (
    PersistenceAdmissionState as State,
)
from .generated.receiver_enums_generated import (
    QuarantineFailureReason as Reason,
)
from .reading_persistence import PersistenceIdentityCollision
from .sqlite_database import DatabaseFailure, StorageUnavailable, database_failure


@dataclass(frozen=True, slots=True)
class EntityFailure:
    reason: Reason
    operation: DiagnosticOperation
    sqlite_primary_code: int | None = None
    sqlite_extended_code: int | None = None
    os_errno: int | None = None


def classify_entity_failure(
    error: Exception, operation: DiagnosticOperation
) -> EntityFailure | None:
    """Return only a candidate; isolation must exclude global causes and reproduce it.

    The caller invokes this only during preparation/binding of a particular
    queue unit, never for BEGIN, COMMIT, ROLLBACK, quarantine or checkpoint I/O.
    """
    if isinstance(error, PersistenceIdentityCollision):
        return None
    if isinstance(error, (sqlite3.Error, OSError)):
        failure = database_failure(error)
        if (
            failure.sqlite_primary_code == sqlite3.SQLITE_CONSTRAINT
            and failure.os_errno is None
            and failure.sqlite_extended_code
            not in (
                sqlite3.SQLITE_CONSTRAINT_PRIMARYKEY,
                sqlite3.SQLITE_CONSTRAINT_UNIQUE,
                sqlite3.SQLITE_CONSTRAINT_ROWID,
            )
        ):
            return EntityFailure(
                Reason.UNEXPECTED_SQL_CONSTRAINT,
                operation,
                failure.sqlite_primary_code,
                failure.sqlite_extended_code,
            )
        return None
    if isinstance(error, OverflowError):
        reason = Reason.SQL_RANGE_VIOLATION
    elif isinstance(error, (TypeError, AttributeError)):
        reason = (
            Reason.SQL_BINDING_INVARIANT
            if operation is DiagnosticOperation.APPEND
            else Reason.ENTITY_DECODING_INVARIANT
        )
    elif isinstance(error, ValueError):
        reason = (
            Reason.ENTITY_DECODING_INVARIANT
            if operation is DiagnosticOperation.DECODE
            else Reason.PERSISTENCE_DERIVATION_INVARIANT
        )
    else:
        return None
    return EntityFailure(reason, operation)


def classify_global_failure(error: Exception) -> DatabaseFailure:
    if isinstance(error, StorageUnavailable):
        return error.failure
    if isinstance(error, PersistenceIdentityCollision):
        return DatabaseFailure(State.UNAVAILABLE_INCOMPATIBLE_SCHEMA)
    failure = database_failure(error)
    if (
        failure.admission_state is State.UNAVAILABLE_IO
        and failure.os_errno is None
        and failure.sqlite_extended_code
        in (
            sqlite3.SQLITE_SCHEMA,
            sqlite3.SQLITE_CONSTRAINT_PRIMARYKEY,
            sqlite3.SQLITE_CONSTRAINT_UNIQUE,
            sqlite3.SQLITE_CONSTRAINT_ROWID,
        )
    ):
        return DatabaseFailure(
            State.UNAVAILABLE_INCOMPATIBLE_SCHEMA,
            failure.sqlite_primary_code,
            failure.sqlite_extended_code,
        )
    return failure
