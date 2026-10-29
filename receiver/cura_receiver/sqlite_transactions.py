"""Concrete SQLite operation boundary; fault tests override real operations."""

from __future__ import annotations

import sqlite3


class SqliteTransactions:
    """Each call uses the persistence owner's current validated connection.

    COMMIT returning normally confirms durability. An exception after entering
    commit cannot establish noncommit and must be reconciled by the owner.
    """

    def begin(self, connection: sqlite3.Connection) -> None:
        connection.execute("BEGIN IMMEDIATE")

    def commit(self, connection: sqlite3.Connection) -> None:
        connection.execute("COMMIT")

    def rollback(self, connection: sqlite3.Connection) -> None:
        if connection.in_transaction:
            connection.execute("ROLLBACK")

    def checkpoint(self, connection: sqlite3.Connection) -> tuple[int, int, int]:
        """Run one explicit maintenance operation without waiting on readers."""
        return connection.execute("PRAGMA main.wal_checkpoint(PASSIVE)").fetchone()


class ObservedSqliteTransactions(SqliteTransactions):
    """Observe COMMIT entry even when an injected operation loses its outcome.

    The worker shares this adapter between ordinary, quarantine and controls.
    Observation sits outside the concrete operation, so overrides do not need
    to call super() for possible work to remain tracked.
    """

    def __init__(self, transactions, maintenance):
        self._transactions = transactions
        self._maintenance = maintenance

    def begin(self, connection):
        return self._transactions.begin(connection)

    def commit(self, connection):
        self._maintenance.mark_possible_work()
        return self._transactions.commit(connection)

    def rollback(self, connection):
        return self._transactions.rollback(connection)

    def checkpoint(self, connection):
        return self._transactions.checkpoint(connection)
