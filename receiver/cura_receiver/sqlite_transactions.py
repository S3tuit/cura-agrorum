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
