import hashlib
import sqlite3
from dataclasses import replace

import pytest

from cura_receiver.communicator_state_persistence import CommunicatorStatePolicy
from cura_receiver.generated.receiver_entities_generated import (
    communicator_state_v1_parameters,
)
from cura_receiver.persistence_control_execution import (
    ControlCommand,
    ControlRequest,
    ControlCommandKind as Kind,
    ControlExecutionState as Phase,
)
from cura_receiver.persistence_control_operations import PersistenceControlOperations
from cura_receiver.persistence_control_values import (
    CommunicatorStateCommitDisposition as D,
    CommunicatorStateCommitFailureKind as F,
    CommunicatorStateCondition as Condition,
)
from cura_receiver.receiver_configuration import (
    PersistenceControlInterfaceViolation as Violation,
)
from cura_receiver.receiver_startup import ReceiverInstanceStart
from cura_receiver.sqlite_database import open_receiver_database
from cura_receiver.sqlite_transactions import SqliteTransactions
from tests.support.builders.persistence import INSTANCE, GROUP
from tests.support.builders.persistence_control import state


@pytest.fixture
def controls(setup):
    path, connection, queue, clock, _ = setup
    database = open_receiver_database(path, GROUP, minimum_free_bytes=0).database
    operations = PersistenceControlOperations(
        database,
        instance=ReceiverInstanceStart(INSTANCE, 0),
        queue=queue,
        clock=clock,
        policy=CommunicatorStatePolicy(),
    )

    def command(kind=Kind.COMMIT_STATE, deadline=None):
        return ControlCommand(
            ControlRequest(
                kind,
                clock.now_monotonic_us() + 1_000_000 if deadline is None else deadline,
            ),
            clock,
        )

    yield operations, connection, clock, command
    database.close()


from tests.support.builders.persistence_control import synthetic


# Missing history only accepts the exact worst-case generation one, then deterministic generations.
def test_state_creation_and_generation_rules(controls):
    operations, connection, _, command = controls
    assert (
        operations.commit_state(state(), command()).interface_violation
        is Violation.INVALID_STATE
    )
    assert operations.commit_state(synthetic(), command()).disposition is D.COMMITTED
    assert (
        operations.commit_state(synthetic(), command()).disposition
        is D.ALREADY_COMMITTED
    )
    assert (
        operations.commit_state(state(), command()).interface_violation
        is Violation.GENERATION_CONTENT_CONFLICT
    )
    assert (
        operations.commit_state(state(generation=3), command()).interface_violation
        is Violation.GENERATION_GAP
    )
    assert (
        operations.commit_state(state(generation=2), command()).disposition
        is D.COMMITTED
    )
    assert (
        operations.commit_state(synthetic(), command()).interface_violation
        is Violation.STALE_GENERATION
    )
    loaded = operations.load_state(command(Kind.LOAD_STATE))
    assert loaded.state == state(generation=2)
    assert connection.execute("SELECT count(*) FROM communicator_state").fetchone() == (
        1,
    )


# Corrupt raw relation recovery preserves every exact SQLite value and installs one valid row atomically.
def test_corrupt_relation_archive(controls):
    operations, connection, _, command = controls
    raw = [(None, "bad", 1.5, 4, None), (1, 2, 1, b"wrong", b"short")]
    connection.executemany("INSERT INTO communicator_state VALUES (?, ?, ?, ?, ?)", raw)
    assert operations.commit_state(synthetic(), command()).disposition is D.COMMITTED
    assert (
        connection.execute(
            "SELECT observed_singleton_id, observed_state_format_version, observed_generation, observed_state_blob, observed_state_sha256 FROM quarantined_communicator_states ORDER BY quarantined_state_id"
        ).fetchall()
        == raw
    )
    assert connection.execute(
        "SELECT calculated_blob_sha256 FROM quarantined_communicator_states ORDER BY quarantined_state_id"
    ).fetchall() == [(None,), (hashlib.sha256(b"wrong").digest(),)]
    assert (
        operations.commit_state(synthetic(), command()).disposition
        is D.ALREADY_COMMITTED
    )
    assert connection.execute(
        "SELECT count(*) FROM quarantined_communicator_states"
    ).fetchone() == (2,)


# F-002: each invalid-TEXT envelope field and every duplicate row survives SQL-side archival exactly.
@pytest.mark.parametrize(
    "column",
    [
        "singleton_id",
        "state_format_version",
        "generation",
        "state_blob",
        "state_sha256",
    ],
)
@pytest.mark.parametrize("invalid_text", [b"\x80", b"\x00\xff", b"\xed\xa0\x80"])
def test_invalid_text_archive_preserves_exact_values(controls, column, invalid_text):
    operations, connection, _, command = controls
    original = communicator_state_v1_parameters(state())
    connection.execute(
        "INSERT INTO communicator_state VALUES (?, ?, ?, ?, ?)", original
    )
    connection.execute(
        f"UPDATE communicator_state SET {column} = CAST(? AS TEXT)", (invalid_text,)
    )
    connection.execute(
        "INSERT INTO communicator_state SELECT * FROM communicator_state"
    )
    assert connection.execute(
        f"SELECT typeof({column}), hex({column}) FROM communicator_state"
    ).fetchall() == [
        ("text", invalid_text.hex().upper()),
        ("text", invalid_text.hex().upper()),
    ]
    assert connection.execute("PRAGMA integrity_check").fetchall() == [("ok",)]
    columns = (
        "singleton_id",
        "state_format_version",
        "generation",
        "state_blob",
        "state_sha256",
    )
    before = connection.execute(
        "SELECT "
        + ", ".join(f"typeof({name}), hex({name})" for name in columns)
        + " FROM communicator_state ORDER BY rowid"
    ).fetchall()
    assert (
        operations.load_state(command(Kind.LOAD_STATE)).state_condition
        is Condition.CORRUPT
    )
    assert operations.commit_state(synthetic(), command()).disposition is D.COMMITTED
    after = connection.execute(
        "SELECT "
        + ", ".join(
            f"typeof(observed_{name}), hex(observed_{name})" for name in columns
        )
        + " FROM quarantined_communicator_states ORDER BY quarantined_state_id"
    ).fetchall()
    assert after == before
    expected_digest = (
        None if column == "state_blob" else hashlib.sha256(original[3]).digest()
    )
    assert (
        connection.execute(
            "SELECT calculated_blob_sha256, preserved_by_receiver_instance_id, preserved_at_monotonic_us, database_schema_version "
            "FROM quarantined_communicator_states ORDER BY quarantined_state_id"
        ).fetchall()
        == [(expected_digest, INSTANCE, 100, 12)] * 2
    )
    assert operations.load_state(command(Kind.LOAD_STATE)).state == synthetic()


# F-002: actual rejection after SQL archival rolls back invalid TEXT and every archive row together.
def test_invalid_text_archive_rolls_back(controls):
    operations, connection, _, command = controls
    connection.execute(
        "INSERT INTO communicator_state VALUES (1, 1, 1, CAST(X'80' AS TEXT), NULL)"
    )
    connection.execute(
        "CREATE TRIGGER reject_replacement BEFORE INSERT ON communicator_state "
        "BEGIN SELECT RAISE(ABORT, 'test replacement boundary'); END"
    )
    result = operations.commit_state(synthetic(), command())
    assert (result.disposition, result.failure_kind) == (
        D.NOT_INSTALLED,
        F.DATABASE_ERROR,
    )
    assert connection.execute(
        "SELECT count(*) FROM quarantined_communicator_states"
    ).fetchone() == (0,)
    assert connection.execute(
        "SELECT typeof(state_blob), hex(state_blob) FROM communicator_state"
    ).fetchall() == [("text", "80")]
    assert not operations.database.connection.in_transaction


# F-002: unknown archival commits reconcile exact state and never duplicate preserved TEXT rows.
@pytest.mark.parametrize("committed", [False, True])
def test_invalid_text_archive_unknown_outcome(controls, committed):
    operations, connection, _, command = controls
    connection.execute(
        "INSERT INTO communicator_state VALUES (1, 1, 1, CAST(X'80' AS TEXT), NULL)"
    )

    class Unknown(SqliteTransactions):
        def commit(self, db):
            if committed:
                super().commit(db)
            raise sqlite3.OperationalError("lost archival commit confirmation")

    operations.transactions = Unknown()
    assert (
        operations.commit_state(synthetic(), command()).disposition is D.OUTCOME_UNKNOWN
    )
    loaded = operations.load_state(command(Kind.LOAD_STATE))
    assert loaded.state == (synthetic() if committed else None)
    assert loaded.state_condition is (
        Condition.NONE if committed else Condition.CORRUPT
    )
    assert connection.execute(
        "SELECT count(*) FROM quarantined_communicator_states"
    ).fetchone() == (int(committed),)
    operations.transactions = SqliteTransactions()
    assert operations.commit_state(synthetic(), command()).disposition is (
        D.ALREADY_COMMITTED if committed else D.COMMITTED
    )
    assert connection.execute(
        "SELECT typeof(observed_state_blob), hex(observed_state_blob), calculated_blob_sha256 FROM quarantined_communicator_states"
    ).fetchall() == [("text", "80", None)]


# A real SQL failure after archival rolls back both archive rows and singleton replacement.
def test_archive_and_install_are_atomic(controls):
    operations, connection, _, command = controls
    original = (1, 1, 1, b"bad", b"short")
    connection.execute(
        "INSERT INTO communicator_state VALUES (?, ?, ?, ?, ?)", original
    )
    connection.execute(
        "CREATE TRIGGER reject_state BEFORE INSERT ON communicator_state BEGIN SELECT RAISE(ABORT, 'test boundary'); END"
    )
    result = operations.commit_state(synthetic(), command())
    assert (
        result.disposition is D.NOT_INSTALLED
        and result.failure_kind is F.DATABASE_ERROR
    )
    assert connection.execute("SELECT * FROM communicator_state").fetchall() == [
        original
    ]
    assert (
        connection.execute("SELECT * FROM quarantined_communicator_states").fetchall()
        == []
    )
    assert not operations.database.connection.in_transaction


# Unknown version and policy mismatch require an explicit empty generation-one replacement.
@pytest.mark.parametrize(
    "condition", [Condition.UNSUPPORTED_VERSION, Condition.POLICY_MISMATCH]
)
def test_guarded_state_replacement(controls, condition):
    operations, connection, _, command = controls
    blob = b"\x02\x00"
    row = (
        (1, 2, 1, blob, hashlib.sha256(blob).digest())
        if condition is Condition.UNSUPPORTED_VERSION
        else communicator_state_v1_parameters(state(tx_airtime_budget_us=35_000_000))
    )
    connection.execute("INSERT INTO communicator_state VALUES (?, ?, ?, ?, ?)", row)
    rejected = operations.commit_state(synthetic(), command())
    assert (
        rejected.disposition is D.NOT_INSTALLED
        and rejected.state_condition is condition
    )
    assert operations.commit_state(state(), command()).disposition is D.COMMITTED
    assert connection.execute(
        "SELECT count(*) FROM quarantined_communicator_states"
    ).fetchone() == (1,)


# Both possible durable outcomes after COMMIT was invoked reconcile through exact state loads.
@pytest.mark.parametrize("committed", [False, True])
def test_unknown_state_commit(controls, committed):
    operations, connection, _, command = controls

    class UnknownCommit(SqliteTransactions):
        def commit(self, connection):
            if committed:
                super().commit(connection)
            raise sqlite3.OperationalError("lost commit confirmation")

    operations.transactions = UnknownCommit()
    result = operations.commit_state(synthetic(), command())
    assert (
        result.disposition is D.OUTCOME_UNKNOWN
        and result.failure_kind is F.DATABASE_ERROR
    )
    operations.transactions = SqliteTransactions()
    loaded = operations.load_state(command(Kind.LOAD_STATE))
    if committed:
        assert loaded.state == synthetic()
        assert (
            operations.commit_state(synthetic(), command()).disposition
            is D.ALREADY_COMMITTED
        )
    else:
        assert loaded.state_condition is Condition.MISSING
        assert (
            operations.commit_state(synthetic(), command()).disposition is D.COMMITTED
        )


# Expiry or cancellation after BEGIN prevents COMMIT and rolls back all durable effects.
@pytest.mark.parametrize("cancel", [False, True])
def test_running_precommit_cancellation(controls, cancel):
    operations, connection, clock, command = controls
    request = command()

    class ExpireAfterBegin(SqliteTransactions):
        def begin(self, connection):
            super().begin(connection)
            if cancel:
                assert request.expire()[0] is Phase.RUNNING_PRECOMMIT
            else:
                clock.advance_elapsed_us(1_000_000)

        def commit(self, connection):
            pytest.fail("a cancelled command crossed COMMIT")

    operations.transactions = ExpireAfterBegin()
    result = operations.commit_state(synthetic(), request)
    assert (
        result.disposition is D.NOT_INSTALLED
        and result.failure_kind is F.DEADLINE_EXCEEDED
    )
    assert connection.execute("SELECT * FROM communicator_state").fetchall() == []
    assert not operations.database.connection.in_transaction


# The atomic effect boundary classifies timeout as uncertain before entering COMMIT's backend.
def test_commit_boundary_is_atomic(controls):
    operations, _, _, command = controls
    request = command()

    class ObserveCommit(SqliteTransactions):
        def commit(self, connection):
            assert request.expire()[0] is Phase.COMMIT_MAY_HAVE_RUN
            super().commit(connection)

    operations.transactions = ObserveCommit()
    assert operations.commit_state(synthetic(), request).disposition is D.COMMITTED


# Queued expiry performs no state write, and bounded lock waits are restored after control work.
def test_queued_expiry_and_busy_timeout(controls):
    operations, connection, clock, command = controls
    result = operations.commit_state(
        synthetic(), command(deadline=clock.now_monotonic_us())
    )
    assert (
        result.disposition is D.NOT_INSTALLED
        and result.failure_kind is F.DEADLINE_EXCEEDED
    )
    assert connection.execute("SELECT * FROM communicator_state").fetchall() == []
    request = command(deadline=clock.now_monotonic_us() + 1500)

    class CheckWait(SqliteTransactions):
        def begin(self, connection):
            assert connection.execute("PRAGMA busy_timeout").fetchone() == (1,)
            super().begin(connection)

    operations.transactions = CheckWait()
    assert operations.commit_state(synthetic(), request).disposition is D.COMMITTED
    assert operations.database.connection.execute("PRAGMA busy_timeout").fetchone() == (
        250,
    )


# Clean-stop validates queue ownership and authoritative generation before marking this instance.
def test_clean_stop_preconditions_and_idempotency(controls):
    from cura_receiver.persistence_control_values import (
        ReceiverCleanStopV1,
        ReceiverCleanStopCommitDisposition as SD,
    )

    operations, connection, _, command = controls
    marker = ReceiverCleanStopV1(INSTANCE, 100, 0)
    assert (
        operations.commit_clean_stop(
            marker, command(Kind.CLEAN_STOP)
        ).interface_violation
        is Violation.CLEAN_STOP_PRECONDITION
    )
    operations.queue.close()
    assert (
        operations.commit_clean_stop(
            replace(marker, communicator_state_generation=1), command(Kind.CLEAN_STOP)
        ).interface_violation
        is Violation.CLEAN_STOP_PRECONDITION
    )
    assert (
        operations.commit_clean_stop(marker, command(Kind.CLEAN_STOP)).disposition
        is SD.COMMITTED
    )
    assert (
        operations.commit_clean_stop(marker, command(Kind.CLEAN_STOP)).disposition
        is SD.ALREADY_COMMITTED
    )
    assert (
        operations.commit_clean_stop(
            replace(marker, stopped_at_monotonic_us=101), command(Kind.CLEAN_STOP)
        ).interface_violation
        is Violation.CLEAN_STOP_CONFLICT
    )
    assert connection.execute(
        "SELECT clean_stopped_at_monotonic_us, clean_stop_state_generation FROM receiver_instances"
    ).fetchall() == [(100, 0)]


# An outstanding reservation keeps a closed queue ineligible until its owner cancels it.
def test_clean_stop_cannot_ignore_reservation(controls):
    from cura_receiver.persist_queue import PersistenceAdmissionSnapshot
    from cura_receiver.generated.receiver_enums_generated import (
        PersistenceAdmissionState,
    )
    from cura_receiver.persist_queue_entities import PROFILE_ONLY_V1_SPEC
    from cura_receiver.persistence_control_values import (
        ReceiverCleanStopV1,
        ReceiverCleanStopCommitDisposition as SD,
    )

    operations, _, _, command = controls
    operations.queue.publish_admission_state(
        PersistenceAdmissionSnapshot(1, PersistenceAdmissionState.AVAILABLE, 1)
    )
    reservation = operations.queue.try_reserve_one(PROFILE_ONLY_V1_SPEC).reservation
    operations.queue.close()
    marker = ReceiverCleanStopV1(INSTANCE, 100, 0)
    assert (
        operations.commit_clean_stop(
            marker, command(Kind.CLEAN_STOP)
        ).interface_violation
        is Violation.CLEAN_STOP_PRECONDITION
    )
    reservation.cancel()
    assert (
        operations.commit_clean_stop(marker, command(Kind.CLEAN_STOP)).disposition
        is SD.COMMITTED
    )


# A loaded state requires its actual generation; conservative zero applies only to unavailable state.
@pytest.mark.parametrize(
    "raw_condition", ["valid", "corrupt", "unsupported", "mismatch"]
)
def test_clean_stop_generation_conditions(controls, raw_condition):
    from cura_receiver.persistence_control_values import (
        ReceiverCleanStopV1,
        ReceiverCleanStopCommitDisposition as SD,
    )

    operations, connection, _, command = controls
    if raw_condition in ("valid", "mismatch"):
        row = communicator_state_v1_parameters(
            state(
                generation=4,
                tx_airtime_budget_us=(
                    36_000_000 if raw_condition == "valid" else 35_000_000
                ),
            )
        )
    elif raw_condition == "unsupported":
        blob = b"\x02\x00"
        row = (1, 2, 1, blob, hashlib.sha256(blob).digest())
    else:
        row = (None, None, None, None, None)
    connection.execute("INSERT INTO communicator_state VALUES (?, ?, ?, ?, ?)", row)
    operations.queue.close()
    generation = 4 if raw_condition == "valid" else 0
    marker = ReceiverCleanStopV1(INSTANCE, 100, generation)
    assert (
        operations.commit_clean_stop(marker, command(Kind.CLEAN_STOP)).disposition
        is SD.COMMITTED
    )


# Lost clean-stop confirmation reconciles by repeating the exact marker without a second transition.
@pytest.mark.parametrize("committed", [False, True])
def test_unknown_clean_stop_reconciles_exactly(controls, committed):
    from cura_receiver.persistence_control_values import (
        ReceiverCleanStopV1,
        ReceiverCleanStopCommitDisposition as SD,
    )

    operations, connection, _, command = controls
    operations.queue.close()
    marker = ReceiverCleanStopV1(INSTANCE, 100, 0)

    class Unknown(SqliteTransactions):
        def commit(self, connection):
            if committed:
                super().commit(connection)
            raise sqlite3.OperationalError("lost confirmation")

    operations.transactions = Unknown()
    assert (
        operations.commit_clean_stop(marker, command(Kind.CLEAN_STOP)).disposition
        is SD.OUTCOME_UNKNOWN
    )
    operations.transactions = SqliteTransactions()
    result = operations.commit_clean_stop(marker, command(Kind.CLEAN_STOP))
    assert result.disposition is (SD.ALREADY_COMMITTED if committed else SD.COMMITTED)
    assert connection.execute(
        "SELECT clean_stopped_at_monotonic_us FROM receiver_instances"
    ).fetchall() == [(100,)]


# Precommit clean-stop cancellation leaves both marker columns absent.
def test_clean_stop_precommit_deadline(controls):
    from cura_receiver.persistence_control_values import (
        ReceiverCleanStopV1,
        ReceiverCleanStopCommitDisposition as SD,
        ReceiverCleanStopCommitFailureKind as SF,
    )

    operations, connection, clock, command = controls
    operations.queue.close()

    class Expire(SqliteTransactions):
        def begin(self, connection):
            super().begin(connection)
            clock.advance_elapsed_us(1_000_000)

    operations.transactions = Expire()
    result = operations.commit_clean_stop(
        ReceiverCleanStopV1(INSTANCE, 100, 0), command(Kind.CLEAN_STOP)
    )
    assert (
        result.disposition is SD.NOT_COMMITTED
        and result.failure_kind is SF.DEADLINE_EXCEEDED
    )
    assert connection.execute(
        "SELECT clean_stopped_at_monotonic_us, clean_stop_state_generation FROM receiver_instances"
    ).fetchall() == [(None, None)]
