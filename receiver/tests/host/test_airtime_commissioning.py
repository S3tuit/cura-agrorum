"""One-use commissioning authorization across actual SQLite and restart boundaries."""

import sqlite3

import pytest

from cura_receiver.airtime_commissioning import (
    AIRTIME_COMMISSIONING_TOKEN as TOKEN,
    AirtimeCommissioningState as Marker,
)
from cura_receiver.communicator_state_owner import CommunicatorStateOwner
from cura_receiver.database_initializer import initialize_database
from cura_receiver.generated import receiver_enums_generated as E
from cura_receiver.generated.receiver_entities_generated import communicator_state_v2_parameters
from cura_receiver.persistence_control_values import (
    CommunicatorStateCommitDisposition as Disposition,
    CommunicatorStateCondition as Condition,
    CommunicatorStateLoadResult,
    CommunicatorStateLoadStatus as LoadStatus,
)
from cura_receiver.sqlite_database import open_receiver_database
from cura_receiver.tx_airtime import AirtimeReason as Reason
from tests.support.builders.persistence import GROUP
from tests.support.builders.persistence_control import state
from tests.support.coordination.persistence_worker import prepare_worker_files
from tests.support.coordination.state_commit import LostStateReply


def recover(policy, clock):
    return policy.recover(deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000)


def reload_owner(policy, worker, clock):
    loaded = worker.control.load_communicator_state(
        deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000)
    policy.owner = CommunicatorStateOwner.from_load(control=worker.control, loaded=loaded)
    return loaded


@pytest.mark.parametrize('known_empty', [False, True])
def test_creation_mode_is_explicit_and_durable(tmp_path, known_empty):
    path = tmp_path / 'receiver.db'
    initialize_database(path, GROUP, known_empty_airtime=known_empty)
    with sqlite3.connect(path) as db:
        assert db.execute('SELECT request FROM airtime_commissioning').fetchall() == (
            [(TOKEN,)] if known_empty else [])
        assert db.execute('SELECT * FROM communicator_state').fetchall() == []
        assert db.execute('PRAGMA integrity_check').fetchone() == ('ok',)
    with pytest.raises(FileExistsError):
        initialize_database(path, GROUP, known_empty_airtime=True)


@pytest.mark.parametrize('value', [None, 0, 1, 'true'])
def test_creation_rejects_non_bool_authorization(tmp_path, value):
    path = tmp_path / 'receiver.db'
    with pytest.raises(TypeError):
        initialize_database(path, GROUP, known_empty_airtime=value)
    assert not path.exists()


@pytest.mark.parametrize('trusted', [False, True])
def test_commissioning_installs_empty_history_before_ack_admission(airtime_component, trusted):
    policy, worker, path, clock, loaded = airtime_component(known_empty_airtime=True)
    assert loaded.commissioning is Marker.PENDING
    if not trusted:
        policy.update_time(None, rtc_health=E.RtcHealth.MISSING)
    assert policy.try_spend().reason is Reason.STATE_UNAVAILABLE
    assert recover(policy, clock).reason is Reason.STATE_READY
    assert policy.total_used == 0
    assert policy.state.generation == 1
    assert policy.state.rtc_provenance is None
    assert (policy.state.airtime_snapshot is not None) == trusted
    assert all(entry.remaining_us == 0 for entry in policy.state.entries)
    assert not policy.owner.commissioning_pending
    with sqlite3.connect(path) as db:
        assert db.execute('SELECT * FROM communicator_state').fetchone() == communicator_state_v2_parameters(policy.state)
        assert db.execute('SELECT * FROM airtime_commissioning').fetchall() == []
    assert policy.try_spend().reason is Reason.ALLOWED


@pytest.mark.parametrize('rows', [
    [], [None], [0], [1.0], [TOKEN.decode()], [b'bad'], [b'x' * len(TOKEN)],
    [TOKEN, TOKEN], [TOKEN, None], [bytes(1_000_000)],
], ids=['absent', 'null', 'integer', 'real', 'text', 'short-blob', 'wrong-blob',
        'duplicate', 'extra-null', 'large-blob'])
def test_missing_or_malformed_contents_do_not_commission(airtime_component, rows):
    policy, worker, path, clock, _ = airtime_component(known_empty_airtime=True)
    with sqlite3.connect(path) as db:
        db.execute('DELETE FROM airtime_commissioning')
        db.executemany('INSERT INTO airtime_commissioning VALUES (?)', [(v,) for v in rows])
        assert db.execute('PRAGMA integrity_check').fetchone() == ('ok',)
    loaded = reload_owner(policy, worker, clock)
    assert loaded.commissioning is (Marker.INVALID if rows else Marker.ABSENT)
    assert recover(policy, clock).reason is Reason.STATE_READY
    assert policy.total_used == 36_000_000
    assert policy.try_spend().reason is Reason.BUDGET_EXHAUSTED
    with sqlite3.connect(path) as db:
        assert db.execute('SELECT * FROM airtime_commissioning').fetchall() == []


def test_invalid_utf8_marker_is_local_data_failure(airtime_component):
    policy, worker, path, clock, _ = airtime_component(known_empty_airtime=True)
    with sqlite3.connect(path) as db:
        db.execute("UPDATE airtime_commissioning SET request = CAST(X'80ff' AS TEXT)")
        assert db.execute('PRAGMA integrity_check').fetchone() == ('ok',)
    assert reload_owner(policy, worker, clock).commissioning is Marker.INVALID
    assert recover(policy, clock).reason is Reason.STATE_READY
    assert policy.total_used == 36_000_000


@pytest.mark.parametrize('damage', ['table', 'column', 'storage'])
def test_missing_schema_and_damaged_storage_keep_database_failure_policy(tmp_path, damage):
    path, _, _ = prepare_worker_files(tmp_path, known_empty_airtime=True)
    if damage == 'storage':
        path.write_bytes(b'not a SQLite database')
    else:
        with sqlite3.connect(path) as db:
            db.execute('DROP TABLE airtime_commissioning')
            if damage == 'column':
                db.execute('CREATE TABLE airtime_commissioning (wrong ANY) STRICT')
    result = open_receiver_database(path, GROUP, minimum_free_bytes=0)
    assert result.database is None
    assert result.failure.admission_state is (
        E.PersistenceAdmissionState.UNAVAILABLE_CORRUPT if damage == 'storage'
        else E.PersistenceAdmissionState.UNAVAILABLE_INCOMPATIBLE_SCHEMA)


@pytest.mark.parametrize('condition', [Condition.NONE, Condition.CORRUPT,
                                      Condition.UNSUPPORTED_VERSION, Condition.POLICY_MISMATCH])
def test_existing_history_takes_precedence(airtime_component, condition):
    policy, worker, path, clock, _ = airtime_component(
        condition, known_empty_airtime=True,
        initial_state=state() if condition is Condition.NONE else None)
    assert not policy.owner.commissioning_pending
    if condition in (Condition.UNSUPPORTED_VERSION, Condition.POLICY_MISMATCH):
        assert recover(policy, clock).reason is Reason.RECOVERY_WAIT
        with sqlite3.connect(path) as db:
            assert db.execute('SELECT request FROM airtime_commissioning').fetchall() == [(TOKEN,)]
        policy.confirm_transmitter_disabled()
        clock.advance_elapsed_us(3_613_320_000)
    assert recover(policy, clock).reason is Reason.STATE_READY
    assert policy.total_used == (2_000_000 if condition is Condition.NONE
                                 else 36_000_000 if condition is Condition.CORRUPT else 0)
    with sqlite3.connect(path) as db:
        assert db.execute('SELECT * FROM airtime_commissioning').fetchall() == []


@pytest.mark.parametrize('change', ['delete-token', 'invalid-token', 'existing', 'corrupt', 'incompatible'])
def test_stale_authorization_cannot_replace_changed_baseline(airtime_component, change):
    policy, worker, path, clock, _ = airtime_component(known_empty_airtime=True)
    with sqlite3.connect(path) as db:
        if change == 'delete-token':
            db.execute('DELETE FROM airtime_commissioning')
        elif change == 'invalid-token':
            db.execute('UPDATE airtime_commissioning SET request = NULL')
        elif change == 'corrupt':
            db.execute('INSERT INTO communicator_state VALUES (NULL,NULL,NULL,NULL,NULL)')
        else:
            value = state(generation=2, rolling_window_us=(3_500_000_000 if change == 'incompatible' else 3_600_000_000))
            db.execute('INSERT INTO communicator_state VALUES (?,?,?,?,?)', communicator_state_v2_parameters(value))
        original = db.execute('SELECT * FROM communicator_state').fetchall()
    assert recover(policy, clock).reason is Reason.PERSISTENCE_FAILED
    assert policy.try_spend().reason is not Reason.ALLOWED
    with sqlite3.connect(path) as db:
        assert db.execute('SELECT * FROM communicator_state').fetchall() == original
        assert db.execute('SELECT * FROM quarantined_communicator_states').fetchall() == []


def test_token_delete_failure_rolls_back_state_and_retry_keeps_same_snapshot(airtime_component):
    policy, worker, path, clock, _ = airtime_component(known_empty_airtime=True)
    with sqlite3.connect(path) as db:
        db.execute("CREATE TRIGGER fail_token_delete BEFORE DELETE ON airtime_commissioning BEGIN SELECT RAISE(ABORT, 'test'); END")
    assert recover(policy, clock).reason is Reason.PERSISTENCE_FAILED
    frozen = policy._prepared.requested
    assert policy.try_spend().reason is not Reason.ALLOWED
    with sqlite3.connect(path) as db:
        assert db.execute('SELECT * FROM communicator_state').fetchall() == []
        assert db.execute('SELECT request FROM airtime_commissioning').fetchall() == [(TOKEN,)]
        db.execute('DROP TRIGGER fail_token_delete')
    clock.advance_elapsed_us(1_000_000)
    assert recover(policy, clock).reason is Reason.STATE_READY
    assert policy.state is frozen
    assert policy.total_used == 0


@pytest.mark.parametrize('installed', [False, True])
def test_lost_reply_reconciles_both_effects_and_reuses_frozen_request(airtime_component, installed):
    policy, worker, path, clock, loaded = airtime_component(known_empty_airtime=True)
    channel = LostStateReply(worker.control, installed=installed)
    policy.owner = CommunicatorStateOwner.from_load(control=channel, loaded=loaded)
    assert recover(policy, clock).reason is Reason.PERSISTENCE_PENDING
    frozen = policy.owner.pending.requested
    assert policy.owner.pending.commissioning
    assert policy.try_spend().reason is Reason.PERSISTENCE_PENDING
    channel.fail_load = True
    assert recover(policy, clock).reason is Reason.PERSISTENCE_PENDING
    channel.fail_load = False
    clock.advance_elapsed_us(30_000_000)
    policy.update_time(None, rtc_health=E.RtcHealth.MISSING)
    assert recover(policy, clock).reason is Reason.STATE_READY
    assert policy.state is frozen and policy.total_used == 0
    assert all(request is frozen for request in channel.requests)
    with sqlite3.connect(path) as db:
        assert db.execute('SELECT * FROM airtime_commissioning').fetchall() == []
    assert policy.try_spend().reason is Reason.ALLOWED


@pytest.mark.parametrize('installed', [False, True])
@pytest.mark.parametrize('token', [TOKEN, None])
def test_unknown_commit_with_inconsistent_token_never_confirms(airtime_component, installed, token):
    policy, worker, path, clock, loaded = airtime_component(known_empty_airtime=True)
    channel = LostStateReply(worker.control, installed=installed)
    policy.owner = CommunicatorStateOwner.from_load(control=channel, loaded=loaded)
    assert recover(policy, clock).reason is Reason.PERSISTENCE_PENDING
    with sqlite3.connect(path) as db:
        db.execute('DELETE FROM airtime_commissioning')
        if installed or token is None:
            db.execute('INSERT INTO airtime_commissioning VALUES (?)', (token,))
    assert recover(policy, clock).reason is Reason.RECONCILIATION_CONFLICT
    assert policy.try_spend().reason is Reason.RECONCILIATION_CONFLICT


def test_exact_commissioning_replay_requires_consumed_token(airtime_component):
    policy, worker, path, clock, _ = airtime_component(known_empty_airtime=True)
    assert recover(policy, clock).reason is Reason.STATE_READY
    value = policy.state
    def replay():
        return worker.control.commit_communicator_state(value, commissioning=True,
            deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000)
    assert replay().disposition is Disposition.ALREADY_COMMITTED
    with sqlite3.connect(path) as db:
        db.execute('INSERT INTO airtime_commissioning VALUES (?)', (TOKEN,))
    assert replay().disposition is Disposition.NOT_INSTALLED


@pytest.mark.parametrize('flag', [1, None, 'true'])
def test_commit_rejects_non_bool_intent_without_consuming_token(airtime_component, flag):
    policy, worker, path, clock, _ = airtime_component(known_empty_airtime=True)
    result = worker.control.commit_communicator_state(state(), commissioning=flag,
        deadline_monotonic_us=5_000_100)
    assert result.disposition is Disposition.NOT_INSTALLED
    with sqlite3.connect(path) as db:
        assert db.execute('SELECT request FROM airtime_commissioning').fetchall() == [(TOKEN,)]
        assert db.execute('SELECT * FROM communicator_state').fetchall() == []


def test_commissioning_intent_cannot_replay_an_ordinary_generation(airtime_component):
    value = state(generation=2)
    policy, worker, path, clock, _ = airtime_component(initial_state=value)
    result = worker.control.commit_communicator_state(value, commissioning=True,
        deadline_monotonic_us=5_000_100)
    assert result.disposition is Disposition.NOT_INSTALLED


def test_unknown_commissioning_with_changed_history_is_conflict(airtime_component):
    policy, worker, path, clock, loaded = airtime_component(known_empty_airtime=True)
    channel = LostStateReply(worker.control, installed=False)
    policy.owner = CommunicatorStateOwner.from_load(control=channel, loaded=loaded)
    assert recover(policy, clock).reason is Reason.PERSISTENCE_PENDING
    with sqlite3.connect(path) as db:
        db.execute('INSERT INTO communicator_state VALUES (NULL,NULL,NULL,NULL,NULL)')
    assert recover(policy, clock).reason is Reason.RECONCILIATION_CONFLICT
    assert policy.try_spend().reason is Reason.RECONCILIATION_CONFLICT


@pytest.mark.parametrize('commissioning', [None, True, 'pending', 1])
def test_load_marker_classification_is_strictly_typed(commissioning):
    with pytest.raises(TypeError):
        CommunicatorStateLoadResult(LoadStatus.STATE_UNAVAILABLE, E.DiagnosticOperation.READ,
            state_condition=Condition.MISSING, commissioning=commissioning)


def test_failed_load_cannot_claim_observed_token():
    with pytest.raises(ValueError):
        CommunicatorStateLoadResult(LoadStatus.DATABASE_ERROR, E.DiagnosticOperation.READ,
            commissioning=Marker.PENDING)
