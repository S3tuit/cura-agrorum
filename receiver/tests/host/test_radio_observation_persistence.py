from dataclasses import replace

from cura_receiver.generated import receiver_enums_generated as E
from cura_receiver.ordinary_persistence import OrdinaryBatchCommitOutcome as Outcome
from cura_receiver.persist_queue_entities import RADIO_OBSERVATION_V1_SPEC, RECEIVER_HEALTH_REQUEST_V1_SPEC
from cura_receiver.sqlite_transactions import SqliteTransactions
from tests.support.builders.persistence import _health_request, _radio_observation


def publish(queue, entity, spec):
    reserved = queue.try_reserve_one(spec)
    assert reserved.status is E.AdmissionResult.RESERVED
    reserved.reservation.publish(entity)


def test_health_and_snapshot_roll_back_together_then_retry_identically(setup, monkeypatch):
    _, db, queue, clock, create = setup
    persistence = create()
    persistence.enable_admission()
    health = _health_request()
    publish(queue, health, RECEIVER_HEALTH_REQUEST_V1_SPEC)
    repository_type = type(persistence._repository)
    original = repository_type.insert_receiver_health

    def fail_after_snapshot(self, entity):
        assert db.execute('SELECT chip_status FROM radio_observations').fetchone() == (0x52,)
        raise OSError('injected health write failure')

    monkeypatch.setattr(repository_type, 'insert_receiver_health', fail_after_snapshot)
    assert persistence.attempt(max_entities=1).outcome is Outcome.NOT_COMMITTED
    assert db.execute('SELECT count(*) FROM radio_observations').fetchone() == (0,)
    assert db.execute('SELECT count(*) FROM receiver_health').fetchone() == (0,)
    monkeypatch.setattr(repository_type, 'insert_receiver_health', original)
    clock.advance_elapsed_us(1_000_000)
    assert persistence.attempt(max_entities=1).acknowledged_entities == 1
    assert db.execute('SELECT h.radio_observation_sequence, o.chip_status FROM receiver_health h '
                      'JOIN radio_observations o USING (receiver_instance_id)').fetchone() == (1, 0x52)
    assert db.execute('PRAGMA foreign_key_check').fetchall() == []


def test_unknown_commit_preserves_both_effects_and_exact_capture(setup):
    _, db, queue, clock, create = setup

    class LostReply(SqliteTransactions):
        lost = False

        def commit(self, connection):
            super().commit(connection)
            if not self.lost:
                self.lost = True
                raise OSError('lost reply')

    persistence = create(LostReply())
    persistence.enable_admission()
    publish(queue, _health_request(), RECEIVER_HEALTH_REQUEST_V1_SPEC)
    assert persistence.attempt(max_entities=1).outcome is Outcome.OUTCOME_UNKNOWN
    before = db.execute('SELECT * FROM radio_observations').fetchall()
    clock.advance_elapsed_us(1_000_000)
    assert persistence.attempt(max_entities=1).acknowledged_entities == 1
    assert db.execute('SELECT * FROM radio_observations').fetchall() == before
    assert db.execute('SELECT count(*) FROM receiver_health').fetchone() == (1,)


def test_partial_standalone_observation_preserves_failure_and_nulls(setup):
    _, db, queue, _, create = setup
    persistence = create()
    persistence.enable_admission()
    observation = _radio_observation()
    snapshot = replace(observation.snapshot, capture_status=E.RadioCaptureStatus.PARTIAL,
                       chip_status=None, chip_mode=None, failure_code=2,
                       failure_stage=1, failure_opcode=0xC0, os_errno=5)
    publish(queue, replace(observation, snapshot=snapshot), RADIO_OBSERVATION_V1_SPEC)
    assert persistence.attempt(max_entities=1).acknowledged_entities == 1
    assert db.execute('SELECT chip_status, chip_mode_id, os_errno, irq_status FROM radio_observations').fetchone() == (None, None, 5, 0)


def test_health_foreign_instance_is_quarantined_without_snapshot_side_effect(setup):
    _, db, queue, clock, create = setup
    persistence = create()
    persistence.enable_admission()
    health = _health_request()
    publish(queue, replace(health, radio_observation=replace(health.radio_observation,
                   receiver_instance_id=b'x'*16)), RECEIVER_HEALTH_REQUEST_V1_SPEC)
    for _ in range(4):
        persistence.attempt(max_entities=1)
        clock.advance_elapsed_us(1_000_000)
    assert db.execute('SELECT count(*) FROM radio_observations').fetchone() == (0,)
    assert db.execute('SELECT count(*) FROM receiver_health').fetchone() == (0,)
    assert db.execute('SELECT count(*) FROM quarantined_entities').fetchone() == (1,)


def test_existing_unpaired_observation_is_not_repaired_into_health(setup):
    _, db, queue, clock, create = setup
    persistence = create()
    persistence.enable_admission()
    health = _health_request()
    publish(queue, health.radio_observation, RADIO_OBSERVATION_V1_SPEC)
    assert persistence.attempt(max_entities=1).acknowledged_entities == 1
    publish(queue, health, RECEIVER_HEALTH_REQUEST_V1_SPEC)
    for _ in range(4):
        persistence.attempt(max_entities=1)
        clock.advance_elapsed_us(1_000_000)
    assert db.execute('SELECT count(*) FROM radio_observations').fetchone() == (1,)
    assert db.execute('SELECT count(*) FROM receiver_health').fetchone() == (0,)
    assert db.execute('SELECT count(*) FROM quarantined_entities').fetchone() == (0,)
    assert queue.snapshot().admission_snapshot.state is E.PersistenceAdmissionState.UNAVAILABLE_INCOMPATIBLE_SCHEMA
    assert queue.snapshot().claimed_entities == 1


def test_replay_with_changed_snapshot_stops_admission_without_overwriting_pair(setup):
    _, db, queue, clock, create = setup
    persistence = create()
    persistence.enable_admission()
    health = _health_request()
    publish(queue, health, RECEIVER_HEALTH_REQUEST_V1_SPEC)
    assert persistence.attempt(max_entities=1).acknowledged_entities == 1
    captured = health.radio_observation
    changed = replace(captured, snapshot=replace(captured.snapshot, irq_status=0x20))
    publish(queue, replace(health, radio_observation=changed), RECEIVER_HEALTH_REQUEST_V1_SPEC)
    for _ in range(4):
        persistence.attempt(max_entities=1)
        clock.advance_elapsed_us(1_000_000)
    assert db.execute('SELECT irq_status FROM radio_observations').fetchone() == (0,)
    assert db.execute('SELECT count(*) FROM receiver_health').fetchone() == (1,)
    assert db.execute('SELECT count(*) FROM quarantined_entities').fetchone() == (0,)
    assert queue.snapshot().admission_snapshot.state is E.PersistenceAdmissionState.UNAVAILABLE_INCOMPATIBLE_SCHEMA
    assert queue.snapshot().claimed_entities == 1
