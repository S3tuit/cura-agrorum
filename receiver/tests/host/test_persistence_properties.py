from __future__ import annotations

import shutil
import tempfile
from dataclasses import replace
from pathlib import Path

from hypothesis import settings, strategies as st
from hypothesis.stateful import RuleBasedStateMachine, invariant, precondition, rule
from cura_receiver.database_initializer import initialize_database
from cura_receiver.ordinary_persistence import OrdinaryPersistence
from cura_receiver.persist_queue import PersistQueue
from cura_receiver.persist_queue_entities import (
    CLOCK_OBSERVATION_V1_SPEC,
    MEASUREMENT_PROFILE_V1_SPEC,
    PROFILE_ONLY_V1_SPEC,
    ProfileOnlyUnitV1,
)
from cura_receiver.receiver_startup import (
    ReceiverInstanceStart,
    insert_receiver_instance_start,
)
from cura_receiver.sqlite_database import open_receiver_database
from cura_receiver.sqlite_transactions import SqliteTransactions
from tests.support.builders.persistence import (
    GROUP,
    _measurement,
    _observation,
    _profile,
)
from tests.support.fakes.os_clock import FakeOsClock
from tests.support.models.ordinary_persistence import DurableModel, Work


class FaultTransactions(SqliteTransactions):
    mode = "none"

    def begin(self, connection):
        if self.mode == "begin":
            raise OSError(5, "injected begin failure")
        super().begin(connection)

    def commit(self, connection):
        if self.mode == "commit_absent":
            raise OSError(5, "unconfirmed absent commit")
        super().commit(connection)
        if self.mode == "commit_durable":
            raise OSError(5, "unconfirmed durable commit")


# Generated actions compare independent effects, admission, immutable history and FIFO claims after every step.
class PersistenceMachine(RuleBasedStateMachine):
    def __init__(self):
        super().__init__()
        self.root = Path(tempfile.mkdtemp(prefix="cura-persistence-model-"))
        self.path = self.root / "receiver.db"
        self.checked = False
        initialize_database(self.path, GROUP)
        self.model = DurableModel()
        self.clock = FakeOsClock(monotonic_us=100)
        self.fault = FaultTransactions()
        self.epoch = 0
        self.uid = 0
        self.history = {}
        self._open()

    def _open(self):
        self.epoch += 1
        self.instance = bytes.fromhex(f"00112233445546778899{self.epoch:012x}")
        self.connection = open_receiver_database(
            self.path, GROUP, minimum_free_bytes=0
        ).connection
        instance = ReceiverInstanceStart(self.instance, 0)
        insert_receiver_instance_start(self.connection, instance, b"b" * 16)
        self.queue = PersistQueue()
        self.owner = OrdinaryPersistence(
            self.connection,
            self.queue,
            instance=instance,
            database_path=self.path,
            group_id=GROUP,
            clock=self.clock,
            transactions=self.fault,
        )
        self.owner.enable_admission()

    def _publish(self, work, entity, spec):
        self.checked = False
        self.queue.try_reserve_one(spec).reservation.publish(entity)
        self.model.queue.append(work)

    @precondition(lambda self: self.model.state == "AVAILABLE")
    @rule(
        kind=st.sampled_from(["clock", "profile", "reading"]),
        message=st.integers(100, 103),
        sample=st.integers(200, 202),
        soil=st.sampled_from([1000, 1001]),
        domain=st.sampled_from([1, 2]),
        node=st.sampled_from([b"n" * 8, b"m" * 8]),
        poison=st.booleans(),
    )
    def enqueue(self, kind, message, sample, soil, domain, node, poison):
        self.uid += 1
        work = Work(self.uid, self.instance, self.uid, kind)
        if kind == "clock":
            entity = replace(
                _observation(sequence=self.uid), receiver_instance_id=self.instance
            )
            spec = CLOCK_OBSERVATION_V1_SPEC
        elif kind == "profile":
            entity = ProfileOnlyUnitV1(
                replace(
                    _profile(sequence=self.uid),
                    receiver_instance_id=self.instance,
                    busy_wait_count=-1 if poison else 0,
                )
            )
            work = replace(work, frame=bytes(255), poison=poison)
            spec = PROFILE_ONLY_V1_SPEC
        else:
            entity = _measurement(
                sequence=self.uid,
                instance=self.instance,
                message=message,
                sample=sample,
                soil=soil,
                domain=domain,
                node=node,
            )
            # The oracle receives the same immutable raw input facts, never a
            # production-derived classification, projection or decoded SQL row.
            work = replace(
                work,
                node=node,
                message=message,
                sample=sample,
                frame=entity.profile.received_frame,
                body=entity.candidate.reading_body,
            )
            spec = MEASUREMENT_PROFILE_V1_SPEC
        self._publish(work, entity, spec)

    @rule(
        count=st.integers(1, 5),
        fault=st.sampled_from(["none", "begin", "commit_absent", "commit_durable"]),
    )
    def attempt(self, count, fault):
        self.checked = False
        if self.owner.retry_deadline_monotonic_us is not None:
            self.clock.advance_elapsed_us(
                max(
                    0,
                    self.owner.retry_deadline_monotonic_us
                    - self.clock.now_monotonic_us(),
                )
            )
        expected = self.model.attempt(count, fault)
        self.fault.mode = fault
        try:
            observed = self.owner.attempt(max_entities=count)
        finally:
            self.fault.mode = "none"
        names = {
            "COMMITTED": "committed",
            "NOT_COMMITTED": "failed",
            "OUTCOME_UNKNOWN": "unknown",
        }
        assert (None if observed is None else names[observed.outcome.name]) == expected

    @precondition(
        lambda self: self.model.state == "AVAILABLE" and bool(self.model.rows["clocks"])
    )
    @rule()
    def identity_collision(self):
        (instance, sequence), value = next(iter(self.model.rows["clocks"].items()))
        self.uid += 1
        entity = replace(
            _observation(sequence=sequence),
            receiver_instance_id=instance,
            sampled_at_monotonic_us=value + 1,
        )
        self._publish(
            Work(self.uid, instance, sequence, "clock", value=value + 1),
            entity,
            CLOCK_OBSERVATION_V1_SPEC,
        )

    @rule()
    def restart(self):
        self.checked = False
        # SIGKILL boundaries are tested separately. Here process restart is the
        # observable loss of all volatile queue/derived state plus a new instance.
        self.owner.close()
        self.model.restart()
        self._open()

    @invariant()
    def compare(self):
        expected_state = {
            "AVAILABLE": "AVAILABLE",
            "IO": "UNAVAILABLE_IO",
            "INCOMPATIBLE": "UNAVAILABLE_INCOMPATIBLE_SCHEMA",
        }[self.model.state]
        snapshot = self.queue.snapshot()
        assert snapshot.admission_snapshot.state.name == expected_state
        assert snapshot.published_entities == len(self.model.queue)
        assert snapshot.claimed_entities == self.model.claimed
        assert {
            (i, s): v
            for i, s, v in self.connection.execute(
                "SELECT receiver_instance_id,observation_sequence,sampled_at_monotonic_us FROM clock_observations"
            )
        } == self.model.rows["clocks"]
        assert {
            (i, s): (f, c)
            for i, s, f, c in self.connection.execute(
                "SELECT receiver_instance_id,occurrence_sequence,received_frame,persistence_classification_id FROM message_profiles"
            )
        } == self.model.rows["profiles"]
        assert {
            (n, m): tuple(rest)
            for n, m, *rest in self.connection.execute(
                "SELECT node_id,message_id,sample_id,reading_body,is_canonical_for_sample,first_receiver_instance_id,first_occurrence_sequence FROM reading_messages"
            )
        } == self.model.rows["readings"]
        assert self.connection.execute(
            "SELECT count(*) FROM quarantined_entities"
        ).fetchone() == (len(self.model.rows["quarantine"]),)
        # Every column of every previously durable row must remain byte-for-byte
        # stable, including fields outside the model's policy projection.
        for table in (
            "clock_observations",
            "message_profiles",
            "reading_messages",
            "quarantined_entities",
        ):
            current = set(self.connection.execute(f"SELECT * FROM {table}").fetchall())
            assert self.history.get(table, set()) <= current
            self.history[table] = current
        assert self.connection.execute("PRAGMA foreign_key_check").fetchall() == []
        self.checked = True

    def teardown(self):
        self.owner.close()
        if self.checked:
            shutil.rmtree(self.root)


TestPersistenceMachine = PersistenceMachine.TestCase
TestPersistenceMachine.settings = settings(
    max_examples=60, stateful_step_count=45, deadline=None
)
