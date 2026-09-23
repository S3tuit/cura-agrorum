from dataclasses import replace
from pathlib import Path
from queue import Queue
import sqlite3
import tempfile

from hypothesis import given, settings, strategies as st

from cura_receiver.generated.receiver_enums_generated import (
    AdmissionResult,
    PersistenceAdmissionState as State,
)
from cura_receiver.persist_queue_entities import PROFILE_ONLY_V1_SPEC, ProfileOnlyUnitV1
from cura_receiver.receiver_startup import ReceiverInstanceStart
from cura_receiver.sqlite_transactions import SqliteTransactions
from tests.support.builders.persistence import INSTANCE, _profile
from tests.support.builders.persistence_control import synthetic
from tests.support.coordination.persistence_worker import CheckedPersistenceWorker
from tests.support.fakes.os_clock import FakeOsClock


class Model:
    """Primitive oracle for this schedule family; no production policy calls."""

    def __init__(self):
        self.now = 100
        self.deadline = None
        self.delay = 0
        self.checkpoint = False
        self.pending = []
        self.rows = set()
        self.generation = 0
        self.sequence = 0

    def fail(self, control):
        self.checkpoint |= control
        if self.deadline is None:
            self.delay = 250_000
            self.deadline = self.now + 250925

    def publish(self, mode):
        if self.deadline is not None:
            return False
        self.sequence += 1
        if mode != "ordinary_error":
            self.rows.add(self.sequence)
        if mode != "publish":
            self.pending.append(self.sequence)
            self.fail(False)
        return True

    def control(self, mode):
        if mode != "control_error":
            self.generation += 1
        if mode != "control_commit":
            self.fail(True)

    def advance(self, checkpoint_failure=False):
        if self.deadline is None:
            return
        self.now = self.deadline
        self.rows.update(self.pending)
        self.pending.clear()
        if self.checkpoint and checkpoint_failure:
            self.delay = min(5_000_000, self.delay * 2)
            self.deadline = self.now + (self.delay * 1003700 + 999999) // 1000000
        else:
            self.deadline = None
            self.delay = 0
            self.checkpoint = False


# Reviewed oracle example: control success cannot clear a previous fault, and due failures alone double backoff.
def test_control_recovery_model_example():
    model = Model()
    model.control("control_error")
    assert (model.deadline, model.generation, model.checkpoint) == (251025, 0, True)
    model.now = 100100
    model.control("control_commit")
    model.control("control_error")
    assert (model.deadline, model.generation) == (251025, 1)
    model.advance(True)
    assert model.deadline == 752875
    model.advance()
    assert model.deadline is None and model.generation == 1


# Reviewed oracle example: exact reconciliation removes volatile ownership without duplicating durable work.
def test_ordinary_unknown_model_example():
    model = Model()
    assert model.publish("ordinary_unknown")
    model.control("control_error")
    assert model.pending == [1] and model.rows == {1}
    assert not model.publish("publish")
    model.advance()
    assert model.pending == [] and model.rows == {1} and model.deadline is None


# Generated real-thread schedules compare generations, rows, FIFO ownership and exact deadlines after every safe boundary.
def test_generated_worker_schedules(tmp_path, worker_file_factory):
    @settings(max_examples=25, deadline=None, derandomize=True)
    @given(
        st.lists(
            st.sampled_from(
                [
                    "publish",
                    "ordinary_error",
                    "ordinary_unknown",
                    "control_commit",
                    "control_error",
                    "control_unknown",
                    "load",
                    "advance",
                    "checkpoint_error",
                    "wake",
                ]
            ),
            min_size=40,
            max_size=80,
        )
    )
    def exercise(actions):
        # Keep failing databases/WAL/SHM beside the minimal Hypothesis schedule.
        root = Path(tempfile.mkdtemp(prefix="worker-schedule-", dir=tmp_path))
        path, config, boot = worker_file_factory(root)
        waits = Queue()

        class Faults(SqliteTransactions):
            mode = ""

            def begin(self, db):
                if self.mode in ("ordinary_error", "control_error"):
                    raise OSError(5, "scheduled begin failure")
                super().begin(db)

            def commit(self, db):
                super().commit(db)
                if self.mode in ("ordinary_unknown", "control_unknown"):
                    raise OSError(5, "scheduled lost commit reply")

            def checkpoint(self, db):
                if self.mode == "checkpoint_error":
                    raise OSError(5, "scheduled checkpoint failure")
                return super().checkpoint(db)

        class Observed(CheckedPersistenceWorker):
            def _wait_for_work(self, timeout):
                waits.put(timeout)
                assert self._wake.wait(5)

        clock = FakeOsClock(monotonic_us=100)
        backend = Faults()
        owner = Observed(
            instance=ReceiverInstanceStart(INSTANCE, 0),
            database_path=path,
            configuration_path=config,
            boot_id_path=boot,
            clock=clock,
            transactions=backend,
            wake_threshold_entities=1,
            batch_limit_entities=3,
            checkpoint_threshold_bytes=1 << 40,
        )
        model = Model()
        owner.start()
        try:
            assert owner.wait_started(deadline_monotonic_us=5_000_100)
            assert waits.get(timeout=5) is None
            for index, action in enumerate(actions):
                backend.mode = action
                woke = True
                if action in ("publish", "ordinary_error", "ordinary_unknown"):
                    expected_admission = model.publish(action)
                    admission = owner.queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
                    assert (
                        admission.status is AdmissionResult.RESERVED
                    ) == expected_admission
                    if expected_admission:
                        admission.reservation.publish(
                            ProfileOnlyUnitV1(_profile(sequence=model.sequence))
                        )
                    else:
                        woke = False
                elif action.startswith("control_"):
                    value = replace(synthetic(), generation=model.generation + 1)
                    result = owner.control.commit_communicator_state(
                        value, deadline_monotonic_us=model.now + 5_000_000
                    )
                    expected = {
                        "control_commit": "COMMITTED",
                        "control_error": "NOT_INSTALLED",
                        "control_unknown": "OUTCOME_UNKNOWN",
                    }[action]
                    assert result.disposition.name == expected
                    model.control(action)
                elif action == "load":
                    loaded = owner.control.load_communicator_state(
                        deadline_monotonic_us=model.now + 5_000_000
                    )
                    assert (
                        loaded.state.generation if loaded.state else 0
                    ) == model.generation
                else:
                    if action != "wake":
                        model.advance(action == "checkpoint_error")
                        clock.advance_elapsed_us(model.now - clock.now_monotonic_us())
                    owner._wake.set()
                if woke:
                    waits.get(timeout=5)
                snapshot = owner.queue.snapshot()
                assert snapshot.published_entities == len(model.pending), (
                    index,
                    actions[: index + 1],
                )
                assert snapshot.admission_snapshot.state is (
                    State.AVAILABLE if model.deadline is None else State.UNAVAILABLE_IO
                )
                assert owner._recovery.retry_deadline_monotonic_us == model.deadline
                assert owner._recovery.checkpoint_pending == model.checkpoint
                with sqlite3.connect(path) as db:
                    assert db.execute(
                        "SELECT occurrence_sequence FROM message_profiles ORDER BY occurrence_sequence"
                    ).fetchall() == [(n,) for n in sorted(model.rows)]
                    assert db.execute(
                        "SELECT generation FROM communicator_state"
                    ).fetchall() == ([(model.generation,)] if model.generation else [])
            (root / "schedule.txt").write_text(repr(actions))
        finally:
            owner.finish_test()

    exercise()
