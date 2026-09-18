"""Controlled stop through the production application and real persistence worker."""

import sqlite3
from dataclasses import replace
from threading import Event

import pytest

from cura_receiver.persistence_control_values import ReceiverCleanStopCommitDisposition as D, ReceiverCleanStopCommitFailureKind as F
from cura_receiver.generated import receiver_enums_generated as E
from tests.host.test_application import make_application


def wait_for_worker(app):
    def wait(until):
        # Let the actual worker run, with a bounded virtual deadline as fallback.
        Event().wait(0.001)
        app.clock.advance_elapsed_us(max(0, until-app.clock.now_monotonic_us()))
    return wait


def test_clean_stop_drains_marks_and_is_idempotent(tmp_path):
    app, _, database, _ = make_application(tmp_path)
    try:
        assert app.start().ready
        app.runtime.step()
        app.request_stop()
        deadline = app.stop_deadline
        result = app.shutdown(clean_requested=True, wait=wait_for_worker(app))
        assert result.radio_safe and result.queue_drained and result.worker_stopped
        assert result.clean_stop_confirmed and result.failure is None
        assert result.authoritative_generation == 0
        assert app.stop_deadline == deadline
        assert app.shutdown(clean_requested=True) is result
        with sqlite3.connect(database) as db:
            assert db.execute('SELECT clean_stopped_at_monotonic_us FROM receiver_instances').fetchone()[0] is not None
    finally:
        app.worker.finish_test()


@pytest.mark.parametrize('installed', [True, False])
def test_unknown_clean_marker_retries_identical_request(tmp_path, monkeypatch, installed):
    app, _, _, _ = make_application(tmp_path)
    try:
        assert app.start().ready
        real = app.worker.control.commit_receiver_clean_stop
        requests = []
        def commit(marker, **kwargs):
            requests.append(marker)
            if len(requests) == 1:
                result = real(marker, **kwargs) if installed else real(marker, deadline_monotonic_us=0)
                return replace(result, disposition=D.OUTCOME_UNKNOWN, failure_kind=F.DEADLINE_EXCEEDED, operation=E.DiagnosticOperation.CLEANUP)
            return real(marker, **kwargs)
        monkeypatch.setattr(app.worker.control, 'commit_receiver_clean_stop', commit)
        result = app.shutdown(clean_requested=True, wait=wait_for_worker(app))
        assert result.clean_stop_confirmed
        assert len(requests) == 2 and requests[0] is requests[1]
    finally:
        app.worker.finish_test()


def test_unexpected_marker_exception_still_stops_disk_owner(tmp_path, monkeypatch):
    app, _, _, _ = make_application(tmp_path)
    try:
        assert app.start().ready
        def fail(*args, **kwargs):
            raise RuntimeError('marker transport escape')
        monkeypatch.setattr(app.worker.control, 'commit_receiver_clean_stop', fail)
        result = app.shutdown(clean_requested=True, wait=wait_for_worker(app))
        assert not result.clean_stop_confirmed and result.worker_stopped
        assert result.failure is not None
    finally:
        app.worker.finish_test()


def test_fatal_exit_never_marks_clean(tmp_path):
    app, _, database, _ = make_application(tmp_path)
    try:
        assert app.start().ready
        app.runtime._terminate(RuntimeError('injected fatal'))
        result = app.shutdown(clean_requested=False, wait=wait_for_worker(app))
        assert not result.clean_stop_confirmed and result.worker_stopped
        with sqlite3.connect(database) as db:
            assert db.execute('SELECT clean_stopped_at_monotonic_us FROM receiver_instances').fetchone() == (None,)
    finally:
        app.worker.finish_test()


def test_unsafe_radio_cleanup_forbids_marker(tmp_path):
    app, io, database, _ = make_application(tmp_path)
    try:
        assert app.start().ready
        io.busy_forever = True
        result = app.shutdown(clean_requested=True, wait=wait_for_worker(app))
        assert not result.radio_safe and not result.clean_stop_confirmed
        assert result.worker_stopped and result.failure == 'RADIO_UNSAFE'
        with sqlite3.connect(database) as db:
            assert db.execute('SELECT clean_stopped_at_monotonic_us FROM receiver_instances').fetchone() == (None,)
    finally:
        app.worker.finish_test()


def test_unknown_marker_expires_without_reopening_diagnostics(tmp_path, monkeypatch):
    app, _, _, _ = make_application(tmp_path)
    try:
        assert app.start().ready
        real = app.worker.control.commit_receiver_clean_stop
        requests, counts = [], []
        def uncertain(marker, **kwargs):
            assert app.worker.queue.snapshot().closed
            requests.append(marker)
            counts.append(app.admission.counts)
            result = real(marker, deadline_monotonic_us=0)
            app.clock.advance_elapsed_us(1_000_000)
            return replace(result, disposition=D.OUTCOME_UNKNOWN)
        monkeypatch.setattr(app.worker.control, 'commit_receiver_clean_stop', uncertain)
        result = app.shutdown(clean_requested=True, wait=wait_for_worker(app))
        assert not result.clean_stop_confirmed and result.failure == 'CLEAN_STOP_UNRESOLVED'
        assert requests and all(marker is requests[0] for marker in requests)
        assert all(count == counts[0] for count in counts)
        assert app.clock.now_monotonic_us() >= app.stop_deadline
    finally:
        app.worker.finish_test()


@pytest.mark.parametrize('fault', ['checkpoint', 'close'])
def test_durable_marker_survives_final_storage_cleanup_failure(tmp_path, monkeypatch, fault):
    from cura_receiver.sqlite_database import ReceiverDatabase
    from cura_receiver.sqlite_transactions import SqliteTransactions
    app, _, database, _ = make_application(tmp_path)
    trace = []
    class Transactions(SqliteTransactions):
        def checkpoint(self, db):
            assert db.execute('SELECT clean_stopped_at_monotonic_us FROM receiver_instances').fetchone()[0] is not None
            trace.append('checkpoint')
            if fault == 'checkpoint':
                raise OSError(5, 'checkpoint failure')
            return super().checkpoint(db)
    app.worker._transactions = Transactions()
    original = ReceiverDatabase.close
    def close(db):
        trace.append('close')
        original(db)
        if fault == 'close':
            raise OSError(5, 'close failure')
    monkeypatch.setattr(ReceiverDatabase, 'close', close)
    try:
        assert app.start().ready
        result = app.shutdown(clean_requested=True, wait=wait_for_worker(app))
        assert result.clean_stop_confirmed and result.worker_stopped
        assert trace == ['checkpoint', 'close']
        with sqlite3.connect(database) as db:
            assert db.execute('SELECT clean_stopped_at_monotonic_us FROM receiver_instances').fetchone()[0] is not None
    finally:
        if fault == 'close':
            with pytest.raises(OSError, match='close failure'):
                app.worker.finish_test()
        else:
            app.worker.finish_test()


def test_unresolved_queue_reservation_cannot_get_clean_marker(tmp_path):
    from cura_receiver.persist_queue_entities import PROFILE_ONLY_V1_SPEC
    app, _, database, _ = make_application(tmp_path)
    try:
        assert app.start().ready
        reserved = app.admission.try_reserve_one(PROFILE_ONLY_V1_SPEC)
        assert reserved.reservation is not None
        result = app.shutdown(clean_requested=True, wait=wait_for_worker(app))
        assert not result.queue_drained and not result.clean_stop_confirmed
        assert result.failure == 'QUEUE_NOT_DRAINED'
        with sqlite3.connect(database) as db:
            assert db.execute('SELECT clean_stopped_at_monotonic_us FROM receiver_instances').fetchone() == (None,)
    finally:
        app.worker.finish_test()


def _application_crash_child(root, boundary, pipe):
    app, _, _, _ = make_application(root)
    assert app.start().ready
    if boundary == 'after_startup':
        pipe.send('ready')
        pipe.recv()
    else:
        real = app.worker.control.commit_receiver_clean_stop
        def committed(marker, **kwargs):
            result = real(marker, **kwargs)
            assert result.disposition is D.COMMITTED
            pipe.send('ready')
            pipe.recv()
            return result
        app.worker.control.commit_receiver_clean_stop = committed
        app.shutdown(clean_requested=True, wait=wait_for_worker(app))


@pytest.mark.parametrize('boundary', ['after_startup', 'after_clean_commit'])
def test_application_sigkill_and_restart_preserve_marker_meaning(tmp_path, boundary):
    import multiprocessing
    context = multiprocessing.get_context('spawn')
    parent, child = context.Pipe()
    process = context.Process(target=_application_crash_child, args=(tmp_path, boundary, child))
    process.start()
    child.close()
    try:
        assert parent.poll(10), 'application did not reach crash boundary'
        assert parent.recv() == 'ready'
        process.kill()
        process.join(5)
        assert process.exitcode == -9
        with sqlite3.connect(tmp_path/'worker.db') as db:
            marker = db.execute('SELECT clean_stopped_at_monotonic_us FROM receiver_instances').fetchone()[0]
            assert (marker is not None) == (boundary == 'after_clean_commit')
        restarted, _, database, _ = make_application(tmp_path, reuse_storage=True, instance_id=bytes.fromhex('112233445566478899aabbccddeeff00'))
        try:
            assert restarted.start().ready
            assert restarted.runtime.communicator.occurrence_sequence == 0
            assert restarted.runtime.telemetry.health_sequence == 0
            assert restarted.runtime.telemetry.diagnostic_sequence <= 1
            stopped = restarted.shutdown(clean_requested=True, wait=wait_for_worker(restarted))
            assert stopped.clean_stop_confirmed
            with sqlite3.connect(database) as db:
                rows = db.execute('SELECT receiver_instance_id FROM receiver_instances').fetchall()
                assert len(rows) == 2 and rows[0] != rows[1]
        finally:
            restarted.worker.finish_test()
    finally:
        if process.is_alive():
            process.kill()
            process.join(5)
        parent.close()


def test_unknown_complete_state_prevents_clean_marker(tmp_path):
    from tests.support.coordination.state_commit import LostStateReply
    from tests.support.builders.persistence_control import synthetic
    app, _, database, _ = make_application(tmp_path)
    try:
        assert app.start().ready
        owner = app.runtime.communicator.airtime.owner
        lost = LostStateReply(app.worker.control, installed=True)
        lost.fail_load = True
        owner._control = lost
        owner.commit(synthetic(), purpose=E.PersistenceControlPurpose.RTC_PROVENANCE,
            deadline_monotonic_us=app.clock.now_monotonic_us()+1_000_000)
        pending = owner.pending
        assert pending is not None
        result = app.shutdown(clean_requested=True, wait=wait_for_worker(app))
        assert not result.clean_stop_confirmed and result.authoritative_generation is None
        assert owner.pending is pending
        with sqlite3.connect(database) as db:
            assert db.execute('SELECT clean_stopped_at_monotonic_us FROM receiver_instances').fetchone() == (None,)
    finally:
        app.worker.finish_test()


def test_application_stop_during_rtc_read_cancels_before_write(tmp_path):
    from tests.host.test_runtime_time import sample, tracking, UTC
    from cura_receiver.ports.ds3231 import Ds3231ReadResult, Ds3231ReadStatus as R
    app, _, _, _ = make_application(tmp_path)
    try:
        assert app.start().ready
        c = app.runtime.communicator
        sample(c.time, app.kernel)
        c.time.sample_network(tracking(c.time))
        c.airtime.update_time(c.time.airtime_correlation(), rtc_health=c.time.state.rtc_health)
        c.airtime.recover(deadline_monotonic_us=app.clock.now_monotonic_us()+1_000_000)
        assert c.time.durable_state is not None
        app.clock.advance_elapsed_us(max(0, c.time.next_rtc_attempt_monotonic_us-app.clock.now_monotonic_us()))
        app.runtime.scheduler.next_airtime = app.clock.now_monotonic_us()+10_000_000
        app.runtime.scheduler.initial_health_pending = False
        def read():
            app.request_stop()
            start = app.clock.now_monotonic_us()
            app.clock.advance_elapsed_us(2_900_000)
            return Ds3231ReadResult(R.OK, start, app.clock.now_monotonic_us(), UTC//1_000_000)
        app.rtc.read_results.append(read)
        turn = app.runtime.step()
        assert turn.work.name == "RTC", turn
        assert app.stop_event.is_set()
        assert app.run() == 0
        assert c.time.rtc_refresh_episode is not None
        deadline = app.stop_deadline
        result = app.shutdown(clean_requested=True, wait=wait_for_worker(app))
        assert result.radio_safe and result.worker_stopped and result.clean_stop_confirmed
        assert c.time.rtc_refresh_episode is None
        assert not any(call[0] == 'write' for call in app.rtc.calls)
        assert app.stop_deadline == deadline and app.clock.now_monotonic_us() < deadline
    finally:
        app.worker.finish_test()
