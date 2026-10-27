import sqlite3

import pytest

from cura_receiver.clock_correlation import AnalysisInstance, ClockCorrelation, clock_observation_from_row
from cura_receiver.generated import receiver_enums_generated as E
from cura_receiver.generated.receiver_entities_generated import CLOCK_OBSERVATION_V1_COLUMNS
from cura_receiver.ports.radio import RadioBackendError, RadioFailure, Error, Stage
from tests.host.test_communicator import composition, runtime_dispatch
from tests.host.test_application import make_application, start_application


@pytest.mark.parametrize('path', ['fatal_cleanup', 'terminal_radio'])
def test_final_health_fences_expiry_without_another_scheduler_time_turn(composition, path):
    c = composition()
    runtime = runtime_dispatch(c)
    expiry = min(c.time.schedule.trust_expires_at_monotonic_us, c.time.tracking_poll_deadline)
    c.clock.advance_elapsed_us(expiry - c.clock.now_monotonic_us() - 1)
    # Physical calls during capture/cleanup cross expiry after the last normal
    # scheduling boundary. Final telemetry must establish the clock fence itself.
    c.io.after_transfer = lambda command: c.clock.advance_elapsed_us(100)
    if path == 'fatal_cleanup':
        original = RuntimeError('fatal owner failure')
        runtime._terminate(original)
        assert runtime.failure.original is original
    else:
        c.radio.shutdown()
        runtime.step()
    assert runtime.terminal and runtime.telemetry.health_sequence == 1
    c.worker.request_stop(deadline_monotonic_us=c.clock.now_monotonic_us() + 5_000_000)
    c.worker.join(5)
    assert c.worker.failure is None
    with sqlite3.connect(c.database) as db:
        sampled, quality = db.execute('SELECT communicator_sampled_at_monotonic_us, '
                                     'system_time_quality_id FROM receiver_health').fetchone()
        observations = [clock_observation_from_row(row) for row in db.execute(
            'SELECT ' + ','.join(CLOCK_OBSERVATION_V1_COLUMNS)
            + ' FROM clock_observations ORDER BY observation_sequence')]
        instance = AnalysisInstance(*db.execute('SELECT instance_ordinal, receiver_instance_id, '
            'linux_boot_id, started_at_monotonic_us FROM receiver_instances').fetchone())
    assert sampled > expiry
    assert quality == E.SystemTimeQuality.UNTRUSTED.value
    assert len(observations) == 2
    boundary = observations[-1]
    assert boundary.sampled_at_monotonic_us == expiry
    assert boundary.system_time_quality is E.SystemTimeQuality.UNTRUSTED
    assert ClockCorrelation([instance], observations).correlate(instance.receiver_instance_id, sampled) is None


def test_runtime_failure_captures_before_shutdown_and_health_never_reopens(composition):
    c = composition()
    runtime = runtime_dispatch(c)
    c.io.irq = 0x20
    original = RuntimeError('fatal owner failure')
    runtime._terminate(original)
    assert runtime.failure.original is original
    assert runtime.telemetry.health_sequence == 1
    assert sum(call[0] == 'open' for call in c.io.calls) == 1
    calls = list(c.io.calls)
    runtime.final_failure_health()
    assert c.io.calls == calls
    c.worker.request_stop(deadline_monotonic_us=c.clock.now_monotonic_us() + 5_000_000)
    c.worker.join(5)
    assert c.worker.failure is None
    with sqlite3.connect(c.database) as db:
        rows = db.execute('SELECT software_state_id, capture_status_id, irq_status FROM radio_observations ORDER BY observation_sequence').fetchall()
        assert rows[0] == (E.RadioState.RX_SINGLE.value, E.RadioCaptureStatus.COMPLETE.value, 0x20)
        assert rows[-1] == (E.RadioState.SHUTDOWN.value, E.RadioCaptureStatus.UNAVAILABLE.value, None)
        assert db.execute('SELECT consistency_id FROM receiver_health').fetchone() == (E.HealthConsistency.INDETERMINATE.value,)


@pytest.mark.parametrize('missing', [False, True])
def test_initialization_failure_retains_pre_cleanup_and_final_health(tmp_path, missing):
    app, io, database, _ = make_application(tmp_path)
    def fail(command):
        del io.hooks[0x8a]
        io.irq = 0x20
        raise RadioBackendError(RadioFailure(Error.IO, Stage.WRITE_COMMAND, os_errno=5, hardware_missing=missing))
    io.hooks[0x8a] = fail
    try:
        assert not start_application(app).ready
        assert app.runtime.telemetry.health_sequence == 1
        app.worker.request_stop(deadline_monotonic_us=app.clock.now_monotonic_us() + 5_000_000)
        app.worker.join(5)
        assert app.worker.failure is None
        with sqlite3.connect(database) as db:
            rows = db.execute('SELECT software_state_id, irq_status, capture_status_id FROM radio_observations ORDER BY observation_sequence').fetchall()
            assert rows[0][:2] == (E.RadioState.INITIALIZING.value, 0x20)
            expected = E.RadioState.HARDWARE_MISSING if missing else E.RadioState.INITIALIZATION_FAILED
            assert rows[-1] == (expected.value, None, E.RadioCaptureStatus.UNAVAILABLE.value)
            assert db.execute('SELECT count(*) FROM receiver_health').fetchone() == (1,)
    finally:
        app.worker.finish_test()


def test_capture_failure_does_not_replace_original_or_block_cleanup(composition, monkeypatch):
    c = composition()
    runtime = runtime_dispatch(c)
    def broken(*args, **kwargs):
        raise MemoryError('cannot allocate observation')
    monkeypatch.setattr(c.radio, 'observe', broken)
    original = RuntimeError('original')
    runtime._terminate(original)
    assert runtime.failure.original is original
    assert c.radio.state is E.RadioState.SHUTDOWN
    assert c.queue.snapshot().reserved_entities == 0


@pytest.mark.parametrize('gate', ['closed', 'reservation', 'clock', 'expired', 'unsound'])
def test_final_health_respects_admission_and_shutdown_gates(composition, monkeypatch, gate):
    c = composition(isolated_queue=True)
    runtime = runtime_dispatch(c)
    if gate == 'closed':
        c.queue.close()
    elif gate == 'clock':
        c.clock.advance_elapsed_us(60_000_000)
        c.time.expire_due()
    elif gate == 'expired':
        runtime.scheduler.shutdown_deadline = lambda: c.clock.now_monotonic_us()
    elif gate == 'unsound':
        from types import SimpleNamespace
        runtime.failure = SimpleNamespace(queue_sound=False)
    else:
        from cura_receiver.persist_queue_entities import DIAGNOSTIC_V1_SPEC
        c.queue.claim_batch(max_entities=1).acknowledge_durable(completed_entities=1)
        reservation = c.communicator.queue.try_reserve_one(DIAGNOSTIC_V1_SPEC).reservation
    def forbidden(*args, **kwargs):
        pytest.fail('gated final health must not touch hardware')
    monkeypatch.setattr(c.radio, 'observe', forbidden)
    runtime.final_failure_health()
    assert runtime.telemetry.health_sequence == 0
    if gate == 'reservation':
        reservation.cancel()
