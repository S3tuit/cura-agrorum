"""Actual process signals exercise production owners and real SQLite.

Named pipe/primitive barriers establish ordering. Parent timeouts detect
deadlock only; they do not manufacture the tested interleaving.
"""

import multiprocessing
import os
from pathlib import Path
import signal
import sqlite3
import time
import traceback

import pytest

from cura_receiver.generated import receiver_enums_generated as E
from cura_receiver.platform import linux_signal_wait as wait_module
from cura_receiver.platform.linux_clocks import LinuxOsClock
from cura_receiver.platform.linux_signal_wait import LinuxSignalWait
from cura_receiver.generated.receiver_entities_generated import ClockObservationV1
from tests.host.test_application import make_application


def signal_application(connection, directory, boundary):
    clock = LinuxOsClock()  # No fake clock lock may be acquired by a handler.
    app, io, database, _ = make_application(Path(directory), clock=clock, radio_wait=clock)
    real_poll = wait_module.select.select
    try:
        with LinuxSignalWait(clock, app.stop_intent) as wait:
            assert app.start().ready
            if boundary in ('before_wait', 'before_poll', 'blocked_poll'):
                # Establish the retained clock-boundary retry state that uses
                # the application's waiter. Ordinary idle RX waits in Radio.
                time = app.runtime.communicator.time
                time.pending_observation = ClockObservationV1(
                    app.instance.receiver_instance_id, time.observation_sequence + 1,
                    time.state.generation, clock.now_monotonic_us(), None, False,
                    time.state.quality, time.state.rtc_health)
                app.runtime.scheduler.boundary_retry = clock.now_monotonic_us() + 2_000_000
                notified = False
                def barrier():
                    nonlocal notified
                    if not notified:
                        notified = True
                        connection.send('ready')
                        if boundary != 'blocked_poll':
                            assert connection.recv() == 'continue'
                if boundary == 'before_wait':
                    real_wait = wait.wait_until_monotonic_us
                    def waiting(deadline):
                        barrier()
                        real_wait(deadline)
                    wait.wait_until_monotonic_us = waiting
                else:
                    def polling(*args):
                        barrier()
                        return real_poll(*args)
                    wait_module.select.select = polling
                assert app.run(wait=wait) == 0, app.runtime.failure
            else:
                def stop_during_transfer(_command):
                    io.after_transfer = None
                    # This is the actual radio Event lock the old handler set.
                    with app.radio._stop._cond:
                        connection.send('ready')
                        assert connection.recv() == 'continue'
                        assert app.stop_intent.is_requested()
                io.after_transfer = stop_during_transfer
                result = app.radio.rearm()
                assert result.state is E.RadioState.SHUTDOWN and result.safe_shutdown
            deadline = app.stop_deadline
            assert deadline == app.stop_intent.requested_at_monotonic_us + app.settings.shutdown_budget_us
            # A second actual signal during owner cleanup cannot reset its bound.
            original_shutdown = app.radio.shutdown
            def shutdown(**kwargs):
                os.kill(os.getpid(), signal.SIGINT)
                assert app.stop_deadline == deadline
                return original_shutdown(**kwargs)
            app.radio.shutdown = shutdown
            stopped = app.shutdown(clean_requested=True)
            assert stopped.radio_safe and stopped.queue_drained and stopped.worker_stopped
            assert stopped.clean_stop_confirmed and stopped.failure is None
            assert app.stop_deadline == deadline and clock.now_monotonic_us() < deadline
            assert not any(command[0] == 0x83 for command in io.commands)
        with sqlite3.connect(database) as db:
            marked = db.execute('SELECT clean_stopped_at_monotonic_us FROM receiver_instances').fetchone()[0]
        assert marked is not None
        connection.send('clean')
    except BaseException:
        connection.send(('error', traceback.format_exc()))
        raise
    finally:
        wait_module.select.select = real_poll
        app.worker.finish_test()
        connection.close()


@pytest.mark.parametrize('boundary', ['before_wait', 'before_poll', 'blocked_poll', 'radio_lock'])
def test_real_sigterm_wakes_owner_and_preserves_clean_stop_deadline(tmp_path, boundary):
    context = multiprocessing.get_context('spawn')
    parent, child = context.Pipe()
    process = context.Process(target=signal_application, args=(child, str(tmp_path), boundary))
    process.start()
    child.close()
    try:
        assert parent.poll(10), 'application did not reach the named signal boundary'
        assert parent.recv() == 'ready'
        if boundary == 'blocked_poll':
            # Observe the main thread in its kernel poll before delivery; the
            # earlier before_poll case separately covers the submission race.
            limit = time.monotonic() + 5
            while 'poll_schedule_timeout' not in Path(f'/proc/{process.pid}/wchan').read_text():
                assert time.monotonic() < limit, 'owner did not enter the kernel poll'
                assert not parent.poll(0.005), 'child exited before blocking poll'
        os.kill(process.pid, signal.SIGTERM)
        if boundary != 'blocked_poll':
            parent.send('continue')
        assert parent.poll(10), f'SIGTERM deadlocked at {boundary}'
        assert parent.recv() == 'clean'
        process.join(10)
        assert process.exitcode == 0
    finally:
        if process.is_alive():
            process.kill()
            process.join(5)
        parent.close()
