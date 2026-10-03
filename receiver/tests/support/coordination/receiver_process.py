"""Named process barriers around the guarded entry point and real SQLite owners."""

import os
from pathlib import Path
import sys
from threading import Event
import traceback

from cura_receiver import __main__ as entry
from cura_receiver.airtime_ledger import AirtimeCorrelation
from cura_receiver.generated import receiver_enums_generated as E
from cura_receiver.ports.ds3231 import Ds3231ReadResult, Ds3231ReadStatus
from cura_receiver.time_observations import TrustedTimeSample
from cura_receiver.tx_airtime import AirtimeReason, TxCertainty
from tests.support.coordination.persistence_worker import CheckedPersistenceWorker
from tests.support.fakes.chrony import FakeChronyControl
from tests.support.fakes.ds3231 import FakeDs3231Control
from tests.support.fakes.kernel_clock import FakeKernelClock
from tests.support.fakes.os_clock import FakeOsClock
from tests.support.fakes.radio_io import PhysicalPort, Wait


def run_receiver(pipe, directory, boundary, *, trusted=False, spend=False):
    """Child-only platform substitution; process guard and entry point stay real."""
    root = Path(directory)
    sys.argv = ['receiver', '--rtc-helper-sha256', '0' * 64,
                '--rtc-kernel-bound-us', '3000000']
    os.environ.update({
        'CURA_RECEIVER_CONFIGURATION': str(root / 'test-group.json'),
        'CURA_RECEIVER_DATABASE': str(root / 'worker.db'),
        'SQLITE_TMPDIR': str(root / 'sqlite-temp'),
        'CURA_RECEIVER_TEST_ROOT': str(root),
    })
    clock = FakeOsClock(monotonic_us=100)
    applications = []

    def arrive(name, **evidence):
        if name == boundary:
            pipe.send({'boundary': name, **evidence})
            assert pipe.recv() == 'continue'

    def runtime_clock():
        arrive('before_runtime')
        if boundary == 'construction_failed':
            raise RuntimeError('injected construction failure')
        return clock

    def worker(**kwargs):
        kwargs.update(boot_id_path=root / 'boot-id', minimum_free_bytes=0)
        return CheckedPersistenceWorker(**kwargs)

    rtc = FakeDs3231Control()
    rtc.read_results.append(Ds3231ReadResult(Ds3231ReadStatus.MISSING, 100, 100))
    real_sx1262 = entry.Sx1262

    class Application(entry.ReceiverApplication):
        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            applications.append(self)

        def start(self):
            result = super().start()
            arrive('after_start', ready=result.ready)
            return result

        def run(self, *, wait):
            policy = self.runtime.communicator.airtime
            now = clock.now_monotonic_us()
            if trusted:
                policy.update_time(AirtimeCorrelation(
                    TrustedTimeSample(now, now, 1, E.SystemTimeQuality.NETWORK_SYNCED, 1),
                    1, now + 10_000_000_000), rtc_health=E.RtcHealth.MISSING)
            assert policy.recover(deadline_monotonic_us=now + 5_000_000).reason is AirtimeReason.STATE_READY
            initial_used = policy.total_used
            if spend:
                for _ in range(28):
                    result = policy.try_spend()
                    assert result.reason is AirtimeReason.ALLOWED
                    policy.report_tx(result.token, TxCertainty.STARTED)
                    clock.advance_elapsed_us(100_000)
                assert policy.used_since_save_us == 1_900_248 and not policy.save_required
            arrive('running', generation=policy.state.generation, initial_used=initial_used,
                   unsaved=policy.used_since_save_us, available=policy.available_charge_us)
            self.request_stop()
            return 0

        def shutdown(self, **kwargs):
            arrive('shutdown')
            # Simulate a disk primitive outliving the application's stop budget.
            # The real daemon worker, shutdown orchestration and final join run.
            if boundary == 'returned_with_worker':
                entered = Event()
                blocked = Event()
                def stalled_checkpoint(_db):
                    entered.set()
                    blocked.wait()
                self.worker._transactions.checkpoint = stalled_checkpoint
                real_join = self.worker.join
                def expired_join(_timeout):
                    assert entered.wait(5), 'worker did not reach final checkpoint'
                    clock.advance_elapsed_us(self.settings.shutdown_budget_us)
                    real_join(0)
                self.worker.join = expired_join
            def advancing_wait(until):
                Event().wait(0.001)
                clock.advance_elapsed_us(max(0, until - clock.now_monotonic_us()))
            return super().shutdown(**kwargs, wait=advancing_wait)

    entry.LinuxOsClock = runtime_clock
    entry.LinuxDs3231Control = lambda *args, **kwargs: rtc
    entry.LinuxChronyControl = lambda *args, **kwargs: FakeChronyControl()
    entry.LinuxKernelClock = lambda _clock: FakeKernelClock()
    entry.LinuxRadioIo = PhysicalPort
    entry.Sx1262 = lambda io, clock, wait, configuration: real_sx1262(io, clock, Wait(clock), configuration)
    entry.PersistenceWorker = worker
    entry.ReceiverApplication = Application
    try:
        try:
            code = entry.main()
        except RuntimeError:
            if boundary != 'construction_failed':
                raise
            arrive('construction_failed')
            return
        if boundary == 'returned_with_worker':
            assert code == 1
            assert applications[0].worker.is_alive()
            assert applications[0].shutdown_result.failure == 'WORKER_NOT_STOPPED'
        arrive(boundary if boundary in ('returned', 'returned_with_worker') else 'finished',
               code=code)
    except BaseException:
        pipe.send({'error': traceback.format_exc()})
        raise
    finally:
        pipe.close()
