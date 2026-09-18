"""Installed receiver entry point. No provisioning or database initialization."""

import argparse
import os
import signal

from .application import ReceiverApplication
from .application_settings import ApplicationSettings
from .persistence_worker import PersistenceWorker
from .platform.linux_chrony import LinuxChronyControl
from .platform.linux_clocks import LinuxOsClock
from .platform.linux_ds3231 import LinuxDs3231Control
from .platform.linux_kernel_clock import LinuxKernelClock
from .platform.linux_radio import LinuxRadioIo
from .radio import Radio
from .receiver_startup import create_receiver_instance
from .sx1262 import Sx1262


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rtc-helper-sha256', required=True)
    parser.add_argument('--rtc-kernel-bound-us', type=int, required=True)
    args = parser.parse_args()
    settings = ApplicationSettings()
    clock = LinuxOsClock()
    instance = create_receiver_instance(clock)
    rtc = LinuxDs3231Control(clock, kernel_operation_bound_us=args.rtc_kernel_bound_us,
        helper_sha256=args.rtc_helper_sha256, receiver_gid=os.getegid())
    chrony = LinuxChronyControl(clock, socket_path='/run/chrony/chronyd.sock',
        deadline_monotonic_us=clock.now_monotonic_us() + settings.time_settings.command_budget_us)
    worker = PersistenceWorker(instance=instance, database_path=settings.database_path,
        configuration_path=settings.configuration_path, expected_owner_uid=os.geteuid(), clock=clock,
        policy=settings.airtime_policy, minimum_free_bytes=settings.minimum_free_bytes)
    radio = Radio(Sx1262(LinuxRadioIo(clock), clock, clock, settings.radio))
    app = ReceiverApplication(instance=instance, settings=settings, worker=worker, clock=clock,
        kernel=LinuxKernelClock(clock), rtc=rtc, chrony=chrony, radio=radio)
    for number in (signal.SIGTERM, signal.SIGINT):
        signal.signal(number, lambda *_: app.request_stop())
    result = 1
    try:
        startup = app.start()
        if startup.ready:
            result = app.run()
        else:
            print('receiver startup:', startup.failure, flush=True)
    except Exception:
        # Fixed bounded service evidence never includes configuration/key objects.
        print('receiver application failed', flush=True)
        if app.runtime is not None:
            import sys
            app.runtime._terminate(sys.exception())
    finally:
        stopped = app.shutdown(clean_requested=result == 0 and app.stop_event.is_set())
        if stopped.failure is not None:
            print('receiver shutdown:', stopped.failure, flush=True)
            result = 1
    return result


if __name__ == '__main__':
    raise SystemExit(main())
