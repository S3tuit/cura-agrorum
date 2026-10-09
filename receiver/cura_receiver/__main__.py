"""Installed receiver entry point. No provisioning or database initialization."""

import argparse
import os
import re

from .application import ReceiverApplication
from .application_environment import settings_from_environment
from .persistence_worker import PersistenceWorker
from .platform.linux_chrony import LinuxChronyControl
from .platform.linux_clocks import LinuxOsClock
from .platform.linux_ds3231 import LinuxDs3231Control
from .platform.linux_kernel_clock import LinuxKernelClock
from .platform.linux_process_ownership import claim_receiver_process, ReceiverAlreadyRunning
from .platform.linux_radio import LinuxRadioIo
from .platform.linux_signal_wait import CompletionNotification, LinuxSignalWait
from .radio import Radio
from .receiver_startup import create_receiver_instance
from .sx1262 import Sx1262
from .stop_intent import StopIntent
from .service_evidence import emit_service_evidence
from . import radio_investigation


class ServicePersistenceWorker(PersistenceWorker):
    """Keep unexpected thread exit off Python's blocking traceback stream."""

    def run(self):
        try:
            super().run()
        except Exception:
            emit_service_evidence(dict(format_version=1, event='receiver_persistence',
                outcome='WORKER_FAILED', receiver_instance_id=self._instance.receiver_instance_id.hex()))


def helper_digest(value):
    if re.fullmatch(r'[0-9a-f]{64}', value) is None:
        raise argparse.ArgumentTypeError('helper digest must be 64 lowercase hexadecimal characters')
    return bytes.fromhex(value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rtc-helper-sha256', type=helper_digest, required=True)
    parser.add_argument('--rtc-kernel-bound-us', type=int, required=True)
    args = parser.parse_args()
    # Only fixed classifications and immutable startup evidence reach the sink.
    # One startup emission is attempted even if composition fails before launch.
    startup_record = dict(format_version=1, event='receiver_startup',
                          receiver_instance_id=None, outcome='APPLICATION_SETUP_FAILED')
    startup_emitted = False
    app = None
    result = 1
    try:
        try:
            settings = settings_from_environment(os.environ)
        except ValueError:
            startup_record['outcome'] = 'DEPLOYMENT_CONFIGURATION_REJECTED'
            return 2
        try:
            claim_receiver_process()
        except ReceiverAlreadyRunning:
            startup_record['outcome'] = 'RECEIVER_ALREADY_RUNNING'
            return 1
        except OSError:
            startup_record['outcome'] = 'PROCESS_OWNERSHIP_FAILED'
            return 1
        os.environ['SQLITE_TMPDIR'] = str(settings.sqlite_temporary_directory)
        clock = LinuxOsClock()
        instance = create_receiver_instance(clock)
        startup_record['receiver_instance_id'] = instance.receiver_instance_id.hex()
        stop = StopIntent(clock, settings.shutdown_budget_us)
        with LinuxSignalWait(clock, stop) as wait, CompletionNotification() as notification:
            rtc = LinuxDs3231Control(clock, kernel_operation_bound_us=args.rtc_kernel_bound_us,
                helper_sha256=args.rtc_helper_sha256, receiver_gid=os.getegid())
            chrony = LinuxChronyControl(clock, socket_path='/run/chrony/chronyd.sock',
                deadline_monotonic_us=clock.now_monotonic_us() + settings.time_settings.command_budget_us)
            worker = ServicePersistenceWorker(instance=instance, database_path=settings.database_path,
                configuration_path=settings.configuration_path, expected_owner_uid=os.geteuid(), clock=clock,
                policy=settings.airtime_policy, minimum_free_bytes=settings.minimum_free_bytes,
                startup_notification=notification)
            backend = Sx1262(LinuxRadioIo(clock), clock, clock, settings.radio)
            radio_investigation.configure(backend, instance.receiver_instance_id,
                                          os.environ.get("CURA_RADIO_INVESTIGATION_DIR"))
            radio = Radio(backend, stop_requested=stop.is_requested)
            app = ReceiverApplication(instance=instance, settings=settings, worker=worker, clock=clock,
                kernel=LinuxKernelClock(clock), rtc=rtc, chrony=chrony, radio=radio, stop_intent=stop)
            try:
                try:
                    startup = app.start(wait=wait)
                finally:
                    if app.startup_decision is not None:
                        startup_record = app.startup_decision.summary(instance.receiver_instance_id)
                        startup_record['event'] = 'receiver_startup'
                    else:
                        startup_record['outcome'] = 'APPLICATION_STARTUP_FAILED'
                    started = app.start_result
                    startup_record['application_outcome'] = (
                        'APPLICATION_STARTUP_FAILED' if started is None else
                        'SUCCESS' if started.ready else started.failure)
                    startup_emitted = True
                    emit_service_evidence(startup_record)
                if startup.ready:
                    result = app.run(wait=wait)
            except Exception as error:
                if app.runtime is not None:
                    app.runtime._terminate(error)
                emit_service_evidence(dict(format_version=1, event='receiver_application',
                    outcome='APPLICATION_FAILED', receiver_instance_id=instance.receiver_instance_id.hex()))
            finally:
                stopped = app.shutdown(clean_requested=result == 0 and stop.is_requested())
                if stopped.failure is not None:
                    emit_service_evidence(dict(format_version=1, event='receiver_shutdown',
                        outcome=stopped.failure, receiver_instance_id=instance.receiver_instance_id.hex(),
                        worker_stopped=stopped.worker_stopped,
                        clean_stop_confirmed=stopped.clean_stop_confirmed))
                    result = 1
        return result
    except Exception:
        # Includes construction, notification setup and exceptional shutdown.
        # Do not let a traceback or buffered print turn failure into a hang.
        if startup_emitted:
            emit_service_evidence(dict(format_version=1, event='receiver_application',
                                      outcome='APPLICATION_TEARDOWN_FAILED'))
        return 1
    finally:
        radio_investigation.close()
        if not startup_emitted:
            emit_service_evidence(startup_record)


if __name__ == '__main__':
    raise SystemExit(main())
