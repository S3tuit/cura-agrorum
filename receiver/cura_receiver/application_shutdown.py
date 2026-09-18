"""Bounded controlled-stop orchestration over the existing production owners."""

from dataclasses import dataclass
from threading import Event

from .persist_queue import PersistQueueInterfaceError
from .persistence_control_values import ReceiverCleanStopV1, ReceiverCleanStopCommitDisposition as D
from .persistence_control_values import CommunicatorStateCondition


@dataclass(frozen=True, slots=True)
class ApplicationStopResult:
    radio_safe: bool
    queue_drained: bool
    authoritative_generation: int | None
    clean_stop_confirmed: bool
    worker_stopped: bool
    failure: str | None = None


def shutdown_application(app, *, clean_requested, wait=None):
    app.request_stop()
    clock, worker, runtime = app.clock, app.worker, app.runtime
    deadline = app.stop_deadline
    waiter = Event()
    wait = wait or (lambda until: waiter.wait(max(0, until - clock.now_monotonic_us()) / 1_000_000))
    def remaining():
        return clock.now_monotonic_us() < deadline
    def pause():
        wait(min(deadline, clock.now_monotonic_us() + 50_000))
    radio_safe = False
    failure = None
    result = None
    try:
        if app._radio_started:
            result = app.radio.shutdown(deadline_monotonic_us=deadline)
            radio_safe = result.safe_shutdown is True
        if runtime is not None and runtime.failure is not None and not runtime.failure.queue_sound:
            # Ownership is uncertain: no snapshot, close, drain or control call
            # through this queue. The daemon disk owner ends with process exit.
            return ApplicationStopResult(radio_safe, False, None, False, not worker.is_alive(), 'QUEUE_UNSOUND')
        if runtime is not None:
            c = runtime.communicator
            cancelled = c.time.cancel_rtc_refresh()
            if cancelled is not None and cancelled.failure is not None:
                runtime._pending.append((runtime.telemetry.time, cancelled.failure))
            if c._active is None:
                c._active = c.ingress.active_occurrence
            if c._active is not None or c._failed_receive is not None:
                c._record(result)
                if c._active is not None:
                    c._finish()
                else:
                    c._finish_failed_receive()
            if result is not None:
                runtime._pending.extend((runtime.telemetry.radio, item) for item in result.episodes)
            runtime.flush_diagnostics()
            if remaining():
                c.airtime.settle(deadline_monotonic_us=app._deadline(c.time.settings.control_budget_us), precharge=False)
                while c.airtime.owner.pending is not None and remaining() and worker.is_alive():
                    pause()
                    if remaining():
                        c.airtime.reconcile(deadline_monotonic_us=app._deadline(c.time.settings.control_budget_us))
            runtime._collect_control(terminal=True)
            runtime.flush_diagnostics()
    except Exception as error:
        if isinstance(error, PersistQueueInterfaceError) and not error.queue_known_sound:
            return ApplicationStopResult(radio_safe, False, None, False, not worker.is_alive(), 'QUEUE_UNSOUND')
        failure = 'SHUTDOWN_FINALIZATION_FAILED'
        clean_requested = False
    worker.queue.close()
    if runtime is not None:
        runtime._pending.clear()
    while remaining() and worker.is_alive() and not worker.queue.snapshot().closed_and_drained:
        pause()
    drained = worker.queue.snapshot().closed_and_drained
    generation = None
    if runtime is not None:
        owner = runtime.communicator.airtime.owner
        if owner.pending is None:
            if owner.state is not None:
                generation = owner.state.generation
            elif owner.condition is not CommunicatorStateCondition.NONE:
                generation = 0
    confirmed = False
    try:
        if clean_requested and radio_safe and drained and generation is not None and remaining() and worker.is_alive():
            marker = ReceiverCleanStopV1(app.instance.receiver_instance_id, clock.now_monotonic_us(), generation)
            while remaining() and worker.is_alive():
                outcome = worker.control.commit_receiver_clean_stop(marker,
                    deadline_monotonic_us=app._deadline(app.settings.time_settings.control_budget_us))
                if outcome.disposition in (D.COMMITTED, D.ALREADY_COMMITTED):
                    confirmed = True
                    break
                if outcome.disposition is not D.OUTCOME_UNKNOWN:
                    failure = 'CLEAN_STOP_NOT_COMMITTED'
                    break
                failure = 'CLEAN_STOP_UNRESOLVED'
                pause()
            if confirmed:
                failure = None
    except Exception:
        failure = 'CLEAN_STOP_COMMIT_FAILED'
    worker.request_stop(deadline_monotonic_us=deadline)
    if worker.ident is not None:
        worker.join(max(0, deadline - clock.now_monotonic_us()) / 1_000_000)
    if failure is None:
        if not radio_safe and app._radio_started:
            failure = 'RADIO_UNSAFE'
        elif not drained:
            failure = 'QUEUE_NOT_DRAINED'
        elif worker.is_alive():
            failure = 'WORKER_NOT_STOPPED'
        elif clean_requested and not confirmed:
            failure = 'CLEAN_STOP_NOT_CONFIRMED'
    return ApplicationStopResult(radio_safe, drained, generation, confirmed, not worker.is_alive(), failure)
