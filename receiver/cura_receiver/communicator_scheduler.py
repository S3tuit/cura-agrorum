"""Bounded safe-boundary scheduling for the single communicator owner."""

from dataclasses import dataclass
from enum import Enum, auto

from .elapsed_duration import checked_monotonic_deadline, minimum_wait_monotonic_us
from .generated import receiver_enums_generated as E
from .runtime_time import ChronyStepState
from .time_observations import rtc_refresh_due_us


class Work(Enum):
    RADIO = auto()
    TIME = auto()
    AIRTIME = auto()
    RTC = auto()
    HEALTH = auto()
    WAIT = auto()
    TERMINAL = auto()


@dataclass(frozen=True, slots=True)
class ScheduledTurn:
    work: Work
    exchange: object | None = None
    update: object | None = None
    wait_until_monotonic_us: int | None = None


_RECEIVING = (E.RadioState.RX_SINGLE, E.RadioState.RX_EVENT_PENDING)
_TERMINAL = (E.RadioState.SHUTDOWN, E.RadioState.INITIALIZATION_FAILED,
             E.RadioState.RECOVERY_EXHAUSTED, E.RadioState.HARDWARE_MISSING)


class CommunicatorScheduler:
    """Radio work precedes housekeeping; no control call belongs to an exchange.

    One turn performs at most one background action. WAIT gives the application
    an absolute monotonic bound for an interruptible stop-aware wait.
    """

    def __init__(self, communicator, *, chrony, rtc, health_interval_us, initial_health_pending=False, stop_requested=lambda: False, shutdown_deadline=lambda: None):
        self.communicator = communicator
        self.time = communicator.time
        self.clock = communicator.clock
        self.chrony, self.rtc = chrony, rtc
        self.health_interval_us = health_interval_us
        self.initial_health_pending = initial_health_pending
        self.stop_requested, self.shutdown_deadline = stop_requested, shutdown_deadline
        now = self.clock.now_monotonic_us()
        self.next_health = self.next_airtime = self.boundary_retry = now
        self.failure_location = (E.CorePhase.IDLE, E.CoreFailureStage.INVOKE_ADAPTER, E.DiagnosticOperation.RECEIVE)
        self.last_exchange = None

    def _later(self, duration):
        return checked_monotonic_deadline(self.clock.now_monotonic_us(),
            minimum_wait_monotonic_us(duration, rate_bound_ppm=self.time.policy.monotonic_elapsed_rate_bound_ppm))

    def _rtc_due(self):
        if self.time.rtc_refresh_episode is not None:
            return self.clock.now_monotonic_us()
        if self.time.state.quality is not E.SystemTimeQuality.NETWORK_SYNCED:
            return self.time.next_rtc_read_start()
        if self.time.sample is None:
            return None
        due = rtc_refresh_due_us(self.time.sample,
            last_refresh_monotonic_us=self.time.last_refresh_monotonic_us, policy=self.time.policy)
        return None if due is None else max(due, self.time.next_rtc_attempt_monotonic_us)

    def _call(self, phase, operation, function, *args, **kwargs):
        self.failure_location = (phase, E.CoreFailureStage.INVOKE_ADAPTER, operation)
        return function(*args, **kwargs)

    def _poll_time(self):
        operation = (E.DiagnosticOperation.SYNC if self.time.step_state is ChronyStepState.STEP_COMMAND_PENDING
                     else E.DiagnosticOperation.READ)
        return self._call(E.CorePhase.PERIODIC_TIME, operation, self.time.poll_chrony, self.chrony)

    def _receive(self, deadline):
        result = self._call(E.CorePhase.PACKET_PROCESSING, E.DiagnosticOperation.RECEIVE,
                            self.communicator.receive_once, deadline_monotonic_us=deadline)
        self.last_exchange = result
        return result

    def run_once(self):
        self.last_exchange = None
        c, t = self.communicator, self.time
        now = self.clock.now_monotonic_us()
        if self.stop_requested():
            return ScheduledTurn(Work.TERMINAL, update=t.cancel_rtc_refresh())
        if c.radio.state in _TERMINAL:
            return ScheduledTurn(Work.TERMINAL)
        if t.ordinary_admission_blocked and now < self.boundary_retry:
            return ScheduledTurn(Work.WAIT, wait_until_monotonic_us=self.boundary_retry)
        previous_sequence = c.occurrence_sequence
        exchange = self._receive(now)
        if self.stop_requested():
            return ScheduledTurn(Work.TERMINAL, exchange, t.cancel_rtc_refresh())
        if c.radio.state in _TERMINAL:
            return ScheduledTurn(Work.TERMINAL, exchange)
        if (c.occurrence_sequence != previous_sequence or exchange.finalization is not None
                or exchange.radio_episodes or c.radio.state not in _RECEIVING):
            return ScheduledTurn(Work.RADIO, exchange)
        now = self.clock.now_monotonic_us()
        if t.ordinary_admission_blocked:
            # A changed RTC health under network time needs a fresh observation;
            # a retained observation itself must be retried before any new work.
            update = None
            if t.pending_observation is None:
                update = self._poll_time()
            self.boundary_retry = self._later(t.settings.retry_initial_us)
            return ScheduledTurn(Work.TIME, exchange, update, self.boundary_retry)
        if self.stop_requested():
            return ScheduledTurn(Work.TERMINAL, exchange, t.cancel_rtc_refresh())
        if self.initial_health_pending:
            self.initial_health_pending = False
            self.next_health = self._later(self.health_interval_us)
            return ScheduledTurn(Work.HEALTH, exchange)
        if t.step_state is ChronyStepState.STEP_COMMAND_PENDING or now >= t.next_tracking_start():
            return ScheduledTurn(Work.TIME, exchange, self._poll_time())
        if now >= self.next_airtime:
            self.failure_location = (E.CorePhase.AIRTIME_STATE, E.CoreFailureStage.COMMIT_STATE, E.DiagnosticOperation.WRITE)
            c.airtime.update_time(t.airtime_correlation(), rtc_health=t.state.rtc_health)
            update = c.airtime.acquire_grant(deadline_monotonic_us=t.deadline(t.settings.control_budget_us))
            self.next_airtime = self._later(t.settings.retry_initial_us)
            return ScheduledTurn(Work.AIRTIME, exchange, update)
        rtc_due = self._rtc_due()
        if rtc_due is not None and now >= rtc_due:
            self.failure_location = (E.CorePhase.PERIODIC_TIME, E.CoreFailureStage.INVOKE_ADAPTER,
                E.DiagnosticOperation.SYNC if t.state.quality is E.SystemTimeQuality.NETWORK_SYNCED else E.DiagnosticOperation.READ)
            if t.rtc_refresh_episode is not None:
                self.failure_location = (E.CorePhase.PERIODIC_TIME, E.CoreFailureStage.INVOKE_ADAPTER,
                    t.rtc_refresh_episode.operation or E.DiagnosticOperation.SYNC)
            update = (t.advance_rtc_refresh(self.rtc, c.airtime.snapshot,
                        stop_requested=self.stop_requested(), shutdown_deadline=self.shutdown_deadline())
                      if t.rtc_refresh_episode is not None or t.state.quality is E.SystemTimeQuality.NETWORK_SYNCED
                      else t.observe_rtc(self.rtc))
            return ScheduledTurn(Work.RTC, exchange, update)
        if now >= self.next_health:
            self.next_health = self._later(self.health_interval_us)
            return ScheduledTurn(Work.HEALTH, exchange)
        deadlines = [self.next_airtime, self.next_health, t.next_tracking_start()]
        if rtc_due is not None:
            deadlines.append(rtc_due)
        # Radio owns its bounded DIO1 wait; caller regains control within 500 ms.
        deadline = min(deadlines)
        exchange = self._receive(deadline)
        return ScheduledTurn(Work.RADIO, exchange)
