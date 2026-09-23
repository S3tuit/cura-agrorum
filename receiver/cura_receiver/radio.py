"""Synchronous single-owner radio component; caller owns protocol and airtime."""

from dataclasses import dataclass, replace
from functools import wraps
import threading

from .generated import receiver_enums_generated as E
from .ports.radio import (BusyMetrics, Error, Outcome, RadioBackendError, RadioFailure, RadioLifecycleError,
                         RadioTxAuthorization, RadioTxFacts, Stage, integer)
from .radio_diagnostics import Operation, RadioEpisodeBuilder, Reason, State
from .sx1262 import IRQ_CRC_ERROR, IRQ_HEADER_ERROR, IRQ_RX_DONE, IRQ_TIMEOUT, IRQ_TX_DONE
from .elapsed_duration import checked_monotonic_deadline, maximum_lifetime_monotonic_us

_TERMINAL = (State.SHUTDOWN, State.INITIALIZATION_FAILED, State.RECOVERY_EXHAUSTED, State.HARDWARE_MISSING)


def _failures(error):
    if isinstance(error, RadioLifecycleError):
        result = error.lifecycle
        return ((result.primary_failure,) if result.primary_failure is not None else ()) + result.release_failures
    return (error.failure,)


class _ShutdownRequested(Exception):
    pass


def _operation(*states):
    def decorate(method):
        @wraps(method)
        def run(self, *args, **kwargs):
            self._claim()
            if self.state in _TERMINAL:
                raise RuntimeError("terminal radio cannot operate")
            if self._stop.is_set():
                return self.shutdown()
            if self.state not in states:
                return self._invalid(Error.INVALID_STATE)
            try:
                return method(self, *args, **kwargs)
            except _ShutdownRequested:
                return self.shutdown()
        return run
    return decorate


@dataclass(frozen=True, slots=True)
class RadioReceiveEvent:
    edge_timestamp_ns: int
    t1_handler_started_monotonic_us: int
    usable_for_ingress: bool = False
    frame: bytes | None = None
    rssi_dbm_x2: int | None = None
    snr_db_x4: int | None = None
    irq_status: int | None = None
    device_errors: int | None = None
    t2_packet_copied_monotonic_us: int | None = None
    busy: BusyMetrics = BusyMetrics()

    @property
    def received_at_monotonic_us(self):
        return self.edge_timestamp_ns // 1000


@dataclass(frozen=True, slots=True)
class RadioCounters:
    recovery_attempts: int = 0
    recovery_successes: int = 0
    recovery_failures: int = 0
    recovery_attempts_by_reason: tuple = (0,) * 8
    header_errors: int = 0
    crc_errors: int = 0


@dataclass(frozen=True, slots=True)
class RadioTxResult:
    ack_tx_result: E.AckTxResult
    facts: RadioTxFacts
    t4_set_tx_attempted_monotonic_us: int | None = None
    t5_tx_done_monotonic_us: int | None = None


@dataclass(frozen=True, slots=True)
class RadioResult:
    state: State
    receive_event: RadioReceiveEvent | None = None
    episodes: tuple = ()
    t6_set_rx_issued_monotonic_us: int | None = None
    busy: BusyMetrics = BusyMetrics()
    tx: RadioTxResult | None = None
    safe_shutdown: bool | None = None


class Radio:
    def __init__(self, backend):
        self.backend = backend
        self.clock = backend.clock
        self._state = State.INITIALIZING
        self._owner = None
        self._edge = None
        self._episode = None
        self._t6 = None
        self._opened = False
        self._counters = RadioCounters()
        self._can_ack = False
        self._prepared = None
        self._authorization = None
        self._sequence = None
        self._tx = None
        self._tx_deadline = None
        self._completed = ()
        self._restoring_set_rx = False
        self._stop = threading.Event()
        self._cleaning = False
        self._safe_shutdown = None
        self._installing_tx = False
        backend.checkpoint = self._checkpoint

    @property
    def state(self):
        return self._state

    @property
    def counters(self):
        return self._counters

    def _claim(self):
        current = threading.get_ident()
        if self._owner is None:
            self._owner = current
        elif current != self._owner:
            raise RuntimeError("radio state belongs to another thread")

    def _require(self, *states):
        self._claim()
        if self.state not in states:
            raise RuntimeError("invalid radio operation state")

    def _result(self, *, receive_event=None, episodes=()):
        result = RadioResult(self.state, receive_event, self._completed + episodes, self._t6, self.backend.metrics, self._tx, self._safe_shutdown)
        self._completed = ()
        return result

    def _bump(self, name):
        value = integer(getattr(self._counters, name) + 1)
        self._counters = replace(self._counters, **{name: value})

    def _builder(self, operation, failure):
        return RadioEpisodeBuilder(operation, failure, self.state, self.clock.now_monotonic_us(),
            sequence=self._sequence, bucket=self._authorization.airtime_bucket_expiration_utc_us if self._authorization else None)

    def _reason(self, failure, *, tx_uncertain=False, restoring=False):
        if failure.hardware_missing:
            return Reason.HARDWARE_UNREACHABLE
        if tx_uncertain:
            return Reason.TX_OUTCOME_UNCERTAIN
        if restoring:
            return Reason.SET_RX_FAILED if self._restoring_set_rx else Reason.RX_PROFILE_RESTORE_FAILED
        if failure.code is Error.BUSY_TIMEOUT:
            return Reason.BUSY_TIMEOUT
        if failure.code is Error.IO:
            return Reason.SPI_FAILURE
        if failure.code is Error.UNEXPECTED_IRQ:
            return Reason.UNEXPECTED_IRQ
        return Reason.STATUS_UNCONFIRMED

    def _recovering(self, operation, failure, *, episode=None, restoring=False, tx_uncertain=False, receive_event=None):
        self._episode = episode or self._builder(operation, failure)
        reason = self._reason(failure, restoring=restoring, tx_uncertain=tx_uncertain)
        self._episode.enter_recovery(reason)
        self._state = State.RECOVERING
        self._edge = None
        self._bump("recovery_attempts")
        reasons = list(self._counters.recovery_attempts_by_reason)
        reasons[reason.value - 1] = integer(reasons[reason.value - 1] + 1)
        self._counters = replace(self._counters, recovery_attempts_by_reason=tuple(reasons))
        return self._result(receive_event=receive_event)

    @_operation(State.INITIALIZING)
    def initialize(self):
        self._require(State.INITIALIZING)
        deadline = self.backend.deadline(2_000_000)
        try:
            self.backend.open(deadline)
            self._opened = True
            self.backend.initialize(deadline)
            self.backend.arm_receive(deadline)
            self._t6 = self.backend.last_set_rx_issued_us
            self._state = State.RX_SINGLE
            return self._result()
        except RadioBackendError as error:
            episode = self._builder(Operation.INITIALIZE, error.failure)
            terminal = State.HARDWARE_MISSING if error.failure.hardware_missing else State.INITIALIZATION_FAILED
            cleanup = [self._builder(Operation.CLEANUP, failure) for failure in _failures(error)[1:]]
            if cleanup:
                self._safe_shutdown = False
            self._cleaning = True
            try:
                if self._opened and not error.failure.hardware_missing:
                    self._safe_shutdown = False
                    try:
                        self.backend.safe_standby(deadline)
                        self._safe_shutdown = True
                    except RadioBackendError as failure:
                        cleanup.append(self._builder(Operation.CLEANUP, failure.failure))
            finally:
                try:
                    self.backend.close()
                    if self.clock.now_monotonic_us() > deadline:
                        raise RadioBackendError(RadioFailure(Error.DEADLINE, Stage.DETACH_IRQ, hardware_touched=True))
                except RadioBackendError as failure:
                    self._safe_shutdown = False
                    cleanup.extend(self._builder(Operation.CLEANUP, item) for item in _failures(failure))
                except BaseException:
                    self._safe_shutdown = False
                    raise
                finally:
                    self._opened = False
                    self._state = terminal
                    self._cleaning = False
            finished = self.clock.now_monotonic_us()
            return self._result(episodes=tuple(
                item.finish(terminal, finished) for item in (episode, *cleanup)
            ))

    def _restore(self, deadline):
        self._t6 = self.backend.last_set_rx_issued_us = None
        self._restoring_set_rx = False
        try:
            self.backend.install_profile(transmit=False, payload_length=255, deadline=deadline)
            self._restoring_set_rx = True
            self.backend.arm_receive(deadline)
        finally:
            self._t6 = self.backend.last_set_rx_issued_us
        self._state = State.RX_SINGLE

    @_operation(State.RX_EVENT_PENDING)
    def rearm(self):
        self._require(State.RX_EVENT_PENDING)
        self._can_ack = False
        self._prepared = None
        try:
            self._restore(self.backend.deadline(500_000))
        except RadioBackendError as error:
            return self._recovering(Operation.RECEIVE, error.failure, restoring=True)
        return self._result()

    @_operation(State.RX_SINGLE, State.RX_EVENT_PENDING)
    def receive(self, *, deadline_monotonic_us):
        self._require(State.RX_SINGLE, State.RX_EVENT_PENDING)
        if self.state is State.RX_EVENT_PENDING and self._can_ack:
            return self._invalid(Error.INVALID_STATE)
        receive_event = None
        try:
            integer(deadline_monotonic_us)
        except (TypeError, ValueError):
            return self._invalid(Error.INVALID_ARGUMENT)
        try:
            if self._edge is None:
                self._edge = self.backend.wait_edge(deadline_monotonic_us=self.backend.deadline(500_000, deadline_monotonic_us))
                self._checkpoint()
            if self._edge is None:
                return self._result()
            self._state = State.RX_EVENT_PENDING
            edge, self._edge = self._edge, None
            self._t6 = None
            self._tx = self._prepared = self._authorization = self._sequence = None
            self.backend.last_set_tx_issued_us = None
            self.backend.set_tx_outcome = Outcome.NOT_APPLICABLE
            self._can_ack = False
            self.backend.reset_metrics()
            started = self.clock.now_monotonic_us()
            if edge.timestamp_ns > started * 1000 + 999 or edge.monotonic_us < self.backend.last_set_rx_issued_us:
                raise RadioBackendError(RadioFailure(Error.MALFORMED_RESPONSE, Stage.CAPTURE_TIME, hardware_touched=True))
            receive_event = RadioReceiveEvent(edge.timestamp_ns, started)
            deadline = self.backend.deadline(500_000)
            event = self.backend.observe_event(deadline)
            receive_event = replace(receive_event, irq_status=event.irq_status, device_errors=event.device_errors)
            self.backend.validate_event(event, transmit=False)
            irq = event.irq_status
            if not irq or irq & ~(IRQ_RX_DONE | IRQ_HEADER_ERROR | IRQ_CRC_ERROR):
                failure = RadioFailure(Error.UNEXPECTED_IRQ, Stage.READ_IRQ, opcode=0x12, irq_status=irq, hardware_touched=True)
                episode = self._builder(Operation.RECEIVE, failure)
                try:
                    self.backend.standby(deadline)
                    self.backend.clear_irq(irq, deadline)
                    self._restore(deadline)
                except RadioBackendError as error:
                    return self._recovering(Operation.RECEIVE, error.failure, episode=episode, restoring=True,
                                            receive_event=replace(receive_event, busy=self.backend.metrics))
                self._completed = (episode.finish(State.RX_SINGLE, self.clock.now_monotonic_us()),)
                return self._result(receive_event=replace(receive_event, busy=self.backend.metrics))
            self.backend.standby(deadline)
            if irq & (IRQ_HEADER_ERROR | IRQ_CRC_ERROR):
                if irq & IRQ_HEADER_ERROR:
                    self._bump("header_errors")
                if irq & IRQ_CRC_ERROR:
                    self._bump("crc_errors")
                self.backend.clear_irq(irq, deadline)
                restored = self.rearm()
                return replace(restored, receive_event=replace(receive_event, busy=self.backend.metrics))
            frame, copied = self.backend.copy_packet(deadline)
            receive_event = replace(receive_event, frame=frame, t2_packet_copied_monotonic_us=copied)
            rssi, snr = self.backend.read_packet_status(deadline)
            receive_event = replace(receive_event, rssi_dbm_x2=rssi, snr_db_x4=snr)
            self.backend.finish_receive(deadline)
            self.backend.clear_irq(irq, deadline)
            self._can_ack = True
            return self._result(receive_event=replace(receive_event, usable_for_ingress=True, busy=self.backend.metrics))
        except RadioBackendError as error:
            if receive_event is not None:
                receive_event = replace(receive_event, busy=self.backend.metrics)
            return self._recovering(Operation.RECEIVE, error.failure, receive_event=receive_event)

        except _ShutdownRequested:
            terminal = self.shutdown()
            if receive_event is not None:
                receive_event = replace(receive_event, busy=self.backend.metrics)
            return replace(terminal, receive_event=receive_event)

    def _tx_failure(self, failure, *, uncertain):
        self._can_ack = False
        self._prepared = None
        self._tx = RadioTxResult(
            E.AckTxResult.TX_UNCONFIRMED if uncertain else E.AckTxResult.SET_TX_FAILED,
            RadioTxFacts(self.backend.set_tx_outcome, uncertain and self.backend.last_set_tx_issued_us is None),
            self.backend.last_set_tx_issued_us,
        )
        if uncertain:
            if self.backend.last_set_tx_issued_us is not None:
                self._state = State.TX_ACTIVE
            return self._recovering(Operation.TRANSMIT, failure, tx_uncertain=True)
        episode = self._builder(Operation.TRANSMIT, failure)
        try:
            self._restore(self.backend.deadline(500_000))
        except RadioBackendError as error:
            return self._recovering(Operation.TRANSMIT, error.failure, episode=episode, restoring=True)
        self._completed = (episode.finish(State.RX_SINGLE, self.clock.now_monotonic_us()),)
        return self._result()

    @_operation(State.RX_EVENT_PENDING)
    def prepare_ack(self, frame, *, occurrence_sequence=None):
        """Write caller-selected bytes before the caller tentatively charges airtime."""
        self._require(State.RX_EVENT_PENDING)
        if not self._can_ack or self._prepared is not None:
            return self._invalid(Error.INVALID_STATE)
        if type(frame) is not bytes or not 1 <= len(frame) <= 255:
            return self._invalid(Error.INVALID_ARGUMENT)
        if occurrence_sequence is not None:
            try:
                integer(occurrence_sequence, 1)
            except (TypeError, ValueError):
                return self._invalid(Error.INVALID_ARGUMENT)
        self._sequence = occurrence_sequence
        self.backend.last_set_tx_issued_us = None
        self.backend.set_tx_outcome = Outcome.DEFINITELY_NOT_APPLIED
        try:
            self.backend.write_buffer(frame, self.backend.deadline(500_000))
        except RadioBackendError as error:
            return self._tx_failure(error.failure, uncertain=False)
        self._prepared = frame
        return self._result()

    @_operation(State.RX_EVENT_PENDING)
    def start_ack(self, authorization):
        """Caller has already consumed allowance; no grant or refund policy here."""
        self._require(State.RX_EVENT_PENDING)
        if self._prepared is None:
            return self._invalid(Error.INVALID_STATE)
        if type(authorization) is not RadioTxAuthorization:
            return self._invalid(Error.INVALID_ARGUMENT)
        if self._sequence is not None and authorization.occurrence_sequence not in (None, self._sequence):
            return self._invalid(Error.INVALID_ARGUMENT)
        self._sequence = authorization.occurrence_sequence or self._sequence
        self._authorization = authorization
        deadline = self.backend.deadline(500_000, authorization.submission_deadline_monotonic_us)
        self._installing_tx = True
        try:
            self.backend.install_profile(transmit=True, payload_length=len(self._prepared), deadline=deadline)
            self._installing_tx = False
            self.backend.start_tx(deadline)
        except RadioBackendError as error:
            uncertain = error.failure.outcome is not Outcome.DEFINITELY_NOT_APPLIED
            return self._tx_failure(error.failure, uncertain=uncertain)
        self._can_ack = False
        self._prepared = None
        self._state = State.TX_ACTIVE
        self._tx_deadline = checked_monotonic_deadline(self.backend.last_set_tx_issued_us, maximum_lifetime_monotonic_us(250_000))
        self._tx = None
        return self._result()

    @_operation(State.TX_ACTIVE)
    def finish_ack(self):
        self._require(State.TX_ACTIVE)
        try:
            edge = self.backend.wait_edge(deadline_monotonic_us=self._tx_deadline)
            self._checkpoint()
            if edge is None or edge.timestamp_ns > self._tx_deadline * 1000:
                raise RadioBackendError(RadioFailure(Error.DEADLINE, Stage.WAIT_IRQ,
                    Outcome.UNCERTAIN, 0x83, hardware_touched=True))
            if edge.timestamp_ns < self.backend.last_set_tx_issued_us * 1000:
                raise RadioBackendError(RadioFailure(Error.UNEXPECTED_IRQ, Stage.WAIT_IRQ,
                    Outcome.UNCERTAIN, 0x83, hardware_touched=True))
            if edge.timestamp_ns > self.clock.now_monotonic_us() * 1000 + 999:
                raise RadioBackendError(RadioFailure(Error.MALFORMED_RESPONSE, Stage.CAPTURE_TIME,
                    Outcome.UNCERTAIN, hardware_touched=True))
            deadline = self.backend.deadline(500_000)
            event = self.backend.observe_event(deadline)
            self.backend.validate_event(event, transmit=True)
            irq = event.irq_status
            if irq not in (IRQ_TX_DONE, IRQ_TIMEOUT):
                raise RadioBackendError(RadioFailure(Error.UNEXPECTED_IRQ, Stage.READ_IRQ,
                    Outcome.UNCERTAIN, 0x12, irq_status=irq, hardware_touched=True))
            self._tx = RadioTxResult(E.AckTxResult.TX_DONE if irq == IRQ_TX_DONE else E.AckTxResult.TX_TIMEOUT,
                RadioTxFacts(Outcome.CONFIRMED_APPLIED), self.backend.last_set_tx_issued_us,
                edge.monotonic_us if irq == IRQ_TX_DONE else None)
            self.backend.standby(deadline)
            self.backend.clear_irq(irq, deadline)
        except RadioBackendError as error:
            if self._tx is None:
                self._tx = RadioTxResult(E.AckTxResult.TX_UNCONFIRMED, RadioTxFacts(self.backend.set_tx_outcome), self.backend.last_set_tx_issued_us)
            return self._recovering(Operation.TRANSMIT, error.failure, tx_uncertain=self._tx.ack_tx_result is E.AckTxResult.TX_UNCONFIRMED)
        try:
            self._restore(deadline)
        except RadioBackendError as error:
            return self._recovering(Operation.RECEIVE, error.failure, restoring=True)
        return self._result()

    def _recovery_terminal(self, state):
        self._state = state
        self._bump("recovery_failures")
        try:
            self.backend.close()
        except RadioBackendError:
            # The terminal radio failure remains authoritative; no further TX.
            pass
        self._opened = False
        completed = self._episode.finish(state, self.clock.now_monotonic_us())
        self._episode = None
        return self._result(episodes=(completed,))

    @_operation(State.RECOVERING)
    def recover(self):
        self._require(State.RECOVERING)
        if self._episode.trigger_failure.hardware_missing:
            return self._recovery_terminal(State.HARDWARE_MISSING)
        for hard in (False, True):
            self._episode.start_level(hard=hard)
            deadline = self.backend.deadline(2_000_000 if hard else 500_000)
            self.backend.last_set_rx_issued_us = None
            try:
                if hard:
                    self.backend.initialize(deadline)
                    self.backend.arm_receive(deadline, resynchronize=True)
                else:
                    self.backend.soft_restore(deadline)
            except RadioBackendError as error:
                if self.backend.last_set_rx_issued_us is not None:
                    self._t6 = self.backend.last_set_rx_issued_us

                self._episode.finish_level(error.failure)
                if error.failure.hardware_missing:
                    return self._recovery_terminal(State.HARDWARE_MISSING)
                if hard:
                    return self._recovery_terminal(State.RECOVERY_EXHAUSTED)
            else:
                self._t6 = self.backend.last_set_rx_issued_us
                self._episode.finish_level()
                self._state = State.RX_SINGLE
                self._bump("recovery_successes")
                self._completed += (self._episode.finish(State.RX_SINGLE, self.clock.now_monotonic_us()),)
                self._episode = None
                return self._result()
            finally:
                if self.backend.last_set_rx_issued_us is not None:
                    self._t6 = self.backend.last_set_rx_issued_us

    def request_shutdown(self):
        """May be called from another thread: record intent, perform no hardware I/O."""
        self._stop.set()

    def _checkpoint(self):
        if self._stop.is_set() and not self._cleaning:
            raise _ShutdownRequested()

    def _invalid(self, code):
        failure = RadioFailure(code, Stage.VALIDATE_INPUT if code is Error.INVALID_ARGUMENT else Stage.STATE_CHECK)
        episode = self._builder(Operation.VALIDATE, failure)
        if code is Error.INVALID_STATE and (self.state is State.RECOVERING or (self._opened and self.backend.profile is None)):
            if self.state is State.RECOVERING:
                existing = self.recover()
                self._completed += existing.episodes
                if self.state in _TERMINAL:
                    return self._result(episodes=(episode.finish(self.state, self.clock.now_monotonic_us(), safe=False),))
            else:
                self._recovering(Operation.VALIDATE, failure, episode=episode)
                return self.recover()
        result = self.shutdown()
        return replace(result, episodes=result.episodes + (episode.finish(State.SHUTDOWN, self.clock.now_monotonic_us(), safe=False),))

    def shutdown(self, *, deadline_monotonic_us=None):
        self._claim()
        if self.state in _TERMINAL:
            return self._result()
        self._stop.set()
        self._cleaning = True
        self._can_ack = False
        if self._tx is None:
            if self.backend.last_set_tx_issued_us is not None:
                self._tx = RadioTxResult(E.AckTxResult.UNKNOWN_INTERRUPTED,
                    RadioTxFacts(self.backend.set_tx_outcome), self.backend.last_set_tx_issued_us)
            elif self._prepared is not None:
                uncertain = self._installing_tx and self.backend.command_uncertain
                self._tx = RadioTxResult(E.AckTxResult.TX_UNCONFIRMED if uncertain else E.AckTxResult.SET_TX_FAILED,
                    RadioTxFacts(Outcome.DEFINITELY_NOT_APPLIED, uncertain))
        self._prepared = None
        self._edge = None
        if self._episode is not None:
            self._episode.interrupt()
        deadline = self.backend.deadline(500_000)
        if deadline_monotonic_us is not None:
            deadline = min(deadline, deadline_monotonic_us)
        cleanup = []
        safe = False
        try:
            try:
                if not self._opened:
                    self.backend.open(deadline)
                    self._opened = True
                self.backend.safe_standby(deadline)
                safe = True
            except RadioBackendError as error:
                cleanup.extend(self._builder(Operation.CLEANUP, item) for item in _failures(error))
                if self._opened and not error.failure.hardware_missing:
                    try:
                        self.backend.reset(deadline)
                        self.backend.safe_standby(deadline)
                        safe = True
                    except RadioBackendError:
                        pass
        finally:
            try:
                self.backend.close()
                if self.clock.now_monotonic_us() > deadline:
                    raise RadioBackendError(RadioFailure(Error.DEADLINE, Stage.DETACH_IRQ, hardware_touched=True))
            except RadioBackendError as error:
                cleanup.extend(self._builder(Operation.CLEANUP, item) for item in _failures(error))
                safe = False
            except BaseException:
                safe = False
                raise
            finally:
                self._opened = False
                self._state = State.SHUTDOWN
                self._safe_shutdown = safe
                self._cleaning = False
        if self._episode is not None:
            self._bump("recovery_failures")
            self._completed += (self._episode.finish(State.SHUTDOWN, self.clock.now_monotonic_us(), safe=safe),)
            self._episode = None
        self._completed += tuple(
            item.finish(State.SHUTDOWN, self.clock.now_monotonic_us(), safe=safe) for item in cleanup
        )
        return self._result()
