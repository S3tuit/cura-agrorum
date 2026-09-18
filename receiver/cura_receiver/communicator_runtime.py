"""Communicator turn dispatch and nonrecursive terminal diagnostic handling."""

from dataclasses import dataclass

from .communicator import CommunicatorFailure
from .communicator_scheduler import Work
from .communicator_telemetry import CommunicatorTelemetry
from .core_diagnostics import exception_episode
from .generated import receiver_enums_generated as E
from .persist_queue import PersistQueueInterfaceError
from .ports.radio import Outcome


@dataclass(frozen=True, slots=True)
class RuntimeFailure:
    original: Exception
    radio_result: object | None
    exchange: object | None
    diagnostic_result: object | None
    queue_sound: bool


class CommunicatorRuntime:
    """Single-thread owner; no worker, device creation or hidden waiting.

    Completed episodes wait behind the mandatory clock boundary. While blocked,
    the scheduler performs only the bounded boundary action, so normal producers
    cannot accumulate a second stream of deferred records. Closure discards them.
    """

    def __init__(self, scheduler, *, control_episodes):
        self.scheduler = scheduler
        self.communicator = scheduler.communicator
        self.telemetry = CommunicatorTelemetry(self.communicator)
        self.control_episodes = control_episodes
        self._pending = []
        self._emitting_diagnostic = False
        self.failure = None
        self.terminal = False

    def _collect_exchange(self, exchange, *, include_radio=True):
        if exchange is None:
            return
        if include_radio:
            self._pending.extend((self.telemetry.radio, item) for item in exchange.radio_episodes)
        for update in exchange.time_updates:
            if update is not None and update.failure is not None:
                self._pending.append((self.telemetry.time, update.failure))

    def _collect_control(self, *, terminal=False):
        self._pending.extend((self.telemetry.control, item)
            for item in self.control_episodes.take_ready(terminal=terminal))

    def flush_diagnostics(self):
        # Packet completion retains the sole reservation; diagnostic publication
        # runs only after that completion has released ownership.
        if self.communicator.ingress.active_occurrence is not None:
            return
        while self._pending:
            emitter, episode = self._pending[0]
            self._emitting_diagnostic = True
            result = emitter(episode)
            self._emitting_diagnostic = False
            if result is None:
                return
            del self._pending[0]

    def step(self):
        if self.terminal:
            raise RuntimeError('terminal communicator cannot run another turn')
        try:
            self.flush_diagnostics()
            turn = self.scheduler.run_once()
            self._collect_exchange(turn.exchange)
            if turn.update is not None and getattr(turn.update, 'failure', None) is not None:
                self._pending.append((self.telemetry.time, turn.update.failure))
            self._collect_control(terminal=turn.work is Work.TERMINAL)
            self.flush_diagnostics()
            if turn.work is Work.HEALTH:
                self.scheduler.failure_location = (E.CorePhase.PERIODIC_HEALTH,
                    E.CoreFailureStage.CONSTRUCT_ENTITY, E.DiagnosticOperation.APPEND)
                self.telemetry.health()
            self.terminal = turn.work is Work.TERMINAL
            return turn
        except Exception as error:
            self._terminate(error)
            return None

    def _terminate(self, error):
        c = self.communicator
        observed = c.clock.now_monotonic_us()
        exchange = None
        radio_result = None
        original = error
        diagnostic_result = None
        if isinstance(error, CommunicatorFailure):
            original, exchange = error.original, error.result
            observed = error.observed_at if error.observed_at is not None else observed
            radio_result = exchange.radio_result
        else:
            # Packet composition already does this on its own fatal boundary.
            c.radio.request_shutdown()
            try:
                radio_result = c.radio.shutdown()
            except Exception:
                pass
        self.terminal = True
        # Never probe a queue whose ownership is not explicitly known sound.
        sound = not isinstance(original, PersistQueueInterfaceError) or original.queue_known_sound
        if isinstance(error, CommunicatorFailure) and isinstance(error.cleanup_error, PersistQueueInterfaceError):
            sound = sound and error.cleanup_error.queue_known_sound
        try:
            if sound and not self._emitting_diagnostic:
                if isinstance(error, CommunicatorFailure):
                    self._pending.extend((self.telemetry.radio, item) for item in error.completed_episodes)
                    self._collect_exchange(exchange, include_radio=False)
                self._collect_control(terminal=True)
                self.flush_diagnostics()
                entity = (exchange.finalization.published_entity if exchange is not None
                          and exchange.finalization is not None else None)
                active = c.ingress.active_occurrence
                profile = entity.profile if entity is not None else active.pre_tx_profile if active is not None else None
                candidate = entity.candidate if entity is not None and hasattr(entity, 'candidate') else active.candidate if active is not None else None
                entity_kind = (E.PersistQueueEntityKind.MEASUREMENT_PROFILE if candidate is not None
                               else E.PersistQueueEntityKind.PROFILE_ONLY if profile is not None else None)
                tx = None if radio_result is None else radio_result.tx
                state = c.airtime.owner.state
                phase, stage, operation = (error.location if isinstance(error, CommunicatorFailure)
                    and error.location is not None else self.scheduler.failure_location)
                bucket = c.airtime.outstanding_bucket_expiration_utc_us
                episode = exception_episode(original, phase=phase, stage=stage, operation=operation,
                    started=observed, finished=c.clock.now_monotonic_us(),
                    safe_radio=radio_result is not None and radio_result.safe_shutdown is True,
                    related_occurrence_sequence=(c.occurrence_sequence if exchange is not None and c.occurrence_sequence else None),
                    related_entity_kind=entity_kind, occurrence_accepted=candidate is not None,
                    ack_selected=profile is not None and profile.ack_selected is not E.AckSelection.NONE,
                    tx_may_have_started=tx is not None and tx.t4_set_tx_attempted_monotonic_us is not None
                        and tx.facts.set_tx_outcome is not Outcome.DEFINITELY_NOT_APPLIED,
                    airtime_grant_outstanding=bucket is not None,
                    airtime_bucket_expiration_utc_us=bucket,
                    communicator_state_generation=None if state is None else state.generation,
                    profile_published=exchange is not None and exchange.finalization is not None
                        and exchange.finalization.published_entity is not None)
                # A CORE trigger owns cleanup; no second RADIO root is created.
                if episode is not None and active is None:
                    diagnostic_result = self.telemetry.core(episode)
        except Exception:
            # Allocation, construction or admission failure cannot recurse.
            pass
        self._pending.clear()
        self.failure = RuntimeFailure(original, radio_result, exchange, diagnostic_result, sound)
