"""Single-owner health and best-effort diagnostic publication."""

from enum import Enum, auto

from .elapsed_duration import checked_monotonic_deadline
from .generated import receiver_enums_generated as E
from .persist_queue_entities import (ReceiverHealthRequestV1, RECEIVER_HEALTH_REQUEST_V1_SPEC,
                                    DIAGNOSTIC_V1_SPEC)
from .ports.chrony import ChronyStepDisposition
from .ports.ds3231 import Ds3231WriteDisposition
from .radio_diagnostics import radio_diagnostic
from .time_diagnostics import time_diagnostic
from .control_diagnostics import control_diagnostic
from .core_diagnostics import core_diagnostic, CoreFault


class EmissionSkipped(Enum):
    CLOSED = auto()


class CommunicatorTelemetry:
    def __init__(self, communicator):
        self.communicator = communicator
        self.health_sequence = self.diagnostic_sequence = 0

    def health(self):
        c = self.communicator
        if c.queue.closed:
            return EmissionSkipped.CLOSED
        if c.time.ordinary_admission_blocked:
            return None
        self.health_sequence = checked_monotonic_deadline(self.health_sequence, 1)
        reserved = c.queue.try_reserve_one(RECEIVER_HEALTH_REQUEST_V1_SPEC)
        if reserved.status is not E.AdmissionResult.RESERVED:
            return reserved.status
        try:
            t, r = c.time, c.radio.counters
            entity = ReceiverHealthRequestV1(
                c.instance_id, self.health_sequence, c.clock.now_monotonic_us(), c.radio.state,
                r.recovery_attempts, r.recovery_successes, r.recovery_failures, r.recovery_attempts_by_reason,
                t.state.quality, t.state.rtc_health,
                t.time_quality_transition_count, t.rtc_health_transition_count,
                t.last_time_quality_transition_monotonic_us, t.last_rtc_health_transition_monotonic_us,
                tuple(t.step_command_counts[v] for v in (ChronyStepDisposition.SUBMITTED,
                    ChronyStepDisposition.NOT_SUBMITTED, ChronyStepDisposition.OUTCOME_UNKNOWN)),
                tuple(t.rtc_write_counts[v] for v in (Ds3231WriteDisposition.COMPLETED,
                    Ds3231WriteDisposition.NOT_APPLIED, Ds3231WriteDisposition.OUTCOME_UNKNOWN)),
                t.rtc_readback_verified_count, t.rtc_trust_invalidated_count, c.queue.counts,
            )
        except MemoryError:
            reserved.reservation.cancel()
            raise
        except Exception as error:
            reserved.reservation.cancel()
            raise CoreFault(E.CoreDiagnosticErrorCode.REPRESENTATION_INVARIANT,
                E.DiagnosticOperation.APPEND, E.CorePhase.PERIODIC_HEALTH,
                E.CoreFailureStage.CONSTRUCT_ENTITY) from error
        reserved.reservation.publish(entity)
        return reserved.status

    def diagnostic(self, factory, episode):
        """None means clock-gated: caller retains the completed episode.

        CLOSED permanently ends emission; the caller discards the episode.
        Every operational result, including rejection, consumes this one attempt.
        The caller must not resubmit it or construct a diagnostic about rejection.
        """
        c = self.communicator
        if c.queue.closed:
            return EmissionSkipped.CLOSED
        if c.time.ordinary_admission_blocked:
            return None
        self.diagnostic_sequence = checked_monotonic_deadline(self.diagnostic_sequence, 1)
        diagnostic = factory(episode, receiver_instance_id=c.instance_id,
                             diagnostic_sequence=self.diagnostic_sequence)
        result = c.queue.try_reserve_one(DIAGNOSTIC_V1_SPEC)
        if result.status is E.AdmissionResult.RESERVED:
            result.reservation.publish(diagnostic)
        return result.status

    def radio(self, episode):
        return self.diagnostic(radio_diagnostic, episode)

    def time(self, episode):
        return self.diagnostic(time_diagnostic, episode)

    def control(self, episode):
        return self.diagnostic(control_diagnostic, episode)

    def core(self, episode):
        return self.diagnostic(core_diagnostic, episode)
