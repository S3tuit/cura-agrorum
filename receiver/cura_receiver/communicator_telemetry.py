"""Single-owner health and best-effort diagnostic publication.

Only the communicator allocates diagnostic identities. Packet profiles publish
before their separate diagnostic attempt. Queue pressure and persistence outages
are counted at the original admission attempt, never diagnosed recursively.
Closure ends emission, including completed episodes waiting on clock admission;
there is no alternate database or side-file diagnostic path.
"""

from enum import Enum, auto

from .elapsed_duration import checked_monotonic_deadline
from .generated import receiver_enums_generated as E
from .persist_queue_entities import (ReceiverHealthRequestV1, RECEIVER_HEALTH_REQUEST_V1_SPEC,
                                    DIAGNOSTIC_V1_SPEC, RADIO_OBSERVATION_V1_SPEC)
from .ports.chrony import ChronyStepDisposition
from .ports.ds3231 import Ds3231WriteDisposition
from .radio_diagnostics import radio_diagnostic
from .time_diagnostics import time_diagnostic
from .control_diagnostics import control_diagnostic
from .core_diagnostics import core_diagnostic
from .health_observations import read_rtc_for_health
from .health_assessment import assess_health


class EmissionSkipped(Enum):
    CLOSED = auto()


class CommunicatorTelemetry:
    def __init__(self, communicator, *, rtc):
        self.communicator = communicator
        self.rtc = rtc
        self.health_sequence = self.diagnostic_sequence = 0

    def _publication_allowed(self):
        """Fence expiry before telemetry, without borrowing an owned reservation.

        Capture and cleanup can cross a trust deadline after the scheduler's
        last check. Only RuntimeTime publishes the clock boundary; a failed
        attempt leaves telemetry blocked for the scheduler's normal retry.
        """
        c = self.communicator
        if (c.queue.closed or c.ingress.active_occurrence is not None
                or c.queue.reservation_active or c.time.ordinary_admission_blocked):
            return False
        c.time.expire_due()
        return not c.time.ordinary_admission_blocked

    def _publish_prepared(self, entity, spec):
        c = self.communicator
        if not self._publication_allowed():
            return EmissionSkipped.CLOSED if c.queue.closed else None
        admitted = c.queue.try_reserve_one(spec)
        if admitted.status is E.AdmissionResult.RESERVED:
            admitted.reservation.publish(entity)
        return admitted.status

    def publish_observations(self):
        c = self.communicator
        if c.queue.closed:
            c.observations_dropped += len(c.pending_radio_observations)
            c.pending_radio_observations.clear()
            return EmissionSkipped.CLOSED
        while c.pending_radio_observations:
            observation = c.pending_radio_observations[0]
            result = self._publish_prepared(observation, RADIO_OBSERVATION_V1_SPEC)
            if result is EmissionSkipped.CLOSED:
                c.observations_dropped += len(c.pending_radio_observations)
                c.pending_radio_observations.clear()
                return result
            if result is None:
                return None
            if result is not E.AdmissionResult.RESERVED:
                c.observations_dropped += 1
            c.pending_radio_observations.popleft()

    def health(self, *, final=False, deadline=None):
        c = self.communicator
        # Start the sample before the gate's clock check, so expiry at this
        # timestamp is covered even if the gate itself crosses the deadline.
        sampled = c.clock.now_monotonic_us()
        if not self._publication_allowed():
            return EmissionSkipped.CLOSED if c.queue.closed else None
        self.health_sequence = checked_monotonic_deadline(self.health_sequence, 1)
        t, r = c.time, c.radio.counters
        software_time = t.state
        trigger = E.RadioObservationTrigger.FINAL_FAILURE if final else E.RadioObservationTrigger.HEALTH
        observation = c.identify_radio_observation(c.radio.observe(trigger, deadline=deadline))
        status, errno, start, finish, utc = read_rtc_for_health(t, self.rtc, deadline=deadline)
        # All fallible construction precedes admission. These counts therefore
        # describe attempts preceding this health unit, including discarded pilots.
        entity = ReceiverHealthRequestV1(
            receiver_instance_id=c.instance_id, health_sequence=self.health_sequence,
            communicator_sampled_at_monotonic_us=sampled, radio_observation=observation,
            consistency=assess_health(observation.snapshot, software_time.rtc_health, status),
            radio_header_errors_observed=r.header_errors_observed,
            radio_header_errors_handled=r.header_errors_handled,
            radio_crc_errors_observed=r.crc_errors_observed,
            radio_crc_errors_handled=r.crc_errors_handled,
            radio_observations_dropped=c.observations_dropped,
            radio_recovery_attempts=r.recovery_attempts, radio_recovery_successes=r.recovery_successes,
            radio_recovery_failures=r.recovery_failures, radio_recovery_attempts_by_reason=r.recovery_attempts_by_reason,
            system_time_quality=software_time.quality, rtc_health=software_time.rtc_health,
            rtc_read_status=status, rtc_read_errno=errno, rtc_read_started_at_monotonic_us=start,
            rtc_read_finished_at_monotonic_us=finish, rtc_read_utc_s=utc,
            time_quality_transition_count=t.time_quality_transition_count,
            rtc_health_transition_count=t.rtc_health_transition_count,
            last_time_quality_transition_monotonic_us=t.last_time_quality_transition_monotonic_us,
            last_rtc_health_transition_monotonic_us=t.last_rtc_health_transition_monotonic_us,
            chrony_step_command_results=tuple(t.step_command_counts[v] for v in (ChronyStepDisposition.SUBMITTED,
                ChronyStepDisposition.NOT_SUBMITTED, ChronyStepDisposition.OUTCOME_UNKNOWN)),
            rtc_write_results=tuple(t.rtc_write_counts[v] for v in (Ds3231WriteDisposition.COMPLETED,
                Ds3231WriteDisposition.NOT_APPLIED, Ds3231WriteDisposition.OUTCOME_UNKNOWN)),
            rtc_write_readback_verified_count=t.rtc_readback_verified_count,
            rtc_write_trust_invalidated_count=t.rtc_trust_invalidated_count,
            persist_queue_admission_counts=c.queue.counts,
        )
        # The prepared sample predates the second gate: expiry during a probe
        # publishes its boundary without relabeling earlier software evidence.
        result = self._publish_prepared(entity, RECEIVER_HEALTH_REQUEST_V1_SPEC)
        if result is not E.AdmissionResult.RESERVED:
            c.observations_dropped += 1
        return result

    def diagnostic(self, factory, episode):
        """None means clock-gated: caller retains the completed episode.

        CLOSED permanently ends emission; the caller discards the episode.
        Every operational result, including rejection, consumes this one attempt.
        The caller must not resubmit it or construct a diagnostic about rejection.
        """
        c = self.communicator
        if not self._publication_allowed():
            return EmissionSkipped.CLOSED if c.queue.closed else None
        self.diagnostic_sequence = checked_monotonic_deadline(self.diagnostic_sequence, 1)
        diagnostic = factory(episode, receiver_instance_id=c.instance_id,
                             diagnostic_sequence=self.diagnostic_sequence)
        return self._publish_prepared(diagnostic, DIAGNOSTIC_V1_SPEC)

    def radio(self, episode):
        return self.diagnostic(radio_diagnostic, episode)

    def time(self, episode):
        return self.diagnostic(time_diagnostic, episode)

    def control(self, episode):
        return self.diagnostic(control_diagnostic, episode)

    def core(self, episode):
        return self.diagnostic(core_diagnostic, episode)
