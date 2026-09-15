"""Communicator-owned time state; callers schedule operations outside radio work."""

from dataclasses import dataclass, replace
from enum import Enum, auto

from .elapsed_duration import (
    checked_monotonic_deadline,
    checked_monotonic_elapsed,
    maximum_lifetime_monotonic_us,
    minimum_wait_monotonic_us,
    checked_correlated_utc,
    checked_whole_second_midpoint,
    checked_utc_difference,
    checked_absolute_us,
)
from .generated import receiver_enums_generated as E
from .generated.receiver_entities_generated import (
    ClockObservationV1,
    encode_communicator_state_v1,
)
from .persist_queue_entities import CLOCK_OBSERVATION_V1_SPEC
from .ports.ds3231 import (
    Ds3231ReadResult,
    Ds3231WriteResult,
    Ds3231ReadStatus as R,
    Ds3231WriteDisposition as W,
    Ds3231Failure as RF,
)
from .ports.kernel_clock import KernelSampleStatus as K
from .ports.chrony import (
    ChronyQueryStatus as Q,
    ChronyStepDisposition as SD,
    ChronyStepResult,
)
from .time_diagnostics import (
    BackendStatus,
    ReceiverTimeEpisodeContextV1,
    TimeFailureEpisode,
    TimeFailureLatch,
    TimeFlags,
    integer,
    encoded_status,
)
from .time_observations import (
    network_observation,
    observation_schedule,
    rtc_observation,
    rtc_refresh_source_error_us,
    rtc_provenance_candidate,
    rtc_refresh_due_us,
)
from .persistence_control_values import (
    CommunicatorStateCommitResult,
    CommunicatorStateLoadResult,
    require_immutable_state,
    CommunicatorStateCommitDisposition as CD,
    CommunicatorStateLoadStatus as LS,
)
from .receiver_startup import ReceiverInstanceStart
from .time_policy import (
    startup_clock_state,
    advance_clock_state,
    network_error_bound_us,
    network_tracking_decision,
)


@dataclass(frozen=True, slots=True)
class RuntimeTimeSettings:
    tracking_budget_us: int = 250_000
    kernel_budget_us: int = 250_000
    rtc_read_budget_us: int = 5_000_000
    rtc_recovery_window_us: int = 3_000_000
    command_budget_us: int = 1_000_000
    control_budget_us: int = 1_000_000
    step_episode_budget_us: int = 30_000_000
    stable_poll_period_us: int = 1_000_000
    retry_initial_us: int = 1_000_000
    retry_cap_us: int = 60_000_000

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            integer(getattr(self, name), 1)
        if self.retry_initial_us > self.retry_cap_us:
            raise ValueError("retry initial bound exceeds its cap")


@dataclass(frozen=True, slots=True)
class TimeUpdate:
    failure: TimeFailureEpisode | None = None
    admission_result: E.AdmissionResult | None = None
    observation: ClockObservationV1 | None = None
    step_result: ChronyStepResult | None = None


class ChronyStepState(Enum):
    IDLE = auto()
    STEP_COMMAND_PENDING = auto()
    WAITING_FOR_STABLE_TIME = auto()
    RETRY_BACKOFF = auto()


class RtcRefreshStatus(Enum):
    DEFERRED = auto()
    TRUST_INVALIDATED = auto()
    PREWRITE_READ_FAILED = auto()
    WRITE_FAILED = auto()
    READBACK_FAILED = auto()
    READBACK_MISMATCH = auto()
    PERSISTENCE_FAILED = auto()
    VERIFIED = auto()


@dataclass(frozen=True, slots=True)
class RtcRefreshResult:
    status: RtcRefreshStatus
    failure: TimeFailureEpisode | None = None
    write_result: Ds3231WriteResult | None = None
    read_result: Ds3231ReadResult | None = None
    commit_result: CommunicatorStateCommitResult | None = None
    reconciliation_result: CommunicatorStateLoadResult | None = None
    prewrite_read_result: Ds3231ReadResult | None = None


@dataclass(frozen=True, slots=True)
class RtcReadRecoveryResult:
    result: Ds3231ReadResult
    operation_started_at_monotonic_us: int
    operation_finished_at_monotonic_us: int
    attempts: int
    first_failure: Ds3231ReadResult | None = None


def recover_rtc_read(rtc, *, clock, deadline_monotonic_us, attempt_budget_us):
    """Communicator policy: retry reads, retaining a separate final-call bound."""
    integer(deadline_monotonic_us)
    integer(attempt_budget_us, 1)
    start = now = clock.now_monotonic_us()
    attempts = 0
    first_failure = None
    result = Ds3231ReadResult(R.DEADLINE_EXCEEDED, start, start)
    while now < deadline_monotonic_us:
        result = rtc.read_time(
            deadline_monotonic_us=checked_monotonic_deadline(now, attempt_budget_us)
        )
        attempts += 1
        now = clock.now_monotonic_us()
        checked_monotonic_elapsed(result.operation_finished_at_monotonic_us, now)
        if first_failure is None and result.status in (R.IO_ERROR, R.DEADLINE_EXCEEDED):
            first_failure = result
        if now >= deadline_monotonic_us:
            result = replace(result, status=R.DEADLINE_EXCEEDED, rtc_utc_s=None)
            break
        if result.status in (R.OK, R.INVALID):
            break
    if first_failure is None and result.status is R.DEADLINE_EXCEEDED:
        first_failure = result
    return RtcReadRecoveryResult(result, start, now, attempts, first_failure)


def rtc_health(result):
    if type(result) is not Ds3231ReadResult:
        raise TypeError("a fresh RTC port result is required")
    return (
        E.RtcHealth.PRESENT
        if result.status is R.OK
        else (
            E.RtcHealth.INVALID if result.status is R.INVALID else E.RtcHealth.MISSING
        )
    )


class RuntimeTime:
    """One communicator caller owns this object and every injected platform port."""

    def __init__(
        self,
        *,
        receiver_instance_id,
        clock,
        kernel,
        queue,
        policy,
        startup_rtc_result,
        durable_state=None,
        settings=None,
    ):
        ReceiverInstanceStart(receiver_instance_id, 0)
        self.instance = receiver_instance_id
        self.clock, self.kernel, self.queue, self.policy = clock, kernel, queue, policy
        self.settings = settings or RuntimeTimeSettings()
        checked_monotonic_elapsed(
            startup_rtc_result.operation_finished_at_monotonic_us,
            clock.now_monotonic_us(),
        )
        self.state = startup_clock_state(rtc_health(startup_rtc_result))
        self.startup_rtc_result = startup_rtc_result
        self.durable_state = durable_state
        if durable_state is not None:
            require_immutable_state(durable_state)
        self.rtc_provenance = (
            None if durable_state is None else durable_state.rtc_provenance
        )
        self._health_observation_pending = False
        self.sample = None
        self.schedule = None
        self.tracking_poll_deadline = None
        self.observation_sequence = 0
        self.pending_observation = None
        self._network_latch = TimeFailureLatch()
        self._rtc_latch = TimeFailureLatch()
        self._step_boundary_published = False
        self.step_state = ChronyStepState.IDLE
        self.operation_generation = 0
        self.step_started_at_monotonic_us = None
        self.step_deadline_monotonic_us = None
        self.next_status_poll_monotonic_us = None
        self.retry_not_before_monotonic_us = None
        self._step_failure = None
        self._step_before = None
        self._step_clock_generation = None
        self._step_authorized_until = None
        self._retry_delay = self.settings.retry_initial_us
        self.step_command_counts = {disposition: 0 for disposition in SD}
        self.last_refresh_monotonic_us = None
        self.next_rtc_attempt_monotonic_us = 0
        self.rtc_trust_invalidated_count = 0
        self.rtc_write_counts = {disposition: 0 for disposition in W}
        self.rtc_readback_verified_count = 0

    @property
    def ordinary_admission_blocked(self):
        return self.pending_observation is not None or self._health_observation_pending

    def deadline(self, budget_us, *, cap=None):
        duration = maximum_lifetime_monotonic_us(
            budget_us, rate_bound_ppm=self.policy.monotonic_elapsed_rate_bound_ppm
        )
        deadline = checked_monotonic_deadline(self.clock.now_monotonic_us(), duration)
        return deadline if cap is None else min(deadline, cap)

    def _sequence(self):
        self.observation_sequence = integer(
            self.observation_sequence + 1, 1, (1 << 63) - 1
        )
        return self.observation_sequence

    def _failure(
        self,
        code,
        component,
        stage,
        start,
        finish,
        *,
        before,
        operation=E.DiagnosticOperation.READ,
        primary_status=None,
        os_errno=None,
        kernel_status_bits=None,
        flags=0,
        **details,
    ):
        flags |= (
            int(TimeFlags.TRUST_SUPPRESSED)
            if self.state.quality is E.SystemTimeQuality.UNTRUSTED
            else 0
        )
        flags |= (
            int(TimeFlags.QUALITY_CHANGED)
            if before.quality is not self.state.quality
            else 0
        )
        flags |= (
            int(TimeFlags.RTC_HEALTH_CHANGED)
            if before.rtc_health is not self.state.rtc_health
            else 0
        )
        context = ReceiverTimeEpisodeContextV1(
            component,
            stage,
            checked_monotonic_elapsed(start, finish),
            primary_status=primary_status,
            quality=(before.quality, self.state.quality),
            rtc_health=(before.rtc_health, self.state.rtc_health),
            flags=flags,
            os_errno=os_errno,
            kernel_status_bits=kernel_status_bits,
            clock_state_generation=self.state.generation,
            **details,
        )
        return TimeFailureEpisode(operation, code, finish, context)

    def _untrusted(self, at, *, tracking_processed=False, step=False, health=None):
        self.state = advance_clock_state(
            self.state,
            quality=E.SystemTimeQuality.UNTRUSTED,
            rtc_health=self.state.rtc_health if health is None else health,
            tracking_processed=tracking_processed,
            step_boundary=step,
        )
        self.sample = self.schedule = None
        if self.pending_observation is None:
            self.pending_observation = ClockObservationV1(
                self.instance,
                0,
                self.state.generation,
                at,
                None,
                step,
                self.state.quality,
                self.state.rtc_health,
            )
        return self.publish_pending()

    def publish_pending(self):
        pending = self.pending_observation
        if pending is None:
            return TimeUpdate()
        pending = replace(pending, observation_sequence=self._sequence())
        reservation = self.queue.try_reserve_one(CLOCK_OBSERVATION_V1_SPEC)
        self.pending_observation = pending
        if reservation.status is not E.AdmissionResult.RESERVED:
            return TimeUpdate(admission_result=reservation.status)
        reservation.reservation.publish(pending)
        self.pending_observation = None
        self._health_observation_pending = False
        if pending.step_discontinuity_boundary:
            self._step_boundary_published = True
        return TimeUpdate(admission_result=reservation.status, observation=pending)

    def expire_due(self):
        if self.sample is None:
            return TimeUpdate()
        bounds = [self.schedule.trust_expires_at_monotonic_us]
        if self.state.quality is E.SystemTimeQuality.NETWORK_SYNCED:
            bounds.append(self.tracking_poll_deadline)
        deadline = min((v for v in bounds if v is not None), default=None)
        if deadline is not None and self.clock.now_monotonic_us() >= deadline:
            return self._untrusted(deadline)
        return TimeUpdate()

    def _accept(
        self, sample, *, tracking_processed=False, tracking_start=None, health=None
    ):
        before = self.state
        candidate = advance_clock_state(
            before,
            quality=sample.quality,
            rtc_health=before.rtc_health if health is None else health,
            tracking_processed=tracking_processed,
        )
        schedule = observation_schedule(
            sample, self.policy, tracking_started_at_monotonic_us=tracking_start
        )
        now = self.clock.now_monotonic_us()
        if (
            schedule.trust_expires_at_monotonic_us is not None
            and now >= schedule.trust_expires_at_monotonic_us
        ) or (
            schedule.tracking_poll_due_at_monotonic_us is not None
            and now >= schedule.tracking_poll_due_at_monotonic_us
        ):
            return self._untrusted(
                now, tracking_processed=tracking_processed, health=candidate.rtc_health
            )
        sequence = self._sequence()
        reservation = self.queue.try_reserve_one(CLOCK_OBSERVATION_V1_SPEC)
        if before.generation != self.state.generation:
            if reservation.reservation is not None:
                reservation.reservation.cancel()
            return self._untrusted(now, tracking_processed=tracking_processed)
        if reservation.status is not E.AdmissionResult.RESERVED:
            self.state = advance_clock_state(
                before,
                quality=E.SystemTimeQuality.UNTRUSTED,
                rtc_health=candidate.rtc_health,
                tracking_processed=tracking_processed,
            )
            self.sample = self.schedule = None
            self.pending_observation = ClockObservationV1(
                self.instance,
                sequence,
                self.state.generation,
                now,
                None,
                False,
                self.state.quality,
                self.state.rtc_health,
            )
            return TimeUpdate(admission_result=reservation.status)
        observation = ClockObservationV1(
            self.instance,
            sequence,
            candidate.generation,
            sample.monotonic_us,
            sample.utc_us,
            False,
            candidate.quality,
            candidate.rtc_health,
        )
        reservation.reservation.publish(observation)
        self.state = candidate
        self._health_observation_pending = False
        self.sample = replace(sample, generation=candidate.generation)
        self.schedule = schedule
        if tracking_start is not None:
            self.tracking_poll_deadline = schedule.tracking_poll_due_at_monotonic_us
        return TimeUpdate(admission_result=reservation.status, observation=observation)

    def sample_network(self, tracking):
        """Process one completed tracking result and its actual read-only kernel bracket."""
        before = self.state
        if self.pending_observation is not None:
            self.state = advance_clock_state(
                before,
                quality=E.SystemTimeQuality.UNTRUSTED,
                rtc_health=before.rtc_health,
                tracking_processed=True,
            )
            return self.publish_pending()
        evidence = tracking.evidence()
        start = tracking.sample_started_at_monotonic_us
        finish = self.clock.now_monotonic_us()
        poll_deadline = checked_monotonic_deadline(
            start, self.policy.chrony_tracking_poll_period_cap_us
        )
        self.tracking_poll_deadline = poll_deadline
        try:
            if evidence is not None:
                network_error_bound_us(
                    evidence.remaining_correction_us, evidence.root_distance_us
                )
            decision = network_tracking_decision(
                before,
                evidence,
                now_monotonic_us=finish,
                required_poll_deadline_us=poll_deadline,
                policy=self.policy,
            )
        except OverflowError:
            update = self._untrusted(finish, tracking_processed=True)
            failure = self._failure(
                E.TimeDiagnosticErrorCode.CALCULATION_RANGE,
                E.TimeComponent.TIME_POLICY,
                E.TimeFailureStage.CALCULATE_ERROR_BOUND,
                start,
                finish,
                before=before,
                operation=E.DiagnosticOperation.VALIDATE,
            )
            return replace(update, failure=self._network_latch.failed(failure))
        if decision.candidate_quality is not E.SystemTimeQuality.NETWORK_SYNCED:
            if tracking.status is Q.OK:
                self._network_latch.succeeded()
            if decision.candidate_quality is E.SystemTimeQuality.RTC_HOLDOVER:
                self.state = advance_clock_state(
                    before,
                    quality=before.quality,
                    rtc_health=before.rtc_health,
                    tracking_processed=True,
                )
                if self.sample is not None:
                    self.sample = replace(self.sample, generation=self.state.generation)
                return TimeUpdate()
            return self._untrusted(finish, tracking_processed=True)
        kernel = self.kernel.sample(
            deadline_monotonic_us=self.deadline(
                self.settings.kernel_budget_us,
                cap=min(
                    poll_deadline,
                    checked_monotonic_deadline(
                        start, self.policy.time_sampling_margin_us
                    ),
                ),
            )
        )
        finish = self.clock.now_monotonic_us()
        if before.generation != self.state.generation:
            return self._untrusted(finish, tracking_processed=True)
        sample = network_observation(
            before,
            self.state,
            evidence,
            operation_started_at_monotonic_us=kernel.operation_started_at_monotonic_us,
            operation_finished_at_monotonic_us=kernel.operation_finished_at_monotonic_us,
            sampled_utc_us=(
                kernel.sampled_utc_us if kernel.sampled_utc_us is not None else 0
            ),
            kernel_sample_usable=kernel.status is K.OK,
            required_poll_deadline_us=poll_deadline,
            policy=self.policy,
        )
        if sample is not None:
            self._network_latch.succeeded()
            return self._accept(sample, tracking_processed=True, tracking_start=start)
        update = self._untrusted(finish, tracking_processed=True)
        codes = {
            K.IO_ERROR: E.TimeDiagnosticErrorCode.IO,
            K.DEADLINE_EXCEEDED: E.TimeDiagnosticErrorCode.DEADLINE,
            K.INVALID_RESPONSE: E.TimeDiagnosticErrorCode.INVALID_RESPONSE,
            K.CLOCK_INTERFERENCE: E.TimeDiagnosticErrorCode.CLOCK_INTERFERENCE,
        }
        if kernel.status in codes:
            status = (
                None
                if kernel.adjtimex_return is None
                else BackendStatus(
                    E.TimeBackendStatusKind.ADJTIMEX_RETURN, kernel.adjtimex_return
                )
            )
            failure = self._failure(
                codes[kernel.status],
                E.TimeComponent.KERNEL_CLOCK,
                E.TimeFailureStage.SAMPLE_SYSTEM_CLOCK,
                kernel.operation_started_at_monotonic_us,
                kernel.operation_finished_at_monotonic_us,
                before=before,
                primary_status=status,
                os_errno=kernel.os_errno,
                kernel_status_bits=kernel.kernel_status_bits,
            )
            update = replace(update, failure=self._network_latch.failed(failure))
        return update

    def observe_rtc(self, rtc, *, startup=False):
        """Direct RTC observations never read CLOCK_REALTIME or write either clock."""
        if self.pending_observation is not None:
            return self.publish_pending()
        before = self.state
        recovery = None if startup else self.recover_rtc(rtc)
        result = self.startup_rtc_result if startup else recovery.result
        now = self.clock.now_monotonic_us()
        if before.generation != self.state.generation:
            return TimeUpdate()
        health = rtc_health(result)
        failure = None
        sample = None
        if (
            result.status is R.OK
            and self.step_state is ChronyStepState.IDLE
            and before.quality is not E.SystemTimeQuality.NETWORK_SYNCED
        ):
            try:
                sample = rtc_observation(
                    before,
                    self.state,
                    self.rtc_provenance,
                    rtc_health=health,
                    rtc_utc_seconds=result.rtc_utc_s,
                    operation_started_at_monotonic_us=result.operation_started_at_monotonic_us,
                    operation_finished_at_monotonic_us=result.operation_finished_at_monotonic_us,
                    policy=self.policy,
                    report_calculation_failure=True,
                )
            except OverflowError:
                failure = E.TimeDiagnosticErrorCode.CALCULATION_RANGE
        if before.quality is E.SystemTimeQuality.NETWORK_SYNCED:
            self.state = advance_clock_state(
                before, quality=before.quality, rtc_health=health
            )
            if self.sample is not None:
                self.sample = replace(self.sample, generation=self.state.generation)
            self._health_observation_pending |= health is not before.rtc_health
            update = self.expire_due()
        elif sample is not None:
            update = self._accept(sample, health=health)
        else:
            update = self._untrusted(now, health=health)
        trigger = recovery.first_failure if recovery is not None else None
        trigger = trigger or result
        if trigger.status in (R.IO_ERROR, R.DEADLINE_EXCEEDED):
            failure = (
                E.TimeDiagnosticErrorCode.IO
                if trigger.status is R.IO_ERROR
                else E.TimeDiagnosticErrorCode.DEADLINE
            )
        if failure is not None:
            arithmetic = failure is E.TimeDiagnosticErrorCode.CALCULATION_RANGE
            episode = self._failure(
                failure,
                E.TimeComponent.TIME_POLICY if arithmetic else E.TimeComponent.DS3231,
                (
                    E.TimeFailureStage.CALCULATE_ERROR_BOUND
                    if arithmetic
                    else E.TimeFailureStage.READ_RTC
                ),
                trigger.operation_started_at_monotonic_us,
                now,
                before=before,
                operation=(
                    E.DiagnosticOperation.VALIDATE
                    if arithmetic
                    else E.DiagnosticOperation.READ
                ),
                primary_status=encoded_status(
                    E.TimeBackendStatusKind.DS3231_READ_STATUS, trigger.status
                ),
                secondary_status=encoded_status(
                    E.TimeBackendStatusKind.DS3231_READ_STATUS, result.status
                ),
                os_errno=trigger.os_errno,
            )
            update = replace(update, failure=self._rtc_latch.failed(episode))
        if result.status is R.OK:
            self._rtc_latch.succeeded()
        self.next_rtc_attempt_monotonic_us = checked_monotonic_deadline(
            now,
            minimum_wait_monotonic_us(
                self.settings.retry_initial_us,
                rate_bound_ppm=self.policy.monotonic_elapsed_rate_bound_ppm,
            ),
        )
        return update

    def recover_rtc(self, rtc):
        return recover_rtc_read(
            rtc,
            clock=self.clock,
            deadline_monotonic_us=self.deadline(self.settings.rtc_recovery_window_us),
            attempt_budget_us=maximum_lifetime_monotonic_us(
                self.settings.rtc_read_budget_us,
                rate_bound_ppm=self.policy.monotonic_elapsed_rate_bound_ppm,
            ),
        )

    def next_rtc_read_start(self):
        if (
            self.sample is not None
            and self.state.quality is E.SystemTimeQuality.RTC_HOLDOVER
        ):
            lead = maximum_lifetime_monotonic_us(
                self.settings.rtc_recovery_window_us + self.settings.rtc_read_budget_us,
                rate_bound_ppm=self.policy.monotonic_elapsed_rate_bound_ppm,
            )
            due = self.schedule.observation_due_at_monotonic_us
            return max(self.next_rtc_attempt_monotonic_us, due - lead)
        return self.next_rtc_attempt_monotonic_us

    def _commit_rtc_state(self, provenance, control, snapshot, generation):
        """The callback supplies the real state owner's complete immutable snapshot."""
        now = self.clock.now_monotonic_us()
        utc = checked_correlated_utc(self.sample.utc_us, self.sample.monotonic_us, now)
        requested = snapshot(
            provenance=provenance,
            snapshot_monotonic_us=now,
            snapshot_utc_us=utc,
            previous_state=self.durable_state,
        )
        if requested is None:
            return False, None, None
        require_immutable_state(requested)
        if (
            requested.generation != self.durable_state.generation + 1
            or requested.rtc_provenance != provenance
            or requested.airtime_snapshot_utc_us != utc
        ):
            raise ValueError("complete state callback violated the time handoff")
        if self.state.generation != generation or not self._rtc_source_valid(
            generation, allow_health_pending=True
        ):
            return False, None, None
        result = control.commit_communicator_state(
            requested,
            deadline_monotonic_us=self.deadline(self.settings.control_budget_us),
        )
        loaded = None
        acknowledged = result.disposition in (CD.COMMITTED, CD.ALREADY_COMMITTED)
        if result.disposition is CD.OUTCOME_UNKNOWN:
            loaded = control.load_communicator_state(
                deadline_monotonic_us=self.deadline(self.settings.control_budget_us)
            )
            acknowledged = loaded.status is LS.LOADED and encode_communicator_state_v1(
                loaded.state
            ) == encode_communicator_state_v1(requested)
        if acknowledged:
            self.durable_state = requested
            if provenance is None:
                self.rtc_provenance = None
        return acknowledged, result, loaded

    def _rtc_source_valid(self, generation, *, allow_health_pending=False):
        if (
            self.sample is None
            or generation != self.state.generation
            or self.step_state is not ChronyStepState.IDLE
            or self.pending_observation is not None
            or (self._health_observation_pending and not allow_health_pending)
        ):
            return False
        return (
            rtc_refresh_source_error_us(
                self.sample,
                self.state,
                now_monotonic_us=self.clock.now_monotonic_us(),
                required_poll_deadline_us=self.tracking_poll_deadline,
                policy=self.policy,
                report_calculation_failure=True,
            )
            is not None
        )

    def refresh_rtc(self, rtc, control, snapshot):
        """One invalidate/write/read-back/provenance episode; no blind device retries."""
        before = self.state
        generation = before.generation
        start = self.clock.now_monotonic_us()
        root = None
        write = read = prewrite = commit = reconciliation = None

        def finish(status):
            nonlocal root
            if root is not None:
                root = root.finish(
                    started_at_monotonic_us=start,
                    finished_at_monotonic_us=self.clock.now_monotonic_us(),
                    secondary_status=(
                        encoded_status(
                            E.TimeBackendStatusKind.DS3231_READ_STATUS, read.status
                        )
                        if read is not None
                        else None
                    ),
                    quality=(before.quality, self.state.quality),
                    rtc_health=(before.rtc_health, self.state.rtc_health),
                    flags=root.context.flags | (
                        int(TimeFlags.COMMAND_MAY_HAVE_APPLIED)
                        if write is not None and write.disposition is W.OUTCOME_UNKNOWN
                        else 0
                    ),
                )
            if (
                status is RtcRefreshStatus.TRUST_INVALIDATED
                and write is not None
                and write.disposition is not W.NOT_APPLIED
            ):
                self.rtc_trust_invalidated_count = min(
                    (1 << 63) - 1, self.rtc_trust_invalidated_count + 1
                )
            self.next_rtc_attempt_monotonic_us = checked_monotonic_deadline(
                self.clock.now_monotonic_us(),
                minimum_wait_monotonic_us(
                    self.settings.retry_initial_us,
                    rate_bound_ppm=self.policy.monotonic_elapsed_rate_bound_ppm,
                ),
            )
            return RtcRefreshResult(
                status, root, write, read, commit, reconciliation, prewrite
            )

        def record_read_failure(recovery, stage):
            nonlocal root
            trigger = recovery.first_failure
            if root is None and trigger is not None:
                root = self._failure(
                    E.TimeDiagnosticErrorCode.IO
                    if trigger.status is R.IO_ERROR else E.TimeDiagnosticErrorCode.DEADLINE,
                    E.TimeComponent.DS3231,
                    stage,
                    trigger.operation_started_at_monotonic_us,
                    recovery.operation_finished_at_monotonic_us,
                    before=before,
                    operation=E.DiagnosticOperation.SYNC,
                    primary_status=encoded_status(
                        E.TimeBackendStatusKind.DS3231_READ_STATUS, trigger.status
                    ),
                    secondary_status=encoded_status(
                        E.TimeBackendStatusKind.DS3231_READ_STATUS, recovery.result.status
                    ),
                    os_errno=trigger.os_errno,
                )

        try:
            if self.durable_state is None or not self._rtc_source_valid(generation):
                return finish(RtcRefreshStatus.DEFERRED)
            due = rtc_refresh_due_us(
                self.sample,
                last_refresh_monotonic_us=self.last_refresh_monotonic_us,
                policy=self.policy,
            )
            if due is None or start < due or start < self.next_rtc_attempt_monotonic_us:
                return RtcRefreshResult(RtcRefreshStatus.DEFERRED)
            recovered = self.recover_rtc(rtc)
            prewrite = recovered.result
            record_read_failure(recovered, E.TimeFailureStage.READ_RTC)
            if generation != self.state.generation:
                return finish(RtcRefreshStatus.TRUST_INVALIDATED)
            if prewrite.status is not R.OK:
                health = rtc_health(prewrite)
                self.state = advance_clock_state(
                    self.state, quality=self.state.quality, rtc_health=health
                )
                self.sample = replace(self.sample, generation=self.state.generation)
                self._health_observation_pending |= health is not before.rtc_health
                self.expire_due()
                return finish(RtcRefreshStatus.PREWRITE_READ_FAILED)
            if not self._rtc_source_valid(generation):
                return finish(RtcRefreshStatus.TRUST_INVALIDATED)
            if self.durable_state.rtc_provenance is not None:
                acknowledged, commit, reconciliation = self._commit_rtc_state(
                    None, control, snapshot, generation
                )
                if not acknowledged:
                    if not self._rtc_source_valid(
                        generation, allow_health_pending=True
                    ):
                        return finish(RtcRefreshStatus.TRUST_INVALIDATED)
                    return finish(
                        RtcRefreshStatus.PERSISTENCE_FAILED
                        if commit is not None
                        else RtcRefreshStatus.DEFERRED
                    )
            if not self._rtc_source_valid(generation):
                return finish(RtcRefreshStatus.TRUST_INVALIDATED)
            value = (
                checked_correlated_utc(
                    self.sample.utc_us,
                    self.sample.monotonic_us,
                    self.clock.now_monotonic_us(),
                )
                // 1_000_000
            )
            write = rtc.write_time(
                rtc_utc_s=value,
                deadline_monotonic_us=self.deadline(self.settings.command_budget_us),
            )
            self.rtc_write_counts[write.disposition] = min(
                (1 << 63) - 1, self.rtc_write_counts[write.disposition] + 1
            )
            matched = generation == self.state.generation
            if write.disposition is not W.COMPLETED:
                code = (
                    E.TimeDiagnosticErrorCode.OUTCOME_UNKNOWN
                    if write.disposition is W.OUTCOME_UNKNOWN
                    else (
                        E.TimeDiagnosticErrorCode.DEADLINE
                        if write.failure is RF.DEADLINE_EXCEEDED
                        else E.TimeDiagnosticErrorCode.IO
                    )
                )
                if root is None and (
                    write.failure is not RF.MISSING
                    or write.disposition is W.OUTCOME_UNKNOWN
                ):
                    root = self._failure(
                        code,
                        E.TimeComponent.RTC_WRITE_HELPER,
                        E.TimeFailureStage.EXECUTE_RTC_HELPER,
                        write.operation_started_at_monotonic_us,
                        write.operation_finished_at_monotonic_us,
                        before=before,
                        operation=E.DiagnosticOperation.SYNC,
                        primary_status=encoded_status(
                            E.TimeBackendStatusKind.DS3231_WRITE_DISPOSITION,
                            write.disposition,
                        ),
                        os_errno=write.os_errno,
                        flags=(
                            int(TimeFlags.COMMAND_MAY_HAVE_APPLIED)
                            if write.disposition is W.OUTCOME_UNKNOWN
                            else 0
                        ),
                    )
            if write.disposition is W.NOT_APPLIED:
                return finish(RtcRefreshStatus.WRITE_FAILED)
            # Read-back is mandatory after possible application, even if the generation changed.
            recovered = self.recover_rtc(rtc)
            read = recovered.result
            record_read_failure(recovered, E.TimeFailureStage.READ_BACK_RTC)
            matched &= generation == self.state.generation
            if matched:
                health = rtc_health(read)
                self.state = advance_clock_state(
                    self.state, quality=self.state.quality, rtc_health=health
                )
                generation = self.state.generation
                self.sample = replace(self.sample, generation=generation)
                self._health_observation_pending |= health is not before.rtc_health
            if not matched:
                return finish(RtcRefreshStatus.TRUST_INVALIDATED)
            if read.status is not R.OK:
                if root is None and read.status in (R.IO_ERROR, R.DEADLINE_EXCEEDED):
                    root = self._failure(
                        (
                            E.TimeDiagnosticErrorCode.IO
                            if read.status is R.IO_ERROR
                            else E.TimeDiagnosticErrorCode.DEADLINE
                        ),
                        E.TimeComponent.DS3231,
                        E.TimeFailureStage.READ_BACK_RTC,
                        read.operation_started_at_monotonic_us,
                        read.operation_finished_at_monotonic_us,
                        before=before,
                        operation=E.DiagnosticOperation.SYNC,
                        primary_status=encoded_status(
                            E.TimeBackendStatusKind.DS3231_READ_STATUS, read.status
                        ),
                        os_errno=read.os_errno,
                    )
                return finish(RtcRefreshStatus.READBACK_FAILED)
            midpoint = (
                read.operation_started_at_monotonic_us
                + (
                    read.operation_finished_at_monotonic_us
                    - read.operation_started_at_monotonic_us
                )
                // 2
            )
            intended = checked_correlated_utc(
                self.sample.utc_us, self.sample.monotonic_us, midpoint
            )
            actual = checked_whole_second_midpoint(read.rtc_utc_s)
            difference = checked_absolute_us(checked_utc_difference(actual, intended))
            if difference > self.policy.time_sampling_margin_us:
                if root is None:
                    root = self._failure(
                        E.TimeDiagnosticErrorCode.RTC_READBACK_MISMATCH,
                        E.TimeComponent.DS3231,
                        E.TimeFailureStage.VERIFY_RTC_READBACK,
                        start,
                        self.clock.now_monotonic_us(),
                        before=before,
                        operation=E.DiagnosticOperation.SYNC,
                        observed_value_us=actual,
                        comparison_value_us=intended,
                        threshold_us=self.policy.time_sampling_margin_us,
                    )
                return finish(RtcRefreshStatus.READBACK_MISMATCH)
            if not self._rtc_source_valid(generation, allow_health_pending=True):
                return finish(RtcRefreshStatus.TRUST_INVALIDATED)
            self.rtc_readback_verified_count = min(
                (1 << 63) - 1, self.rtc_readback_verified_count + 1
            )
            provenance = rtc_provenance_candidate(
                self.sample,
                self.state,
                verified_by_receiver_instance_id=self.instance,
                episode_started_at_monotonic_us=start,
                read_started_at_monotonic_us=read.operation_started_at_monotonic_us,
                read_finished_at_monotonic_us=read.operation_finished_at_monotonic_us,
                commit_at_monotonic_us=self.clock.now_monotonic_us(),
                rtc_utc_seconds=read.rtc_utc_s,
                required_poll_deadline_us=self.tracking_poll_deadline,
                policy=self.policy,
                report_calculation_failure=True,
            )
            if provenance is None:
                return finish(RtcRefreshStatus.TRUST_INVALIDATED)
            acknowledged, commit, reconciliation = self._commit_rtc_state(
                provenance, control, snapshot, generation
            )
            if not acknowledged:
                if not self._rtc_source_valid(generation, allow_health_pending=True):
                    return finish(RtcRefreshStatus.TRUST_INVALIDATED)
                return finish(
                    RtcRefreshStatus.PERSISTENCE_FAILED
                    if commit is not None
                    else RtcRefreshStatus.DEFERRED
                )
            if not self._rtc_source_valid(generation, allow_health_pending=True):
                return finish(RtcRefreshStatus.TRUST_INVALIDATED)
            self.rtc_provenance = provenance
            self.last_refresh_monotonic_us = self.clock.now_monotonic_us()
            return finish(RtcRefreshStatus.VERIFIED)
        except OverflowError:
            if root is None:
                root = self._failure(
                    E.TimeDiagnosticErrorCode.CALCULATION_RANGE,
                    E.TimeComponent.TIME_POLICY,
                    E.TimeFailureStage.CALCULATE_ERROR_BOUND,
                    start,
                    self.clock.now_monotonic_us(),
                    before=before,
                    operation=E.DiagnosticOperation.SYNC,
                )
            return finish(RtcRefreshStatus.TRUST_INVALIDATED)

    def next_tracking_start(self):
        if self.step_state is ChronyStepState.WAITING_FOR_STABLE_TIME:
            return min(
                self.next_status_poll_monotonic_us, self.step_deadline_monotonic_us
            )
        if self.step_state is ChronyStepState.RETRY_BACKOFF:
            return self.retry_not_before_monotonic_us
        if self.tracking_poll_deadline is None:
            return self.clock.now_monotonic_us()
        lead = maximum_lifetime_monotonic_us(
            self.settings.tracking_budget_us + self.settings.kernel_budget_us,
            rate_bound_ppm=self.policy.monotonic_elapsed_rate_bound_ppm,
        )
        return max(0, self.tracking_poll_deadline - lead)

    def _enter_backoff(self):
        now = self.clock.now_monotonic_us()
        self.step_state = ChronyStepState.RETRY_BACKOFF
        self.retry_not_before_monotonic_us = checked_monotonic_deadline(
            now,
            minimum_wait_monotonic_us(
                self._retry_delay,
                rate_bound_ppm=self.policy.monotonic_elapsed_rate_bound_ppm,
            ),
        )
        self._retry_delay = min(self.settings.retry_cap_us, self._retry_delay * 2)
        self._step_clock_generation = None
        self._step_authorized_until = None

    def _finish_step_failure(self, update):
        failure = self._step_failure
        self._step_failure = None
        if failure is not None:
            failure = failure.finish(
                started_at_monotonic_us=self.step_started_at_monotonic_us,
                finished_at_monotonic_us=self.clock.now_monotonic_us(),
                quality=(self._step_before.quality, self.state.quality),
                rtc_health=(self._step_before.rtc_health, self.state.rtc_health),
            )
        return replace(update, failure=failure)

    def _step_timeout(self):
        update = self._untrusted(self.step_deadline_monotonic_us)
        if self._step_failure is None:
            self._step_failure = self._failure(
                E.TimeDiagnosticErrorCode.DEADLINE,
                E.TimeComponent.CHRONY,
                E.TimeFailureStage.WAIT_STABLE_TIME,
                self.step_started_at_monotonic_us,
                self.clock.now_monotonic_us(),
                before=self._step_before,
                operation=E.DiagnosticOperation.SYNC,
                operation_generation=self.operation_generation,
            )
        self._enter_backoff()
        return self._finish_step_failure(update)

    def _submit_step(self, chrony):
        now = self.clock.now_monotonic_us()
        if self.pending_observation is not None:
            return self.publish_pending()
        if (
            not self._step_boundary_published
            or self.state.generation != self._step_clock_generation
            or now >= self._step_authorized_until
        ):
            self._enter_backoff()
            return TimeUpdate()
        result = chrony.apply_pending_correction_by_step(
            deadline_monotonic_us=self.deadline(
                self.settings.command_budget_us, cap=self.step_deadline_monotonic_us
            )
        )
        self.step_command_counts[result.disposition] = min(
            (1 << 63) - 1, self.step_command_counts[result.disposition] + 1
        )
        self._step_clock_generation = (
            None  # This operation can never submit a second command.
        )
        update = TimeUpdate(step_result=result)
        if result.disposition is not SD.SUBMITTED:
            code = (
                E.TimeDiagnosticErrorCode.OUTCOME_UNKNOWN
                if result.disposition is SD.OUTCOME_UNKNOWN
                else E.TimeDiagnosticErrorCode.COMMAND_REJECTED
            )
            self._step_failure = self._failure(
                code,
                E.TimeComponent.CHRONY,
                E.TimeFailureStage.SUBMIT_STEP,
                result.operation_started_at_monotonic_us,
                result.operation_finished_at_monotonic_us,
                before=self._step_before,
                operation=E.DiagnosticOperation.SYNC,
                primary_status=encoded_status(
                    E.TimeBackendStatusKind.CHRONY_STEP_DISPOSITION, result.disposition
                ),
                operation_generation=self.operation_generation,
                flags=(
                    int(TimeFlags.COMMAND_MAY_HAVE_APPLIED)
                    if result.disposition is SD.OUTCOME_UNKNOWN
                    else 0
                ),
            )
        if result.disposition is SD.NOT_SUBMITTED:
            self._enter_backoff()
            return self._finish_step_failure(update)
        # Even a generation change during submission cannot make a possibly applied command retryable.
        if self.state.quality is not E.SystemTimeQuality.UNTRUSTED:
            transition = self._untrusted(self.clock.now_monotonic_us())
            update = replace(transition, step_result=result)
        self.step_state = ChronyStepState.WAITING_FOR_STABLE_TIME
        self.next_status_poll_monotonic_us = self.clock.now_monotonic_us()
        return update

    def poll_chrony(self, chrony):
        """One bounded scheduling action, never waitsync or a blocking retry loop."""
        now = self.clock.now_monotonic_us()
        expiry = self.expire_due()
        if self.pending_observation is not None:
            return (
                expiry
                if expiry.admission_result is not None
                else self.publish_pending()
            )
        if self.step_state is ChronyStepState.STEP_COMMAND_PENDING:
            return self._submit_step(chrony)
        waiting = self.step_state is ChronyStepState.WAITING_FOR_STABLE_TIME
        if waiting and now >= self.step_deadline_monotonic_us:
            return self._step_timeout()
        if waiting and now < self.next_status_poll_monotonic_us:
            return expiry
        if (
            self.step_state is ChronyStepState.RETRY_BACKOFF
            and now < self.retry_not_before_monotonic_us
        ):
            return expiry
        before = self.state
        query = chrony.read_tracking(
            deadline_monotonic_us=self.deadline(
                self.settings.tracking_budget_us,
                cap=self.step_deadline_monotonic_us if waiting else None,
            )
        )
        finish = self.clock.now_monotonic_us()
        if before.generation != self.state.generation:
            return self._untrusted(finish, tracking_processed=True)
        if waiting and finish >= self.step_deadline_monotonic_us:
            self.state = advance_clock_state(
                self.state,
                quality=E.SystemTimeQuality.UNTRUSTED,
                rtc_health=self.state.rtc_health,
                tracking_processed=True,
            )
            return self._step_timeout()
        decision = network_tracking_decision(
            before,
            query.evidence(),
            now_monotonic_us=finish,
            required_poll_deadline_us=checked_monotonic_deadline(
                query.sample_started_at_monotonic_us,
                self.policy.chrony_tracking_poll_period_cap_us,
            ),
            policy=self.policy,
        )
        if decision.step_required and not waiting:
            self.operation_generation = integer(
                self.operation_generation + 1, 1, (1 << 64) - 1
            )
            self.step_started_at_monotonic_us = finish
            self.step_deadline_monotonic_us = self.deadline(
                self.settings.step_episode_budget_us
            )
            self._step_authorized_until = checked_monotonic_deadline(
                query.sample_started_at_monotonic_us,
                self.policy.time_sampling_margin_us,
            )
            self._step_before = before
            self._step_failure = None
            self._step_boundary_published = False
            self.step_state = ChronyStepState.STEP_COMMAND_PENDING
            update = self._untrusted(finish, tracking_processed=True, step=True)
            self._step_clock_generation = self.state.generation
            return update
        update = self.sample_network(query)
        if query.status in (Q.DEADLINE_EXCEEDED, Q.INVALID_RESPONSE):
            code = (
                E.TimeDiagnosticErrorCode.DEADLINE
                if query.status is Q.DEADLINE_EXCEEDED
                else E.TimeDiagnosticErrorCode.INVALID_RESPONSE
            )
            failure = self._failure(
                code,
                E.TimeComponent.CHRONY,
                E.TimeFailureStage.QUERY_TRACKING,
                query.sample_started_at_monotonic_us,
                query.sample_finished_at_monotonic_us,
                before=before,
                primary_status=encoded_status(
                    E.TimeBackendStatusKind.CHRONY_QUERY_STATUS, query.status
                ),
            )
            update = replace(update, failure=self._network_latch.failed(failure))
        if waiting:
            if self._step_failure is None:
                self._step_failure = update.failure
            update = replace(update, failure=None)
            if self.state.quality is E.SystemTimeQuality.NETWORK_SYNCED:
                self.step_state = ChronyStepState.IDLE
                self._retry_delay = self.settings.retry_initial_us
                self.last_refresh_monotonic_us = None
                return self._finish_step_failure(update)
            self.next_status_poll_monotonic_us = checked_monotonic_deadline(
                finish,
                minimum_wait_monotonic_us(
                    self.settings.stable_poll_period_us,
                    rate_bound_ppm=self.policy.monotonic_elapsed_rate_bound_ppm,
                ),
            )
        elif self.step_state is ChronyStepState.RETRY_BACKOFF:
            if self.state.quality is E.SystemTimeQuality.NETWORK_SYNCED:
                self.step_state = ChronyStepState.IDLE
                self._retry_delay = self.settings.retry_initial_us
            else:
                self._enter_backoff()
        return update
