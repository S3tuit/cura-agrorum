"""Receiver airtime state transitions; radio execution and scheduling stay outside."""

from dataclasses import dataclass, replace
from enum import Enum, auto

from .generated import receiver_enums_generated as E
from .airtime_ledger import AirtimeCorrelation, AirtimeLedger
from .communicator_state_persistence import (
    COMMUNICATOR_STATE_BUCKET_CAPACITY as CAPACITY,
    CommunicatorStatePolicy,
)
from .elapsed_duration import (
    MONOTONIC_ELAPSED_RATE_BOUND_PPM,
    checked_monotonic_deadline,
    checked_monotonic_elapsed,
    checked_utc_difference,
    minimum_wait_monotonic_us,
)

from .generated.receiver_entities_generated import (
    CommunicatorStateV1,
    TxAirtimeBucketV1,
)
from .generated.receiver_enums_generated import RtcHealth
from .persistence_control_values import (
    CommunicatorStateCommitDisposition as CD,
    CommunicatorStateCommitResult,
    CommunicatorStateCondition as Condition,
    CommunicatorStateLoadResult,
    CommunicatorStateLoadStatus as LS,
)

# protocol/protocol-v2-lora/README.md: Pilot airtime constants, 61,696 us plus 10%.
ACK_CHARGED_AIRTIME_US = 67_866


class AirtimeReason(Enum):
    STATE_READY = auto()
    STATE_UNAVAILABLE = auto()
    RECOVERY_WAIT = auto()
    PERSISTENCE_PENDING = auto()
    PERSISTENCE_FAILED = auto()
    RECONCILIATION_CONFLICT = auto()
    INVALID_STATE = auto()
    GRANT_REQUIRED = auto()
    GRANT_EXPIRED = auto()
    BUDGET_EXHAUSTED = auto()
    CAPACITY_EXCEEDED = auto()
    ALLOWED = auto()


@dataclass(frozen=True, slots=True)
class AirtimeUpdate:
    reason: AirtimeReason
    commit_result: CommunicatorStateCommitResult | None = None
    load_result: CommunicatorStateLoadResult | None = None


class TxCertainty(Enum):
    NOT_STARTED = auto()
    STARTED = auto()
    UNCERTAIN = auto()


@dataclass(frozen=True, slots=True)
class AirtimeSpend:
    reason: AirtimeReason
    token: object | None = None
    bucket_expiration_utc_us: int | None = None
    grant_deadline_monotonic_us: int | None = None


@dataclass(slots=True)
class _BucketGrant:
    bucket_start: int
    baseline_us: int
    increment_us: int
    unspent_us: int
    generation: int
    deadline_monotonic_us: int


@dataclass(frozen=True, slots=True)
class _GrantTransition:
    requested: CommunicatorStateV1
    ledger: AirtimeLedger
    grant: _BucketGrant | None
    preceding_ledger: AirtimeLedger
    preceding_grant: _BucketGrant | None
    next_reason: AirtimeReason


def recovery_state(policy, *, snapshot, quality, rtc_health, synthetic):
    """Construct the specified positional recovery value, including without UTC."""
    charges = ()
    if synthetic:
        q, r = divmod(policy.tx_airtime_budget_us, policy.bucket_charge_limit_us)
        charges = ((r,) if r else ()) + (policy.bucket_charge_limit_us,) * q
        if len(charges) > CAPACITY:
            raise ValueError("synthetic ledger exceeds capacity")
    return CommunicatorStateV1(
        generation=1,
        last_observed_system_time_quality=quality,
        last_observed_rtc_health=rtc_health,
        rtc_provenance=None,
        rolling_window_us=policy.rolling_window_us,
        tx_airtime_budget_us=policy.tx_airtime_budget_us,
        bucket_width_us=policy.bucket_width_us,
        bucket_charge_limit_us=policy.bucket_charge_limit_us,
        airtime_snapshot=snapshot,
        buckets=(TxAirtimeBucketV1(0),) * (CAPACITY - len(charges)) +
                tuple(TxAirtimeBucketV1(charge) for charge in charges),
    )


class TxAirtimePolicy:
    """One process's policy, using the same complete-state owner as RuntimeTime."""

    def __init__(
        self,
        *,
        state_owner,
        clock,
        policy=None,
        rate_bound_ppm=MONOTONIC_ELAPSED_RATE_BOUND_PPM,
    ):
        self.owner, self.clock = state_owner, clock
        self.policy = policy if policy is not None else CommunicatorStatePolicy()
        if type(self.policy) is not CommunicatorStatePolicy:
            raise TypeError("airtime requires validated policy values")
        minimum_wait_monotonic_us(0, rate_bound_ppm=rate_bound_ppm)
        self.rate = rate_bound_ppm
        self._correlation = None
        self._rtc_health = RtcHealth.MISSING
        self._ledger = None
        self._known_state = None
        self._pending_requested = None
        self._disabled_since = None
        self._grant = None
        self._transition = None
        self._settlement_frozen = False
        self._spends = {}
        self._external_snapshot = None
        self._external_ledger = None

    @property
    def total_used(self):
        return (
            None
            if self._ledger is None or self.owner.pending is not None
            else self._ledger.total_used
        )

    @property
    def state(self):
        return self.owner.state

    @property
    def grant_outstanding(self):
        """Diagnostic ownership fact; an outstanding grant may be expired/frozen."""
        return self._grant is not None

    @property
    def available_charge_us(self):
        return (
            self._grant.unspent_us
            if self._grant_reason() is AirtimeReason.ALLOWED
            else 0
        )

    def _grant_reason(self):
        if self.owner.pending is not None:
            return (
                AirtimeReason.RECONCILIATION_CONFLICT
                if self.owner.reconciliation_conflict
                else AirtimeReason.PERSISTENCE_PENDING
            )
        self._sync_external_snapshot()
        if self.owner.state is None:
            return AirtimeReason.STATE_UNAVAILABLE
        if (
            self._settlement_frozen
            or self.owner.state is not self._known_state
        ):
            return AirtimeReason.GRANT_REQUIRED
        if self._grant is None:
            return AirtimeReason.GRANT_REQUIRED
        if self.clock.now_monotonic_us() >= self._grant.deadline_monotonic_us:
            return AirtimeReason.GRANT_EXPIRED
        if self._grant.unspent_us < ACK_CHARGED_AIRTIME_US:
            return AirtimeReason.BUDGET_EXHAUSTED
        return AirtimeReason.ALLOWED

    def _sync_external_snapshot(self):
        if (
            self._external_snapshot is not None
            and self.owner.state is self._external_snapshot
        ):
            self._known_state = self.owner.state
            self._ledger = self._external_ledger
            self._external_snapshot = self._external_ledger = (
                None
            )

    def snapshot(self, *, provenance, snapshot_monotonic_us, snapshot_utc_us,
                 previous_state):
        """RTC's complete-state callback preserves precharges and live deadlines."""
        self._sync_external_snapshot()
        if (self.owner.pending is not None or self._transition is not None
                or previous_state is not self.owner.state or previous_state is None):
            return None
        try:
            evidence = self._snapshot_time(snapshot_monotonic_us)
            if (evidence is None or evidence.utc_us != snapshot_utc_us
                    or previous_state.generation >= (1 << 63) - 1):
                return None
            if self._ledger is None:
                self._restore(previous_state, evidence, snapshot_monotonic_us)
            if previous_state is not self._known_state:
                return None
            ledger = self._ledger.copy()
            ledger.advance(snapshot_monotonic_us)
            requested = replace(
                previous_state, generation=previous_state.generation + 1,
                rtc_provenance=provenance, airtime_snapshot=evidence,
                last_observed_system_time_quality=self._quality(evidence),
                last_observed_rtc_health=self._rtc_health,
                buckets=ledger.snapshot_buckets())
        except (ValueError, OverflowError):
            return None
        self._external_snapshot, self._external_ledger = requested, ledger
        return requested

    def update_time(self, correlation, *, rtc_health):
        if correlation is not None and type(correlation) is not AirtimeCorrelation:
            raise TypeError("time handoff must be an AirtimeCorrelation or None")
        if type(rtc_health) is not RtcHealth:
            raise TypeError("current RTC health must be supplied explicitly")
        self._correlation, self._rtc_health = correlation, rtc_health

    def _snapshot_time(self, now_monotonic_us):
        if self._correlation is None:
            return None
        try:
            return self._correlation.evidence_at(
                now_monotonic_us, policy=self.policy, rate_bound_ppm=self.rate)
        except (TypeError, ValueError, OverflowError):
            return None

    def _quality(self, snapshot):
        return (E.SystemTimeQuality.UNTRUSTED if snapshot is None
                else self._correlation.sample.quality)

    def confirm_transmitter_disabled(self):
        """Caller assertion of physical inability; software policy cannot establish it."""
        if self._disabled_since is None:
            self._disabled_since = self.clock.now_monotonic_us()

    def _restore(self, state, snapshot, monotonic_us):
        for name in ("rolling_window_us", "tx_airtime_budget_us",
                     "bucket_width_us", "bucket_charge_limit_us"):
            if getattr(state, name) != getattr(self.policy, name):
                raise ValueError("loaded state does not match the active policy")
        elapsed = 0
        if state.airtime_snapshot is not None and snapshot is not None:
            try:
                elapsed = max(0, checked_utc_difference(
                    snapshot.utc_us, state.airtime_snapshot.utc_us)
                    - snapshot.error_bound_us - state.airtime_snapshot.error_bound_us)
            except OverflowError:
                # No representable correlation is unknown elapsed time, not empty history.
                elapsed = 0
        self._ledger = AirtimeLedger(
            self.policy, state.buckets, monotonic_us=monotonic_us,
            elapsed_us=elapsed, rate_bound_ppm=self.rate)
        self._known_state = state

    def _adopt_recovery_result(self, result, requested, *, loaded=None):
        if result.disposition in (CD.COMMITTED, CD.ALREADY_COMMITTED):
            self._pending_requested = None
            now = self.clock.now_monotonic_us()
            snapshot = self._snapshot_time(now)
            try:
                self._restore(requested, snapshot, now)
            except (ValueError, OverflowError):
                return AirtimeUpdate(AirtimeReason.INVALID_STATE, result, loaded)
            return AirtimeUpdate(AirtimeReason.STATE_READY, result, loaded)
        if result.disposition is CD.NOT_INSTALLED:
            self._pending_requested = None
            return AirtimeUpdate(AirtimeReason.PERSISTENCE_FAILED, result, loaded)
        return AirtimeUpdate(AirtimeReason.PERSISTENCE_PENDING, result, loaded)

    def reconcile(self, *, deadline_monotonic_us):
        loaded = self.owner.reconcile(deadline_monotonic_us=deadline_monotonic_us)
        if self.owner.pending is not None:
            if self.owner.reconciliation_conflict:
                return AirtimeUpdate(
                    AirtimeReason.RECONCILIATION_CONFLICT, load_result=loaded
                )
            if (
                self._pending_requested is not None
                and self.owner.pending.preceding is None
                and loaded is not None
                and loaded.status is LS.STATE_UNAVAILABLE
                and loaded.state_condition is self.owner.pending.preceding_condition
            ):
                requested = self._pending_requested
                result = self.owner.retry_pending_recovery(
                    deadline_monotonic_us=deadline_monotonic_us
                )
                return self._adopt_recovery_result(result, requested, loaded=loaded)
            return AirtimeUpdate(AirtimeReason.PERSISTENCE_PENDING, load_result=loaded)
        if self._transition is not None:
            return self._finish_transition(loaded=loaded)
        if self._grant is not None:
            return AirtimeUpdate(self._grant_reason(), load_result=loaded)
        self._pending_requested = None
        self._sync_external_snapshot()
        now = self.clock.now_monotonic_us()
        if self.owner.state is None:
            return AirtimeUpdate(AirtimeReason.STATE_UNAVAILABLE, load_result=loaded)
        try:
            if self._ledger is None:
                self._restore(self.owner.state, self._snapshot_time(now), now)
            elif self.owner.state is self._known_state:
                self._ledger.advance(now)
            else:
                return AirtimeUpdate(AirtimeReason.INVALID_STATE, load_result=loaded)
        except (ValueError, OverflowError):
            return AirtimeUpdate(AirtimeReason.INVALID_STATE, load_result=loaded)
        return AirtimeUpdate(AirtimeReason.STATE_READY, load_result=loaded)

    def recover(self, *, deadline_monotonic_us):
        """Establish durable history; this operation never grants spendable allowance."""
        if self.owner.pending is not None:
            return self.reconcile(deadline_monotonic_us=deadline_monotonic_us)
        if self._transition is not None:
            return self._finish_transition()
        self._sync_external_snapshot()
        if self._grant is not None:
            reason = self._grant_reason()
            return AirtimeUpdate(
                AirtimeReason.STATE_READY if reason is AirtimeReason.ALLOWED else reason
            )
        now = self.clock.now_monotonic_us()
        snapshot = self._snapshot_time(now)
        if self.owner.state is not None:
            try:
                if self._ledger is None:
                    self._restore(self.owner.state, snapshot, now)
                else:
                    self._ledger.advance(now)
            except (ValueError, OverflowError):
                return AirtimeUpdate(AirtimeReason.INVALID_STATE)
            return AirtimeUpdate(AirtimeReason.STATE_READY)
        condition = self.owner.condition
        if condition is Condition.NONE:
            return AirtimeUpdate(AirtimeReason.STATE_UNAVAILABLE)
        incompatible = condition in (
            Condition.UNSUPPORTED_VERSION,
            Condition.POLICY_MISMATCH,
        )
        try:
            if incompatible:
                if self._disabled_since is None:
                    return AirtimeUpdate(AirtimeReason.RECOVERY_WAIT)
                wait = minimum_wait_monotonic_us(
                    self.policy.rolling_window_us, rate_bound_ppm=self.rate
                )
                waited = checked_monotonic_elapsed(
                    self._disabled_since, self.clock.now_monotonic_us()
                )
                if waited < wait:
                    return AirtimeUpdate(AirtimeReason.RECOVERY_WAIT)
            requested = recovery_state(
                self.policy,
                snapshot=snapshot,
                quality=self._quality(snapshot),
                rtc_health=self._rtc_health,
                synthetic=not incompatible,
            )
        except (ValueError, OverflowError):
            return AirtimeUpdate(AirtimeReason.INVALID_STATE)
        self._pending_requested = requested
        result = self.owner.commit(
            requested, deadline_monotonic_us=deadline_monotonic_us,
            purpose=E.PersistenceControlPurpose.AIRTIME_HISTORY_RECOVERY
        )
        return self._adopt_recovery_result(result, requested)

    def _finish_transition(self, *, committed=None, loaded=None):
        transition = self._transition
        if self.owner.pending is not None:
            return AirtimeUpdate(AirtimeReason.PERSISTENCE_PENDING, committed, loaded)
        installed = self.owner.state is transition.requested
        if installed:
            self._ledger = transition.ledger
            self._grant = transition.grant
        else:
            self._ledger = transition.preceding_ledger
            self._grant = transition.preceding_grant
        self._known_state = self.owner.state
        self._transition = None
        self._pending_requested = None
        self._external_snapshot = self._external_ledger = None
        self._settlement_frozen = False
        reason = self._grant_reason()
        if installed and transition.grant is None:
            reason = transition.next_reason
        if committed is not None and committed.disposition is CD.NOT_INSTALLED:
            reason = AirtimeReason.PERSISTENCE_FAILED
        return AirtimeUpdate(reason, committed, loaded)

    def acquire_grant(self, *, deadline_monotonic_us):
        """Obtain allowance, settling an exhausted/frozen grant when necessary."""
        if self.owner.pending is not None:
            return self.reconcile(deadline_monotonic_us=deadline_monotonic_us)
        if self._transition is not None:
            return self._finish_transition()
        if self._grant_reason() is AirtimeReason.ALLOWED:
            return AirtimeUpdate(AirtimeReason.ALLOWED)
        if self._grant is not None:
            return self.settle(
                deadline_monotonic_us=deadline_monotonic_us, precharge=True
            )
        recovered = self.recover(deadline_monotonic_us=deadline_monotonic_us)
        if recovered.reason is not AirtimeReason.STATE_READY:
            return recovered
        return self._prepare_transition(
            deadline_monotonic_us=deadline_monotonic_us, precharge=True
        )

    def try_spend(self):
        """Tentatively charge one pilot ACK; the returned token precedes SetTx."""
        reason = self._grant_reason()
        if reason is not AirtimeReason.ALLOWED:
            return AirtimeSpend(reason)
        try:
            self._ledger.advance(self.clock.now_monotonic_us())
        except (ValueError, OverflowError):
            self._settlement_frozen = True
            return AirtimeSpend(AirtimeReason.INVALID_STATE)
        self._grant.unspent_us -= ACK_CHARGED_AIRTIME_US
        token = object()
        self._spends[token] = self._grant
        self._disabled_since = None
        return AirtimeSpend(
            AirtimeReason.ALLOWED,
            token,
            None,
            self._grant.deadline_monotonic_us,
        )

    def report_tx(self, token, certainty):
        """Resolve one issued token; missing reports remain charged at settlement."""
        if type(certainty) is not TxCertainty:
            raise TypeError("radio certainty must use the policy certainty enum")
        if type(token) is not object or token not in self._spends:
            raise ValueError("foreign, settled or already resolved spend token")
        grant = self._spends.pop(token)
        if certainty is TxCertainty.NOT_STARTED:
            grant.unspent_us += ACK_CHARGED_AIRTIME_US

    def settle(self, *, deadline_monotonic_us, precharge=True):
        """Freeze, commit exact possible use, and optionally precharge the current grid bucket."""
        if type(precharge) is not bool:
            raise TypeError("precharge must be Boolean")
        if self.owner.pending is not None:
            return self.reconcile(deadline_monotonic_us=deadline_monotonic_us)
        if self._transition is not None:
            return self._finish_transition()
        if self._grant is None:
            return (
                self.acquire_grant(deadline_monotonic_us=deadline_monotonic_us)
                if precharge
                else self.recover(deadline_monotonic_us=deadline_monotonic_us)
            )
        self._settlement_frozen = True
        # Freeze all unresolved outcomes as possible TX before constructing immutable bytes.
        self._spends.clear()
        return self._prepare_transition(
            deadline_monotonic_us=deadline_monotonic_us, precharge=precharge
        )

    def _prepare_transition(self, *, deadline_monotonic_us, precharge):
        self._sync_external_snapshot()
        now = self.clock.now_monotonic_us()
        evidence = self._snapshot_time(now)
        if self.owner.state is not self._known_state:
            return AirtimeUpdate(AirtimeReason.INVALID_STATE)
        try:
            if self.owner.state.generation >= (1 << 63) - 1:
                raise OverflowError("state generation exhausted")
            ledger = self._ledger.copy()
            ledger.advance(now)
            if self._grant is not None:
                old = self._grant
                baseline = ledger.charge_at(old.bucket_start)
                if baseline:
                    if baseline != old.baseline_us + old.increment_us:
                        raise ValueError("durable grant baseline changed outside airtime policy")
                    ledger.set_charge(old.bucket_start,
                        old.baseline_us + old.increment_us - old.unspent_us)
            grant = None
            next_reason = AirtimeReason.STATE_READY
            if precharge:
                if now >= ledger.grant_deadline:
                    next_reason = AirtimeReason.GRANT_EXPIRED
                else:
                    baseline = ledger.charge_at(ledger.current_start)
                    increment = min(self.policy.bucket_charge_limit_us - baseline,
                                    self.policy.tx_airtime_budget_us - ledger.total_used)
                    if increment < ACK_CHARGED_AIRTIME_US:
                        next_reason = AirtimeReason.BUDGET_EXHAUSTED
                    else:
                        ledger.set_charge(ledger.current_start, baseline + increment)
                        grant = _BucketGrant(ledger.current_start, baseline, increment,
                                             increment, self.owner.state.generation + 1,
                                             ledger.grant_deadline)
                        next_reason = AirtimeReason.ALLOWED
                if grant is None and self._grant is None:
                    return AirtimeUpdate(next_reason)
            requested = replace(self.owner.state,
                generation=self.owner.state.generation + 1,
                airtime_snapshot=evidence,
                last_observed_system_time_quality=self._quality(evidence),
                last_observed_rtc_health=self._rtc_health,
                buckets=ledger.snapshot_buckets())
        except (ValueError, OverflowError):
            return AirtimeUpdate(AirtimeReason.INVALID_STATE)
        self._transition = _GrantTransition(requested, ledger, grant, self._ledger,
                                             self._grant, next_reason)
        self._pending_requested = requested
        result = self.owner.commit(requested, deadline_monotonic_us=deadline_monotonic_us,
            purpose=(E.PersistenceControlPurpose.AIRTIME_BUCKET_GRANT if grant is not None
                     else E.PersistenceControlPurpose.AIRTIME_BUCKET_SETTLEMENT))
        return self._finish_transition(committed=result)
