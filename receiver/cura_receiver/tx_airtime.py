"""Receiver fixed-charge airtime recovery; radio execution stays outside."""
from dataclasses import dataclass
from enum import Enum, auto

from .generated import receiver_enums_generated as E
from .airtime_ledger import AirtimeCorrelation, AirtimeLedger
from .communicator_state_owner import StateCommitResolution as Resolution
from .communicator_state_persistence import CommunicatorStatePolicy, AIRTIME_TX_COMPLETION_US
from .elapsed_duration import (
    MONOTONIC_ELAPSED_RATE_BOUND_PPM, checked_monotonic_deadline,
    checked_monotonic_elapsed, checked_utc_difference, minimum_wait_monotonic_us,
    maximum_lifetime_monotonic_us,
)
from .generated.receiver_entities_generated import CommunicatorStateV2
from .persistence_control_values import (
    CommunicatorStateCommitResult, CommunicatorStateLoadResult,
    CommunicatorStateCondition as Condition, CommunicatorStateLoadStatus as LS,
)

ACK_CHARGED_AIRTIME_US = 67_866


class AirtimeReason(Enum):
    STATE_READY = auto()
    STATE_UNAVAILABLE = auto()
    RECOVERY_WAIT = auto()
    PERSISTENCE_PENDING = auto()
    PERSISTENCE_FAILED = auto()
    RECONCILIATION_CONFLICT = auto()
    INVALID_STATE = auto()
    SAVE_REQUIRED = auto()
    BUDGET_EXHAUSTED = auto()
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
    submission_deadline_monotonic_us: int | None = None


@dataclass(slots=True)
class _PreparedSave:
    requested: CommunicatorStateV2
    covered_us: int = 0
    tokens: frozenset = frozenset()
    receipt: object = None
    recovery_ledger: AirtimeLedger | None = None


class TxAirtimePolicy:
    """One process's policy; shares the exact complete-state owner with RTC."""

    def __init__(self, *, state_owner, clock, policy=None,
                 rate_bound_ppm=MONOTONIC_ELAPSED_RATE_BOUND_PPM,
                 longest_packet_charge_us=ACK_CHARGED_AIRTIME_US):
        self.owner, self.clock = state_owner, clock
        self.policy = policy if policy is not None else CommunicatorStatePolicy()
        if type(self.policy) is not CommunicatorStatePolicy:
            raise TypeError("airtime requires validated policy values")
        if (type(longest_packet_charge_us) is not int or
                not 0 < longest_packet_charge_us <= self.policy.entry_charge_us):
            raise ValueError("longest packet must fit in one entry")
        minimum_wait_monotonic_us(0, rate_bound_ppm=rate_bound_ppm)
        self.rate, self.longest_packet_charge_us = rate_bound_ppm, longest_packet_charge_us
        self._correlation = None
        self._rtc_health = E.RtcHealth.MISSING
        self._ledger = self._known_state = self._prepared = None
        self._disabled_since = None
        self._spends = {}
        self._required = False

    @property
    def state(self):
        return self.owner.state

    @property
    def total_used(self):
        self._finish_save()
        if self._ledger is None or self.owner.pending is not None:
            return None
        self._ledger.advance(self.clock.now_monotonic_us())
        return self._ledger.total_used

    @property
    def group_outstanding(self):
        self._finish_save()
        return self._ledger is not None and self._ledger.current_entry is not None

    @property
    def used_since_save_us(self):
        self._finish_save()
        return 0 if self._ledger is None else self._ledger.used_since_save_us

    def update_time(self, correlation, *, rtc_health):
        if correlation is not None and type(correlation) is not AirtimeCorrelation:
            raise TypeError("time handoff must be an AirtimeCorrelation or None")
        if type(rtc_health) is not E.RtcHealth:
            raise TypeError("current RTC health must be supplied explicitly")
        self._correlation, self._rtc_health = correlation, rtc_health

    def _snapshot_time(self, now):
        if self._correlation is None:
            return None
        try:
            return self._correlation.evidence_at(now, policy=self.policy, rate_bound_ppm=self.rate)
        except (TypeError, ValueError, OverflowError):
            return None

    def _quality(self, snapshot):
        return E.SystemTimeQuality.UNTRUSTED if snapshot is None else self._correlation.sample.quality

    def confirm_transmitter_disabled(self):
        """Caller assertion of physical inability, required by incompatible recovery."""
        if self._disabled_since is None:
            self._disabled_since = self.clock.now_monotonic_us()

    def _state_value(self, ledger, now, evidence, previous, *, provenance=None):
        if previous is not None and previous.generation >= (1 << 63)-1:
            raise OverflowError("state generation exhausted")
        return CommunicatorStateV2(
            generation=1 if previous is None else previous.generation + 1,
            last_observed_system_time_quality=self._quality(evidence),
            last_observed_rtc_health=self._rtc_health,
            rtc_provenance=provenance,
            rolling_window_us=self.policy.rolling_window_us,
            tx_airtime_budget_us=self.policy.tx_airtime_budget_us,
            entry_charge_us=self.policy.entry_charge_us,
            airtime_snapshot=evidence, entries=ledger.snapshot_entries(now))

    def _recover_ledger(self, state, evidence, now):
        ledger = AirtimeLedger(self.policy, monotonic_us=now, rate_bound_ppm=self.rate)
        usable = (state is not None and state.airtime_snapshot is not None and evidence is not None
                  and state.last_observed_system_time_quality in
                  (E.SystemTimeQuality.CHRONY_SYNCED, E.SystemTimeQuality.RTC_HOLDOVER))
        if usable:
            try:
                for name in ('rolling_window_us', 'tx_airtime_budget_us', 'entry_charge_us'):
                    if getattr(state, name) != getattr(self.policy, name):
                        raise ValueError("loaded policy differs from active policy")
                delta = checked_utc_difference(evidence.utc_us, state.airtime_snapshot.utc_us)
                error = checked_monotonic_deadline(evidence.error_bound_us,
                                                   state.airtime_snapshot.error_bound_us)
                if delta + error < 0:
                    raise ValueError("impossible negative elapsed time")
                ledger = AirtimeLedger(self.policy, state.entries, monotonic_us=now,
                    elapsed_us=max(0, delta-error), rate_bound_ppm=self.rate)
                ledger.add_recovery()
                return ledger
            except (TypeError, ValueError, OverflowError):
                pass
        ledger.exhaust()
        return ledger

    def _finish_save(self):
        prepared = self._prepared
        if prepared is None or prepared.receipt is None:
            return None
        resolution = prepared.receipt.resolution
        if resolution is Resolution.PENDING:
            return None
        if resolution is Resolution.INSTALLED:
            if prepared.recovery_ledger is not None:
                self._ledger = prepared.recovery_ledger
            else:
                self._ledger.confirm_coverage(prepared.covered_us)
                for token in prepared.tokens:
                    self._spends.pop(token, None)
            self._known_state = prepared.requested
            self._prepared = None
            self._required = self._threshold()
            return True
        # Keep a rejected startup snapshot frozen at its original correlation.
        if prepared.recovery_ledger is not None:
            prepared.receipt = None
        else:
            self._prepared = None
        return False

    def _threshold(self):
        return (self._ledger is not None and self._ledger.used_since_save_us > 0 and
                self._ledger.used_since_save_us + self.longest_packet_charge_us >=
                self.policy.entry_charge_us)

    def _commit_prepared(self, deadline, purpose):
        self._prepared.receipt = self.owner.commit(self._prepared.requested,
            deadline_monotonic_us=deadline, purpose=purpose)
        result = self._prepared.receipt.commit_result
        completed = self._finish_save()
        reason = (AirtimeReason.STATE_READY if completed else
                  AirtimeReason.PERSISTENCE_FAILED if completed is False else
                  AirtimeReason.PERSISTENCE_PENDING)
        return AirtimeUpdate(reason, result)

    def recover(self, *, deadline_monotonic_us):
        if self.owner.pending is not None:
            return self.reconcile(deadline_monotonic_us=deadline_monotonic_us)
        self._finish_save()
        if self._ledger is not None:
            return AirtimeUpdate(AirtimeReason.STATE_READY)
        if self._prepared is not None:
            return self._commit_prepared(deadline_monotonic_us, E.PersistenceControlPurpose.AIRTIME_HISTORY_RECOVERY)
        previous, condition = self.owner.state, self.owner.condition
        if previous is None and condition is Condition.NONE:
            return AirtimeUpdate(AirtimeReason.STATE_UNAVAILABLE)
        now = self.clock.now_monotonic_us()
        incompatible = condition in (Condition.UNSUPPORTED_VERSION, Condition.POLICY_MISMATCH)
        try:
            if incompatible:
                if self._disabled_since is None or checked_monotonic_elapsed(self._disabled_since, now) < minimum_wait_monotonic_us(self.policy.rolling_window_us, rate_bound_ppm=self.rate):
                    return AirtimeUpdate(AirtimeReason.RECOVERY_WAIT)
            evidence = self._snapshot_time(now)
            ledger = (AirtimeLedger(self.policy, monotonic_us=now, rate_bound_ppm=self.rate)
                      if incompatible or self.owner.commissioning_pending
                      else self._recover_ledger(previous, evidence, now))
            requested = self._state_value(ledger, now, evidence, previous,
                                         provenance=None if previous is None else previous.rtc_provenance)
            self._prepared = _PreparedSave(requested, recovery_ledger=ledger)
        except (TypeError, ValueError, OverflowError):
            return AirtimeUpdate(AirtimeReason.INVALID_STATE)
        return self._commit_prepared(deadline_monotonic_us, E.PersistenceControlPurpose.AIRTIME_HISTORY_RECOVERY)

    def reconcile(self, *, deadline_monotonic_us):
        loaded = self.owner.reconcile(deadline_monotonic_us=deadline_monotonic_us)
        if self.owner.pending is not None:
            if self.owner.reconciliation_conflict:
                return AirtimeUpdate(AirtimeReason.RECONCILIATION_CONFLICT, load_result=loaded)
            if (self._prepared is not None and self._prepared.recovery_ledger is not None
                    and self.owner.pending.preceding is None and loaded is not None
                    and loaded.status is LS.STATE_UNAVAILABLE
                    and loaded.state_condition is self.owner.pending.preceding_condition):
                self.owner.retry_pending_recovery(deadline_monotonic_us=deadline_monotonic_us)
            if self.owner.pending is not None:
                return AirtimeUpdate(AirtimeReason.PERSISTENCE_PENDING, load_result=loaded)
        completed = self._finish_save()
        return AirtimeUpdate(AirtimeReason.PERSISTENCE_FAILED if completed is False
                             else AirtimeReason.STATE_READY, load_result=loaded)

    @property
    def maintenance_required(self):
        self._finish_save()
        return self._ledger is None or self._required or self.owner.pending is not None

    @property
    def save_required(self):
        self._finish_save()
        return self._required

    def _reason(self):
        self._finish_save()
        if self.owner.pending is not None:
            return (AirtimeReason.RECONCILIATION_CONFLICT if self.owner.reconciliation_conflict
                    else AirtimeReason.PERSISTENCE_PENDING)
        if self._prepared is not None:
            return AirtimeReason.PERSISTENCE_PENDING
        if self._ledger is None:
            return AirtimeReason.STATE_UNAVAILABLE
        if self.owner.state is not self._known_state:
            return AirtimeReason.INVALID_STATE
        if self._required:
            return AirtimeReason.SAVE_REQUIRED
        try:
            self._ledger.advance(self.clock.now_monotonic_us())
        except (ValueError, OverflowError):
            return AirtimeReason.INVALID_STATE
        return (AirtimeReason.ALLOWED if self._ledger.available_charge_us else
                AirtimeReason.BUDGET_EXHAUSTED)

    @property
    def available_charge_us(self):
        return self._ledger.available_charge_us if self._reason() is AirtimeReason.ALLOWED else 0

    def try_spend(self, charge_us=ACK_CHARGED_AIRTIME_US):
        """Reserve and extend before radio execution; no persistence in this path."""
        if type(charge_us) is not int or not 0 < charge_us <= self.longest_packet_charge_us:
            raise ValueError("packet exceeds configured longest receiver charge")
        reason = self._reason()
        if reason is not AirtimeReason.ALLOWED:
            return AirtimeSpend(reason)
        now = self.clock.now_monotonic_us()
        try:
            deadline = checked_monotonic_deadline(now,
                maximum_lifetime_monotonic_us(max(0, AIRTIME_TX_COMPLETION_US - charge_us), rate_bound_ppm=self.rate))
            if not self._ledger.reserve(charge_us, now):
                return AirtimeSpend(AirtimeReason.BUDGET_EXHAUSTED)
        except (ValueError, OverflowError):
            return AirtimeSpend(AirtimeReason.INVALID_STATE)
        token = object()
        self._spends[token] = charge_us
        self._required = self._threshold()
        self._disabled_since = None
        return AirtimeSpend(AirtimeReason.ALLOWED, token, deadline)

    def report_tx(self, token, certainty):
        if type(certainty) is not TxCertainty:
            raise TypeError("radio certainty must use the policy certainty enum")
        if type(token) is not object or token not in self._spends:
            raise ValueError("foreign, saved or already resolved spend token")
        charge = self._spends.pop(token)
        if certainty is TxCertainty.NOT_STARTED:
            self._ledger.refund(charge)
            self._required = self._threshold()

    def _prepare_save(self, now, evidence, *, provenance):
        if self.owner.state is not self._known_state or self._known_state.generation >= (1 << 63)-1:
            raise ValueError("unknown state or exhausted generation")
        requested = self._state_value(self._ledger, now, evidence, self.owner.state,
                                      provenance=provenance)
        self._prepared = _PreparedSave(requested, self._ledger.used_since_save_us,
                                      frozenset(self._spends))
        return requested

    def snapshot(self, *, provenance, snapshot_monotonic_us, snapshot_utc_us, previous_state):
        """Prepare RTC's complete state at its supplied single correlation.

        The caller must call snapshot_receipt with the resulting exact receipt,
        or None if it abandons this prepared snapshot before submission.
        """
        self._finish_save()
        if (self._ledger is None or self._prepared is not None or self.owner.pending is not None
                or previous_state is not self.owner.state):
            return None
        evidence = self._snapshot_time(snapshot_monotonic_us)
        if evidence is None or evidence.utc_us != snapshot_utc_us:
            return None
        try:
            return self._prepare_save(snapshot_monotonic_us, evidence, provenance=provenance)
        except (TypeError, ValueError, OverflowError):
            return None

    def snapshot_receipt(self, receipt):
        prepared = self._prepared
        if prepared is None:
            # Re-delivery of a resolved receipt must never credit usage twice.
            return
        if receipt is None:
            if prepared.receipt is not None or self.owner.pending is not None:
                raise ValueError("cannot abandon a submitted complete-state request")
            self._prepared = None
            return
        if receipt.requested is not prepared.requested:
            raise ValueError("receipt does not cover the prepared airtime snapshot")
        prepared.receipt = receipt
        self._finish_save()

    def save(self, *, deadline_monotonic_us):
        if self.owner.pending is not None:
            return self.reconcile(deadline_monotonic_us=deadline_monotonic_us)
        self._finish_save()
        if self._ledger is None:
            return self.recover(deadline_monotonic_us=deadline_monotonic_us)
        if self._prepared is not None:
            return AirtimeUpdate(AirtimeReason.PERSISTENCE_PENDING)
        if not self._ledger.used_since_save_us:
            return AirtimeUpdate(AirtimeReason.STATE_READY)
        try:
            now = self.clock.now_monotonic_us()
            self._prepare_save(now, self._snapshot_time(now), provenance=self.owner.state.rtc_provenance)
        except (TypeError, ValueError, OverflowError):
            return AirtimeUpdate(AirtimeReason.INVALID_STATE)
        return self._commit_prepared(deadline_monotonic_us, E.PersistenceControlPurpose.AIRTIME_USAGE_SAVE)

    def maintain(self, *, deadline_monotonic_us):
        if self.owner.pending is not None:
            return self.reconcile(deadline_monotonic_us=deadline_monotonic_us)
        self._finish_save()
        if self._ledger is None:
            return self.recover(deadline_monotonic_us=deadline_monotonic_us)
        if self._required:
            return self.save(deadline_monotonic_us=deadline_monotonic_us)
        return AirtimeUpdate(self._reason())
