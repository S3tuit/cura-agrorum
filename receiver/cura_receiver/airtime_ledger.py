"""Fixed-charge monotonic airtime history; no persistence, radio or clock I/O."""

from dataclasses import dataclass

from .communicator_state_persistence import (
    AIRTIME_TX_COMPLETION_US,
    AIRTIME_UTC_ERROR_CEILING_US,
    COMMUNICATOR_STATE_ENTRY_CAPACITY as CAPACITY,
    CommunicatorStatePolicy,
)
from .elapsed_duration import (
    MONOTONIC_ELAPSED_RATE_BOUND_PPM,
    checked_correlated_utc,
    checked_duration_us,
    checked_monotonic_deadline,
    checked_monotonic_elapsed,
    minimum_wait_monotonic_us,
    maximum_physical_duration_us,
    rate_growth_us,
)
from .generated.receiver_entities_generated import AirtimeSnapshotV1, AirtimeEntryV2
from .time_observations import TrustedTimeSample


@dataclass(frozen=True, slots=True)
class AirtimeCorrelation:
    """Optional trusted UTC evidence, never an authorization to spend airtime."""

    sample: TrustedTimeSample
    clock_state_generation: int
    valid_until_monotonic_us: int

    def __post_init__(self):
        if type(self.sample) is not TrustedTimeSample:
            raise TypeError("airtime requires a production trusted-time sample")
        checked_duration_us(self.clock_state_generation)
        checked_duration_us(self.valid_until_monotonic_us)

    def evidence_at(self, now_monotonic_us, *, policy, rate_bound_ppm):
        elapsed = checked_monotonic_elapsed(self.sample.monotonic_us, now_monotonic_us)
        if (self.sample.generation != self.clock_state_generation
                or now_monotonic_us >= self.valid_until_monotonic_us):
            raise ValueError("airtime correlation is no longer current")
        error = checked_monotonic_deadline(
            self.sample.error_bound_us, rate_growth_us(rate_bound_ppm, elapsed)
        )
        if error >= min(AIRTIME_UTC_ERROR_CEILING_US, policy.receiver_utc_error_budget_us):
            raise ValueError("airtime UTC error bound is exhausted")
        return AirtimeSnapshotV1(
            checked_correlated_utc(self.sample.utc_us, self.sample.monotonic_us,
                                   now_monotonic_us), error
        )

    def utc_at(self, now_monotonic_us, *, policy, rate_bound_ppm):
        return self.evidence_at(now_monotonic_us, policy=policy,
                                rate_bound_ppm=rate_bound_ppm).utc_us


class AirtimeLedger:
    """18 reservations, one open group and one unsaved-usage counter.

    Deadlines are process-local. Saved entries are physical lifetimes measured
    at one correlation. Neither persistence acknowledgement nor UTC can rebase
    a live deadline; only a TX or startup recovery extends a reservation.
    """

    def __init__(self, policy, entries=None, *, monotonic_us,
                 elapsed_us=0, rate_bound_ppm=MONOTONIC_ELAPSED_RATE_BOUND_PPM):
        if type(policy) is not CommunicatorStatePolicy:
            raise TypeError("a validated airtime policy is required")
        self.policy = policy
        self._now = checked_duration_us(monotonic_us)
        self._rate = rate_bound_ppm
        self.hold = minimum_wait_monotonic_us(checked_monotonic_deadline(
            policy.rolling_window_us, AIRTIME_TX_COMPLETION_US), rate_bound_ppm=rate_bound_ppm)
        elapsed_us = checked_duration_us(elapsed_us)
        if entries is None:
            entries = (AirtimeEntryV2(0),) * CAPACITY
        if type(entries) is not tuple or len(entries) != CAPACITY:
            raise ValueError("ledger requires exactly 18 immutable entries")
        for entry in entries:
            if type(entry) is not AirtimeEntryV2:
                raise TypeError("ledger entry has a foreign type")
            checked_duration_us(entry.remaining_us)
        self.deadlines = []
        for entry in entries:
            left = max(0, entry.remaining_us - elapsed_us)
            self.deadlines.append(checked_monotonic_deadline(monotonic_us,
                minimum_wait_monotonic_us(left, rate_bound_ppm=rate_bound_ppm)) if left else 0)
        self.current_entry = None
        self.used_since_save_us = 0

    @property
    def total_used(self):
        return sum(d > self._now for d in self.deadlines) * self.policy.entry_charge_us

    @property
    def available_charge_us(self):
        if self.current_entry is None and all(self.deadlines):
            return 0
        return self.policy.entry_charge_us - self.used_since_save_us

    def advance(self, now_monotonic_us):
        checked_monotonic_elapsed(self._now, now_monotonic_us)
        self._now = now_monotonic_us
        for index, deadline in enumerate(self.deadlines):
            if deadline <= now_monotonic_us:
                self.deadlines[index] = 0
                if self.current_entry == index:
                    self.current_entry = None

    def reserve(self, charge_us, now_monotonic_us):
        charge_us = checked_duration_us(charge_us)
        if not charge_us or charge_us > self.policy.entry_charge_us:
            raise ValueError("packet charge must fit in one entry")
        self.advance(now_monotonic_us)
        if charge_us > self.available_charge_us:
            return False
        deadline = checked_monotonic_deadline(now_monotonic_us, self.hold)
        index = self.current_entry
        if index is None:
            index = self.deadlines.index(0)
        self.deadlines[index] = max(self.deadlines[index], deadline)
        self.current_entry = index
        self.used_since_save_us += charge_us
        return True

    def refund(self, charge_us):
        charge_us = checked_duration_us(charge_us)
        if charge_us > self.used_since_save_us:
            raise ValueError("refund exceeds unsaved usage")
        self.used_since_save_us -= charge_us
        # Keep the reservation even if its first attempted TX never started.

    def confirm_coverage(self, covered_us):
        covered_us = checked_duration_us(covered_us)
        if covered_us > self.used_since_save_us:
            raise ValueError("confirmed coverage exceeds unsaved usage")
        self.used_since_save_us -= covered_us
        if covered_us or not self.used_since_save_us:
            # Full coverage closes even a zero-use group left by a refund.
            self.current_entry = None

    def add_recovery(self):
        deadline = checked_monotonic_deadline(self._now, self.hold)
        index = (self.deadlines.index(0) if 0 in self.deadlines else
                 min(range(CAPACITY), key=self.deadlines.__getitem__))
        self.deadlines[index] = max(self.deadlines[index], deadline)

    def exhaust(self):
        deadline = checked_monotonic_deadline(self._now, self.hold)
        self.deadlines = [deadline] * CAPACITY
        self.current_entry = None

    def snapshot_entries(self, now_monotonic_us):
        self.advance(now_monotonic_us)
        return tuple(AirtimeEntryV2(maximum_physical_duration_us(
            max(0, deadline - now_monotonic_us), rate_bound_ppm=self._rate))
            for deadline in self.deadlines)
