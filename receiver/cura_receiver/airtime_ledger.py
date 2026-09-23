"""Positional monotonic airtime history; no persistence, radio or clock I/O."""

from collections import deque
from dataclasses import dataclass

from .communicator_state_persistence import (
    AIRTIME_TX_COMPLETION_US,
    AIRTIME_UTC_ERROR_CEILING_US,
    COMMUNICATOR_STATE_BUCKET_CAPACITY as CAPACITY,
    CommunicatorStatePolicy,
)
from .elapsed_duration import (
    MONOTONIC_ELAPSED_RATE_BOUND_PPM,
    checked_correlated_utc,
    checked_duration_us,
    checked_monotonic_deadline,
    checked_monotonic_elapsed,
    minimum_wait_monotonic_us,
    maximum_lifetime_monotonic_us,
    rate_growth_us,
)
from .generated.receiver_entities_generated import AirtimeSnapshotV1, TxAirtimeBucketV1
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


@dataclass(frozen=True, slots=True)
class _LiveBucket:
    charge: int
    retention_deadline: int


class AirtimeLedger:
    """Fixed circular history with immutable deadlines and cached charge.

    A bucket identity is its current-process virtual interval start, possibly
    negative for reconstructed history near boot. It is never serialized.
    Admission does not iterate the ring; advance only retires elapsed heads.
    """

    def __init__(self, policy, buckets, *, monotonic_us, elapsed_us=0,
                 rate_bound_ppm=MONOTONIC_ELAPSED_RATE_BOUND_PPM):
        if type(policy) is not CommunicatorStatePolicy:
            raise TypeError("a validated airtime policy is required")
        if type(buckets) is not tuple or len(buckets) != CAPACITY:
            raise ValueError("ledger requires exactly 62 immutable positional buckets")
        self.policy = policy
        self._now = checked_duration_us(monotonic_us)
        checked_duration_us(elapsed_us)
        self._rate = rate_bound_ppm
        self.spacing = self._wait(policy.bucket_width_us)
        self._lifetime = maximum_lifetime_monotonic_us(
            policy.bucket_width_us, rate_bound_ppm=rate_bound_ppm)
        self._horizon = checked_monotonic_deadline(
            checked_monotonic_deadline(policy.rolling_window_us, policy.bucket_width_us),
            AIRTIME_TX_COMPLETION_US)
        whole, phase = divmod(elapsed_us, policy.bucket_width_us)
        self.current_start = monotonic_us - self._wait(phase)
        self._ring = [None] * CAPACITY
        self._head = 0
        self._active = deque()
        self._total = 0
        # Validate the entire snapshot before pruning can hide bad historical charges.
        total = 0
        for bucket in buckets:
            if type(bucket) is not TxAirtimeBucketV1:
                raise TypeError("ledger entry has a foreign type")
            charge = checked_duration_us(bucket.charged_airtime_us)
            if charge > policy.bucket_charge_limit_us:
                raise ValueError("loaded charge exceeds bucket limit")
            total = checked_monotonic_deadline(total, charge)
        if total > policy.tx_airtime_budget_us:
            raise ValueError("loaded charge exceeds global budget")
        for position, bucket in enumerate(buckets):
            distance = CAPACITY - 1 - position
            remaining = self._horizon - distance * policy.bucket_width_us - elapsed_us
            if not bucket.charged_airtime_us or remaining <= 0:
                continue
            age = distance + whole
            if age >= CAPACITY:
                raise ValueError("retained history exceeds positional capacity")
            start = self.current_start - age * self.spacing
            deadline = checked_monotonic_deadline(monotonic_us, self._wait(remaining))
            self._ring[self._index(start)] = _LiveBucket(bucket.charged_airtime_us, deadline)
            self._active.append(start)
            self._total += bucket.charged_airtime_us

    def _wait(self, duration):
        return minimum_wait_monotonic_us(duration, rate_bound_ppm=self._rate)

    @property
    def total_used(self):
        return self._total

    @property
    def grant_deadline(self):
        # A reconstructed, already closed interval can have a negative deadline.
        return max(0, self.current_start + self._lifetime)

    def _index(self, start):
        if type(start) is not int:
            raise TypeError("bucket identity must be an integer")
        distance, remainder = divmod(self.current_start - start, self.spacing)
        if remainder or not 0 <= distance < CAPACITY:
            raise ValueError("bucket is outside the represented monotonic grid")
        return (self._head + CAPACITY - 1 - distance) % CAPACITY

    def charge_at(self, start):
        # A previously held grant can have aged completely out of the ring.
        if start < self.current_start - (CAPACITY - 1) * self.spacing:
            return 0
        bucket = self._ring[self._index(start)]
        return 0 if bucket is None else bucket.charge

    def retention_deadline(self, start):
        bucket = self._ring[self._index(start)]
        if bucket is None:
            raise ValueError("unopened bucket has no retention deadline")
        return bucket.retention_deadline

    def set_charge(self, start, charge):
        checked_duration_us(charge)
        if charge > self.policy.bucket_charge_limit_us:
            raise ValueError("charge exceeds bucket limit")
        index = self._index(start)
        old = self._ring[index]
        total = checked_monotonic_deadline(self._total - (old.charge if old else 0), charge)
        if total > self.policy.tx_airtime_budget_us:
            raise ValueError("charge exceeds global budget")
        if old is None:
            if not charge:
                return
            if start != self.current_start:
                raise ValueError("only the current interval can acquire new charge")
            deadline = start + self._wait(self._horizon)
            checked_duration_us(deadline)
            if deadline <= self._now:
                raise ValueError("cannot reopen an expired interval")
            self._active.append(start)
        else:
            deadline = old.retention_deadline
        self._ring[index] = _LiveBucket(charge, deadline)
        self._total = total

    def advance(self, now_monotonic_us):
        checked_monotonic_elapsed(self._now, now_monotonic_us)
        self._now = now_monotonic_us
        while self._active:
            start = self._active[0]
            index = self._index(start)
            bucket = self._ring[index]
            if now_monotonic_us < bucket.retention_deadline:
                break
            self._total -= bucket.charge
            self._ring[index] = None
            self._active.popleft()
        steps = (now_monotonic_us - self.current_start) // self.spacing
        if steps >= CAPACITY:
            if self._active:
                raise ValueError("unexpired history would be overwritten")
            self._ring = [None] * CAPACITY
            self._head = 0
        elif steps:
            for offset in range(steps):
                if self._ring[(self._head + offset) % CAPACITY] is not None:
                    raise ValueError("unexpired history would be overwritten")
            self._head = (self._head + steps) % CAPACITY
        self.current_start += steps * self.spacing

    def snapshot_buckets(self):
        return tuple(TxAirtimeBucketV1(0 if bucket is None else bucket.charge)
                     for index in range(CAPACITY)
                     for bucket in (self._ring[(self._head + index) % CAPACITY],))

    def copy(self):
        clone = object.__new__(type(self))
        clone.__dict__.update(self.__dict__)
        clone._ring = self._ring.copy()
        clone._active = self._active.copy()
        return clone
