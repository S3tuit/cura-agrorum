"""Bounded in-memory bucket accounting; no persistence, radio or clock I/O."""

from dataclasses import dataclass

from .communicator_state_persistence import (
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
    checked_utc_difference,
    checked_utc_offset,
    checked_utc_us,
    minimum_wait_monotonic_us,
    rate_growth_us,
)
from .generated.receiver_entities_generated import TxAirtimeBucketV1
from .time_observations import TrustedTimeSample


class SnapshotDeferred(ValueError):
    """Monotonic retention has not yet caught up with the snapshot's UTC."""


@dataclass(frozen=True, slots=True)
class AirtimeCorrelation:
    """A caller's live time handoff; the validity deadline includes source freshness."""

    sample: TrustedTimeSample
    clock_state_generation: int
    valid_until_monotonic_us: int

    def __post_init__(self):
        if type(self.sample) is not TrustedTimeSample:
            raise TypeError("airtime requires a production trusted-time sample")
        checked_duration_us(self.clock_state_generation)
        checked_duration_us(self.valid_until_monotonic_us)

    @property
    def offset_us(self):
        # Python's exact difference is a comparison key, not an encoded duration.
        return self.sample.utc_us - self.sample.monotonic_us

    def utc_at(self, now_monotonic_us, *, policy, rate_bound_ppm):
        elapsed = checked_monotonic_elapsed(self.sample.monotonic_us, now_monotonic_us)
        if (
            self.sample.generation != self.clock_state_generation
            or now_monotonic_us >= self.valid_until_monotonic_us
        ):
            raise ValueError("airtime correlation is no longer current")
        error = checked_monotonic_deadline(
            self.sample.error_bound_us, rate_growth_us(rate_bound_ppm, elapsed)
        )
        if error >= min(
            AIRTIME_UTC_ERROR_CEILING_US, policy.receiver_utc_error_budget_us
        ):
            raise ValueError("airtime UTC error bound is exhausted")
        return checked_correlated_utc(
            self.sample.utc_us, self.sample.monotonic_us, now_monotonic_us
        )


class AirtimeLedger:
    """One fixed grid with cached charge, reconstructed in the current clock domain.

    Iteration is confined to construction, snapshot preparation and copying.
    Admission reads total_used/charge_at and ages only elapsed head intervals.
    """

    def __init__(
        self,
        policy: CommunicatorStatePolicy,
        buckets,
        *,
        utc_us,
        monotonic_us,
        rate_bound_ppm=MONOTONIC_ELAPSED_RATE_BOUND_PPM
    ):
        if type(policy) is not CommunicatorStatePolicy:
            raise TypeError("a validated airtime policy is required")
        if type(buckets) is not tuple or len(buckets) != CAPACITY:
            raise ValueError("ledger requires exactly 64 immutable buckets")
        self.policy = policy
        self._utc = checked_utc_us(utc_us)
        self._monotonic = checked_duration_us(monotonic_us)
        self._now = self._monotonic
        self._rate = rate_bound_ppm
        minimum_wait_monotonic_us(0, rate_bound_ppm=rate_bound_ppm)
        self._charges = [0] * CAPACITY
        self._first = self._last = None
        self._head = 0
        self._total = 0
        previous = first = None
        empty_seen = False
        total = 0
        retained = []
        for bucket in buckets:
            if type(bucket) is not TxAirtimeBucketV1:
                raise TypeError("ledger entry has a foreign type")
            charge = checked_duration_us(bucket.charged_airtime_us)
            expiration = checked_utc_us(bucket.expires_at_utc_us)
            if charge == 0:
                if expiration != 0:
                    raise ValueError("empty bucket is not canonical")
                empty_seen = True
                continue
            if empty_seen or charge > policy.bucket_charge_limit_us:
                raise ValueError("invalid bucket order or charge")
            self.bucket_end(expiration)
            if previous is not None:
                delta = checked_utc_difference(expiration, previous)
                if delta <= 0 or delta % self.width:
                    raise ValueError(
                        "bucket expirations do not form one increasing grid"
                    )
            if first is None:
                first = expiration
            if checked_utc_difference(expiration, first) // self.width >= CAPACITY:
                raise ValueError("ledger grid span exceeds capacity")
            total = checked_monotonic_deadline(total, charge)
            previous = expiration
            if expiration > self._utc:
                retained.append(bucket)
        if total > policy.tx_airtime_budget_us:
            raise ValueError("loaded charge exceeds the global budget")
        for bucket in retained:
            self.set_charge(bucket.expires_at_utc_us, bucket.charged_airtime_us)

    @property
    def width(self):
        return self.policy.bucket_width_us

    @property
    def total_used(self):
        return self._total

    @property
    def empty(self):
        return self._first is None

    def bucket_end(self, expiration):
        return checked_utc_offset(
            checked_utc_offset(expiration, -self.policy.rolling_window_us),
            -self.policy.bucket_expiration_guard_us,
        )

    def expiration(self, end):
        return checked_utc_offset(
            checked_utc_offset(end, self.policy.rolling_window_us),
            self.policy.bucket_expiration_guard_us,
        )

    def retention_deadline(self, expiration):
        remaining = checked_utc_difference(expiration, self._utc)
        return checked_monotonic_deadline(
            self._monotonic,
            minimum_wait_monotonic_us(max(0, remaining), rate_bound_ppm=self._rate),
        )

    def current_bucket_end(self, utc_us):
        checked_utc_us(utc_us)
        if self.empty:
            return checked_utc_offset(utc_us, self.width)
        anchor = self.bucket_end(self._first)
        distance = checked_utc_difference(utc_us, anchor)
        # The remainder avoids an overflowing multiplication for distant grid indices.
        return checked_utc_offset(utc_us, self.width - distance % self.width)

    def _index(self, expiration):
        distance = checked_utc_difference(expiration, self._first)
        if distance % self.width:
            raise ValueError("expiration is off the retained grid")
        slot = distance // self.width
        if not 0 <= slot < CAPACITY:
            raise ValueError("logical ring index is outside capacity")
        return (self._head + slot) % CAPACITY

    def charge_at(self, expiration):
        checked_utc_us(expiration)
        if self.empty:
            return 0
        distance = checked_utc_difference(expiration, self._first)
        if distance % self.width:
            raise ValueError("expiration is off the retained grid")
        if expiration < self._first or expiration > self._last:
            return 0
        return self._charges[self._index(expiration)]

    def fits(self, expiration):
        checked_utc_us(expiration)
        if self.empty:
            return True
        if checked_utc_difference(expiration, self._first) % self.width:
            return False
        span = checked_utc_difference(
            max(expiration, self._last), min(expiration, self._first)
        )
        return span // self.width < CAPACITY

    def set_charge(self, expiration, charge):
        """Change a bucket during preparation/settlement, never at a radio command."""
        checked_utc_us(expiration)
        self.bucket_end(expiration)
        checked_duration_us(charge)
        if charge > self.policy.bucket_charge_limit_us:
            raise ValueError("charge exceeds the bucket limit")
        previous = self.charge_at(expiration)
        total = checked_monotonic_deadline(self._total - previous, charge)
        if total > self.policy.tx_airtime_budget_us:
            raise ValueError("charge exceeds the global budget")
        if not self.fits(expiration):
            raise ValueError("new bucket does not fit the ring")
        if charge and (
            expiration <= self._utc or self.retention_deadline(expiration) <= self._now
        ):
            raise ValueError("cannot insert an expired bucket")
        if self.empty:
            if not charge:
                return
            self._first = self._last = expiration
        elif expiration < self._first and charge:
            steps = checked_utc_difference(self._first, expiration) // self.width
            self._head = (self._head - steps) % CAPACITY
            self._first = expiration
        elif expiration > self._last and charge:
            self._last = expiration
        if self._first <= expiration <= self._last:
            self._charges[self._index(expiration)] = charge
        self._total = total
        self._trim_empty_edges()

    def _trim_empty_edges(self):
        if self._total == 0:
            self._first = self._last = None
            self._head = 0
            return
        while self._charges[self._head] == 0:
            self._first = checked_utc_offset(self._first, self.width)
            self._head = (self._head + 1) % CAPACITY
        while self._charges[self._index(self._last)] == 0:
            self._last = checked_utc_offset(self._last, -self.width)

    def advance(self, now_monotonic_us):
        checked_monotonic_elapsed(self._now, now_monotonic_us)
        self._now = now_monotonic_us
        if self.empty:
            return
        if now_monotonic_us >= self.retention_deadline(self._last):
            self._charges = [0] * CAPACITY
            self._first = self._last = None
            self._head = self._total = 0
            return
        while now_monotonic_us >= self.retention_deadline(self._first):
            self._total -= self._charges[self._head]
            self._charges[self._head] = 0
            self._head = (self._head + 1) % CAPACITY
            self._first = checked_utc_offset(self._first, self.width)
        self._trim_empty_edges()

    def entries(self):
        if self.empty:
            return ()
        count = checked_utc_difference(self._last, self._first) // self.width + 1
        result = []
        expiration = self._first
        for index in range(count):
            charge = self._charges[(self._head + index) % CAPACITY]
            if charge:
                result.append(TxAirtimeBucketV1(charge, expiration))
            if index + 1 < count:
                expiration = checked_utc_offset(expiration, self.width)
        return tuple(result)

    def snapshot_buckets(self, utc_us):
        checked_utc_us(utc_us)
        entries = self.entries()
        if any(b.expires_at_utc_us <= utc_us for b in entries):
            raise SnapshotDeferred("snapshot UTC passed a monotonically retained entry")
        return entries + (TxAirtimeBucketV1(0, 0),) * (CAPACITY - len(entries))

    def copy(self):
        clone = object.__new__(type(self))
        clone.__dict__.update(self.__dict__)
        clone._charges = self._charges.copy()
        return clone
