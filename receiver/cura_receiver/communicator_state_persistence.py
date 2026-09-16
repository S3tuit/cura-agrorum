"""Handwritten validation of durable communicator snapshots, without live policy."""

from __future__ import annotations

import hashlib
from dataclasses import dataclass

from .generated.receiver_entities_generated import (
    CommunicatorStateV1,
    decode_communicator_state_v1,
    encode_communicator_state_v1,
)
from .generated.receiver_enums_generated import DiagnosticOperation as Operation
from .persistence_control_values import (
    CommunicatorStateCondition as Condition,
    CommunicatorStateLoadResult as LoadResult,
    CommunicatorStateLoadStatus as LoadStatus,
    require_immutable_state,
)
from .sqlite_repository import (
    COMMUNICATOR_STATE_STORAGE_CLASSES,
    CommunicatorStateRow,
    SqliteRepository,
)

_I64_MIN = -(1 << 63)
_I64_MAX = (1 << 63) - 1
_U64_MAX = (1 << 64) - 1
COMMUNICATOR_STATE_BUCKET_CAPACITY = 64
AIRTIME_UTC_ERROR_CEILING_US = 40_000_000
_POLICY_FIELDS = (
    "rolling_window_us",
    "tx_airtime_budget_us",
    "bucket_width_us",
    "bucket_charge_limit_us",
    "bucket_expiration_guard_us",
)


def _integer(value: int, low: int = 0, high: int = _U64_MAX) -> int:
    if type(value) is not int or not low <= value <= high:
        raise ValueError("state integer is outside its canonical range")
    return value


def _policy_shape(value) -> None:
    for name in _POLICY_FIELDS:
        _integer(getattr(value, name))
    window, budget, width, limit, guard = (
        getattr(value, name) for name in _POLICY_FIELDS
    )
    if window == 0 or width == 0 or not 0 < limit <= budget:
        raise ValueError("invalid state policy durations or charge bounds")
    span = _integer(window + guard)
    grid_count = span // width + bool(span % width) + 1
    synthetic_count = budget // limit + bool(budget % limit)
    if max(grid_count, synthetic_count) > COMMUNICATOR_STATE_BUCKET_CAPACITY:
        raise ValueError("state policy exceeds fixed ledger capacity")
    oldest_offset = _integer((synthetic_count - 1) * width)
    if oldest_offset >= _integer(span + width):
        raise ValueError("synthetic recovery would contain expired buckets")


@dataclass(frozen=True, slots=True)
class CommunicatorStatePolicy:
    """Deployment inputs for validation; no clock, grant, or recovery-wait behavior."""

    rolling_window_us: int = 3_600_000_000
    tx_airtime_budget_us: int = 36_000_000
    bucket_width_us: int = 60_000_000
    bucket_charge_limit_us: int = 8_000_000
    bucket_expiration_guard_us: int = 120_000_000
    receiver_utc_error_budget_us: int = 40_000_000

    def __post_init__(self) -> None:
        _policy_shape(self)
        _integer(self.receiver_utc_error_budget_us, 1, AIRTIME_UTC_ERROR_CEILING_US)
        if self.bucket_expiration_guard_us < 2 * AIRTIME_UTC_ERROR_CEILING_US:
            raise ValueError("airtime guard does not cover both UTC correlations")


def validate_communicator_state(
    state: CommunicatorStateV1,
    repository: SqliteRepository,
    policy: CommunicatorStatePolicy,
    *,
    compare_policy: bool = True,
) -> bytes:
    """Return exact canonical bytes after validating a supplied immutable value.

    Trusted-time provenance of the snapshot and transmitter silence are caller
    invariants. This function neither samples time nor edits the requested state.
    """
    require_immutable_state(state)
    _integer(state.generation, 1, _I64_MAX)
    _integer(state.airtime_snapshot_utc_us, _I64_MIN, _I64_MAX)
    _policy_shape(state)
    provenance = state.rtc_provenance
    if provenance is not None:
        if len(provenance.verified_by_receiver_instance_id) != 16:
            raise ValueError("invalid provenance identity")
        if (
            repository.find_receiver_instance(
                provenance.verified_by_receiver_instance_id
            )
            is None
        ):
            raise ValueError("provenance references a missing lifecycle row")
        _integer(provenance.network_utc_at_verification_us, _I64_MIN, _I64_MAX)
        _integer(provenance.rtc_readback_utc_us, _I64_MIN, _I64_MAX)
        _integer(
            provenance.verification_uncertainty_us,
            0,
            policy.receiver_utc_error_budget_us - 1,
        )
        _integer(provenance.drift_bound_ppm, 1, 999_999)
    previous = None
    first = None
    empty_seen = False
    total = 0
    for bucket in state.buckets:
        charge = _integer(bucket.charged_airtime_us)
        expiration = _integer(bucket.expires_at_utc_us, _I64_MIN, _I64_MAX)
        if charge == 0:
            if expiration != 0:
                raise ValueError("empty bucket must be canonical zero")
            empty_seen = True
            continue
        if empty_seen or charge > state.bucket_charge_limit_us:
            raise ValueError("nonempty bucket follows empty or exceeds charge bound")
        if expiration <= state.airtime_snapshot_utc_us:
            raise ValueError("expired bucket is not a valid snapshot entry")
        _integer(expiration - state.rolling_window_us, _I64_MIN, _I64_MAX)
        _integer(
            expiration - state.rolling_window_us - state.bucket_expiration_guard_us,
            _I64_MIN,
            _I64_MAX,
        )
        if previous is not None:
            delta = _integer(expiration - previous, 1, _I64_MAX)
            if delta % state.bucket_width_us:
                raise ValueError("bucket expirations are not on one increasing grid")
        if first is None:
            first = expiration
        span = _integer(expiration - first, 0, _I64_MAX)
        if span // state.bucket_width_us >= COMMUNICATOR_STATE_BUCKET_CAPACITY:
            raise ValueError("ledger grid span exceeds fixed capacity")
        previous = expiration
        total = _integer(total + charge)
    if total > state.tx_airtime_budget_us:
        raise ValueError("ledger exceeds global airtime budget")
    # Generated grammar owns exact lengths, representation ranges and reserved bytes.
    blob = encode_communicator_state_v1(state)
    if compare_policy and any(
        getattr(state, name) != getattr(policy, name) for name in _POLICY_FIELDS
    ):
        raise ValueError("requested state differs from active deployment policy")
    return blob


def classify_communicator_state_rows(
    raw_rows: tuple[CommunicatorStateRow, ...],
    repository: SqliteRepository,
    policy: CommunicatorStatePolicy,
) -> LoadResult:
    """Apply the exclusive envelope/digest/version/structure/policy decision tree.

    The caller has already validated the database itself. Original SQL values
    remain in the database for explicit atomic archive-and-replace transactions.
    SQLite errors propagate to that caller's control transaction boundary.
    """

    def unavailable(condition):
        return LoadResult(
            LoadStatus.STATE_UNAVAILABLE, Operation.READ, state_condition=condition
        )

    if not raw_rows:
        return unavailable(Condition.MISSING)
    if (
        len(raw_rows) != 1
        or raw_rows[0].storage_classes != COMMUNICATOR_STATE_STORAGE_CLASSES
        or len(raw_rows[0].values) != 5
    ):
        return unavailable(Condition.CORRUPT)
    singleton, version, generation, blob, digest = raw_rows[0].values
    if (
        type(singleton) is not int
        or singleton != 1
        or type(version) is not int
        or not 0 <= version <= 65535
        or type(generation) is not int
        or not 1 <= generation <= _I64_MAX
        or type(blob) is not bytes
        or type(digest) is not bytes
        or len(digest) != 32
        or hashlib.sha256(blob).digest() != digest
        or len(blob) < 2
        or int.from_bytes(blob[:2], "little") != version
    ):
        return unavailable(Condition.CORRUPT)
    if version != 1:
        return unavailable(Condition.UNSUPPORTED_VERSION)
    try:
        state = decode_communicator_state_v1(blob)
        if state.generation != generation:
            return unavailable(Condition.CORRUPT)
        encoded = validate_communicator_state(
            state, repository, policy, compare_policy=False
        )
        if encoded != blob:
            return unavailable(Condition.CORRUPT)
    except (TypeError, ValueError, OverflowError):
        return unavailable(Condition.CORRUPT)
    if any(getattr(state, name) != getattr(policy, name) for name in _POLICY_FIELDS):
        return unavailable(Condition.POLICY_MISMATCH)
    return LoadResult(LoadStatus.LOADED, Operation.NONE, state=state)
