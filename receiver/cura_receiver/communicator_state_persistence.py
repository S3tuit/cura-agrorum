"""Handwritten validation of durable communicator snapshots, without live policy."""

from __future__ import annotations

import hashlib
from dataclasses import dataclass

from .generated.receiver_entities_generated import (
    CommunicatorStateV1,
    decode_communicator_state_v1,
    encode_communicator_state_v1,
)
from .generated.receiver_enums_generated import DiagnosticOperation as Operation, SystemTimeQuality
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
COMMUNICATOR_STATE_BUCKET_CAPACITY = 62
AIRTIME_TX_COMPLETION_US = 250_000
AIRTIME_UTC_ERROR_CEILING_US = 40_000_000
_POLICY_FIELDS = (
    "rolling_window_us",
    "tx_airtime_budget_us",
    "bucket_width_us",
    "bucket_charge_limit_us",
)


def _integer(value: int, low: int = 0, high: int = _U64_MAX) -> int:
    if type(value) is not int or not low <= value <= high:
        raise ValueError("state integer is outside its canonical range")
    return value


def _policy_shape(value) -> None:
    for name in _POLICY_FIELDS:
        _integer(getattr(value, name))
    window, budget, width, limit = (
        getattr(value, name) for name in _POLICY_FIELDS
    )
    if window == 0 or width == 0 or not 0 < limit <= budget:
        raise ValueError("invalid state policy durations or charge bounds")
    span = _integer(window + AIRTIME_TX_COMPLETION_US)
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
    receiver_utc_error_budget_us: int = 40_000_000

    def __post_init__(self) -> None:
        _policy_shape(self)
        _integer(self.receiver_utc_error_budget_us, 1, AIRTIME_UTC_ERROR_CEILING_US)


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
    snapshot = state.airtime_snapshot
    if snapshot is None:
        if state.last_observed_system_time_quality is not SystemTimeQuality.UNTRUSTED:
            raise ValueError("trusted snapshot quality requires UTC/error evidence")
    else:
        if state.last_observed_system_time_quality not in (
            SystemTimeQuality.NETWORK_SYNCED, SystemTimeQuality.RTC_HOLDOVER
        ):
            raise ValueError("snapshot UTC requires trusted historical quality")
        _integer(snapshot.utc_us, _I64_MIN, _I64_MAX)
        _integer(snapshot.error_bound_us, 0,
                 min(policy.receiver_utc_error_budget_us, AIRTIME_UTC_ERROR_CEILING_US) - 1)
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
    if len(state.buckets) != COMMUNICATOR_STATE_BUCKET_CAPACITY:
        raise ValueError("ledger must contain exactly 62 chronological slots")
    total = 0
    for bucket in state.buckets:
        charge = _integer(bucket.charged_airtime_us, 0, state.bucket_charge_limit_us)
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
