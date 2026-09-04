"""Bounded SPSC ownership handoff between receiver owner threads."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, unique
from threading import Event, Lock
from typing import Final

from .generated.receiver_enums_generated import (
    AdmissionResult,
    PersistenceAdmissionState,
    PersistQueueEntityKind,
)
from .persist_queue_entities import (
    SUPPORTED_PERSIST_QUEUE_ENTITY_SPECS,
    PersistQueueEntitySpec,
)


PERSIST_QUEUE_MAX_ENTITIES = 500
_EMPTY: Final = object()


class PersistQueueError(Exception):
    """Base class for queue errors outside operational admission results."""


class PersistQueueConfigurationError(PersistQueueError, ValueError):
    """A queue or batch bound is invalid for the pilot."""


class PersistQueueInterfaceError(PersistQueueError, RuntimeError):
    """A caller violated queue ownership or state-machine rules."""


@dataclass(frozen=True, slots=True)
class PersistenceAdmissionSnapshot:
    generation: int
    state: PersistenceAdmissionState
    changed_at_monotonic_us: int


@dataclass(frozen=True, slots=True)
class PersistQueueSnapshot:
    admission_snapshot: PersistenceAdmissionSnapshot
    capacity_entities: int
    reserved_entities: int
    published_entities: int
    claimed_entities: int
    closed: bool
    closed_and_drained: bool


class PersistQueueReservationToken:
    """Opaque identity used only to detect stale queue ownership."""

    __slots__ = ("_sequence",)

    def __init__(self, sequence: int) -> None:
        self._sequence = sequence

    def __copy__(self) -> PersistQueueReservationToken:
        raise TypeError("PersistQueue reservation tokens are not copyable")

    def __deepcopy__(self, memo: object) -> PersistQueueReservationToken:
        raise TypeError("PersistQueue reservation tokens are not copyable")

    def __repr__(self) -> str:
        return "<PersistQueueReservationToken>"


class PersistQueueReservation:
    """Single-use producer handle bound to one unpublished tail slot."""

    __slots__ = ("_active", "_queue", "_slot_index", "_spec", "_token")

    def __init__(
        self,
        queue: PersistQueue,
        token: PersistQueueReservationToken,
        spec: PersistQueueEntitySpec,
        slot_index: int,
    ) -> None:
        self._queue = queue
        self._token = token
        self._spec = spec
        self._slot_index = slot_index
        self._active = True

    @property
    def token(self) -> PersistQueueReservationToken:
        return self._token

    @property
    def spec(self) -> PersistQueueEntitySpec:
        return self._spec

    def publish(self, entity: object) -> None:
        self._queue._publish_reservation(self, entity)

    def cancel(self) -> None:
        self._queue._cancel_reservation(self)

    def __copy__(self) -> PersistQueueReservation:
        raise TypeError("PersistQueue reservations are not copyable")

    def __deepcopy__(self, memo: object) -> PersistQueueReservation:
        raise TypeError("PersistQueue reservations are not copyable")


@dataclass(frozen=True, slots=True)
class PersistQueueReserveResult:
    status: AdmissionResult
    reservation: PersistQueueReservation | None
    admission_snapshot: PersistenceAdmissionSnapshot
    used_entities_before: int
    capacity_entities: int


@dataclass(frozen=True, slots=True)
class PersistQueueEntryView:
    token: PersistQueueReservationToken
    spec: PersistQueueEntitySpec
    entity: object


@unique
class PersistQueueBatchDisposition(Enum):
    SQLITE_COMMITTED = 1
    QUARANTINED = 2


class PersistQueueBatchLease:
    """Single-use consumer lease over the current published FIFO prefix."""

    __slots__ = ("_active", "_entries", "_queue")

    def __init__(
        self,
        queue: PersistQueue,
        entries: tuple[PersistQueueEntryView, ...],
    ) -> None:
        self._queue = queue
        self._entries = entries
        self._active = True

    @property
    def entries(self) -> tuple[PersistQueueEntryView, ...]:
        return self._entries

    def acknowledge_durable(
        self,
        dispositions: tuple[PersistQueueBatchDisposition, ...],
    ) -> None:
        self._queue._acknowledge_lease(self, dispositions)

    def release_for_retry(self) -> None:
        self._queue._release_lease(self)

    def __enter__(self) -> PersistQueueBatchLease:
        if not self._active:
            raise PersistQueueInterfaceError("stale PersistQueue batch lease")
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> bool:
        if self._active:
            self.release_for_retry()
        return False

    def __copy__(self) -> PersistQueueBatchLease:
        raise TypeError("PersistQueue batch leases are not copyable")

    def __deepcopy__(self, memo: object) -> PersistQueueBatchLease:
        raise TypeError("PersistQueue batch leases are not copyable")


class PersistQueue:
    """Preallocated entity-count-bounded circular SPSC queue."""

    def __init__(
        self,
        capacity_entities: int = PERSIST_QUEUE_MAX_ENTITIES,
        *,
        wake_event: Event | None = None,
    ) -> None:
        if (
            type(capacity_entities) is not int
            or not 1 <= capacity_entities <= PERSIST_QUEUE_MAX_ENTITIES
        ):
            raise PersistQueueConfigurationError(
                "capacity_entities must be an integer in 1..500"
            )
        if wake_event is not None and not isinstance(wake_event, Event):
            raise PersistQueueConfigurationError(
                "wake_event must be a threading.Event"
            )

        self._capacity_entities = capacity_entities
        self._payload_slots: list[object] = [_EMPTY] * capacity_entities
        self._spec_slots: list[PersistQueueEntitySpec | None] = [
            None
        ] * capacity_entities
        self._token_slots: list[PersistQueueReservationToken | None] = [
            None
        ] * capacity_entities

        self._head = 0
        self._published_entities = 0
        self._claimed_entities = 0
        self._reservation: PersistQueueReservation | None = None
        self._active_lease: PersistQueueBatchLease | None = None
        self._next_token_sequence = 1
        self._closed = False
        self._admission_snapshot = PersistenceAdmissionSnapshot(
            generation=0,
            state=PersistenceAdmissionState.UNAVAILABLE_STARTING,
            changed_at_monotonic_us=0,
        )
        self._wake_event = wake_event if wake_event is not None else Event()
        self._lock = Lock()

    @property
    def capacity_entities(self) -> int:
        return self._capacity_entities

    def publish_admission_state(
        self,
        snapshot: PersistenceAdmissionSnapshot,
    ) -> None:
        if type(snapshot) is not PersistenceAdmissionSnapshot:
            raise PersistQueueInterfaceError("invalid persistence admission snapshot")
        if (
            type(snapshot.generation) is not int
            or type(snapshot.state) is not PersistenceAdmissionState
            or type(snapshot.changed_at_monotonic_us) is not int
            or snapshot.generation < 0
            or snapshot.changed_at_monotonic_us < 0
        ):
            raise PersistQueueInterfaceError("invalid persistence admission snapshot")

        with self._lock:
            current = self._admission_snapshot
            if snapshot.generation != current.generation + 1:
                raise PersistQueueInterfaceError(
                    "persistence admission generation must advance exactly once"
                )
            if (
                snapshot.changed_at_monotonic_us
                < current.changed_at_monotonic_us
            ):
                raise PersistQueueInterfaceError(
                    "persistence admission monotonic time regressed"
                )
            self._admission_snapshot = snapshot

    def try_reserve_one(
        self,
        spec: PersistQueueEntitySpec,
    ) -> PersistQueueReserveResult:
        with self._lock:
            if self._closed:
                raise PersistQueueInterfaceError("PersistQueue producer is closed")
            self._require_supported_spec(spec)
            if self._reservation is not None:
                raise PersistQueueInterfaceError(
                    "PersistQueue already has an outstanding reservation"
                )

            admission_snapshot = self._admission_snapshot
            used_entities = self._published_entities
            if admission_snapshot.state is not PersistenceAdmissionState.AVAILABLE:
                return PersistQueueReserveResult(
                    status=AdmissionResult.PERSISTENCE_UNAVAILABLE,
                    reservation=None,
                    admission_snapshot=admission_snapshot,
                    used_entities_before=used_entities,
                    capacity_entities=self.capacity_entities,
                )
            if used_entities >= self.capacity_entities:
                return PersistQueueReserveResult(
                    status=AdmissionResult.QUEUE_FULL,
                    reservation=None,
                    admission_snapshot=admission_snapshot,
                    used_entities_before=used_entities,
                    capacity_entities=self.capacity_entities,
                )

            slot_index = (
                self._head + self._published_entities
            ) % self.capacity_entities
            self._require_empty_slot(slot_index)
            token = PersistQueueReservationToken(self._next_token_sequence)
            reservation = PersistQueueReservation(self, token, spec, slot_index)
            result = PersistQueueReserveResult(
                status=AdmissionResult.RESERVED,
                reservation=reservation,
                admission_snapshot=admission_snapshot,
                used_entities_before=used_entities,
                capacity_entities=self.capacity_entities,
            )

            self._spec_slots[slot_index] = spec
            self._token_slots[slot_index] = token
            self._reservation = reservation
            self._next_token_sequence += 1
            return result

    def _publish_reservation(
        self,
        reservation: PersistQueueReservation,
        entity: object,
    ) -> None:
        with self._lock:
            self._require_live_reservation(reservation)
            slot_index = reservation._slot_index
            if self._payload_slots[slot_index] is not _EMPTY:
                raise PersistQueueInterfaceError(
                    "reserved PersistQueue slot already has a payload"
                )
            self._payload_slots[slot_index] = entity
            self._published_entities += 1
            self._reservation = None
            reservation._active = False
        self._wake_event.set()

    def _cancel_reservation(self, reservation: PersistQueueReservation) -> None:
        should_wake = False
        with self._lock:
            self._require_live_reservation(reservation)
            slot_index = reservation._slot_index
            self._clear_slot(slot_index)
            self._reservation = None
            reservation._active = False
            should_wake = self._closed and self._published_entities == 0
        if should_wake:
            self._wake_event.set()

    def claim_batch(
        self,
        *,
        max_entities: int,
    ) -> PersistQueueBatchLease | None:
        if type(max_entities) is not int or max_entities <= 0:
            raise PersistQueueConfigurationError(
                "max_entities must be a positive integer"
            )
        with self._lock:
            if self._active_lease is not None:
                raise PersistQueueInterfaceError(
                    "PersistQueue already has an active batch lease"
                )
            if self._published_entities == 0:
                return None

            claimed_entities = min(max_entities, self._published_entities)
            views: list[PersistQueueEntryView] = []
            for offset in range(claimed_entities):
                slot_index = (self._head + offset) % self.capacity_entities
                spec = self._spec_slots[slot_index]
                token = self._token_slots[slot_index]
                entity = self._payload_slots[slot_index]
                if spec is None or token is None or entity is _EMPTY:
                    raise PersistQueueInterfaceError(
                        "published PersistQueue slot is incomplete"
                    )
                views.append(PersistQueueEntryView(token, spec, entity))
            lease = PersistQueueBatchLease(self, tuple(views))
            self._claimed_entities = claimed_entities
            self._active_lease = lease
            return lease

    def _acknowledge_lease(
        self,
        lease: PersistQueueBatchLease,
        dispositions: tuple[PersistQueueBatchDisposition, ...],
    ) -> None:
        if type(dispositions) is not tuple or any(
            type(disposition) is not PersistQueueBatchDisposition
            for disposition in dispositions
        ):
            raise PersistQueueInterfaceError(
                "PersistQueue dispositions must be a tuple of known values"
            )
        should_wake = False
        with self._lock:
            self._require_live_lease(lease)
            if len(dispositions) != self._claimed_entities:
                raise PersistQueueInterfaceError(
                    "PersistQueue disposition count does not match the lease"
                )
            for offset, view in enumerate(lease._entries):
                slot_index = (self._head + offset) % self.capacity_entities
                if (
                    self._token_slots[slot_index] is not view.token
                    or self._spec_slots[slot_index] is not view.spec
                    or self._payload_slots[slot_index] is not view.entity
                ):
                    raise PersistQueueInterfaceError(
                        "PersistQueue batch no longer names the queue-head prefix"
                    )

            removed = self._claimed_entities
            for offset in range(removed):
                self._clear_slot((self._head + offset) % self.capacity_entities)
            self._head = (self._head + removed) % self.capacity_entities
            self._published_entities -= removed
            self._claimed_entities = 0
            self._active_lease = None
            lease._active = False
            should_wake = self._closed and self._reservation is None and (
                self._published_entities == 0
            )
        if should_wake:
            self._wake_event.set()

    def _release_lease(self, lease: PersistQueueBatchLease) -> None:
        with self._lock:
            self._require_live_lease(lease)
            self._claimed_entities = 0
            self._active_lease = None
            lease._active = False
        self._wake_event.set()

    def close(self) -> None:
        should_wake = False
        with self._lock:
            if not self._closed:
                self._closed = True
                should_wake = True
        if should_wake:
            self._wake_event.set()

    def snapshot(self) -> PersistQueueSnapshot:
        with self._lock:
            reserved_entities = int(self._reservation is not None)
            published_entities = self._published_entities
            return PersistQueueSnapshot(
                admission_snapshot=self._admission_snapshot,
                capacity_entities=self.capacity_entities,
                reserved_entities=reserved_entities,
                published_entities=published_entities,
                claimed_entities=self._claimed_entities,
                closed=self._closed,
                closed_and_drained=(
                    self._closed
                    and reserved_entities == 0
                    and published_entities == 0
                ),
            )

    @staticmethod
    def _require_supported_spec(spec: PersistQueueEntitySpec) -> None:
        if (
            type(spec) is not PersistQueueEntitySpec
            or type(spec.kind) is not PersistQueueEntityKind
            or type(spec.schema_version) is not int
            or not any(
                spec.kind is supported.kind
                and spec.schema_version == supported.schema_version
                for supported in SUPPORTED_PERSIST_QUEUE_ENTITY_SPECS
            )
        ):
            raise PersistQueueInterfaceError("unsupported PersistQueue entity spec")

    def _require_empty_slot(self, slot_index: int) -> None:
        if (
            self._payload_slots[slot_index] is not _EMPTY
            or self._spec_slots[slot_index] is not None
            or self._token_slots[slot_index] is not None
        ):
            raise PersistQueueInterfaceError("PersistQueue tail slot is not empty")

    def _require_live_reservation(
        self,
        reservation: PersistQueueReservation,
    ) -> None:
        if type(reservation) is not PersistQueueReservation:
            raise PersistQueueInterfaceError("invalid PersistQueue reservation")
        if reservation._queue is not self:
            raise PersistQueueInterfaceError("foreign PersistQueue reservation")
        if not reservation._active or self._reservation is not reservation:
            raise PersistQueueInterfaceError("stale PersistQueue reservation")
        slot_index = reservation._slot_index
        if type(slot_index) is not int or not 0 <= slot_index < self.capacity_entities:
            raise PersistQueueInterfaceError(
                "PersistQueue reservation slot is invalid"
            )
        if (
            self._token_slots[slot_index] is not reservation._token
            or self._spec_slots[slot_index] is not reservation._spec
        ):
            raise PersistQueueInterfaceError(
                "PersistQueue reservation metadata does not match its slot"
            )

    def _require_live_lease(self, lease: PersistQueueBatchLease) -> None:
        if type(lease) is not PersistQueueBatchLease:
            raise PersistQueueInterfaceError("invalid PersistQueue batch lease")
        if lease._queue is not self:
            raise PersistQueueInterfaceError("foreign PersistQueue batch lease")
        if not lease._active or self._active_lease is not lease:
            raise PersistQueueInterfaceError("stale PersistQueue batch lease")

    def _clear_slot(self, slot_index: int) -> None:
        self._payload_slots[slot_index] = _EMPTY
        self._spec_slots[slot_index] = None
        self._token_slots[slot_index] = None


__all__ = [
    "PERSIST_QUEUE_MAX_ENTITIES",
    "PersistQueueError",
    "PersistQueueConfigurationError",
    "PersistQueueInterfaceError",
    "PersistenceAdmissionSnapshot",
    "PersistQueueSnapshot",
    "PersistQueueReservationToken",
    "PersistQueueReservation",
    "PersistQueueReserveResult",
    "PersistQueueEntryView",
    "PersistQueueBatchDisposition",
    "PersistQueueBatchLease",
    "PersistQueue",
]
