"""Small specification model for ``PersistQueue`` state-machine tests.

The model intentionally uses an ordinary logical list rather than copying the
production circular-array implementation.
"""

from __future__ import annotations

from dataclasses import dataclass

from cura_receiver.generated.receiver_enums_generated import AdmissionResult


class ReferenceQueueViolation(RuntimeError):
    pass


@dataclass(frozen=True, slots=True)
class ReferenceEntry:
    token: int
    spec: object
    entity: object


@dataclass(frozen=True, slots=True)
class ReferenceSnapshot:
    capacity_entities: int
    reserved_entities: int
    published_entities: int
    claimed_entities: int
    closed: bool
    closed_and_drained: bool


@dataclass(frozen=True, slots=True)
class ReferenceReserveResult:
    status: AdmissionResult
    token: int | None
    used_entities_before: int


class ReferencePersistQueue:
    def __init__(self, capacity_entities: int) -> None:
        if capacity_entities <= 0:
            raise ValueError("capacity must be positive")
        self.capacity_entities = capacity_entities
        self.available = False
        self.closed = False
        self._next_token = 1
        self._reserved: tuple[int, object] | None = None
        self._published: list[ReferenceEntry] = []
        self._claimed_entities = 0

    def set_available(self, available: bool) -> None:
        self.available = available

    def reserve(self, spec: object) -> ReferenceReserveResult:
        if self.closed:
            raise ReferenceQueueViolation("closed")
        if self._reserved is not None:
            raise ReferenceQueueViolation("reservation already outstanding")
        used = len(self._published)
        if not self.available:
            return ReferenceReserveResult(
                AdmissionResult.PERSISTENCE_UNAVAILABLE,
                None,
                used,
            )
        if used >= self.capacity_entities:
            return ReferenceReserveResult(AdmissionResult.QUEUE_FULL, None, used)
        token = self._next_token
        self._next_token += 1
        self._reserved = (token, spec)
        return ReferenceReserveResult(AdmissionResult.RESERVED, token, used)

    def publish(self, token: int, entity: object) -> None:
        if self._reserved is None or self._reserved[0] != token:
            raise ReferenceQueueViolation("stale token")
        _token, spec = self._reserved
        self._published.append(ReferenceEntry(token, spec, entity))
        self._reserved = None

    def cancel(self, token: int) -> None:
        if self._reserved is None or self._reserved[0] != token:
            raise ReferenceQueueViolation("stale token")
        self._reserved = None

    def claim(self, max_entities: int) -> tuple[ReferenceEntry, ...] | None:
        if max_entities <= 0:
            raise ValueError("max_entities must be positive")
        if self._claimed_entities:
            raise ReferenceQueueViolation("claim already active")
        if not self._published:
            return None
        self._claimed_entities = min(max_entities, len(self._published))
        return tuple(self._published[: self._claimed_entities])

    def acknowledge(self, tokens: tuple[int, ...]) -> None:
        if not self._claimed_entities:
            raise ReferenceQueueViolation("no active claim")
        expected = tuple(
            entry.token for entry in self._published[: self._claimed_entities]
        )
        if tokens != expected:
            raise ReferenceQueueViolation("stale prefix")
        del self._published[: self._claimed_entities]
        self._claimed_entities = 0

    def release(self) -> None:
        if not self._claimed_entities:
            raise ReferenceQueueViolation("no active claim")
        self._claimed_entities = 0

    def close(self) -> None:
        self.closed = True

    def snapshot(self) -> ReferenceSnapshot:
        reserved = int(self._reserved is not None)
        published = len(self._published)
        return ReferenceSnapshot(
            capacity_entities=self.capacity_entities,
            reserved_entities=reserved,
            published_entities=published,
            claimed_entities=self._claimed_entities,
            closed=self.closed,
            closed_and_drained=self.closed and reserved == 0 and published == 0,
        )
