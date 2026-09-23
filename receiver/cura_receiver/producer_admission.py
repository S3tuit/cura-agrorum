"""Communicator-owned ordinary producer boundary and admission accounting."""

from .generated.receiver_enums_generated import AdmissionResult, PersistQueueEntityKind
from .persist_queue import PersistQueue


class ProducerAdmission:
    """One instance shared by all producers in a receiver instance.

    The real queue owns capacity, reservations and publication. This single-owner
    adapter records operational results only; it adds no admission policy.
    """

    def __init__(self, queue: PersistQueue):
        if type(queue) is not PersistQueue:
            raise TypeError("producer admission requires the production PersistQueue")
        self._queue = queue
        self._counts = [[0 for _ in AdmissionResult] for _ in PersistQueueEntityKind]

    @property
    def closed(self):
        return self._queue.snapshot().closed

    @property
    def counts(self):
        return tuple(tuple(row) for row in self._counts)

    def try_reserve_one(self, spec):
        result = self._queue.try_reserve_one(spec)
        row, column = spec.kind.value - 1, result.status.value
        self._counts[row][column] = min((1 << 64) - 1, self._counts[row][column] + 1)
        return result
