"""Communicator-owned complete-state authority; disk I/O stays in the worker."""

from dataclasses import dataclass

from .generated.receiver_entities_generated import (
    CommunicatorStateV1,
    encode_communicator_state_v1,
)
from .persistence_control_values import (
    CommunicatorStateCommitDisposition as CD,
    CommunicatorStateLoadStatus as LS,
    require_immutable_state,
)


@dataclass(frozen=True, slots=True)
class PendingStateCommit:
    preceding: CommunicatorStateV1
    requested: CommunicatorStateV1


class CommunicatorStateOwner:
    """One communicator caller shares this owner among all complete-state users.

    Initial state must come from acknowledged persistence. Baseline creation
    and recovery remain the caller's policy, outside this coordinator.
    """

    def __init__(self, *, control, initial_state=None):
        if initial_state is not None:
            require_immutable_state(initial_state)
        self._control = control
        self._confirmed = initial_state
        self._pending = None
        self._reconciliation_conflict = False

    @property
    def state(self):
        """The usable durable state, absent while a commit is unresolved."""
        return self._confirmed if self._pending is None else None

    @property
    def pending(self):
        return self._pending

    @property
    def reconciliation_conflict(self):
        return self._reconciliation_conflict

    def commit(self, requested, *, deadline_monotonic_us):
        if self._pending is not None:
            raise RuntimeError("reconcile the pending complete-state commit first")
        if self._confirmed is None:
            raise RuntimeError("an acknowledged complete-state baseline is required")
        require_immutable_state(requested)
        if requested.generation != self._confirmed.generation + 1:
            raise ValueError("complete state must use the next acknowledged generation")
        # Retain before entering the channel, including if a caller is interrupted.
        self._pending = PendingStateCommit(self._confirmed, requested)
        result = self._control.commit_communicator_state(
            requested, deadline_monotonic_us=deadline_monotonic_us
        )
        if result.disposition in (CD.COMMITTED, CD.ALREADY_COMMITTED):
            self._resolve(requested)
        elif result.disposition is CD.NOT_INSTALLED:
            self._resolve(self._confirmed)
        return result

    def reconcile(self, *, deadline_monotonic_us):
        if self._pending is None:
            return None
        loaded = self._control.load_communicator_state(
            deadline_monotonic_us=deadline_monotonic_us
        )
        if loaded.status is LS.LOADED:
            actual = encode_communicator_state_v1(loaded.state)
            for expected in (self._pending.requested, self._pending.preceding):
                if actual == encode_communicator_state_v1(expected):
                    self._resolve(expected)
                    break
            else:
                self._reconciliation_conflict = True
        return loaded

    def _resolve(self, state):
        self._confirmed = state
        self._pending = None
        self._reconciliation_conflict = False
