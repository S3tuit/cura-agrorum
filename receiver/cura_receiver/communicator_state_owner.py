"""Communicator-owned complete-state authority; disk I/O stays in the worker."""

from dataclasses import dataclass

from .generated.receiver_entities_generated import (
    CommunicatorStateV1,
    encode_communicator_state_v1,
)
from .persistence_control_values import (
    CommunicatorStateCommitDisposition as CD,
    CommunicatorStateLoadStatus as LS,
    CommunicatorStateCondition as Condition,
    require_immutable_state,
)


@dataclass(frozen=True, slots=True)
class PendingStateCommit:
    preceding: CommunicatorStateV1 | None
    requested: CommunicatorStateV1
    preceding_condition: Condition = Condition.NONE


class CommunicatorStateOwner:
    """One communicator caller shares this owner among all complete-state users.

    Initial state must come from acknowledged persistence. Baseline creation
    and recovery remain the caller's policy, outside this coordinator.
    """

    def __init__(
        self, *, control, initial_state=None, initial_condition=Condition.NONE
    ):
        if type(initial_condition) is not Condition:
            raise TypeError("initial condition must come from the state load")
        if initial_state is not None and initial_condition is not Condition.NONE:
            raise ValueError("a loaded state cannot also have an unavailable condition")
        if initial_state is not None:
            require_immutable_state(initial_state)
        self._control = control
        self._confirmed = initial_state
        self._condition = initial_condition
        self._pending = None
        self._reconciliation_conflict = False
        self._recovery_retry_permitted = False

    @classmethod
    def from_load(cls, *, control, loaded):
        """Start from a serialized usable state or an explicit recovery condition."""
        if loaded.status not in (LS.LOADED, LS.STATE_UNAVAILABLE):
            raise ValueError("a completed state load is required")
        return cls(
            control=control,
            initial_state=loaded.state,
            initial_condition=loaded.state_condition,
        )

    @property
    def condition(self):
        """The preceding primary condition; it does not resolve a pending request."""
        return self._condition

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
        if self._confirmed is None and self._condition is Condition.NONE:
            raise RuntimeError(
                "an acknowledged state or serialized recovery condition is required"
            )
        require_immutable_state(requested)
        expected = 1 if self._confirmed is None else self._confirmed.generation + 1
        if requested.generation != expected or expected > (1 << 63) - 1:
            raise ValueError("complete state must use the next acknowledged generation")
        # Retain before entering the channel, including if a caller is interrupted.
        self._pending = PendingStateCommit(self._confirmed, requested, self._condition)
        return self._submit_pending(deadline_monotonic_us=deadline_monotonic_us)

    def _submit_pending(self, *, deadline_monotonic_us):
        pending = self._pending
        self._recovery_retry_permitted = False
        result = self._control.commit_communicator_state(
            pending.requested, deadline_monotonic_us=deadline_monotonic_us
        )
        if result.disposition in (CD.COMMITTED, CD.ALREADY_COMMITTED):
            self._resolve(pending.requested)
        elif result.disposition is CD.NOT_INSTALLED:
            self._resolve(pending.preceding, pending.preceding_condition)
        return result

    def reconcile(self, *, deadline_monotonic_us):
        if self._pending is None:
            return None
        loaded = self._control.load_communicator_state(
            deadline_monotonic_us=deadline_monotonic_us
        )
        self._recovery_retry_permitted = False
        if loaded.status is LS.LOADED:
            actual = encode_communicator_state_v1(loaded.state)
            for expected in (self._pending.requested, self._pending.preceding):
                if expected is not None and actual == encode_communicator_state_v1(
                    expected
                ):
                    self._resolve(expected)
                    break
            else:
                self._reconciliation_conflict = True
        elif (
            loaded.status is LS.STATE_UNAVAILABLE
            and self._pending.preceding is None
            and loaded.state_condition is self._pending.preceding_condition
            and not self._reconciliation_conflict
        ):
            # Same condition is not exact-row equality. Only reissue the retained request.
            self._recovery_retry_permitted = True
        return loaded

    def retry_pending_recovery(self, *, deadline_monotonic_us):
        """Retry exact generation-one bytes after an unavailable reconciliation load."""
        if not self._recovery_retry_permitted:
            raise RuntimeError("reconcile the exact pending recovery request first")
        return self._submit_pending(deadline_monotonic_us=deadline_monotonic_us)

    def _resolve(self, state, condition=Condition.NONE):
        self._confirmed = state
        self._condition = condition
        self._pending = None
        self._reconciliation_conflict = False
        self._recovery_retry_permitted = False
