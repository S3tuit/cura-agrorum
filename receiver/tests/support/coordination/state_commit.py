"""One lost reply at the existing production persistence-control boundary."""

from dataclasses import replace
from cura_receiver.generated.receiver_enums_generated import DiagnosticOperation as Op
from cura_receiver.persistence_control_values import (
    CommunicatorStateCommitDisposition as CD,
    CommunicatorStateCommitFailureKind as CF,
)


class LostStateReply:
    """Lose one actual control reply, after either commit or queued expiry."""

    def __init__(self, control, *, installed):
        self.control, self.installed = control, installed
        self.lose_next = True
        self.fail_load = False
        self.requests = []

    def commit_communicator_state(self, value, **kwargs):
        self.requests.append(value)
        lose = self.lose_next
        self.lose_next = False
        if lose and not self.installed:
            kwargs["deadline_monotonic_us"] = 0
        result = self.control.commit_communicator_state(value, **kwargs)
        if lose:
            assert result.disposition is (
                CD.COMMITTED if self.installed else CD.NOT_INSTALLED
            )
            return replace(
                result,
                disposition=CD.OUTCOME_UNKNOWN,
                failure_kind=CF.DEADLINE_EXCEEDED,
                operation=Op.WRITE,
            )
        return result

    def load_communicator_state(self, **kwargs):
        if self.fail_load:
            kwargs["deadline_monotonic_us"] = 0
        return self.control.load_communicator_state(**kwargs)
