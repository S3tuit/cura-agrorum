"""Persistence startup evidence and the shared, stop-aware observation boundary."""

from dataclasses import dataclass
from enum import IntEnum

from .elapsed_duration import checked_duration_us, checked_monotonic_deadline


class StartupStage(IntEnum):
    CONFIGURATION_LOAD = 0
    DATABASE_OPEN_VALIDATION = 1
    INSTANCE_COMMIT = 2
    PERSISTENCE_COMPONENT_SETUP = 3
    COMMUNICATOR_STATE_LOAD = 4


@dataclass(frozen=True, slots=True)
class StartupSnapshot:
    begin_monotonic_us: int | None
    stage_entries: tuple[int | None, ...]
    result: object
    observed_at_monotonic_us: int


def startup_failure(result):
    if result.unexpected_failure or result.configuration_load is None:
        return 'UNEXPECTED_INITIALIZATION_ERROR'
    if result.configuration_load.status.name != 'LOADED':
        return result.configuration_load.status.name
    if result.database_failure is not None:
        return result.database_failure.admission_state.name
    if result.state_load is None:
        return 'PERSISTENCE_STARTUP_FAILED'
    return None


@dataclass(frozen=True, slots=True)
class StartupDecision:
    outcome: str
    begin_monotonic_us: int
    deadline_monotonic_us: int
    observed_at_monotonic_us: int
    snapshot: StartupSnapshot | None

    def summary(self, instance_id):
        snapshot = self.snapshot
        entries = snapshot.stage_entries if snapshot else (None,) * len(StartupStage)
        result = snapshot.result if snapshot else None
        completed = result.published_at_monotonic_us if result else None
        last = next((stage for stage in reversed(StartupStage) if entries[stage] is not None), None)
        end = completed if completed is not None else self.observed_at_monotonic_us
        failure = result.database_failure if result else None
        configuration = result.configuration_load if result else None
        instance = result.instance_start if result else None
        return dict(
            format_version=1, receiver_instance_id=instance_id.hex(), outcome=self.outcome,
            startup_begin_monotonic_us=self.begin_monotonic_us,
            startup_deadline_monotonic_us=self.deadline_monotonic_us,
            observed_at_monotonic_us=self.observed_at_monotonic_us,
            published_at_monotonic_us=completed,
            elapsed_us=self.observed_at_monotonic_us - self.begin_monotonic_us,
            completion_elapsed_us=None if completed is None else completed - self.begin_monotonic_us,
            stage_entry_offsets_us={stage.name: None if entries[stage] is None else entries[stage] - self.begin_monotonic_us for stage in StartupStage},
            last_stage=None if last is None else last.name,
            last_stage_elapsed_us=None if last is None else end - entries[last],
            snapshot_available=snapshot is not None,
            worker_failure=None if result is None else startup_failure(result),
            instance_start_disposition=None if instance is None else instance.disposition.name,
            configuration_status=None if configuration is None else configuration.status.name,
            configuration_rejection=None if configuration is None or configuration.protocol_rejection is None else configuration.protocol_rejection.name,
            sqlite_primary_code=None if failure is None else failure.sqlite_primary_code,
            sqlite_extended_code=None if failure is None else failure.sqlite_extended_code,
            os_errno=failure.os_errno if failure else (configuration.os_errno if configuration else None),
        )


def observe_persistence_startup(*, worker, clock, stop_intent, wait, budget_us):
    """Launch once, observe one fixed deadline; cleanup remains with the caller."""
    checked_duration_us(budget_us)
    if budget_us == 0 or worker.startup_notification is None:
        raise ValueError('startup observation requires a positive budget and notification')
    begin = clock.now_monotonic_us()
    deadline = checked_monotonic_deadline(begin, budget_us)
    if not stop_intent.is_requested():
        worker.start(begin_monotonic_us=begin)
        wait.wait_until_monotonic_us(deadline, completion=worker.startup_notification,
                                     completed=worker.startup_completed)
    snapshot = worker.startup_evidence(deadline_monotonic_us=deadline,
                                       nonblocking=stop_intent.is_requested())
    observed = snapshot.observed_at_monotonic_us if snapshot else clock.now_monotonic_us()
    result = snapshot.result if snapshot else None
    if stop_intent.is_requested():
        outcome = 'STOP_REQUESTED'
    elif result is None or result.published_at_monotonic_us >= deadline:
        outcome = 'PERSISTENCE_STARTUP_INCOMPLETE'
    else:
        outcome = startup_failure(result) or 'SUCCESS'
    return StartupDecision(outcome, begin, deadline, observed, snapshot)
