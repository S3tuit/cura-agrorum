"""Incremental refresh cancellation uses real state control and SQLite."""

from dataclasses import replace

import pytest

from cura_receiver.runtime_time import RtcRefreshStatus as RS
from cura_receiver.generated.receiver_enums_generated import PersistenceControlPurpose
from cura_receiver.ports.ds3231 import Ds3231ReadResult, Ds3231ReadStatus as R
from cura_receiver.ports.ds3231 import Ds3231WriteResult, Ds3231WriteDisposition as W, Ds3231Failure as F
from cura_receiver.persistence_control_values import CommunicatorStateCommitDisposition as CD
from tests.host.test_runtime_time import rtc_runtime, UTC, attach_ambiguous_owner


def queue_success(rtc, clock):
    def write():
        now = clock.now_monotonic_us()
        return Ds3231WriteResult(W.COMPLETED, F.NONE, now, now)
    def read():
        now = clock.now_monotonic_us()
        return Ds3231ReadResult(R.OK, now, now, UTC // 1_000_000)
    rtc.write_results.append(write)
    rtc.read_results.append(read)


def test_each_turn_executes_only_one_action(rtc_runtime, monkeypatch):
    rt, clock, _, rtc, worker, snapshot, _ = rtc_runtime
    queue_success(rtc, clock)
    commits = []
    commit = worker.control.commit_communicator_state
    def observed(state, **kwargs):
        commits.append(state)
        return commit(state, **kwargs)
    monkeypatch.setattr(worker.control, 'commit_communicator_state', observed)
    actions = []
    for _ in range(5):
        before = len(commits) + len(rtc.calls)
        result = rt.advance_rtc_refresh(rtc, snapshot)
        actions.append(len(commits) + len(rtc.calls) - before)
    assert actions == [1, 1, 1, 1, 1]
    assert result.status is RS.VERIFIED
    assert rt.rtc_refresh_episode is None
    assert rt.durable_state.generation == 3


@pytest.mark.parametrize('completed_actions', [0, 1, 2, 3, 4])
def test_stop_cancels_without_new_io_or_fabricated_counts(rtc_runtime, completed_actions):
    rt, clock, _, rtc, worker, snapshot, _ = rtc_runtime
    initial = rt.durable_state
    queue_success(rtc, clock)
    for _ in range(completed_actions):
        assert rt.advance_rtc_refresh(rtc, snapshot) is None
    calls = list(rtc.calls)
    result = rt.advance_rtc_refresh(rtc, snapshot, stop_requested=True,
        shutdown_deadline=clock.now_monotonic_us() + 10_000_000)
    assert result.status is RS.SHUTDOWN_CANCELLED and result.failure is None
    assert rtc.calls == calls
    assert sum(rt.rtc_write_counts.values()) == (completed_actions >= 3)
    assert rt.rtc_readback_verified_count == (completed_actions >= 4)
    assert rt.durable_state == (initial if completed_actions < 2 else rt.durable_state)
    assert (rt.rtc_provenance is None) == (completed_actions >= 2)
    assert rt.rtc_refresh_episode is None
    loaded = worker.control.load_communicator_state(deadline_monotonic_us=clock.now_monotonic_us()+5_000_000)
    assert loaded.state == rt.durable_state


def test_stop_during_slow_preread_returns_at_first_boundary(rtc_runtime):
    rt, clock, _, rtc, _, snapshot, _ = rtc_runtime
    initial = rt.durable_state
    stop_deadline = None
    def read():
        nonlocal stop_deadline
        start = clock.now_monotonic_us()
        stop_deadline = start + 10_000_000
        clock.advance_elapsed_us(2_900_000)
        return Ds3231ReadResult(R.OK, start, clock.now_monotonic_us(), UTC // 1_000_000)
    rtc.read_results.clear()
    rtc.read_results.append(read)
    assert rt.advance_rtc_refresh(rtc, snapshot) is None
    result = rt.advance_rtc_refresh(rtc, snapshot, stop_requested=True, shutdown_deadline=stop_deadline)
    assert result.status is RS.SHUTDOWN_CANCELLED
    assert stop_deadline - clock.now_monotonic_us() == 7_100_000
    assert len(rtc.calls) == 1 and rt.durable_state == initial


@pytest.mark.parametrize('write_disposition', [W.COMPLETED, W.OUTCOME_UNKNOWN])
def test_cancel_read_retry_preserves_first_failure_and_write(rtc_runtime, write_disposition):
    rt, clock, _, rtc, _, snapshot, _ = rtc_runtime
    now = clock.now_monotonic_us()
    rtc.write_results.append(Ds3231WriteResult(write_disposition,
        F.NONE if write_disposition is W.COMPLETED else F.IO_ERROR, now, now,
        os_errno=None if write_disposition is W.COMPLETED else 5))
    rtc.read_results.append(Ds3231ReadResult(R.IO_ERROR, now, now, os_errno=5))
    for _ in range(4):
        assert rt.advance_rtc_refresh(rtc, snapshot) is None
    result = rt.cancel_rtc_refresh()
    assert result.status is RS.SHUTDOWN_CANCELLED
    assert result.failure is not None
    assert result.write_result.disposition is write_disposition
    assert result.read_result.status is R.IO_ERROR
    assert rt.rtc_write_counts[write_disposition] == 1
    assert rt.rtc_readback_verified_count == 0 and rt.rtc_provenance is None
    assert [call[0] for call in rtc.calls] == ['read', 'write', 'read']


@pytest.mark.parametrize('generation', [2, 3])
def test_cancel_unknown_commit_retains_exact_state_authority(rtc_runtime, generation):
    rt, clock, _, rtc, worker, snapshot, _ = rtc_runtime
    control = attach_ambiguous_owner(rt, worker, generation)
    queue_success(rtc, clock)
    for _ in range(2 if generation == 2 else 5):
        assert rt.advance_rtc_refresh(rtc, snapshot) is None
    pending = rt.state_owner.pending
    assert pending.requested.generation == generation
    result = rt.cancel_rtc_refresh()
    assert result.status is RS.SHUTDOWN_CANCELLED
    assert result.commit_result.disposition is CD.OUTCOME_UNKNOWN
    assert rt.state_owner.pending is pending and rt.rtc_provenance is None
    control.loads_available = True
    rt.state_owner.reconcile(deadline_monotonic_us=clock.now_monotonic_us()+5_000_000)
    assert rt.durable_state == pending.requested
    assert (rt.rtc_provenance is not None) == (generation == 3)


def test_radio_delay_does_not_restart_read_retry_window(rtc_runtime):
    rt, clock, _, rtc, _, snapshot, _ = rtc_runtime
    now = clock.now_monotonic_us()
    rtc.read_results.clear()
    rtc.read_results.append(Ds3231ReadResult(R.IO_ERROR, now, now, os_errno=5))
    assert rt.advance_rtc_refresh(rtc, snapshot) is None
    clock.advance_elapsed_us(3_000_000)
    result = rt.advance_rtc_refresh(rtc, snapshot)
    assert result.status is RS.PREWRITE_READ_FAILED
    assert result.prewrite_read_result.status is R.DEADLINE_EXCEEDED
    assert len(rtc.calls) == 1 and result.failure is not None


def test_source_expiry_between_invalidation_and_write_prevents_write(rtc_runtime):
    rt, clock, _, rtc, _, snapshot, _ = rtc_runtime
    for _ in range(2):
        assert rt.advance_rtc_refresh(rtc, snapshot) is None
    clock.advance_elapsed_us(60_000_000)
    result = rt.advance_rtc_refresh(rtc, snapshot)
    assert result.status is RS.TRUST_INVALIDATED
    assert len(rtc.calls) == 1 and rt.rtc_provenance is None


# A full operation must fit the inclusive source bound before any device/state mutation.
@pytest.mark.parametrize("extra_error,admitted", [(0, True), (1, False)])
def test_preflight_projects_full_operation_error(rtc_runtime, extra_error, admitted):
    rt, clock, _, rtc, worker, snapshot, _ = rtc_runtime
    initial = rt.durable_state
    # 21 physical seconds => 21,077,700 monotonic us; upward growth is 78,278 us.
    rt.sample = replace(rt.sample, error_bound_us=5_000_000 - 78_278 + extra_error)
    queue_success(rtc, clock)
    result = rt.refresh_rtc(rtc, snapshot)
    if admitted:
        assert result.status is RS.VERIFIED
        assert [call[0] for call in rtc.calls] == ["read", "write", "read"]
        assert 5_000_000 < rt.rtc_provenance.verification_uncertainty_us < 40_000_000
        loaded = worker.control.load_communicator_state(
            deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000)
        assert loaded.state.rtc_provenance == rt.rtc_provenance
    else:
        assert result.status is RS.DEFERRED
        assert rtc.calls == [] and rt.durable_state == initial


# Poll expiry is exclusive even if the projected error remains small.
@pytest.mark.parametrize("spare_us,admitted", [(0, False), (1, True)])
def test_preflight_requires_room_before_poll_deadline(rtc_runtime, spare_us, admitted):
    rt, clock, _, rtc, _, snapshot, _ = rtc_runtime
    initial = rt.durable_state
    rt.tracking_poll_deadline = clock.now_monotonic_us() + 21_077_700 + spare_us
    queue_success(rtc, clock)
    result = rt.refresh_rtc(rtc, snapshot)
    assert result.status is (RS.VERIFIED if admitted else RS.DEFERRED)
    if not admitted:
        assert rtc.calls == [] and rt.durable_state == initial


# Removing the invalidation transition removes its two bounded control calls as well.
def test_preflight_without_old_provenance_uses_shorter_operation(rtc_runtime):
    rt, clock, _, rtc, worker, snapshot, _ = rtc_runtime
    state = replace(rt.durable_state, generation=rt.durable_state.generation + 1,
        rtc_provenance=None)
    rt.state_owner.commit(state,
        purpose=PersistenceControlPurpose.RTC_PROVENANCE,
        deadline_monotonic_us=clock.now_monotonic_us() + 1_000_000)
    assert rt.rtc_provenance is None
    rt.tracking_poll_deadline = clock.now_monotonic_us() + 19_070_301
    queue_success(rtc, clock)
    assert rt.refresh_rtc(rtc, snapshot).status is RS.VERIFIED
