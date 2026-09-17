"""Production Linux adapter tested at the actual gpiod/spidev dependencies."""

from collections import deque
from enum import Enum
import errno
import json
from types import SimpleNamespace as NS
from threading import Thread

import pytest

from cura_receiver.platform import linux_clocks, linux_radio
from cura_receiver.ports.radio import (
    Error, Outcome, RadioBackendError, RadioConfiguration, RadioFailure,
    RadioLifecycleError, RadioLifecycleFailure, Stage,
)
from cura_receiver.radio import Radio, State
from cura_receiver.sx1262 import Sx1262
from tests.support.fakes.os_clock import FakeOsClock
from tests.support.fakes.radio_io import PhysicalPort, Wait


class Value(Enum):
    ACTIVE = 1
    INACTIVE = 0


class Dependencies:
    """Only dependency calls, queued results and explicit named hooks."""

    def __init__(self, clock):
        self.clock = clock
        self.calls = []
        self.hooks = {}
        self.events = deque()
        self.received = [0, 0x24]
        self.level = Value.INACTIVE
        self.line = NS(
            Value=Value, Direction=NS(INPUT="in", OUTPUT="out"),
            Drive=NS(OPEN_DRAIN="open-drain"), Bias=NS(AS_IS="as-is"),
            Edge=NS(RISING="rising"), Clock=NS(MONOTONIC="monotonic"),
        )
        self.gpiod = NS(
            line=self.line, LineSettings=lambda **kwargs: kwargs,
            request_lines=self.request_lines,
            EdgeEvent=NS(Type=NS(RISING_EDGE="rise")),
        )
        owner = self

        class Spi:
            def __setattr__(self, name, value):
                owner.call("property", name, value)

            def open_path(self, path):
                owner.call("open_path", path)

            def xfer2(self, data):
                owner.call("xfer2", data)
                return owner.received

            def close(self):
                owner.call("close")

        self.spidev = NS(SpiDev=Spi)

    def call(self, name, *args):
        self.calls.append((name, *args))
        if name in self.hooks:
            self.hooks[name]()

    def request_lines(self, path, **kwargs):
        self.call("request_lines", path, kwargs)
        return self

    def get_value(self, offset):
        self.call("get_value", offset)
        return self.level

    def set_value(self, offset, value):
        self.call("set_value", offset, value)

    def wait_edge_events(self, *, timeout):
        self.call("wait_edge_events", timeout)
        if self.events:
            return True
        self.clock.advance_elapsed_us(int(timeout.total_seconds() * 1_000_000))
        return False

    def read_edge_events(self, *, max_events):
        self.call("read_edge_events", max_events)
        return [self.events.popleft()]

    def release(self):
        self.call("release")


@pytest.fixture
def device(monkeypatch):
    clock = FakeOsClock(monotonic_us=100)
    dependencies = Dependencies(clock)
    monkeypatch.setattr(linux_radio.importlib, "import_module", lambda name: {
        "gpiod": dependencies.gpiod, "spidev": dependencies.spidev,
    }[name])
    return linux_radio.LinuxRadioIo(clock), dependencies, clock


def failure(error):
    def raise_it():
        raise error
    return raise_it


@pytest.fixture
def component(device, monkeypatch):
    """Wire the existing peripheral fake to actual Linux dependency calls."""
    io, deps, clock = device
    chip = PhysicalPort(clock)

    def transfer(_, data):
        deps.call("xfer2", data)
        return list(chip.transfer(bytes(data), deadline_monotonic_us=(1 << 64) - 1))

    def get_value(offset):
        deps.call("get_value", offset)
        return Value.ACTIVE if (chip.busy() if offset == 24 else chip.dio1()) else Value.INACTIVE

    def set_value(offset, value):
        deps.call("set_value", offset, value)
        chip.set_reset(asserted=value is Value.INACTIVE)

    monkeypatch.setattr(deps.spidev.SpiDev, "xfer2", transfer)
    monkeypatch.setattr(deps, "get_value", get_value)
    monkeypatch.setattr(deps, "set_value", set_value)
    return Radio(Sx1262(io, clock, Wait(clock))), deps, chip, clock


def queue_edge(deps, clock, sequence, **changes):
    deps.events.append(NS(**(dict(event_type="rise", line_offset=23,
        line_seqno=sequence, timestamp_ns=clock.now_monotonic_us() * 1000) | changes)))


# F-003: a gap remains rejected until explicit resynchronization adopts consumed evidence.
@pytest.mark.parametrize("queued", [False, True])
def test_gap_requires_explicit_event_resynchronization(device, queued):
    io, deps, clock = device
    io.open(RadioConfiguration(), deadline_monotonic_us=1000)
    queue_edge(deps, clock, 2)
    with pytest.raises(RadioBackendError) as caught:
        io.wait_edge(deadline_monotonic_us=1000)
    assert caught.value.failure.code is Error.MALFORMED_RESPONSE and io._last_sequence == 0
    if queued:
        queue_edge(deps, clock, 3)
    calls = deps.calls.copy()
    with pytest.raises(RadioBackendError):
        io.wait_edge(deadline_monotonic_us=1000)
    assert deps.calls == calls
    io.resynchronize_events(deadline_monotonic_us=1000)
    assert io._last_sequence == (3 if queued else 2)
    assert clock.now_monotonic_us() == 100
    queue_edge(deps, clock, 4 if queued else 3)
    edge = io.wait_edge(deadline_monotonic_us=1000)
    assert edge.sequence == (4 if queued else 3) and edge.timestamp_ns == 100000


# A full stale buffer is drained; more than one buffer fails without committing a baseline.
@pytest.mark.parametrize("count", [64, 65])
def test_event_resynchronization_buffer_bound(device, count):
    io, deps, clock = device
    io.open(RadioConfiguration(), deadline_monotonic_us=1000)
    for sequence in range(2, count + 2):
        queue_edge(deps, clock, sequence)
    if count == 65:
        with pytest.raises(RadioBackendError) as caught:
            io.resynchronize_events(deadline_monotonic_us=1000)
        assert caught.value.failure.code is Error.UNEXPECTED_IRQ
        assert io._last_sequence == 0 and len(deps.events) == 1
        with pytest.raises(RadioBackendError):
            io.wait_edge(deadline_monotonic_us=1000)
    io.resynchronize_events(deadline_monotonic_us=1000)
    assert io._last_sequence == count + 1 and not deps.events
    assert all(c[1].total_seconds() == 0 for c in deps.calls if c[0] == "wait_edge_events")


# Malformed, regressing or future recovery evidence cannot become a new trusted baseline.
@pytest.mark.parametrize("change", [
    {"line_offset": 24}, {"event_type": "fall"}, {"line_seqno": 2},
    {"line_seqno": 1}, {"line_seqno": True}, {"line_seqno": 0},
    {"line_seqno": 1 << 64}, {"timestamp_ns": 99999},
    {"timestamp_ns": 101000}, {"timestamp_ns": True}, {"timestamp_ns": None},
])
def test_event_resynchronization_rejects_untrusted_metadata(device, change):
    io, deps, clock = device
    io.open(RadioConfiguration(), deadline_monotonic_us=1000)
    queue_edge(deps, clock, 2)
    with pytest.raises(RadioBackendError):
        io.wait_edge(deadline_monotonic_us=1000)
    queue_edge(deps, clock, 3, **change)
    with pytest.raises(RadioBackendError) as caught:
        io.resynchronize_events(deadline_monotonic_us=1000)
    assert caught.value.failure.code is Error.MALFORMED_RESPONSE
    assert io._last_sequence == 0
    calls = deps.calls.copy()
    with pytest.raises(RadioBackendError):
        io.resynchronize_events(deadline_monotonic_us=1000)
    assert deps.calls == calls


# Future-dated consumed evidence stays invalid even after the clock catches up.
def test_retained_future_edge_cannot_age_into_valid_evidence(device):
    io, deps, clock = device
    io.open(RadioConfiguration(), deadline_monotonic_us=1000)
    queue_edge(deps, clock, 2, timestamp_ns=101000)
    with pytest.raises(RadioBackendError):
        io.wait_edge(deadline_monotonic_us=1000)
    clock.advance_elapsed_us(1000)
    with pytest.raises(RadioBackendError) as caught:
        io.resynchronize_events(deadline_monotonic_us=2000)
    assert caught.value.failure.code is Error.MALFORMED_RESPONSE
    assert io._last_sequence == 0


# Before/after syscall bounds stop draining and do not commit a partial baseline.
@pytest.mark.parametrize("point", ["before", "wait_edge_events", "read_edge_events"])
def test_event_resynchronization_deadline(device, point):
    io, deps, clock = device
    io.open(RadioConfiguration(), deadline_monotonic_us=1000)
    queue_edge(deps, clock, 2)
    if point != "before":
        deps.hooks[point] = lambda: clock.advance_elapsed_us(901)
    before = deps.calls.copy()
    with pytest.raises(RadioBackendError) as caught:
        io.resynchronize_events(deadline_monotonic_us=100 if point == "before" else 1000)
    assert caught.value.failure.code is Error.DEADLINE and io._last_sequence == 0
    if point == "before":
        assert deps.calls == before
    deps.hooks.clear()
    io.resynchronize_events(deadline_monotonic_us=2000)
    assert io._last_sequence == 2


# Unknown read consumption cannot be repaired from an empty queue; polling errors can retry.
@pytest.mark.parametrize("point", ["wait_edge_events", "read_edge_events"])
def test_event_resynchronization_io_evidence(device, point):
    io, deps, clock = device
    io.open(RadioConfiguration(), deadline_monotonic_us=1000)
    queue_edge(deps, clock, 2)
    deps.hooks[point] = failure(OSError(errno.EIO, point))
    with pytest.raises(RadioBackendError) as caught:
        io.resynchronize_events(deadline_monotonic_us=1000)
    assert caught.value.failure.os_errno == errno.EIO
    deps.hooks.clear()
    if point == "read_edge_events":
        with pytest.raises(RadioBackendError):
            io.resynchronize_events(deadline_monotonic_us=1000)
        assert io._last_sequence == 0
    else:
        io.resynchronize_events(deadline_monotonic_us=1000)
        assert io._last_sequence == 2


# F-003: real Linux/backend/owner recovery delivers later packets, including immediate RX.
@pytest.mark.parametrize("hard", [False, True])
@pytest.mark.parametrize("queued", [False, True])
@pytest.mark.parametrize("immediate", [False, True])
def test_gap_recovery_restores_packet_delivery(component, hard, queued, immediate):
    radio, deps, chip, clock = component
    assert radio.initialize().state is State.RX_SINGLE
    chip.irq, chip.status = 2, 0x24
    queue_edge(deps, clock, 2)
    result = radio.receive(deadline_monotonic_us=clock.now_monotonic_us() + 1000)
    assert result.state is State.RECOVERING and result.packet is None
    assert radio.backend.io._last_sequence == 0
    if queued:
        queue_edge(deps, clock, 3)
    if hard:
        def fail_soft(_):
            del chip.hooks[0x80]
            raise RadioBackendError(RadioFailure(Error.IO, Stage.WRITE_COMMAND, os_errno=errno.EIO))
        chip.hooks[0x80] = fail_soft
    next_sequence = 4 if queued else 3
    sync_observations = []

    def observe_sync():
        if radio.backend.io._events_need_sync:
            sync_observations.append((chip.status, chip.irq))

    deps.hooks["wait_edge_events"] = observe_sync
    if immediate:
        def immediately_done(command):
            if command[0] == 0x82:
                chip.after_transfer = None
                chip.irq, chip.status = 2, 0x24
                queue_edge(deps, clock, next_sequence)
        chip.after_transfer = immediately_done
    result = radio.recover()
    assert result.state is (State.RX_EVENT_PENDING if immediate else State.RX_SINGLE)
    assert sync_observations and all(pair == (0x24, 0) for pair in sync_observations)
    assert len(result.episodes) == 1
    episode = result.episodes[0]
    assert episode.error_code is Error.MALFORMED_RESPONSE
    assert episode.context.trigger_detail.stage is Stage.WAIT_IRQ
    assert episode.context.soft_recovery_result.name == ("FAILED" if hard else "SUCCEEDED")
    assert episode.context.hard_recovery_result.name == ("SUCCEEDED" if hard else "NOT_ATTEMPTED")
    for sequence in (next_sequence, next_sequence + 1):
        if not (immediate and sequence == next_sequence):
            chip.irq, chip.status = 2, 0x24
            queue_edge(deps, clock, sequence)
        packet = radio.receive(deadline_monotonic_us=clock.now_monotonic_us() + 1000)
        assert packet.packet.frame == b"packet" and packet.episodes == ()
        assert radio.rearm().state is State.RX_SINGLE
    assert (radio.counters.recovery_attempts, radio.counters.recovery_successes, radio.counters.recovery_failures) == (1, 1, 0)


# Untrusted stream metadata exhausts the two levels with one original episode and no new RX.
@pytest.mark.parametrize("change", [{"line_offset": 24}, {"line_seqno": 1}, {"timestamp_ns": -1}])
def test_bad_stream_prevents_recovery_success(component, change):
    radio, deps, chip, clock = component
    assert radio.initialize().state is State.RX_SINGLE
    chip.irq, chip.status = 2, 0x24
    queue_edge(deps, clock, 2)
    assert radio.receive(deadline_monotonic_us=clock.now_monotonic_us()).state is State.RECOVERING
    queue_edge(deps, clock, 3, **change)
    result = radio.recover()
    assert result.state is State.RECOVERY_EXHAUSTED and len(result.episodes) == 1
    episode = result.episodes[0]
    assert episode.error_code is episode.context.last_recovery_error_code is Error.MALFORMED_RESPONSE
    assert episode.context.soft_recovery_result.name == episode.context.hard_recovery_result.name == "FAILED"
    assert (radio.counters.recovery_attempts, radio.counters.recovery_successes, radio.counters.recovery_failures) == (1, 0, 1)
    assert sum(command[0] == 0x82 for command in chip.commands) == 1
    assert sum(call[:2] == ("reset", True) for call in chip.calls) == 2
    calls = deps.calls.copy()
    assert radio.shutdown().episodes == () and deps.calls == calls


# Shutdown requested during the bounded stream primitive prevents the next SetRx.
def test_shutdown_at_event_resynchronization_boundary(component):
    radio, deps, chip, clock = component
    assert radio.initialize().state is State.RX_SINGLE
    chip.irq, chip.status = 2, 0x24
    queue_edge(deps, clock, 2)
    assert radio.receive(deadline_monotonic_us=clock.now_monotonic_us()).state is State.RECOVERING

    def stop_during_sync():
        if radio.backend.io._events_need_sync:
            radio.request_shutdown()

    deps.hooks["wait_edge_events"] = stop_during_sync
    result = radio.recover()
    assert result.state is State.SHUTDOWN and result.safe_shutdown is True
    assert len(result.episodes) == 1 and result.episodes[0].error_code is Error.MALFORMED_RESPONSE
    assert (radio.counters.recovery_successes, radio.counters.recovery_failures) == (0, 1)
    assert sum(command[0] == 0x82 for command in chip.commands) == 1


# A continuously refilled queue exhausts one buffer at each recovery level, never loops forever.
def test_event_flood_exhausts_bounded_recovery(component):
    radio, deps, chip, clock = component
    assert radio.initialize().state is State.RX_SINGLE
    chip.irq, chip.status = 2, 0x24
    queue_edge(deps, clock, 2)
    assert radio.receive(deadline_monotonic_us=clock.now_monotonic_us()).state is State.RECOVERING
    queue_edge(deps, clock, 3)
    sequence = 4

    def refill():
        nonlocal sequence
        queue_edge(deps, clock, sequence)
        sequence += 1

    deps.hooks["read_edge_events"] = refill
    result = radio.recover()
    assert result.state is State.RECOVERY_EXHAUSTED
    assert sequence == 132  # 64 consumed events in each of exactly two attempts.
    assert len(result.episodes) == 1 and result.episodes[0].error_code is Error.MALFORMED_RESPONSE
    assert result.episodes[0].context.last_recovery_error_code is Error.UNEXPECTED_IRQ
    assert (radio.counters.recovery_successes, radio.counters.recovery_failures) == (0, 1)
    assert sum(command[0] == 0x82 for command in chip.commands) == 1


# F-002: lifecycle evidence is bounded, immutable and carries every release failure.
def test_lifecycle_failure_value():
    primary = RadioFailure(Error.IO, Stage.CONFIGURE_SPI, os_errno=errno.EACCES)
    release = RadioFailure(Error.IO, Stage.DETACH_IRQ, os_errno=errno.EIO)
    value = RadioLifecycleFailure(primary, (release,))
    error = RadioLifecycleError(value)
    assert error.failure is primary and error.lifecycle is value
    with pytest.raises(AttributeError):
        value.primary_failure = release
    for first, rest, error_type in [
        (None, (), ValueError), (primary, (release,) * 3, ValueError),
        ("bad", (), TypeError), (primary, [release], TypeError),
        (primary, (None,), TypeError),
    ]:
        with pytest.raises(error_type):
            RadioLifecycleFailure(first, rest)


# F-002: the production owner retains acquisition plus both partial-release failures once.
@pytest.mark.parametrize("point", ["request_lines", "open_path", "property"])
@pytest.mark.parametrize("release_faults", [(), ("close",), ("release",), ("close", "release")])
@pytest.mark.parametrize("opening_errno", [errno.EACCES, errno.ENODEV])
def test_partial_open_owner_preserves_all_failures(component, point, release_faults, opening_errno):
    radio, deps, chip, _ = component
    deps.hooks[point] = failure(OSError(opening_errno, "opening"))
    numbers = {"close": errno.EBADF, "release": errno.EIO}
    for name in release_faults:
        deps.hooks[name] = failure(OSError(numbers[name], name))
    result = radio.initialize()
    expected = State.HARDWARE_MISSING if opening_errno == errno.ENODEV else State.INITIALIZATION_FAILED
    releases = release_faults if point != "request_lines" else ()
    assert result.state is expected
    assert result.safe_shutdown is (False if releases else None)
    assert [e.operation.name for e in result.episodes] == ["INITIALIZE"] + ["CLEANUP"] * len(releases)
    assert [e.context.trigger_detail.backend_status for e in result.episodes] == [opening_errno] + [numbers[n] for n in releases]
    assert [e.context.trigger_detail.stage for e in result.episodes[1:]] == [
        Stage.CONFIGURE_SPI if n == "close" else Stage.DETACH_IRQ for n in releases
    ]
    assert all(e.severity.name == "FATAL" and e.context.terminal_state is expected for e in result.episodes)
    assert [c for c in deps.calls if c[0] in numbers] == ([] if point == "request_lines" else [("close",), ("release",)])
    assert chip.commands == []
    before = deps.calls.copy()
    assert radio.shutdown().episodes == ()
    assert radio.shutdown().safe_shutdown is result.safe_shutdown
    radio.backend.close()
    assert deps.calls == before


# A complete open and a failed open use the same all-release evidence boundary.
@pytest.mark.parametrize("opening_fails", [False, True])
def test_lifecycle_error_preserves_primary_and_release_tuple(device, opening_fails):
    io, deps, _ = device
    if opening_fails:
        deps.hooks["open_path"] = failure(OSError(errno.EACCES, "opening"))
    else:
        io.open(RadioConfiguration(), deadline_monotonic_us=1000)
    deps.hooks["close"] = failure(OSError(errno.EBADF, "close"))
    deps.hooks["release"] = failure(OSError(errno.EIO, "release"))
    with pytest.raises(RadioLifecycleError) as caught:
        if opening_fails:
            io.open(RadioConfiguration(), deadline_monotonic_us=1000)
        else:
            io.close()
    value = caught.value.lifecycle
    assert (value.primary_failure.os_errno if value.primary_failure else None) == (errno.EACCES if opening_fails else None)
    assert [(f.os_errno, f.stage) for f in value.release_failures] == [
        (errno.EBADF, Stage.CONFIGURE_SPI), (errno.EIO, Stage.DETACH_IRQ),
    ]
    assert deps.calls[-2:] == [("close",), ("release",)]
    before = deps.calls.copy()
    io.close()
    assert deps.calls == before


# Full-open startup failure and controlled shutdown also retain both release causes.
@pytest.mark.parametrize("operation", ["startup", "shutdown", "shutdown_with_standby_failure"])
def test_owner_complete_release_failures(component, operation):
    radio, deps, chip, _ = component
    primary = RadioBackendError(RadioFailure(Error.IO, Stage.WRITE_COMMAND, os_errno=errno.EACCES))
    if operation == "startup":
        chip.hooks[0x8A] = lambda _: failure(primary)()
    else:
        assert radio.initialize().state is State.RX_SINGLE
        if operation == "shutdown_with_standby_failure":
            chip.hooks[0x80] = lambda _: failure(primary)()
    deps.hooks["close"] = failure(OSError(errno.EBADF, "close"))
    deps.hooks["release"] = failure(OSError(errno.EIO, "release"))
    result = radio.initialize() if operation == "startup" else radio.shutdown()
    expected = State.INITIALIZATION_FAILED if operation == "startup" else State.SHUTDOWN
    assert result.state is expected and result.safe_shutdown is False
    assert [e.context.trigger_detail.backend_status for e in result.episodes] == (
        ([] if operation == "shutdown" else [errno.EACCES]) + [errno.EBADF, errno.EIO]
    )
    assert all(e.severity.name == "FATAL" for e in result.episodes)
    assert all(e.operation.name == "CLEANUP" for e in result.episodes[1 if operation == "startup" else 0:])
    assert deps.calls.count(("close",)) == deps.calls.count(("release",)) == 1
    before = deps.calls.copy()
    assert radio.shutdown().episodes == () and deps.calls == before


# Unexpected failures remain CORE exceptions without concealing other lifecycle evidence.
@pytest.mark.parametrize("unexpected_at", ["open_path", "close", "release"])
def test_mixed_lifecycle_exceptions_propagate(component, unexpected_at):
    radio, deps, _, _ = component
    unexpected = RuntimeError("implementation bug")
    for name, number in [("open_path", errno.EACCES), ("close", errno.EBADF), ("release", errno.EIO)]:
        deps.hooks[name] = failure(unexpected if name == unexpected_at else OSError(number, name))
    with pytest.raises(BaseExceptionGroup) as caught:
        radio.initialize()
    normalized, bug = caught.value.exceptions
    assert bug is unexpected and isinstance(normalized, RadioLifecycleError)
    assert (normalized.lifecycle.primary_failure is None) is (unexpected_at == "open_path")
    assert len(normalized.lifecycle.release_failures) == (2 if unexpected_at == "open_path" else 1)
    assert deps.calls[-2:] == [("close",), ("release",)]
    before = deps.calls.copy()
    radio.backend.close()
    assert deps.calls == before


# Open requests all lines atomically with kernel monotonic edges and fixed SPI.
def test_open_configuration_and_active_low_reset(device):
    io, deps, _ = device
    io.open(RadioConfiguration(), deadline_monotonic_us=1000)
    config = deps.calls[0][2]
    assert config["config"] == {
        22: dict(direction="out", drive="open-drain", output_value=Value.ACTIVE, bias="as-is"),
        24: dict(direction="in", bias="as-is"),
        23: dict(direction="in", edge_detection="rising", event_clock="monotonic", bias="as-is"),
    }
    assert config["event_buffer_size"] == 64
    assert deps.calls[1:] == [
        ("open_path", "/dev/spidev0.0"), ("property", "mode", 0),
        ("property", "max_speed_hz", 1000000), ("property", "bits_per_word", 8),
        ("property", "lsbfirst", False), ("property", "no_cs", False),
        ("property", "cshigh", False), ("property", "threewire", False),
        ("property", "loop", False),
    ]
    io.set_reset(asserted=True)
    io.set_reset(asserted=False)
    assert deps.calls[-2:] == [("set_value", 22, Value.INACTIVE), ("set_value", 22, Value.ACTIVE)]
    io.close()
    io.close()
    assert deps.calls[-2:] == [("close",), ("release",)]
    with pytest.raises(RuntimeError):
        io.open(RadioConfiguration(), deadline_monotonic_us=1000)


# Run the actual manual hardware case through Linux/SX1262/owner code at the dependency seam.
@pytest.mark.parametrize("release_fails", [False, True])
def test_manual_hardware_case_keeps_fault_and_cleanup_evidence(device, tmp_path, monkeypatch, release_fails):
    from tests.hardware import test_radio as hardware_radio
    from tests.support.fakes.radio_io import Wait

    _, deps, clock = device
    deps.level = Value.ACTIVE
    if release_fails:
        deps.hooks["release"] = failure(OSError(errno.EIO, "release failed"))
    monkeypatch.setattr(clock, "wait_until_monotonic_us", Wait(clock).wait_until_monotonic_us, raising=False)
    monkeypatch.setattr(hardware_radio, "LinuxOsClock", lambda: clock)
    request = NS(node=NS(name="test_manual_busy_held_startup"))
    fixture = hardware_radio.radio_component.__wrapped__(request, (tmp_path, RadioConfiguration()))
    component = next(fixture)
    if release_fails:
        with pytest.raises(AssertionError):
            hardware_radio.test_manual_busy_held_startup(component)
        assert not (tmp_path / "held-busy-observation.json").exists()
    else:
        hardware_radio.test_manual_busy_held_startup(component)
        observation = json.loads((tmp_path / "held-busy-observation.json").read_text())
        assert observation == {
            "expected_fault_observed": True, "safe_shutdown_confirmed": False,
            "manual_restoration_required": True,
        }
    with pytest.raises(pytest.exit.Exception) as stopped:
        next(fixture)
    assert stopped.value.returncode == 1
    assert (tmp_path / "manual-restoration-required.json").exists()
    assert deps.calls[-2:] == [("close",), ("release",)]
    assert not any(call[0] == "xfer2" for call in deps.calls)
    assert deps.calls[0][2]["config"][24]["direction"] == "in"


# Each failed acquisition preserves errno and releases exactly acquired resources.
@pytest.mark.parametrize("point,cleanup", [
    ("request_lines", []), ("open_path", [("close",), ("release",)]),
    ("property", [("close",), ("release",)]),
])
@pytest.mark.parametrize("number,missing", [(errno.EACCES, False), (errno.ENOENT, True), (errno.EIO, False)])
def test_partial_acquisition_failure(device, point, cleanup, number, missing):
    io, deps, _ = device
    deps.hooks[point] = failure(OSError(number, "injected"))
    with pytest.raises(RadioBackendError) as caught:
        io.open(RadioConfiguration(), deadline_monotonic_us=1000)
    assert caught.value.failure.os_errno == number
    assert caught.value.failure.hardware_missing is missing
    assert [call for call in deps.calls if call[0] in ("close", "release")] == cleanup


# A cleanup exception cannot prevent the second resource's release or invent status.
@pytest.mark.parametrize("error", [OSError(errno.EIO, "close"), RuntimeError("implementation")])
def test_close_attempts_both_resources(device, error):
    io, deps, _ = device
    io.open(RadioConfiguration(), deadline_monotonic_us=1000)
    deps.hooks["close"] = failure(error)
    with pytest.raises(RadioBackendError if isinstance(error, OSError) else RuntimeError):
        io.close()
    assert deps.calls[-2:] == [("close",), ("release",)]
    io.close()


# xfer2 holds CS for one transaction and returned storage is copied immediately.
def test_spi_transaction_and_copy(device):
    io, deps, _ = device
    io.open(RadioConfiguration(), deadline_monotonic_us=1000)
    result = io.transfer(b"\xc0\x00", deadline_monotonic_us=1000)
    deps.received[1] = 255
    assert result == b"\x00\x24"
    assert deps.calls[-1] == ("xfer2", [192, 0])


# Invalid dependency responses cannot become apparently valid status bytes.
@pytest.mark.parametrize("response", [None, (), [], [0], [0, 256], [0, True], [0, -1]])
def test_malformed_spi_response(device, response):
    io, deps, _ = device
    io.open(RadioConfiguration(), deadline_monotonic_us=1000)
    deps.received = response
    with pytest.raises(RadioBackendError) as caught:
        io.transfer(b"\xc0\x00", deadline_monotonic_us=1000)
    assert caught.value.failure.code is Error.MALFORMED_RESPONSE
    assert caught.value.failure.outcome is Outcome.UNCERTAIN


# A pre-call deadline forbids submission; an overrun or ioctl error is uncertain.
@pytest.mark.parametrize("fault", ["before", "after", "io"])
def test_transfer_certainty(device, fault):
    io, deps, clock = device
    io.open(RadioConfiguration(), deadline_monotonic_us=1000)
    if fault == "after":
        deps.hooks["xfer2"] = lambda: clock.advance_elapsed_us(901)
    elif fault == "io":
        deps.hooks["xfer2"] = failure(OSError(errno.EIO, "transfer"))
    with pytest.raises(RadioBackendError) as caught:
        io.transfer(b"\x83\x00", deadline_monotonic_us=100 if fault == "before" else 1000)
    assert caught.value.failure.outcome is (Outcome.DEFINITELY_NOT_APPLIED if fault == "before" else Outcome.UNCERTAIN)
    assert (deps.calls[-1][0] == "xfer2") is (fault != "before")


# Queued edges preserve nanoseconds even when their timestamps exceed the bound.
def test_edge_timestamp_and_bounded_idle_wait(device):
    io, deps, clock = device
    io.open(RadioConfiguration(), deadline_monotonic_us=1000)
    deps.events.append(NS(event_type="rise", line_offset=23, line_seqno=1, timestamp_ns=1000001))
    edge = io.wait_edge(deadline_monotonic_us=1000)
    assert (edge.timestamp_ns, edge.monotonic_us, edge.sequence) == (1000001, 1000, 1)
    assert io.wait_edge(deadline_monotonic_us=2000100) is None
    assert clock.now_monotonic_us() == 2000100
    assert all(call[1].total_seconds() <= 1 for call in deps.calls if call[0] == "wait_edge_events")


# Wrong lines, edge kinds, lost events and invalid timestamps invalidate evidence.
@pytest.mark.parametrize("change", [
    {"line_offset": 24}, {"event_type": "fall"}, {"line_seqno": 2},
    {"line_seqno": True}, {"timestamp_ns": -1}, {"timestamp_ns": True},
])
def test_edge_validation(device, change):
    io, deps, _ = device
    io.open(RadioConfiguration(), deadline_monotonic_us=1000)
    fields = dict(event_type="rise", line_offset=23, line_seqno=1, timestamp_ns=101000)
    deps.events.append(NS(**(fields | change)))
    with pytest.raises(RadioBackendError) as caught:
        io.wait_edge(deadline_monotonic_us=1000)
    assert (caught.value.failure.code, caught.value.failure.stage) == (Error.MALFORMED_RESPONSE, Stage.WAIT_IRQ)


# A setter reaching the exact bound prevents the next hardware configuration call.
def test_configuration_deadline_between_calls(device):
    io, deps, clock = device
    deps.hooks["property"] = lambda: clock.advance_elapsed_us(900)
    with pytest.raises(RadioBackendError):
        io.open(RadioConfiguration(), deadline_monotonic_us=1000)
    assert [c for c in deps.calls if c[0] == "property"] == [("property", "mode", 0)]
    assert deps.calls[-2:] == [("close",), ("release",)]


# The real wait adapter rechecks early wakeups, without sleeping in this host test.
def test_linux_wait_rechecks_absolute_deadline(monkeypatch):
    clock = linux_clocks.LinuxOsClock()
    observed = iter([100000, 120000, 150000])
    sleeps = []
    monkeypatch.setattr(linux_clocks.time, "clock_gettime_ns", lambda _: next(observed))
    monkeypatch.setattr(linux_clocks.time, "sleep", sleeps.append)
    clock.wait_until_monotonic_us(150)
    assert sleeps == [0.00005, 0.00003]


# A different thread is rejected before it can access either hardware dependency.
@pytest.mark.parametrize("method", ["busy", "resynchronize_events"])
def test_single_owner(device, method):
    io, deps, _ = device
    io.open(RadioConfiguration(), deadline_monotonic_us=1000)
    before = deps.calls.copy()
    errors = []

    def contender():
        try:
            getattr(io, method)(**({"deadline_monotonic_us": 1000} if method == "resynchronize_events" else {}))
        except BaseException as error:
            errors.append(error)

    worker = Thread(target=contender)
    worker.start()
    worker.join(5)
    assert not worker.is_alive()
    assert len(errors) == 1 and type(errors[0]) is RuntimeError
    assert deps.calls == before


# Every GPIO dependency failure preserves its primitive and the exact host errno.
@pytest.mark.parametrize("method,point,stage", [
    ("busy", "get_value", Stage.WAIT_BUSY),
    ("dio1", "get_value", Stage.READ_IRQ),
    ("wait_edge", "wait_edge_events", Stage.WAIT_IRQ),
    ("set_reset", "set_value", Stage.RESET),
])
def test_gpio_errno(device, method, point, stage):
    io, deps, _ = device
    io.open(RadioConfiguration(), deadline_monotonic_us=1000)
    deps.hooks[point] = failure(OSError(errno.ENODEV, "disconnected"))
    kwargs = {"deadline_monotonic_us": 1000} if method == "wait_edge" else {"asserted": True} if method == "set_reset" else {}
    with pytest.raises(RadioBackendError) as caught:
        getattr(io, method)(**kwargs)
    assert caught.value.failure.stage is stage
    assert caught.value.failure.os_errno == errno.ENODEV
    assert caught.value.failure.hardware_missing
