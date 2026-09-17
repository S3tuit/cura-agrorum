"""Non-peer component hardware only. No case submits SetTx or proves RF timing."""

from dataclasses import asdict, is_dataclass
from enum import Enum
import json

import pytest

from cura_receiver.platform.linux_clocks import LinuxOsClock
from cura_receiver.platform.linux_radio import LinuxRadioIo
from cura_receiver.radio import Radio, State
from cura_receiver.ports.radio import Error, Stage
from cura_receiver.radio_diagnostics import Operation, Severity
from cura_receiver.sx1262 import Sx1262
from tests.hardware.radio_fixture import create_evidence

pytestmark = [pytest.mark.hardware, pytest.mark.radio]


def encoded(value):
    return asdict(value) if is_dataclass(value) else value.name if isinstance(value, Enum) else value.hex() if isinstance(value, bytes) else str(value)


class TracedIo(LinuxRadioIo):
    """Record actual dependency effects without substituting responses or time."""

    def __init__(self, clock, stream):
        super().__init__(clock)
        self.stream = stream
        self.on_reset = None

    def record(self, name, call, **fields):
        started = self.clock.now_monotonic_us()
        try:
            result = call()
        except BaseException as error:
            self.stream.write(json.dumps(dict(operation=name, start_us=started,
                end_us=self.clock.now_monotonic_us(), error=repr(error), **fields)) + "\n")
            self.stream.flush()
            raise
        self.stream.write(json.dumps(dict(operation=name, start_us=started,
            end_us=self.clock.now_monotonic_us(), result=result, **fields), default=encoded) + "\n")
        self.stream.flush()
        return result

    def transfer(self, data, *, deadline_monotonic_us):
        assert data[0] != 0x83, "non-peer hardware cases must never transmit"
        return self.record("spi", lambda: super(TracedIo, self).transfer(data, deadline_monotonic_us=deadline_monotonic_us), tx=data.hex(), deadline_us=deadline_monotonic_us)

    def busy(self):
        return self.record("busy", lambda: super(TracedIo, self).busy())

    def wait_edge(self, *, deadline_monotonic_us):
        return self.record("edge", lambda: super(TracedIo, self).wait_edge(deadline_monotonic_us=deadline_monotonic_us), deadline_us=deadline_monotonic_us)

    def resynchronize_events(self, *, deadline_monotonic_us):
        return self.record("resynchronize_events", lambda: super(TracedIo, self).resynchronize_events(deadline_monotonic_us=deadline_monotonic_us), deadline_us=deadline_monotonic_us)

    def set_reset(self, *, asserted):
        result = self.record("reset", lambda: super(TracedIo, self).set_reset(asserted=asserted), asserted=asserted)
        if asserted and self.on_reset is not None:
            self.on_reset()
        return result

    def close(self):
        return self.record("close", lambda: super(TracedIo, self).close())


@pytest.fixture(scope="module")
def radio_evidence(request):
    return create_evidence(request.config)


@pytest.fixture
def radio_component(request, radio_evidence):
    root, configuration = radio_evidence
    name = request.node.name.replace("[", "-").replace("]", "")
    clock = LinuxOsClock()
    with (root / (name + ".jsonl")).open("x") as stream:
        io = TracedIo(clock, stream)
        radio = Radio(Sx1262(io, clock, clock, configuration))
        try:
            yield radio, io, clock, root
        finally:
            result = radio.shutdown()
            (root / (name + "-teardown.json")).write_text(json.dumps(asdict(result), default=encoded, indent=2) + "\n")
            if result.safe_shutdown is not True:
                (root / "manual-restoration-required.json").write_text(json.dumps({
                    "safe_shutdown": result.safe_shutdown,
                    "terminal_state": result.state.name,
                    "restoration": "required: shut down Pi, remove external power, restore selector 1-2, reboot and run nominal checks",
                }, indent=2) + "\n")
                pytest.exit("radio safe-state teardown unconfirmed; stop all later hardware cases and restore nominal wiring unpowered", returncode=1)


@pytest.fixture
def attached_radio(request, radio_component):
    radio, _, _, root = radio_component
    result = radio.initialize()
    (root / (request.node.name + "-initialization.json")).write_text(json.dumps(asdict(result), default=encoded, indent=2) + "\n")
    assert result.state is State.RX_SINGLE, result
    return radio_component


# The actual service user opens the deployed devices with configured directions/clock and SPI.
def test_device_permission_and_configuration(attached_radio):
    import gpiod
    radio, io, _, root = attached_radio
    with gpiod.Chip(radio.backend.configuration.gpio_chip) as chip:
        reset = chip.get_line_info(22)
        dio = chip.get_line_info(23)
        busy = chip.get_line_info(24)
        assert reset.direction is gpiod.line.Direction.OUTPUT
        assert reset.drive is gpiod.line.Drive.OPEN_DRAIN
        assert dio.direction is busy.direction is gpiod.line.Direction.INPUT
        assert dio.edge_detection is gpiod.line.Edge.RISING
        assert dio.event_clock is gpiod.line.Clock.MONOTONIC
        assert all(info.consumer == "cura-radio" for info in (reset, dio, busy))
        (root / "line-info.txt").write_text("\n".join(str(info) for info in (reset, dio, busy)) + "\n")
    assert (io._spi.mode, io._spi.bits_per_word, io._spi.max_speed_hz) == (0, 8, 1000000)


# Real register readbacks and chip mode verify the complete initialized receive configuration.
def test_initialization_readback(attached_radio):
    radio, _, _, _ = attached_radio
    backend = radio.backend
    deadline = backend.deadline(500000)
    assert backend.read_register(0x0740, 2, deadline) == b"\x14\x24"
    assert backend.read_register(0x08AC, 1, deadline) == b"\x96"
    assert backend.read_register(0x0736, 1, deadline)[0] & 4
    assert backend._status(deadline) >> 4 & 7 == 5
    assert backend.read_device_errors(deadline) == 0


# Finite RX timeout creates a real non-peer IRQ anomaly, followed by complete owner rearm.
def test_finite_rx_irq_and_direct_restore(attached_radio):
    radio, _, clock, _ = attached_radio
    backend = radio.backend
    deadline = backend.deadline(500000)
    backend.install_profile(transmit=False, payload_length=255, deadline=deadline)
    backend.arm_receive(deadline, timeout_ticks=6400)
    result = radio.receive(deadline_monotonic_us=clock.now_monotonic_us() + 300000)
    assert result.state is State.RX_SINGLE and len(result.episodes) == 1
    assert result.episodes[0].error_code.name == "UNEXPECTED_IRQ"
    assert result.episodes[0].context.trigger_detail.irq_status == 0x200


# This is a startup fault; cleanup remains unconfirmed until manual power-off restoration.
@pytest.mark.destructive
@pytest.mark.radio_busy_held
def test_manual_busy_held_startup(radio_component):
    radio, io, clock, root = radio_component
    started = clock.now_monotonic_us()
    result = radio.initialize()
    elapsed = clock.now_monotonic_us() - started
    (root / "held-busy-initialization.json").write_text(json.dumps({
        "result": asdict(result), "elapsed_us": elapsed,
    }, default=encoded, indent=2) + "\n")
    assert result.state is State.INITIALIZATION_FAILED, result
    assert len(result.episodes) == 2
    episode = result.episodes[0]
    assert episode.operation is Operation.INITIALIZE
    assert episode.error_code is Error.BUSY_TIMEOUT
    assert episode.severity is Severity.FATAL
    assert episode.context.trigger_detail.stage is Stage.WAIT_BUSY
    cleanup = result.episodes[1]
    assert cleanup.operation is Operation.CLEANUP
    assert cleanup.error_code is Error.BUSY_TIMEOUT
    assert cleanup.severity is Severity.FATAL
    assert cleanup.context.trigger_detail.stage is Stage.WAIT_BUSY
    assert result.busy.timeout_count == 2
    assert result.tx is None and result.t6_set_rx_issued_monotonic_us is None
    assert result.safe_shutdown is False
    # Approved 2 s startup bound plus 50 ms Linux service-latency allowance.
    assert 0 < elapsed <= 2_050_000
    io.stream.flush()
    events = [json.loads(line) for line in (root / "test_manual_busy_held_startup.jsonl").read_text().splitlines()]
    busy_samples = [event["result"] for event in events if event["operation"] == "busy"]
    assert busy_samples and all(value is True for value in busy_samples)
    assert not any(event["operation"] == "spi" for event in events)
    assert any(event["operation"] == "close" and "error" not in event for event in events)
    assert not any("error" in event for event in events)
    (root / "held-busy-observation.json").write_text(json.dumps({
        "expected_fault_observed": True, "safe_shutdown_confirmed": False,
        "manual_restoration_required": True,
    }, indent=2) + "\n")


# A real isolated BUSY gate forces bounded soft/hard recovery; its release is physical.
@pytest.mark.destructive
@pytest.mark.radio_fault
@pytest.mark.parametrize("hard", [False, True], ids=("soft", "reset"))
def test_physical_busy_recovery(attached_radio, hard):
    import gpiod
    radio, io, clock, root = attached_radio
    with gpiod.request_lines(radio.backend.configuration.gpio_chip, consumer="cura-radio-fault",
        config={5: gpiod.LineSettings(direction=gpiod.line.Direction.OUTPUT, output_value=gpiod.line.Value.INACTIVE)}) as gate:
        backend = radio.backend
        deadline = backend.deadline(500000)
        backend.install_profile(transmit=False, payload_length=255, deadline=deadline)
        backend.arm_receive(deadline, timeout_ticks=6400)
        try:
            gate.set_value(5, gpiod.line.Value.ACTIVE)
            assert io.busy()
            assert radio.receive(deadline_monotonic_us=clock.now_monotonic_us() + 300000).state is State.RECOVERING
            release = lambda: gate.set_value(5, gpiod.line.Value.INACTIVE)
            if hard:
                io.on_reset = release
            else:
                release()
            result = radio.recover()
            (root / ("recovery-reset.json" if hard else "recovery-soft.json")).write_text(json.dumps(asdict(result), default=encoded, indent=2) + "\n")
            assert result.state is State.RX_SINGLE
            assert len(result.episodes) == 1
            assert result.episodes[0].context.hard_recovery_result.name == ("SUCCEEDED" if hard else "NOT_ATTEMPTED")
            assert backend._status(backend.deadline(500000)) >> 4 & 7 == 5
        finally:
            io.on_reset = None
            gate.set_value(5, gpiod.line.Value.INACTIVE)
