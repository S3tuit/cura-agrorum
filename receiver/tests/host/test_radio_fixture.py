"""Fixture authorization/configuration checks without opening any GPIO/SPI device."""

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from tests.hardware import radio_fixture
from tests.hardware import conftest as hardware_config


@pytest.fixture
def selection(tmp_path, monkeypatch):
    monkeypatch.setattr(radio_fixture.os, "geteuid", lambda: 1000)
    monkeypatch.setattr(radio_fixture.pwd, "getpwuid", lambda _: SimpleNamespace(pw_name="cura"))
    path = tmp_path / "fixture.json"
    value = dict(schema=2, board_id="operator-board-id", service_user="cura", wiring_checked=True,
                 power_checked=True, no_pico_fitted=True, receiver_service_stopped=True,
                 fixture_state="radio_nominal", busy_selector_checked=True, configuration={})
    path.write_text(json.dumps(value))
    options = dict(receiver_radio_fixture=str(path), receiver_radio_evidence=str(tmp_path / "new-evidence"), receiver_hardware=True)
    config = SimpleNamespace(getoption=options.get)
    return config, path, value, options


# Valid nominal confirmation authorizes only the documented unprivileged carrier mapping.
def test_valid_nominal_selection(selection):
    config, _, _, _ = selection
    _, configuration, root = radio_fixture.validate_selection(config)
    assert (configuration.reset_line, configuration.dio1_line, configuration.busy_line) == (22, 23, 24)
    assert not root.exists()


# Missing confirmations, foreign pins and unapproved physical faults fail before setup.
@pytest.mark.parametrize("change", [
    {"schema": True}, {"schema": 1}, {"wiring_checked": False}, {"power_checked": False},
    {"no_pico_fitted": False}, {"receiver_service_stopped": False}, {"service_user": "root"},
    {"board_id": ""}, {"configuration": {"reset_line": 17}},
    {"configuration": {"spi_speed_hz": 2000000}}, {"busy_fault_gate_fitted": True}, {"extra": 1},
    {"busy_selector_checked": False}, {"busy_selector_checked": 1},
    {"fixture_state": "radio_fault_ready"}, {"fixture_state": None},
])
def test_invalid_fixture(selection, change):
    config, path, value, _ = selection
    path.write_text(json.dumps(value | change))
    with pytest.raises(pytest.UsageError):
        radio_fixture.validate_selection(config)


# Each manual state authorizes only its own cases; neither is an implicit gate confirmation.
@pytest.mark.parametrize("state", ["radio_nominal", "radio_busy_held"])
def test_manual_state_selection(selection, state):
    config, path, value, _ = selection
    path.write_text(json.dumps(value | {"fixture_state": state}))
    radio_fixture.validate_selection(config, required_state=state)
    other = "radio_nominal" if state == "radio_busy_held" else "radio_busy_held"
    with pytest.raises(pytest.UsageError, match="require fixture_state"):
        radio_fixture.validate_selection(config, required_state=other)


# Existing evidence, absent options and root execution cannot overwrite or mislabel acceptance.
@pytest.mark.parametrize("fault", ["existing", "missing", "root"])
def test_evidence_and_user_interlocks(selection, monkeypatch, fault):
    config, path, _, options = selection
    if fault == "existing":
        options["receiver_radio_evidence"] = str(path.parent)
    elif fault == "missing":
        options["receiver_radio_fixture"] = None
    else:
        monkeypatch.setattr(radio_fixture.os, "geteuid", lambda: 0)
    with pytest.raises(pytest.UsageError):
        radio_fixture.validate_selection(config)


# Collection authorizes selected radio fixtures before checking the target or opening devices.
def test_collection_interlock_precedes_target_probe(selection, monkeypatch):
    config, _, _, options = selection
    options["receiver_radio_fixture"] = None
    item = SimpleNamespace(get_closest_marker=lambda name: name in ("hardware", "radio"))
    def unexpected_target_probe():
        raise AssertionError("target probe reached before fixture authorization")
    monkeypatch.setattr(hardware_config, "_require_target_raspberry_pi", unexpected_target_probe)
    with pytest.raises(pytest.UsageError, match="receiver-radio-fixture"):
        hardware_config.pytest_collection_finish(SimpleNamespace(items=[item], config=config))


def marked_item(*markers):
    return SimpleNamespace(get_closest_marker=lambda name: True if name in markers else None)


# Static fault runs cannot include another case, even an unrelated hardware/host test.
@pytest.mark.parametrize("other", [
    (), ("hardware",), ("hardware", "radio"), ("hardware", "radio", "radio_busy_held"),
])
def test_manual_fault_run_is_exclusive(selection, other):
    config, _, _, _ = selection
    held = marked_item("hardware", "radio", "radio_busy_held", "destructive")
    with pytest.raises(pytest.UsageError, match="must run alone"):
        hardware_config.pytest_collection_finish(SimpleNamespace(items=[held, marked_item(*other)], config=config))


# Gate tests remain explicit unmet obligations, never accepted using the new manual selector.
def test_gate_selection_is_deferred(selection):
    config, _, _, _ = selection
    gate = marked_item("hardware", "radio", "radio_fault", "destructive")
    with pytest.raises(pytest.UsageError, match="gate tests are deferred"):
        hardware_config.pytest_collection_finish(SimpleNamespace(items=[gate], config=config))


# The actual held-case marker requires the held-state confirmation before any target access.
def test_collection_requires_matching_state(selection):
    config, _, _, _ = selection
    held = marked_item("hardware", "radio", "radio_busy_held", "destructive")
    with pytest.raises(pytest.UsageError, match="require fixture_state radio_busy_held"):
        hardware_config.pytest_collection_finish(SimpleNamespace(items=[held], config=config))


# A valid single held case still requires destructive opt-in and the dedicated sentinel root.
def test_held_case_preserves_destructive_interlocks(selection, monkeypatch):
    config, path, value, options = selection
    path.write_text(json.dumps(value | {"fixture_state": "radio_busy_held"}))
    held = marked_item("hardware", "radio", "radio_busy_held", "destructive")
    session = SimpleNamespace(items=[held], config=config)
    with pytest.raises(pytest.UsageError, match="confirm-receiver-destructive"):
        hardware_config.pytest_collection_finish(session)
    options["confirm_receiver_destructive"] = True
    with pytest.raises(pytest.UsageError, match="receiver-test-root"):
        hardware_config.pytest_collection_finish(session)
    options["receiver_test_root"] = str(path.parent)
    (path.parent / ".cura-receiver-test-root").write_text("CURA AGRORUM RECEIVER TEST ROOT\n")
    probes = []
    monkeypatch.setattr(hardware_config, "_require_target_raspberry_pi", lambda: probes.append("target"))
    hardware_config.pytest_collection_finish(session)
    assert probes == ["target"]


# Retained provenance binds the shared rail/selector schematic and physical procedure too.
def test_evidence_binds_manual_fixture(selection):
    config, _, _, _ = selection
    root, _ = radio_fixture.create_evidence(config)
    fixture = json.loads((root / "fixture.json").read_text())
    assert fixture["schema"] == 2 and fixture["fixture_state"] == "radio_nominal"
    manifest = json.loads((root / "sources.json").read_text())
    repo = Path(__file__).resolve().parents[3]
    for name in ("receiver/hardware/TEST_CARRIER.md", "receiver/tests/hardware/RADIO_TESTS.md"):
        assert manifest[name] == radio_fixture.hashlib.sha256((repo / name).read_bytes()).hexdigest()


# The real owner over its physical fake cannot turn expected startup failure into safe cleanup.
def test_held_case_teardown_stops_with_restoration_evidence(tmp_path, monkeypatch):
    from tests.hardware import test_radio as hardware_radio
    from cura_receiver.ports.radio import RadioConfiguration
    from cura_receiver.radio import State
    from tests.support.fakes.os_clock import FakeOsClock
    from tests.support.fakes.radio_io import PhysicalPort, Wait

    class Clock(FakeOsClock):
        def wait_until_monotonic_us(self, deadline_monotonic_us):
            Wait(self).wait_until_monotonic_us(deadline_monotonic_us)

    clock = Clock()
    io = PhysicalPort(clock)
    io.busy_forever = True
    monkeypatch.setattr(hardware_radio, "LinuxOsClock", lambda: clock)
    monkeypatch.setattr(hardware_radio, "TracedIo", lambda *_: io)
    request = SimpleNamespace(node=SimpleNamespace(name="held-probe"))
    fixture = hardware_radio.radio_component.__wrapped__(request, (tmp_path, RadioConfiguration()))
    radio, _, _, _ = next(fixture)
    assert radio.initialize().state is State.INITIALIZATION_FAILED
    with pytest.raises(pytest.exit.Exception, match="restore nominal wiring unpowered") as stopped:
        next(fixture)
    assert stopped.value.returncode == 1
    teardown = json.loads((tmp_path / "held-probe-teardown.json").read_text())
    assert teardown["safe_shutdown"] is False
    pending = json.loads((tmp_path / "manual-restoration-required.json").read_text())
    assert pending["safe_shutdown"] is False
    assert pending["terminal_state"] == "INITIALIZATION_FAILED"
    assert io.calls[-1] == ("close",)
    assert io.commands == []
