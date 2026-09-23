import importlib.util
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

from inputs import APP, REPO
from spec import EPISODES, select_cases, validate_fixture

definition = importlib.util.spec_from_file_location("rf_collection_guard", Path(__file__).resolve().parents[1] / "conftest.py")
guard = importlib.util.module_from_spec(definition)
definition.loader.exec_module(guard)


@pytest.fixture
def fixture():
    return dict(schema=1, c6_fixture="nominal", c6_dut="cc8da2fc0224", c6_uart="/dev/ttyUSB0",
                pi_user="cura", pi_board_id="00000000e0027211", pi_fixture="radio_nominal",
                rtc_shunts_open=True, receiver_service_stopped=True, exclusive_radios=True,
                wiring_checked=True, power_checked=True, antennas_checked=True, operator="test operator",
                c6_transmitter="node module", pi_transmitter="Pi module", operating_envelope_reference="approved test record",
                manual_fault_confirmed=False)


@pytest.mark.parametrize("selection,fixture_name", [(None, "nominal"), ("", "nominal"),
    ("RF-001", "nominal"), ("RF-020.nominal", "nominal"), ("RF-001.exchange,RF-001.exchange", "nominal"),
    ("RF-012.disconnected", "nominal"), ("RF-001.exchange,RF-013.absent", "nominal")])
def test_selection_fails_closed(selection, fixture_name):
    with pytest.raises(ValueError):
        select_cases(selection, fixture_name)


@pytest.mark.parametrize("field,value", [("schema", True), ("rtc_shunts_open", 1),
    ("exclusive_radios", False), ("receiver_service_stopped", False), ("pi_user", "root"),
    ("c6_dut", "unknown"), ("operating_envelope_reference", ""), ("c6_fixture", "radio_absent")])
def test_fixture_rejects_missing_or_mismatched_facts(fixture, field, value):
    fixture[field] = value
    with pytest.raises(ValueError):
        validate_fixture(fixture)


def test_budget_is_only_a_bounded_declaration():
    assert sum(e.charge["c6_us"] for e in EPISODES.values()) == 1467986
    assert sum(e.charge["pi_us"] for e in EPISODES.values()) == 469712
    assert len(EPISODES) == 10
    assert EPISODES["RF-013.absent"].c6_packets == 1


@pytest.fixture
def options(tmp_path, fixture, monkeypatch):
    file = tmp_path / "fixture.json"; file.write_text(json.dumps(fixture))
    sheet = tmp_path / "record.txt"; sheet.write_text("operator-owned history and this batch reservation")
    output = tmp_path / "new-run"
    values = dict(rf_fixture=str(file), rf_cases="RF-001.exchange", rf_run="a"*32, rf_output=str(output),
                  rf_manual_record=str(sheet), rf_session=str(tmp_path / "session.json"), rf_confirm_flash=True, count=1, embedded_services="esp,idf",
                  target="esp32c6", app_path=str(APP), port="/dev/ttyUSB0", port_mac="cc:8d:a2:fc:02:24",
                  build_dir=str(APP / "build"), root_logdir=str(output / "uart"), xmlpath=str(output / "junit.xml"))
    monkeypatch.setattr(guard, "verify_build", lambda _: {"verified": True})
    return values


@pytest.mark.parametrize("change", [{"erase_all": True}, {"erase_nvs": True}, {"skip_autoflash": True},
    {"esp_flash_force": True}, {"encrypt": True}, {"flash_port": "/dev/other"}, {"keyfile": "/tmp/key"},
    {"port_mac": "wrong"}, {"port": "/dev/ttyACM0"}, {"count": 2}, {"rf_ready_run": "b"*32},
    {"rf_confirm_flash": False}, {"app_path": "/tmp/other"}, {"xmlpath": "/tmp/outside.xml"}, {"rf_session": None}, {"rf_session": "relative.json"}])
def test_preflash_interlocks(options, change):
    options.update(change)
    with pytest.raises(ValueError):
        guard.validate_config(SimpleNamespace(getoption=options.get))


def test_build_change_fails_before_any_device_fixture(options, monkeypatch):
    def mismatch(_):
        raise ValueError("source/build seal mismatch")
    monkeypatch.setattr(guard, "verify_build", mismatch)
    with pytest.raises(ValueError, match="seal mismatch"):
        guard.validate_config(SimpleNamespace(getoption=options.get))


def test_zero_collection_does_not_pass_requested_run(options):
    config = SimpleNamespace(getoption=options.get, option=SimpleNamespace(collectonly=True))
    with pytest.raises(pytest.UsageError, match="missing, filtered"):
        guard.pytest_collection_finish(SimpleNamespace(items=[], config=config))


def test_real_cli_rejects_unguarded_hardware_collection(tmp_path):
    result = subprocess.run([sys.executable, "-m", "pytest", "-c", str(REPO / "tests/rf/pytest.ini"),
        "-p", "pytest_embedded.plugin", str(REPO / "tests/rf/pytest_radio.py"), "--collect-only", "-q"],
        cwd=tmp_path, capture_output=True, text=True, timeout=15)
    assert result.returncode == 4
    assert "--rf-fixture is required" in result.stderr
    assert "Connecting" not in result.stdout
