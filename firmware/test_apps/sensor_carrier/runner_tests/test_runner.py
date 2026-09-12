"""Runner regressions at the serial boundary; no simulated sensor readings."""

import hashlib
import json
import os
from types import SimpleNamespace

import pytest
from pytest_embedded.dut import Dut
from pytest_embedded.log import PexpectProcess
from pytest_embedded_idf.dut import IdfDut

import carrier_runner as runner
from pytest_sensor_carrier import test_sensor_carrier as run_hardware_test


@pytest.fixture
def build_dir(tmp_path):
    (tmp_path / "config").mkdir()
    config = {key: value for key, value in runner.GPIO_SETTINGS.values()}
    config.update(IDF_TARGET="esp32c6", ESP_CONSOLE_UART_DEFAULT=True,
                  ESP_CONSOLE_UART_NUM=0, ESP_CONSOLE_UART_BAUDRATE=115200,
                  CURA_DS18B20_0_ROM="1122334455667728",
                  CURA_DS18B20_1_ROM="8877665544332228")
    (tmp_path / "config/sdkconfig.json").write_text(json.dumps(config))
    (tmp_path / "project_description.json").write_text(json.dumps({
        "project_name": "cura_sensor_carrier", "project_version": "runner-unit-test",
    }))
    (tmp_path / "cura_sensor_carrier.elf").write_bytes(b"runner-test-build-identity")
    return tmp_path


def change_config(build_dir, **updates):
    path = build_dir / "config/sdkconfig.json"
    config = json.loads(path.read_text())
    config.update(updates)
    path.write_text(json.dumps(config))


@pytest.mark.parametrize("updates", [
    {"CURA_DS18B20_0_ROM": "0000000000000000"},
    {"CURA_DS18B20_1_ROM": "not-a-rom"},
    {"CURA_DS18B20_1_ROM": "1122334455667728"},
    {"CURA_I2C_SDA_GPIO": 4},
])
def test_bad_acquisition_config_rejected_before_dut_fixture(build_dir, updates):
    change_config(build_dir, **updates)
    options = dict(sensor_operation="acquire", app_path=str(runner.APP),
                   target="esp32c6", embedded_services="esp,idf", erase_all=False,
                   skip_autoflash=False, count=1, port="/dev/null", build_dir=str(build_dir))
    def forbidden_fixture(name):
        pytest.fail(f"configuration rejection must precede requesting {name}")
    request = SimpleNamespace(config=SimpleNamespace(getoption=options.get),
                              getfixturevalue=forbidden_fixture)
    with pytest.raises(ValueError):
        run_hardware_test(request, lambda *args: None)


def test_discovery_allows_unprovisioned_build(build_dir):
    change_config(build_dir, CURA_DS18B20_0_ROM="0000000000000000",
                  CURA_DS18B20_1_ROM="0000000000000000")
    assert runner.load_build(build_dir, "discover").boot_values["rom0"] == "0" * 16


@pytest.mark.parametrize("operation,fixture,revision,ready", [
    (None, None, "rev1", True),
    ("acquire", None, "rev1", True),
    ("repeat", "missing_ds0", "rev1", True),
    ("discover", "nominal", "rev1", True),
    ("discover", None, "", True),
    ("gate-on", "nominal", "rev1", False),
])
def test_incomplete_or_mismatched_selection_fails(operation, fixture, revision, ready):
    with pytest.raises(ValueError):
        runner.validate_selection(operation, fixture, revision, ready)


@pytest.fixture
def scripted_dut(build_dir, tmp_path, monkeypatch):
    # Speed up absent-marker tests only. This fixture never reaches hardware.
    monkeypatch.setattr(runner, "ACTIVE_SECONDS", 0.1)
    monkeypatch.setattr(runner, "HOLD_SECONDS", 0.1)
    build = runner.load_build(build_dir, "acquire")
    read_fd, write_fd = os.pipe()
    process = PexpectProcess(read_fd, timeout=0.1)
    dut = Dut(process, None, SimpleNamespace(app_path=str(runner.APP)),
              str(tmp_path / "serial.log"), "runner-test")
    state = SimpleNamespace(commands=[], resets=0, preflight="PASS", sample="PASS",
                            omit_end=False, omit_ready=False, empty_menu=False,
                            elf=build.elf_sha256, fixture="nominal", inventory="", sample_serial="")

    def feed(text):
        os.write(write_fd, text.encode())

    state.feed = feed

    def boot():
        values = " ".join(f"{key}={value}" for key, value in build.boot_values.items())
        feed(f"CARRIER_BOOT dut=001122334455 elf={state.elf} {values}\n"
             "Press ENTER to see the list of tests.\n")

    def result(name, outcome):
        if outcome == "ZERO":
            feed("\n-----------------------\n0 Tests 0 Failures 0 Ignored\nOK\n")
            return
        feed(f"test_sensor_carrier.c:100:{name}:{outcome}\n"
             f"-----------------------\n1 Tests {int(outcome == 'FAIL')} Failures "
             f"{int(outcome == 'IGNORE')} Ignored\n{'FAIL' if outcome == 'FAIL' else 'OK'}\n")

    def write(command):
        state.commands.append(command)
        if command == "":
            feed("Here's the test menu, pick your combo:\n")
            if not state.empty_menu:
                feed('(1)\t"carrier setup discovery" [sensor_carrier]\n'
                     f'(2)\t"carrier {state.fixture} preflight" [sensor_carrier]\n'
                     '(3)\t"carrier production gate-on hold" [sensor_carrier]\n'
                     '(4)\t"carrier production gate-off hold" [sensor_carrier]\n'
                     f'(5)\t"carrier {state.fixture} acquisition and sample-return hold" [sensor_carrier]\n')
            feed("Enter test for running.\n")
        elif command == "2":
            feed(state.inventory)
            result(f"carrier {state.fixture} preflight", state.preflight)
        elif command == "5":
            feed("CARRIER_HOLD_MODE\n")
        elif command == "auto":
            if state.omit_ready:
                return
            feed(state.sample_serial)
            feed("CARRIER_HOLD_READY sample-return seconds=60\n")
            if not state.omit_end:
                feed("CARRIER_HOLD_END sample-return\n")
                result(f"carrier {state.fixture} acquisition and sample-return hold", state.sample)

    state.boot = boot
    state.result = result

    def reset():
        state.resets += 1
        boot()

    dut.write = write
    dut.serial = SimpleNamespace(hard_reset=reset)
    dut._parse_unity_menu_from_str = IdfDut._parse_unity_menu_from_str
    boot()
    yield dut, build, state
    process.close()
    os.close(write_fd)


def run(script):
    dut, build, _ = script
    runner.run_operation(dut, build, "acquire", lambda *args: None)


def test_acquisition_preflight_then_fresh_boot_then_exactly_one_sample(scripted_dut):
    run(scripted_dut)
    dut, _, state = scripted_dut
    assert state.commands == ["", "2", "", "5", "auto"]
    assert state.resets == 1
    assert [case.result for case in dut.testsuite.testcases] == ["PASS", "PASS"]


@pytest.mark.parametrize("outcome", ["FAIL", "IGNORE", "ZERO"])
def test_failed_or_empty_preflight_cannot_reach_sample(scripted_dut, outcome):
    _, _, state = scripted_dut
    state.preflight = outcome
    with pytest.raises((AssertionError, ValueError)):
        run(scripted_dut)
    assert state.resets == 0
    assert "5" not in state.commands


@pytest.mark.parametrize("outcome", ["FAIL", "IGNORE", "ZERO"])
def test_sample_requires_one_passing_unity_case(scripted_dut, outcome):
    _, _, state = scripted_dut
    state.sample = outcome
    with pytest.raises((AssertionError, ValueError)):
        run(scripted_dut)
    assert state.commands.count("5") == 1


@pytest.mark.parametrize("missing", ["omit_ready", "omit_end"])
def test_timeout_is_incomplete_and_does_not_retry_or_reset_sample(scripted_dut, missing):
    _, _, state = scripted_dut
    setattr(state, missing, True)
    with pytest.raises(RuntimeError, match="failed/incomplete during acquire"):
        run(scripted_dut)
    assert state.commands.count("5") == 1
    assert state.resets == 1  # Only the reset before sampling.


def test_empty_unity_menu_fails_without_access_cases(scripted_dut):
    _, _, state = scripted_dut
    state.empty_menu = True
    with pytest.raises(ValueError, match="selected 0 Unity cases"):
        run(scripted_dut)
    assert state.commands == [""]


def test_wrong_flashed_elf_rejected_before_cases(scripted_dut):
    dut, build, state = scripted_dut
    wrong = runner.Build(build.boot_values, hashlib.sha256(b"other image").hexdigest(),
                         build.config_sha256, build.source_version)
    with pytest.raises(ValueError, match="DUT boot elf"):
        runner.run_operation(dut, wrong, "acquire", lambda *args: None)
    assert state.commands == []
