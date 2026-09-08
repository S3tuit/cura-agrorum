"""Local orchestration for the first sensor-carrier slice."""

from contextlib import contextmanager
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import re
import time

import pexpect
from pytest_embedded.unity import UNITY_SUMMARY_LINE_REGEX


APP = Path(__file__).resolve().parent
ACTIVE_SECONDS = 30
HOLD_SECONDS = 75
CASES = {
    "discover": "carrier setup discovery",
    "preflight": "carrier nominal preflight",
    "gate-on": "carrier production gate-on hold",
    "gate-off": "carrier production gate-off hold",
    "acquire": "carrier nominal acquisition and sample-return hold",
}
GPIO_SETTINGS = {
    "soil0": ("CURA_SOIL_0_GPIO", 0),
    "soil1": ("CURA_SOIL_1_GPIO", 1),
    "gate": ("CURA_SENSOR_POWER_GATE_GPIO", 2),
    "onewire": ("CURA_DS18B20_GPIO", 3),
    "sda": ("CURA_I2C_SDA_GPIO", 21),
    "scl": ("CURA_I2C_SCL_GPIO", 22),
    "stabilization_ms": ("CURA_SENSOR_POWER_STABILIZATION_MS", 200),
}


def validate_selection(operation, fixture, revision, ready):
    if operation not in {"discover", "gate-on", "gate-off", "acquire"}:
        raise ValueError("select --sensor-operation discover, gate-on, gate-off or acquire")
    if operation == "discover":
        if fixture is not None:
            raise ValueError("discovery is setup; omit --sensor-fixture")
    elif fixture != "nominal":
        raise ValueError("this operation requires --sensor-fixture nominal")
    if not revision or not revision.strip():
        raise ValueError("record --carrier-revision for this assembled carrier")
    if not ready:
        raise ValueError("confirm the DUT and wiring are ready with --sensor-fixture-ready")


@dataclass(frozen=True)
class Build:
    boot_values: dict
    elf_sha256: str
    config_sha256: str
    source_version: str


def load_build(build_dir: Path, operation: str) -> Build:
    config_bytes = (build_dir / "config/sdkconfig.json").read_bytes()
    config = json.loads(config_bytes)
    description = json.loads((build_dir / "project_description.json").read_text())
    if description.get("project_name") != "cura_sensor_carrier":
        raise ValueError("build is not the sensor-carrier app")
    expected = {key: value for key, value in GPIO_SETTINGS.values()}
    expected.update(IDF_TARGET="esp32c6", ESP_CONSOLE_UART_DEFAULT=True,
                    ESP_CONSOLE_UART_NUM=0, ESP_CONSOLE_UART_BAUDRATE=115200)
    for key, value in expected.items():
        if config.get(key) != value:
            raise ValueError(f"resolved CONFIG_{key} must be {value!r}; rebuild after menuconfig")
    roms = [config.get(f"CURA_DS18B20_{channel}_ROM", "") for channel in range(2)]
    if operation == "acquire":
        if (any(not re.fullmatch(r"[0-9a-fA-F]{16}", rom) or int(rom, 16) == 0
                for rom in roms) or roms[0].lower() == roms[1].lower()):
            raise ValueError("acquisition requires both distinct provisioned ROMs; run discovery first")
    boot_values = {name: str(value) for name, (_, value) in GPIO_SETTINGS.items()}
    boot_values.update(rom0=roms[0], rom1=roms[1])
    elf = build_dir / "cura_sensor_carrier.elf"
    return Build(boot_values, hashlib.sha256(elf.read_bytes()).hexdigest(),
                 hashlib.sha256(config_bytes).hexdigest(), description["project_version"])


@contextmanager
def phase(name):
    try:
        yield
    except (pexpect.TIMEOUT, pexpect.EOF) as exc:
        raise RuntimeError(
            f"sensor operation failed/incomplete during {name}; preserve the serial log. "
            "The host deadline does not establish target cleanup or BME recovery."
        ) from exc


def remaining(deadline):
    seconds = deadline - time.monotonic()
    if seconds <= 0:
        raise pexpect.TIMEOUT("phase deadline expired")
    return seconds


def boot_menu(dut, build, *, reset=False, expected_dut=None):
    with phase("fresh boot and Unity menu"):
        deadline = time.monotonic() + ACTIVE_SECONDS
        if reset:
            dut.serial.hard_reset()
        match = dut.expect(rb"CARRIER_BOOT ([^\r\n]+)\r?\n", timeout=remaining(deadline))
        values = dict(part.split("=", 1) for part in match.group(1).decode().split())
        for key, expected in {**build.boot_values, "elf": build.elf_sha256}.items():
            if values.get(key) != expected:
                raise ValueError(f"DUT boot {key}={values.get(key)!r}; expected {expected!r}")
        identity = values.get("dut", "")
        if not re.fullmatch(r"[0-9a-f]{12}", identity):
            raise ValueError("missing DUT identity in boot record")
        if expected_dut is not None and identity != expected_dut:
            raise ValueError("DUT identity changed across preflight reset")
        dut.expect_exact("Press ENTER to see the list of tests", timeout=remaining(deadline))
        dut.write("")
        dut.expect_exact("Here's the test menu, pick your combo:", timeout=remaining(deadline))
        menu = dut.expect_exact("Enter test for running.", timeout=remaining(deadline),
                                return_what_before_match=True)
        return identity, dut._parse_unity_menu_from_str(menu.decode().strip())


def select_case(menu, operation):
    selected = [case for case in menu if case.name == CASES[operation]]
    if len(selected) != 1:
        raise ValueError(f"{operation} selected {len(selected)} Unity cases; expected exactly one")
    case = selected[0]
    if case.type != "normal" or case.is_ignored:
        raise ValueError(f"required Unity case {case.name!r} is disabled or has an unexpected type")
    return case


def require_pass(dut, before, name):
    completed = dut.testsuite.testcases[before:]
    if len(completed) != 1 or completed[0].name != name or completed[0].result != "PASS":
        raise AssertionError(f"required Unity case {name!r} did not complete exactly once with PASS")


def execute_case(dut, case, operation):
    with phase(operation):
        before = len(dut.testsuite.testcases)
        deadline = time.monotonic() + ACTIVE_SECONDS
        # Write once: retries could enqueue a second sampling operation.
        dut.write(str(case.index))
        hold = "sample-return" if operation == "acquire" else operation
        if operation in {"gate-on", "gate-off", "acquire"}:
            marker = f"CARRIER_HOLD_READY {hold} seconds=60"
            match = dut.expect([re.escape(marker).encode(), UNITY_SUMMARY_LINE_REGEX],
                               timeout=remaining(deadline))
            prefix = dut.pexpect_proc.before + match.group(0)
            if match.group(0) != marker.encode():
                dut.testsuite.add_unity_test_cases(prefix)
                raise AssertionError(f"{operation} ended before its required observation hold")
            deadline = time.monotonic() + HOLD_SECONDS
            dut.expect_exact(f"CARRIER_HOLD_END {hold}", timeout=remaining(deadline))
            prefix += dut.pexpect_proc.before
            dut.expect_unity_test_output(timeout=remaining(deadline), extra_before=prefix)
        else:
            dut.expect_unity_test_output(timeout=remaining(deadline))
        require_pass(dut, before, case.name)


def run_operation(dut, build, operation, record):
    identity, menu = boot_menu(dut, build)
    record("dut_identity", identity)
    if operation == "acquire":
        # Verify both selections before doing preflight work on the carrier.
        select_case(menu, "acquire")
        execute_case(dut, select_case(menu, "preflight"), "preflight")
        identity, menu = boot_menu(dut, build, reset=True, expected_dut=identity)
    execute_case(dut, select_case(menu, operation), operation)
    record("operation_completed", operation)
    record("electrical_acceptance", "pending_operator_measurements")
