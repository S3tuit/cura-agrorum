"""Local orchestration for the real sensor carrier."""

from contextlib import contextmanager
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import re
import stat
import time
from concurrent.futures import ThreadPoolExecutor

import pexpect
from pytest_embedded.unity import UNITY_SUMMARY_LINE_REGEX


APP = Path(__file__).resolve().parent
ACTIVE_SECONDS = 30
HOLD_SECONDS = 75
GUIDED_HOLD_SECONDS = 195
ACK_HOLDS = {"acquire", "gate-on", "gate-off", "final-cleanup"}
SLEEP_DEADLINE_SECONDS = 615
ACQUISITIONS = {"acquire", "repeat", "final-cleanup", "ds-identity", "adc-reference"}
OPERATIONS = {"discover", "gate-on", "gate-off", *ACQUISITIONS,
              "reset", "held-reset", "deep-sleep"}
CASES = {
    "discover": "carrier setup discovery",
    "preflight": "carrier nominal preflight",
    "gate-on": "carrier production gate-on hold",
    "gate-off": "carrier production gate-off hold",
    "acquire": "carrier nominal acquisition and sample-return hold",
    "repeat": "carrier repeated nominal acquisition",
    "final-cleanup": "carrier final cleanup hold",
    "ds-identity": "carrier nominal acquisition and sample-return hold",
    "adc-reference": "carrier reference acquisition and sample-return hold",
    "reset": "carrier production restart cleanup",
    "held-reset": "carrier enabled rail held reset",
    "deep-sleep": "carrier enabled rail deep sleep",
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
    if operation not in OPERATIONS:
        raise ValueError("select an implemented --sensor-operation")
    if operation == "discover":
        if fixture is not None:
            raise ValueError("discovery is setup; omit --sensor-fixture")
    elif fixture != ("adc_reference" if operation == "adc-reference" else "nominal"):
        raise ValueError("operation and --sensor-fixture do not match")
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
    if operation in ACQUISITIONS | {"reset", "held-reset", "deep-sleep"}:
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


def uart_identity(port):
    if not port:
        return None
    info = Path(port).stat()
    if not stat.S_ISCHR(info.st_mode):
        raise OSError('selected UART is no longer a character device')
    return info.st_dev, info.st_ino, info.st_rdev


def check_uart(dut, port, identity):
    if uart_identity(port) != identity:
        raise OSError('UART device was replaced')
    reader = getattr(getattr(dut, 'serial', None), '_redirect_thread', None)
    if reader is not None and not reader.is_alive():
        raise OSError('UART reader stopped')


def boot_menu(dut, build, *, reset=False, expected_dut=None, deadline=None, boot_record=None):
    with phase("fresh boot and Unity menu"):
        deadline = deadline if deadline is not None else time.monotonic() + ACTIVE_SECONDS
        if reset:
            dut.serial.hard_reset()
        match = boot_record if boot_record is not None else dut.expect(
            rb"CARRIER_BOOT ([^\r\n]+)\r?\n", timeout=remaining(deadline))
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
    expected_type = "multi_stage" if operation in {"reset", "held-reset", "deep-sleep"} else "normal"
    if case.type != expected_type or case.is_ignored:
        raise ValueError(f"required Unity case {case.name!r} is disabled or has an unexpected type")
    return case


def require_pass(dut, before, name):
    completed = dut.testsuite.testcases[before:]
    if len(completed) != 1 or completed[0].name != name or completed[0].result != "PASS":
        raise AssertionError(f"required Unity case {name!r} did not complete exactly once with PASS")


def execute_case(dut, case, operation, *, repeat_count=100, evidence=None, on_hold=None, exploration=False):
    with phase(operation):
        before = len(dut.testsuite.testcases)
        deadline = time.monotonic() + ACTIVE_SECONDS
        # Write once: retries could enqueue a second sampling operation.
        dut.write(str(case.index))
        guided_ack = on_hold is not None and operation in ACK_HOLDS
        if operation in ACK_HOLDS | {"ds-identity"}:
            required_marker(dut, "CARRIER_HOLD_MODE", deadline)
            dut.write("exploration" if exploration else "guided" if guided_ack else "auto")
        if operation == "repeat":
            dut.expect_exact("CARRIER_REPEAT_COUNT", timeout=remaining(deadline))
            dut.write(str(repeat_count))
            overall = time.monotonic() + repeat_count * ACTIVE_SECONDS
            for index in range(1, repeat_count + 1):
                deadline = min(overall, time.monotonic() + ACTIVE_SECONDS)
                match = dut.expect([rb"CARRIER_ITERATION index=(\d+) total=(\d+)\r?\n",
                                    UNITY_SUMMARY_LINE_REGEX, rb"CARRIER_BOOT "],
                                   timeout=remaining(deadline))
                if (len(match.groups()) != 2 or match.group(1) != str(index).encode()
                        or match.group(2) != str(repeat_count).encode()):
                    raise AssertionError("repetition reset, failed or did not complete the requested count in order")
                if evidence:
                    evidence.add("iteration", index=index, requested=repeat_count,
                                 serial=dut.pexpect_proc.before.decode(errors="replace"))
            match = dut.expect([rb"CARRIER_REPEAT_DONE total=(\d+)\r?\n",
                                rb"CARRIER_ITERATION ", UNITY_SUMMARY_LINE_REGEX, rb"CARRIER_BOOT "],
                               timeout=remaining(deadline))
            if len(match.groups()) != 1 or match.group(1) != str(repeat_count).encode():
                raise AssertionError("missing complete count or unexpected additional iteration")
            dut.expect_unity_test_output(timeout=remaining(deadline))
            require_pass(dut, before, case.name)
            return
        hold = "sample-return" if operation in {"acquire", "ds-identity", "adc-reference"} else operation
        if operation in {"gate-on", "gate-off", "acquire", "ds-identity", "adc-reference", "final-cleanup"}:
            duration = "unlimited" if exploration else 180 if guided_ack else 60
            marker = f"CARRIER_HOLD_READY {hold} seconds={duration}"
            match = dut.expect([re.escape(marker).encode(), UNITY_SUMMARY_LINE_REGEX],
                               timeout=remaining(deadline))
            prefix = dut.pexpect_proc.before + match.group(0)
            if match.group(0) != marker.encode():
                dut.testsuite.add_unity_test_cases(prefix)
                raise AssertionError(f"{operation} ended before its required observation hold")
            deadline = time.monotonic() + (GUIDED_HOLD_SECONDS if guided_ack else HOLD_SECONDS)
            if on_hold:
                on_hold(hold, prefix, None if exploration else deadline)
            if exploration:
                deadline = time.monotonic() + ACTIVE_SECONDS
            if guided_ack:
                prefix += acknowledge_hold(dut, hold, deadline, evidence, exploration=exploration)
            dut.expect_exact(f"CARRIER_HOLD_END {hold}", timeout=remaining(deadline))
            prefix += dut.pexpect_proc.before
            dut.expect_unity_test_output(timeout=remaining(deadline), extra_before=prefix)
        else:
            dut.expect_unity_test_output(timeout=remaining(deadline))
        require_pass(dut, before, case.name)


def run_operation(dut, build, operation, record, *, expected_dut=None, repeat_count=100, evidence=None, guided=None):
    identity, menu = boot_menu(dut, build, expected_dut=expected_dut)
    record("dut_identity", identity)
    exploring = bool(getattr(guided, 'exploration', False))
    if exploring:
        guided.bind(dut)
    if operation in ACQUISITIONS or (not exploring and operation in {"reset", "held-reset", "deep-sleep"}):
        # Verify both selections before doing preflight work on the carrier.
        select_case(menu, operation)
        execute_case(dut, select_case(menu, "preflight"), "preflight")
        identity, menu = boot_menu(dut, build, reset=True, expected_dut=identity)
    if operation in {"reset", "held-reset", "deep-sleep"}:
        execute_transition(dut, build, menu, operation, identity, guided, evidence)
        if evidence:
            evidence.add("software_completed", operation=operation)
        record("operation_completed", operation)
        return
    if guided:
        guided.before_sample()
    execute_case(dut, select_case(menu, operation), operation,
                 repeat_count=repeat_count, evidence=evidence,
                 on_hold=guided.hold if guided else None, exploration=exploring)
    if evidence:
        evidence.add("software_completed", operation=operation)
    record("operation_completed", operation)
    record("electrical_acceptance", "pending_operator_measurements")


def select_stage(dut, case, stage, deadline):
    if len(case.subcases) != 2 or [int(sub['index']) for sub in case.subcases] != [1, 2]:
        raise ValueError("expected exactly two transition stages")
    dut.write(str(case.index))
    # Wait for the final submenu line before sending the stage selector.
    dut.expect_exact('"' + case.subcases[-1]['name'] + '"', timeout=remaining(deadline))
    dut.write(str(stage))


def required_marker(dut, marker, deadline):
    match = dut.expect([re.escape(marker).encode(), UNITY_SUMMARY_LINE_REGEX, rb"CARRIER_BOOT "],
                       timeout=remaining(deadline))
    if match.group(0) != marker.encode():
        raise AssertionError(f"operation failed/reset before {marker}")
    return dut.pexpect_proc.before + match.group(0)


def acknowledge_hold(dut, hold, deadline, evidence=None, *, exploration=False):
    if not exploration and time.monotonic() >= deadline - 15:
        raise RuntimeError("readings arrived after the three-minute observation window")
    # Acceptance requires validated measurements; exploration requires /done.
    dut.write(f"CARRIER_HOLD_ACK {hold}")
    match = dut.expect([re.escape(f"CARRIER_HOLD_ACKED {hold}").encode(),
                        rb"CARRIER_HOLD_INCOMPLETE ", UNITY_SUMMARY_LINE_REGEX,
                        rb"CARRIER_BOOT "], timeout=remaining(deadline))
    if match.group(0) != f"CARRIER_HOLD_ACKED {hold}".encode():
        raise AssertionError("DUT did not accept the observation acknowledgement")
    if evidence:
        evidence.add("hold_acknowledged", hold=hold)
    return dut.pexpect_proc.before


def execute_transition(dut, build, menu, operation, identity, guided, evidence=None):
    if guided is None:
        raise ValueError("transition requires guided electrical observations")
    with phase(operation):
        exploring = bool(getattr(guided, 'exploration', False))
        duration = 'unlimited' if exploring else '180'
        case = select_case(menu, operation)
        before = len(dut.testsuite.testcases)
        select_stage(dut, case, 1, time.monotonic() + ACTIVE_SECONDS)
        select_transition_mode(dut, exploring)
        prefix = required_marker(dut, f"CARRIER_HOLD_READY transition-on seconds={duration}",
                                 time.monotonic() + ACTIVE_SECONDS)
        hold_deadline = time.monotonic() + GUIDED_HOLD_SECONDS
        guided.hold("transition-on", prefix, None if exploring else hold_deadline)
        if exploring:
            hold_deadline = time.monotonic() + ACTIVE_SECONDS
        acknowledge_hold(dut, "transition-on", hold_deadline, evidence, exploration=exploring)
        dut.expect_exact("CARRIER_HOLD_END transition-on", timeout=remaining(hold_deadline))
        marker = "CARRIER_TRANSITION " + operation
        if operation == "deep-sleep":
            marker += " seconds=unlimited" if exploring else " end=operator-reset"
        required_marker(dut, marker, time.monotonic() + ACTIVE_SECONDS)
        if operation == 'reset':
            required_marker(dut, 'CARRIER_RESTART_CLEANUP calls=1 result=00000000 '
                            'diagnostic_empty=1 observer_valid=1', time.monotonic() + ACTIVE_SECONDS)
            if evidence:
                evidence.add('restart_cleanup', calls=1, result=0,
                             diagnostic_empty=True, observer_valid=True)
        started = time.monotonic()
        if exploring and operation in {'deep-sleep', 'held-reset'}:
            if operation == 'deep-sleep':
                guided.hold('deep-sleep', b'', None)
                guided.end_reset('deep-sleep')
            else:
                guided.held_reset()
            identity, new_menu = boot_menu(dut, build, expected_dut=identity)
        elif operation == "deep-sleep":
            observed_port, observed_uart = guided.deep_sleep(dut, started, started + SLEEP_DEADLINE_SECONDS)
            reset_deadline = min(started + SLEEP_DEADLINE_SECONDS, time.monotonic() + ACTIVE_SECONDS)
            print(f'Readings and YES saved. Press and release EN/reset now, within '
                  f'{remaining(reset_deadline):.0f} seconds. Keep USB connected. '
                  'The runner will finish automatically after the reboot; no more input is needed.', flush=True)
            identity, new_menu = boot_menu(dut, build, expected_dut=identity,
                                           deadline=reset_deadline)
            check_uart(dut, observed_port, observed_uart)
            reset_elapsed = time.monotonic() - started
        elif operation == "held-reset":
            with ThreadPoolExecutor(max_workers=1) as monitor:
                future, held_until = guided.held_reset(
                    lambda: monitor.submit(timed_boot, dut, time.monotonic() + GUIDED_HOLD_SECONDS))
                boot_record, arrived = future.result()
            if arrived < held_until:
                raise AssertionError("DUT booted during the held-reset observation")
            identity, new_menu = boot_menu(dut, build, expected_dut=identity, boot_record=boot_record)
        else:
            identity, new_menu = boot_menu(dut, build, expected_dut=identity)
        case = select_case(new_menu, operation)
        select_stage(dut, case, 2, time.monotonic() + ACTIVE_SECONDS)
        if operation in {'reset', 'deep-sleep'}:
            select_transition_mode(dut, exploring)
        deadline = time.monotonic() + ACTIVE_SECONDS
        if operation == "reset":
            prefix = required_marker(dut, f"CARRIER_HOLD_READY reset-off seconds={duration}", deadline)
            deadline = time.monotonic() + GUIDED_HOLD_SECONDS
            guided.hold("reset-off", prefix, None if exploring else deadline)
            if exploring:
                deadline = time.monotonic() + ACTIVE_SECONDS
            acknowledge_hold(dut, "reset-off", deadline, evidence, exploration=exploring)
            dut.expect_exact("CARRIER_HOLD_END reset-off", timeout=remaining(deadline))
        dut.expect_unity_test_output(timeout=remaining(deadline))
        require_pass(dut, before, case.name)
        if operation == 'deep-sleep' and not exploring and evidence:
            evidence.add('deep_sleep_operator_reset', reset_reason='POWERON',
                         elapsed_seconds=reset_elapsed)


def timed_boot(dut, deadline):
    match = dut.expect(rb"CARRIER_BOOT ([^\r\n]+)\r?\n", timeout=remaining(deadline))
    return match, time.monotonic()


def select_transition_mode(dut, exploration):
    required_marker(dut, 'CARRIER_HOLD_MODE', time.monotonic() + ACTIVE_SECONDS)
    dut.write('exploration' if exploration else 'guided')
