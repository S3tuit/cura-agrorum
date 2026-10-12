"""Physical DS3231 controller faults; execution and restoration are in README.md."""

from contextlib import contextmanager
from dataclasses import asdict
from decimal import Decimal
import hashlib
import json
import os
from pathlib import Path
import pwd
import select
import signal
import stat
import subprocess
import sys
import tempfile
import time
import pytest
from cura_receiver.platform.linux_clocks import LinuxOsClock
from cura_receiver.platform.linux_ds3231 import LinuxDs3231Control
from cura_receiver.ports.ds3231 import Ds3231ReadStatus as R
from cura_receiver.runtime_time import recover_rtc_read
from cura_receiver.elapsed_duration import maximum_lifetime_monotonic_us
from tests.support.receiver_pytest import _validated_destructive_test_root
from tests.support import time_components
from tests.support.time_components import (
    NORMAL_SOCKET,
    csv_tracking,
    rtc_access,
    run,
    time_test_lock,
)

pytestmark = [pytest.mark.hardware, pytest.mark.destructive]


@pytest.fixture
def controller_fixture(request, operation):
    root = _validated_destructive_test_root(
        request.config.getoption("receiver_test_root")
    )
    path = root / "i2c-fault-wiring.json"
    info = path.lstat()
    assert stat.S_ISREG(info.st_mode) and info.st_uid == 0
    assert info.st_mode & 0o022 == 0
    wiring = json.loads(path.read_text())
    assert wiring["operator_confirmed"] is True
    assert wiring["carrier_revision"] == "receiver-time-r1"
    assert wiring["fixture_state"] == "rtc_fault_ready"
    assert (
        isinstance(wiring["operator_response"], str)
        and wiring["operator_response"].strip()
    )
    assert wiring["spare_gpios_previously_unconnected"] is True
    assert wiring["wired_while_unpowered"] is True
    assert wiring["gpio17_to_scl"] is True and wiring["gpio27_to_sda"] is True
    assert wiring["boot_id"] == (
        Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    )
    with time_test_lock(request) as root:
        with rtc_fault_fixture(root, operation) as descriptor:
            yield descriptor


# This privileged fixture process controls only the two declared spare GPIOs.
# Receiver code continues to use the bound RTC driver, with no GPIO capability.
GPIO_HOLDER = r'''
import json, sys
import gpiod
from gpiod.line import Bias, Direction, Drive, Value
offset = int(sys.argv[1])
assert offset in (17, 27)
with gpiod.Chip('/dev/gpiochip0') as chip:
    original = chip.get_line_info(offset)
    assert not original.used and original.direction == Direction.INPUT
with gpiod.request_lines('/dev/gpiochip0', consumer='cura-receiver-rtc-fault',
    config={offset: gpiod.LineSettings(direction=Direction.OUTPUT,
        drive=Drive.OPEN_DRAIN, bias=Bias.AS_IS, output_value=Value.ACTIVE)}) as request:
    try:
        print('READY', flush=True)
        for command in sys.stdin:
            command = command.strip()
            assert command in ('LOW', 'RELEASE', 'EXIT')
            request.set_value(offset, Value.INACTIVE if command == 'LOW' else Value.ACTIVE)
            print(json.dumps({'command': command, 'value': request.get_value(offset).name}), flush=True)
            if command == 'EXIT':
                break
    finally:
        request.set_value(offset, Value.ACTIVE)
        request.reconfigure_lines({offset: gpiod.LineSettings(
            direction=Direction.INPUT, bias=Bias.AS_IS)})
'''


def bounded_line(process, timeout=5):
    assert select.select([process.stdout], [], [], timeout)[0], "fixture response timeout"
    line = process.stdout.readline()
    assert line, "fixture process exited without a response"
    return line.strip()


@contextmanager
def bus_fault(offset, evidence):
    process = subprocess.Popen(
        ["/usr/bin/python3", "-u", "-c", GPIO_HOLDER, str(offset)],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True,
    )
    try:
        assert bounded_line(process) == "READY"

        def send(command):
            process.stdin.write(command + "\n")
            process.stdin.flush()
            response = json.loads(bounded_line(process))
            evidence["gpio_commands"].append(response)
            assert response["command"] == command
            assert response["value"] in ("INACTIVE", "ACTIVE")
            if command == "LOW":
                assert response["value"] == "INACTIVE"

        send("RELEASE")
        yield send
    finally:
        try:
            if process.poll() is None:
                process.stdin.write("EXIT\n")
                process.stdin.flush()
                response = json.loads(bounded_line(process))
                evidence["gpio_commands"].append(response)
                assert response["command"] == "EXIT"
                assert response["value"] in ("INACTIVE", "ACTIVE")
                process.wait(timeout=5)
        finally:
            if process.poll() is None:
                process.kill()
                process.wait(timeout=5)
            evidence["gpio_exit"] = process.returncode
            evidence["gpio_stderr"] = process.stderr.read()
            process.stdin.close()
            process.stdout.close()
            process.stderr.close()
        assert process.returncode == 0, evidence


def bus_level(pin):
    output = run("/usr/bin/pinctrl", "get", str(pin)).stdout.strip()
    # pinctrl reads the pad level without requesting or reconfiguring its line.
    assert " | hi " in output or " | lo " in output, output
    return {"pin": pin, "high": " | hi " in output, "raw": output}


def fault_gpio_state():
    result = run("/usr/bin/python3", "-c", """
import json, gpiod
with gpiod.Chip('/dev/gpiochip0') as chip:
    lines = [chip.get_line_info(offset) for offset in (17, 27)]
    print(json.dumps([{'offset': line.offset, 'used': line.used,
        'direction': line.direction.name, 'bias': line.bias.name}
        for line in lines]))
""")
    return json.loads(result.stdout)


def normal_chrony_state():
    fields = dict(
        line.split("=", 1) for line in run(
            "systemctl", "show", "chrony.service",
            "--property=MainPID", "--property=ActiveState",
        ).stdout.splitlines()
    )
    pid = int(fields["MainPID"])
    assert fields["ActiveState"] == "active" and pid > 0, fields
    # Start time distinguishes a new process even if its PID is reused.
    start = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[19]
    return {
        "pid": pid, "start_ticks": start,
        "config_sha256": hashlib.sha256(
            Path("/etc/chrony/chrony.conf").read_bytes()
        ).hexdigest(),
        "tracking": csv_tracking(NORMAL_SOCKET),
    }


@contextmanager
def rtc_fault_fixture(root, operation):
    assert os.geteuid() == 0 and operation in ("read", "write")
    session = Path(tempfile.mkdtemp(prefix="time-", dir=root))
    session.chmod(0o755)
    descriptor = session / "fixture.json"
    restoration = {"fixture": "rtc-only", "operation": operation, "restored": False}

    def checkpoint():
        (session / "restoration.json").write_text(
            json.dumps(restoration, default=str, indent=2) + "\n"
        )

    checkpoint()
    try:
        normal = normal_chrony_state()
        restoration["baseline_chrony"] = normal
        tracking = normal["tracking"]
        assert len(tracking) == 14 and tracking[13] == "Normal"
        assert tracking[0] not in ("00000000", "7F7F0101")
        assert abs(Decimal(tracking[4])) < Decimal(".1")
        restoration["baseline_gpio"] = fault_gpio_state()
        assert all(
            not line["used"] and line["direction"] == "INPUT"
            for line in restoration["baseline_gpio"]
        )
        restoration["baseline_pads"] = [bus_level(pin) for pin in (2, 3)]
        assert all(pad["high"] for pad in restoration["baseline_pads"])
        clock = LinuxOsClock()
        rtc = LinuxDs3231Control(
            clock, kernel_operation_bound_us=3_000_000, device_path="/dev/rtc0"
        )

        def recover(label):
            attempts = []

            class RecordedReads:
                def read_time(self, **kwargs):
                    result = rtc.read_time(**kwargs)
                    attempts.append(asdict(result))
                    restoration[label + "_attempts"] = attempts
                    checkpoint()
                    return result

            budget = maximum_lifetime_monotonic_us(3_000_000, rate_bound_ppm=3700)
            attempt_budget = maximum_lifetime_monotonic_us(5_000_000, rate_bound_ppm=3700)
            result = recover_rtc_read(
                RecordedReads(), clock=clock,
                deadline_monotonic_us=clock.now_monotonic_us() + budget,
                attempt_budget_us=attempt_budget,
            )
            restoration[label + "_recovery"] = asdict(result)
            checkpoint()
            return result.result
        saved = rtc.read_time(
            deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000
        )
        saved_monotonic = clock.now_monotonic_us()
        restoration["saved_rtc"] = asdict(saved)
        restoration["saved_monotonic_us"] = saved_monotonic
        assert saved.status is R.OK, saved
        with rtc_access(session, pwd.getpwnam("cura"), restoration) as metadata:
            metadata.update({"fixture": "rtc-only", "operation": operation})
            descriptor.write_text(json.dumps(metadata, indent=2) + "\n")
            descriptor.chmod(0o644)
            checkpoint()
            try:
                yield descriptor
            finally:
                restoration["released_pads"] = [bus_level(pin) for pin in (2, 3)]
                restoration["released_gpio"] = fault_gpio_state()
                assert all(
                    not line["used"] and line["direction"] == "INPUT"
                    for line in restoration["released_gpio"]
                )
                fault_path = session / "controller-fault.json"
                fault = json.loads(fault_path.read_text()) if fault_path.exists() else {}
                authorized = fault.get("go_authorized", False)
                restoration["go_authorized"] = authorized
                restoration["cleanup_write_required"] = operation == "write" and authorized
                released = recover("released")
                restoration["released_rtc"] = asdict(released)
                restoration["recovered_pads"] = [bus_level(pin) for pin in (2, 3)]
                assert all(pad["high"] for pad in restoration["recovered_pads"])
                actual = released
                write_ok = True
                if restoration["cleanup_write_required"]:
                    # Ordinary writes require a recovered read; INVALID is an
                    # explicit operator-recovery stop, never an automatic rewrite.
                    assert released.status is R.OK, released
                    target = saved.rtc_utc_s + (
                        clock.now_monotonic_us() - saved_monotonic
                    ) // 1_000_000
                    restoration["cleanup_write_target_utc_s"] = target
                    checkpoint()
                    write_ok = False
                    try:
                        result = run(metadata["helper"], str(target), check=False)
                        restoration["cleanup_write"] = {
                            "exit": result.returncode,
                            "stdout": result.stdout, "stderr": result.stderr,
                        }
                        write_ok = (
                            result.returncode == 32 and not result.stdout
                            and not result.stderr
                        )
                    except Exception as error:
                        restoration["cleanup_write_error"] = repr(error)
                    actual = recover("verified")
                restoration["verified_rtc"] = asdict(actual)
                restoration["verified_pads"] = [bus_level(pin) for pin in (2, 3)]
                assert all(pad["high"] for pad in restoration["verified_pads"])
                expected = saved.rtc_utc_s + (
                    clock.now_monotonic_us() - saved_monotonic
                ) // 1_000_000
                restoration["expected_rtc_utc_s"] = expected
                assert write_ok, restoration
                assert actual.status is R.OK and abs(actual.rtc_utc_s - expected) <= 1
        restoration["restored"] = True
    except BaseException as error:
        restoration["error"] = repr(error)
        raise
    finally:
        try:
            after = normal_chrony_state()
            restoration["final_chrony"] = after
            tracking = after["tracking"]
            assert len(tracking) == 14 and tracking[13] == "Normal"
            assert abs(Decimal(tracking[4])) < Decimal(".1")
            if "baseline_chrony" in restoration:
                assert all(
                    after[key] == restoration["baseline_chrony"][key]
                    for key in ("pid", "start_ticks", "config_sha256")
                ), "normal Chrony process or configuration changed"
        except BaseException as error:
            restoration["restored"] = False
            restoration["chrony_verification_error"] = repr(error)
            raise
        finally:
            checkpoint()


# Four physical bus faults verify bounded adapter completion and restoration.
@pytest.mark.parametrize("fault_gpio", [17, 27], ids=["scl-low", "sda-low"])
@pytest.mark.parametrize("operation", ["read", "write"])
def test_ds3231_controller_fault(controller_fixture, fault_gpio, operation):
    descriptor = controller_fixture
    value = json.loads(descriptor.read_text())
    path = descriptor.parent / "controller-fault.json"
    observed_pin = 3 if fault_gpio == 17 else 2
    evidence = {
        "carrier_revision": "receiver-time-r1",
        "gpio": fault_gpio, "operation": operation, "gpio_commands": [],
        "levels": [], "syscalls": [], "go_authorized": False, "passed": False,
    }

    def checkpoint():
        path.write_text(json.dumps(evidence, indent=2) + "\n")

    process = None
    try:
        with bus_fault(fault_gpio, evidence) as send:
            before = bus_level(observed_pin)
            evidence["levels"].append(before)
            assert before["high"] is True, before
            argv = [
                "/usr/bin/setpriv", f'--reuid={value["uid"]}',
                f'--regid={value["gid"]}',
                "--groups=" + ",".join(map(str, value["supplementary_gids"])),
                sys.executable, str(Path(time_components.__file__).resolve()),
                "fault-" + operation, str(descriptor),
            ]
            process = subprocess.Popen(
                argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=subprocess.PIPE, text=True,
                env={**os.environ, "PYTHONPATH": os.pathsep.join(sys.path)},
            )
            assert bounded_line(process) == "CURA_FAULT_READY"
            send("LOW")
            low = bus_level(observed_pin)
            evidence["levels"].append(low)
            checkpoint()
            assert low["high"] is False, "declared jumper did not hold the I2C pad low"
            evidence["go_authorized"] = True
            checkpoint()
            process.stdin.write("GO\n")
            process.stdin.flush()
            stop = time.monotonic() + 8
            while process.poll() is None and time.monotonic() < stop:
                pids = [process.pid]
                children = Path(f"/proc/{process.pid}/task/{process.pid}/children")
                try:
                    pids += [int(p) for p in children.read_text().split()]
                except FileNotFoundError:
                    pass
                for pid in pids:
                    proc = Path(f"/proc/{pid}")
                    try:
                        call = (proc / "syscall").read_text().strip()
                        args = call.split()
                        expected_ioctl = (
                            0x80247009 if operation == "read" else 0x4024700A
                        )
                        if (
                            len(args) >= 3
                            and args[0] == "29"
                            and (int(args[2], 0) & 0xFFFFFFFF) == expected_ioctl
                        ):
                            status = (proc / "status").read_text()
                            fields = dict(
                                line.split(":", 1) for line in status.splitlines()
                                if ":" in line
                            )
                            pending = (
                                int(fields["SigPnd"], 16)
                                | int(fields["ShdPnd"], 16)
                            )
                            after_status = (proc / "syscall").read_text().strip()
                            evidence["syscalls"].append({
                                "pid": pid, "syscall": call,
                                "syscall_after_status": after_status,
                                "exe": str((proc / "exe").resolve()),
                                "stat": (proc / "stat").read_text(),
                                "status": status,
                                "sigkill_pending": bool(
                                    pending & (1 << (signal.SIGKILL - 1))
                                    and after_status == call
                                ),
                                "observed_monotonic_ns": time.monotonic_ns(),
                            })
                    except (FileNotFoundError, ProcessLookupError):
                        pass
                time.sleep(0.005)
            assert process.poll() is not None, (
                "component did not finish while the bus remained faulty"
            )
            output, error = process.communicate(timeout=1)
            evidence["component_stdout"], evidence["component_stderr"] = output, error
            evidence["component_exit"] = process.returncode
            checkpoint()
            assert process.returncode == 0, evidence
            result = json.loads(output)
            evidence["result"] = result
            duration = (
                result["operation_finished_at_monotonic_us"]
                - result["operation_started_at_monotonic_us"]
            )
            assert 0 <= duration < 3_000_000, result
            if fault_gpio == 17:
                assert evidence["syscalls"], "no actual in-flight RTC ioctl was observed"
            if operation == "read":
                assert result["status"] != "OK" and result["rtc_utc_s"] is None
            elif fault_gpio == 17:
                assert result["disposition"] == "OUTCOME_UNKNOWN"
                assert result["failure"] == "DEADLINE_EXCEEDED"
                assert any(
                    sample["exe"] == value["helper"] and sample["sigkill_pending"]
                    for sample in evidence["syscalls"]
                ), "no pending SIGKILL was observed inside the native RTC write ioctl"
                assert all(
                    not Path(f'/proc/{sample["pid"]}').exists()
                    for sample in evidence["syscalls"]
                )
            send("RELEASE")
            after = bus_level(observed_pin)
            evidence["levels"].append(after)
            # The RTC/controller may still hold the bus low. Teardown verifies
            # released input ownership, then performs bounded read recovery.
        evidence["passed"] = True
    finally:
        # bus_fault releases the bus before a stuck component or the outer RTC
        # restoration is allowed to finish. Preserve any failure before teardown.
        if process is not None and process.poll() is None:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        if process is not None and "component_stdout" not in evidence:
            output, error = process.communicate(timeout=1)
            evidence["component_stdout"], evidence["component_stderr"] = output, error
            evidence["component_exit"] = process.returncode
        checkpoint()

