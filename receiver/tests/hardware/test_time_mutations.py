"""Destructive, isolated time component cases. See evidence/runtime_time/README.md.

The privileged fixture owns its mutations and restoration. Each component runs
as cura with no capabilities and with the pinned helper as its sole RTC writer.
"""

from contextlib import ExitStack, contextmanager
from dataclasses import asdict, replace
from decimal import Decimal, ROUND_CEILING
import hashlib
import fcntl
import ipaddress
import json
import os
from pathlib import Path
import pwd
import select
import shutil
import signal
import sqlite3
import stat
import subprocess
import sys
import tempfile
import time
from uuid import UUID

import pytest

from cura_receiver.generated import receiver_enums_generated as E
from cura_receiver.generated.receiver_entities_generated import ClockObservationV1
from cura_receiver.platform.linux_chrony import LinuxChronyControl
from cura_receiver.platform.linux_clocks import LinuxOsClock
from cura_receiver.platform.linux_ds3231 import LinuxDs3231Control
from cura_receiver.platform.linux_kernel_clock import LinuxKernelClock
from cura_receiver.ports.ds3231 import (
    Ds3231ReadStatus as R,
    Ds3231WriteDisposition as W,
)
from cura_receiver.ports.chrony import ChronyStepDisposition as SD
from cura_receiver.persistence_control_values import (
    CommunicatorStateCommitDisposition as CD,
)
from cura_receiver.receiver_startup import create_receiver_instance
from cura_receiver.communicator_state_owner import CommunicatorStateOwner
from cura_receiver.runtime_time import (
    RuntimeTime,
    RuntimeTimeSettings,
    ChronyStepState as SS,
    RtcRefreshStatus as RS,
    recover_rtc_read,
)
from cura_receiver.elapsed_duration import maximum_lifetime_monotonic_us
from cura_receiver.time_policy import TimePolicy
from tests.hardware.conftest import _validated_destructive_test_root
from tests.hardware.time_reference import (
    SLEW_MAX_US,
    SLEW_COMPONENT_TIMEOUT_S,
    rate_interval,
    slew_decision,
)
from tests.support.builders.persistence_control import synthetic
from tests.support.coordination.persistence_worker import (
    CheckedPersistenceWorker,
    prepare_worker_files,
)

pytestmark = [pytest.mark.hardware, pytest.mark.destructive]
SOURCE_ROOT = Path(__file__).resolve().parents[3]
NORMAL_SOCKET = "/run/chrony/chronyd.sock"


def run(*argv, check=True, timeout=10, **kwargs):
    return subprocess.run(
        argv,
        check=check,
        capture_output=True,
        text=True,
        timeout=timeout,
        env={**os.environ, "LC_ALL": "C", "PYTHONPATH": os.pathsep.join(sys.path)},
        **kwargs,
    )


def csv_tracking(socket):
    return (
        run("/usr/bin/chronyc", "-n", "-c", "-h", str(socket), "tracking")
        .stdout.strip()
        .split(",")
    )


def wait_tracking(socket, predicate):
    stop = time.monotonic() + 90
    last = None
    while time.monotonic() < stop:
        try:
            last = csv_tracking(socket)
            if len(last) == 14 and last[13] == "Normal" and predicate(last):
                return last
        except subprocess.CalledProcessError:
            pass
        time.sleep(0.25)
    raise AssertionError(f"no qualifying tracking result within 90 seconds: {last}")


@contextmanager
def rtc_access(session, identity, restoration):
    """Own the real helper/device deployment used by both mutating fixtures."""
    device, stable = Path("/dev/rtc0"), Path("/dev/rtc-ds3231")
    info = device.stat()
    helper_dir = Path("/usr/libexec") / ("cura-receiver-" + session.name)
    helper = helper_dir / "ds3231-set"
    created_link = False
    access = {
        "device_uid": info.st_uid, "device_gid": info.st_gid,
        "device_mode": stat.S_IMODE(info.st_mode), "restored": False,
    }
    restoration["access"] = access
    try:
        helper_dir.mkdir(mode=0o755)
        run(
            "cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2",
            str(SOURCE_ROOT / "receiver/native/ds3231-set.c"), "-o", str(helper),
        )
        os.chown(helper, 0, identity.pw_gid)
        helper.chmod(0o750)
        run("/usr/sbin/setcap", "cap_sys_time=ep", str(helper))
        if stable.is_symlink() or stable.exists():
            assert stable.resolve() == device.resolve(), (
                "existing stable path selects a different device"
            )
        else:
            stable.symlink_to(device)
            created_link = True
        os.chown(device, info.st_uid, identity.pw_gid)
        device.chmod(0o660)
        data = session / "component"
        data.mkdir(mode=0o750)
        os.chown(data, identity.pw_uid, identity.pw_gid)
        yield {
            "helper": str(helper),
            "sha256": hashlib.sha256(helper.read_bytes()).hexdigest(),
            "uid": identity.pw_uid, "gid": identity.pw_gid,
            "supplementary_gids": sorted(
                set(os.getgrouplist(identity.pw_name, identity.pw_gid))
            ),
            "data": str(data),
        }
    finally:
        errors = []
        cleanups = [
            lambda: os.chown(device, info.st_uid, info.st_gid),
            lambda: device.chmod(stat.S_IMODE(info.st_mode)),
        ]
        if created_link:
            cleanups.append(stable.unlink)
        if helper_dir.exists():
            cleanups.append(lambda: shutil.rmtree(helper_dir))
        for cleanup in cleanups:
            try:
                cleanup()
            except Exception as error:
                errors.append(repr(error))
        actual = device.stat()
        access.update({
            "cleanup_errors": errors,
            "actual_uid": actual.st_uid, "actual_gid": actual.st_gid,
            "actual_mode": stat.S_IMODE(actual.st_mode),
            "helper_removed": not helper_dir.exists(),
            "created_link_removed": not created_link or not stable.is_symlink(),
        })
        access["restored"] = (
            not errors and access["helper_removed"] and access["created_link_removed"]
            and actual.st_uid == info.st_uid and actual.st_gid == info.st_gid
            and stat.S_IMODE(actual.st_mode) == stat.S_IMODE(info.st_mode)
        )
        assert access["restored"], access


@contextmanager
def isolated_time(root, offset):
    assert (
        os.geteuid() == 0
    ), "privileged fixture must run as root; components drop to cura"
    assert run("systemctl", "is-active", "chrony").stdout.strip() == "active"
    baseline = csv_tracking(NORMAL_SOCKET)
    assert baseline[13] == "Normal" and abs(Decimal(baseline[4])) < Decimal(".5")
    source = str(ipaddress.ip_address(baseline[1]))
    identity, daemon_identity = pwd.getpwnam("cura"), pwd.getpwnam("_chrony")
    session = Path(tempfile.mkdtemp(prefix="time-", dir=root))
    session.chmod(0o755)
    token = session.name
    socket_dir = Path("/run") / ("cura-receiver-" + token)
    device = Path("/dev/rtc0")
    stopped = False
    daemon = None
    helper = None
    access = ExitStack()
    clock = LinuxOsClock()
    rtc = LinuxDs3231Control(
        clock, kernel_operation_bound_us=3_000_000, device_path=str(device)
    )
    saved_rtc = rtc.read_time(
        deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000
    )
    assert saved_rtc.status is R.OK
    saved_monotonic = clock.now_monotonic_us()
    restoration = {
        "baseline_tracking": baseline,
        "saved_rtc": asdict(saved_rtc),
        "offset": offset,
        "restored": False,
    }
    (session / "restoration.json").write_text(
        json.dumps(restoration, default=str, indent=2) + "\n"
    )
    config_bytes = Path("/etc/chrony/chrony.conf").read_bytes()
    try:
        metadata = access.enter_context(rtc_access(session, identity, restoration))
        helper = Path(metadata["helper"])
        # Chrony validates both owner IDs and rejects access by "other".
        # Grant its command group to this test child only, not to the account.
        socket_dir.mkdir(mode=0o770)
        socket_dir.chmod(0o770)
        os.chown(socket_dir, daemon_identity.pw_uid, daemon_identity.pw_gid)
        socket = socket_dir / "chronyd.sock"
        cfg = session / "chrony.conf"
        cfg.write_text(
            f"server {source} iburst minpoll 4 maxpoll 4 offset {offset}\n"
            f"leapsecmode slew\nmaxslewrate 3500\ncorrtimeratio 1\ncmdport 0\nport 0\n"
            f"bindcmdaddress {socket}\npidfile {socket_dir}/chronyd.pid\n"
        )
        cfg.chmod(0o600)
        run("systemctl", "stop", "chrony")
        stopped = True
        log = (session / "chronyd.log").open("w")
        daemon = subprocess.Popen(
            ["/usr/sbin/chronyd", "-d", "-f", str(cfg), "-u", "_chrony"],
            stdout=log,
            stderr=subprocess.STDOUT,
        )
        stop = time.monotonic() + 5
        while not socket.exists() and time.monotonic() < stop:
            assert daemon.poll() is None, "isolated chronyd exited"
            time.sleep(0.02)
        assert socket.exists()
        os.chown(socket, daemon_identity.pw_uid, daemon_identity.pw_gid)
        socket.chmod(0o660)
        ready = wait_tracking(
            socket, lambda f: abs(Decimal(f[4]) - offset) < 5 and Decimal(f[9]) < 1000
        )
        metadata.update({
            "socket": str(socket),
            "supplementary_gids": sorted(
                set(metadata["supplementary_gids"]) | {daemon_identity.pw_gid}
            ),
            "offset": offset,
            "ready_tracking": ready,
        })
        descriptor = session / "fixture.json"
        descriptor.write_text(json.dumps(metadata, indent=2) + "\n")
        descriptor.chmod(0o644)
        yield descriptor
    finally:
        if daemon is not None:
            daemon.terminate()
            try:
                daemon.wait(timeout=5)
            except subprocess.TimeoutExpired:
                daemon.kill()
                daemon.wait(timeout=5)
            log.close()
        try:
            if stopped:
                run("systemctl", "start", "chrony")
                wait_tracking(
                    NORMAL_SOCKET, lambda f: Decimal(f[10]) / 2 + Decimal(f[11]) < 1
                )
                run("/usr/bin/chronyc", "-h", NORMAL_SOCKET, "makestep")
                restoration["restored_tracking"] = wait_tracking(
                    NORMAL_SOCKET, lambda f: abs(Decimal(f[4])) < Decimal(".5")
                )
            if offset == 0 and helper is not None and helper.exists():
                # The RTC refresh case writes the device. Preserve its running value.
                target = (
                    saved_rtc.rtc_utc_s
                    + (clock.now_monotonic_us() - saved_monotonic) // 1_000_000
                )
                outcome = run(str(helper), str(target), check=False)
                assert (
                    outcome.returncode == 32
                    and not outcome.stdout
                    and not outcome.stderr
                )
                actual = rtc.read_time(
                    deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000
                )
                assert actual.status is R.OK and abs(actual.rtc_utc_s - target) <= 1
                restoration["restored_rtc"] = asdict(actual)
            assert Path("/etc/chrony/chrony.conf").read_bytes() == config_bytes
            restoration["restored"] = True
        finally:
            try:
                access.close()
                if socket_dir.exists():
                    shutil.rmtree(socket_dir)
            except BaseException:
                restoration["restored"] = False
                raise
            finally:
                (session / "restoration.json").write_text(
                    json.dumps(restoration, default=str, indent=2) + "\n"
                )
        assert restoration[
            "restored"
        ], "fixture restoration failed; inspect restoration.json"


@contextmanager
def time_test_lock(request):
    root = _validated_destructive_test_root(
        request.config.getoption("receiver_test_root")
    )
    lock_path = Path("/run/lock/cura-receiver-time-test.lock")
    fd = os.open(
        lock_path, os.O_CREAT | os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW, 0o600
    )
    try:
        assert os.fstat(fd).st_uid == 0
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        yield root
    except BaseException:
        request.session.shouldstop = (
            "time fixture setup/restoration failed; dependent tests stopped"
        )
        raise
    finally:
        os.close(fd)


@contextmanager
def owned_time_fixture(request, offset):
    if abs(offset) == 120:
        assert (
            os.environ.get("CURA_LAPTOP_REFERENCE") == "SSH_STDIO_V1"
        ), "run the maximum-slew fixture through the laptop reference bridge"
    with time_test_lock(request) as root:
        with isolated_time(root, offset) as descriptor:
            yield descriptor


@pytest.fixture
def fixture(request):
    with owned_time_fixture(request, request.param) as descriptor:
        yield descriptor


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
                sys.executable, str(Path(__file__).resolve()),
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


def component(descriptor, case, *, stream=False):
    value = json.loads(descriptor.read_text())
    argv = [
        "/usr/bin/setpriv",
        f'--reuid={value["uid"]}',
        f'--regid={value["gid"]}',
        "--groups=" + ",".join(map(str, value["supplementary_gids"])),
        sys.executable,
        str(Path(__file__).resolve()),
        case,
        str(descriptor),
    ]
    if stream:
        # Laptop reference responses travel over the same SSH stdin/stdout channel.
        subprocess.run(
            argv,
            check=True,
            timeout=SLEW_COMPONENT_TIMEOUT_S,
            env={**os.environ, "PYTHONPATH": os.pathsep.join(sys.path)},
        )
    else:
        result = run(*argv, timeout=90)
        (descriptor.parent / (case + "-result.json")).write_text(result.stdout)


@pytest.mark.parametrize("fixture", [0], indirect=True)
def test_network_rtc_refresh_and_helper_privilege(fixture):
    component(fixture, "rtc")


@pytest.mark.parametrize("fixture", [60, -60], indirect=True)
def test_forward_backward_step_component(fixture):
    component(fixture, "step")


@pytest.mark.slow
@pytest.mark.parametrize("fixture", [120, -120], indirect=True)
def test_maximum_slew_against_laptop(fixture):
    assert (
        os.environ.get("CURA_LAPTOP_REFERENCE") == "SSH_STDIO_V1"
    ), "run through the laptop reference bridge"
    component(fixture, "slew", stream=True)


def rtc_ports(value):
    clock = LinuxOsClock()
    rtc = LinuxDs3231Control(
        clock,
        kernel_operation_bound_us=3_000_000,
        helper_path=value["helper"],
        helper_sha256=bytes.fromhex(value["sha256"]),
        receiver_gid=value["gid"],
    )
    return clock, rtc


def ports(value):
    clock, rtc = rtc_ports(value)
    chrony = LinuxChronyControl(
        clock,
        socket_path=value["socket"],
        deadline_monotonic_us=clock.now_monotonic_us() + 2_000_000,
    )
    return clock, rtc, chrony


def execute_component(case, value):
    if case in ("fault-read", "fault-write"):
        # Validate the actual capability-free parent and pinned RTC helper only.
        clock, rtc = rtc_ports(value)
        baseline = rtc.read_time(
            deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000
        )
        assert baseline.status is R.OK, baseline
        print("CURA_FAULT_READY", flush=True)
        assert sys.stdin.readline().strip() == "GO"
        if case == "fault-read":
            result = rtc.read_time(
                deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000
            )
            evidence = asdict(result)
            evidence["status"] = result.status.name
        else:
            result = rtc.write_time(
                rtc_utc_s=clock.now_realtime_us() // 1_000_000,
                deadline_monotonic_us=clock.now_monotonic_us() + 500_000,
            )
            evidence = asdict(result)
            evidence["disposition"] = result.disposition.name
            evidence["failure"] = result.failure.name
        print(json.dumps(evidence), flush=True)
        return
    clock, rtc, chrony = ports(value)
    if case == "slew":
        return measure_slew(value, clock, chrony)
    root = Path(value["data"])
    db, cfg, _ = prepare_worker_files(root)
    instance = create_receiver_instance(clock)
    owner = CheckedPersistenceWorker(
        instance=instance,
        database_path=db,
        configuration_path=cfg,
        clock=clock,
        wake_threshold_entities=1,
    )
    owner.start()
    try:
        started = owner.wait_started(
            deadline_monotonic_us=clock.now_monotonic_us() + 10_000_000
        )
        assert started.instance_start is not None, started
        probe = rtc.read_time(
            deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000
        )
        rt = RuntimeTime(
            receiver_instance_id=instance.receiver_instance_id,
            clock=clock,
            kernel=LinuxKernelClock(clock),
            queue=owner.queue,
            policy=TimePolicy(maximum_network_skew_ppb=1_000_000),
            startup_rtc_result=probe,
            settings=RuntimeTimeSettings(
                rtc_read_budget_us=5_000_000,
                command_budget_us=5_000_000,
                control_budget_us=5_000_000,
            ),
        )
        if case == "rtc":
            update = rt.poll_chrony(chrony)
            assert rt.state.quality is E.SystemTimeQuality.NETWORK_SYNCED, update
            utc = rt.sample.utc_us
            state = synthetic()
            state = replace(
                state,
                airtime_snapshot_utc_us=utc,
                buckets=tuple(
                    (
                        replace(b, expires_at_utc_us=b.expires_at_utc_us + utc)
                        if b.charged_airtime_us
                        else b
                    )
                    for b in state.buckets
                ),
            )
            assert (
                owner.control.commit_communicator_state(
                    state, deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000
                ).disposition
                is CD.COMMITTED
            )
            rt.state_owner = CommunicatorStateOwner(
                control=owner.control, initial_state=state
            )

            def snapshot(**kw):
                return replace(
                    kw["previous_state"],
                    generation=kw["previous_state"].generation + 1,
                    rtc_provenance=kw["provenance"],
                    airtime_snapshot_utc_us=kw["snapshot_utc_us"],
                )

            first = rt.refresh_rtc(rtc, snapshot)
            assert first.status is RS.VERIFIED, first
            assert first.write_result.disposition is W.COMPLETED
            # Repeat with existing proof to exercise the extra durable invalidation.
            rt.last_refresh_monotonic_us = None
            while clock.now_monotonic_us() < rt.next_rtc_attempt_monotonic_us:
                time.sleep(0.01)
            rt.poll_chrony(chrony)

            class InspectWrite:
                def write_time(self, **kw):
                    loaded = owner.control.load_communicator_state(
                        deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000
                    )
                    assert loaded.state.rtc_provenance is None
                    return rtc.write_time(**kw)

                def read_time(self, **kw):
                    return rtc.read_time(**kw)

            second = rt.refresh_rtc(InspectWrite(), snapshot)
            assert second.status is RS.VERIFIED, second
            assert rt.durable_state.generation == 4
            # The production adapter rejects entry expiry without executing its privileged helper.
            assert (
                rtc.write_time(
                    rtc_utc_s=utc // 1_000_000,
                    deadline_monotonic_us=clock.now_monotonic_us(),
                ).disposition
                is W.NOT_APPLIED
            )
            print(
                json.dumps(
                    {
                        "first": asdict(first),
                        "replacement": asdict(second),
                        "state_generation": rt.durable_state.generation,
                    },
                    default=str,
                )
            )
        else:
            boundary = rt.poll_chrony(chrony)
            assert boundary.observation.step_discontinuity_boundary, boundary
            assert rt.step_state is SS.STEP_COMMAND_PENDING
            before_utc, before_mono = clock.now_realtime_us(), clock.now_monotonic_us()
            result = rt.poll_chrony(chrony)
            after_utc, after_mono = clock.now_realtime_us(), clock.now_monotonic_us()
            assert result.step_result.disposition is SD.SUBMITTED, result
            delta = (after_utc - before_utc) - (after_mono - before_mono)
            assert abs(delta - value["offset"] * 1_000_000) < 5_000_000
            updates = [boundary]
            stop = time.monotonic() + 30
            while time.monotonic() < stop and rt.step_state is not SS.IDLE:
                update = rt.poll_chrony(chrony)
                if update.observation is not None:
                    updates.append(update)
                time.sleep(0.02)
            assert (
                rt.step_state is SS.IDLE
                and rt.state.quality is E.SystemTimeQuality.NETWORK_SYNCED
            )
            recovery = updates[-1].observation
            from cura_receiver.clock_correlation import (
                AnalysisInstance,
                ClockCorrelation,
            )

            history = ClockCorrelation(
                [
                    AnalysisInstance(
                        1,
                        instance.receiver_instance_id,
                        UUID(
                            Path("/proc/sys/kernel/random/boot_id").read_text().strip()
                        ).bytes,
                        instance.started_at_monotonic_us,
                    )
                ],
                [u.observation for u in updates],
            )
            assert (
                history.correlate(
                    rt.instance, boundary.observation.sampled_at_monotonic_us
                )
                is None
            )
            assert (
                history.correlate(rt.instance, recovery.sampled_at_monotonic_us).utc_us
                == recovery.sampled_at_utc_us
            )
            stop = time.monotonic() + 5
            rows = []
            while time.monotonic() < stop:
                with sqlite3.connect(db) as conn:
                    rows = conn.execute(
                        "SELECT observation_sequence,step_discontinuity_boundary,sampled_at_utc_us FROM clock_observations ORDER BY observation_sequence"
                    ).fetchall()
                if len(rows) == len(updates):
                    break
                time.sleep(0.02)
            assert (
                rows[0][1:] == (1, None) and rows[-1][2] == recovery.sampled_at_utc_us
            )
            print(
                json.dumps(
                    {
                        "step_delta_us": delta,
                        "observations": [asdict(u.observation) for u in updates],
                        "persisted_rows": rows,
                    },
                    default=str,
                )
            )
    finally:
        owner.finish_test()


def reference_sample(clock, retain):
    sample = {"pi_before_us": clock.now_monotonic_us(), "valid": False}
    try:
        print("\nCURA_REFERENCE_REQUEST", flush=True)
        # The component's outer timeout bounds a lost bridge; EOF fails explicitly.
        sample["raw_reply"] = sys.stdin.readline()
        sample["pi_after_us"] = clock.now_monotonic_us()
        reply = json.loads(sample["raw_reply"])
        sample["reference_reply"] = reply
        assert type(reply) is dict
        assert type(reply.get("version")) is int and reply["version"] == 1
        assert reply.get("source_independent") is True
        assert type(reply.get("utc_us")) is int and reply["utc_us"] >= 0
        assert type(reply.get("error_us")) is int and 0 <= reply["error_us"] <= 100_000
        sample["utc_us"], sample["error_us"] = reply["utc_us"], reply["error_us"]
        sample["valid"] = True
        return sample
    finally:
        sample.setdefault("pi_after_us", clock.now_monotonic_us())
        retain(sample)


def measure_slew(value, clock, chrony):
    sign = 1 if value["offset"] > 0 else -1
    result = {
        "samples": [],
        "intervals": [],
        "tracking": [],
        "offset": value["offset"],
        "status": "RUNNING",
    }
    path = Path(value["data"]) / "maximum-slew.json"

    def checkpoint():
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(result, default=str, indent=2) + "\n")
        temporary.replace(path)

    def retain(sample):
        result["samples"].append(sample)
        checkpoint()

    checkpoint()
    try:
        first = reference_sample(clock, retain)
        until = first["pi_after_us"] + SLEW_MAX_US
        while True:
            tracking = chrony.read_tracking(
                deadline_monotonic_us=clock.now_monotonic_us() + 250_000
            )
            result["tracking"].append(asdict(tracking))
            checkpoint()
            assert (
                tracking.synchronized
                and sign * tracking.remaining_correction_us > 100_000_000
            ), tracking
            time.sleep(min(20, max(0, (until - clock.now_monotonic_us()) / 1_000_000)))
            last = reference_sample(clock, retain)
            low, high = rate_interval(first, last)
            elapsed = last["pi_after_us"] - first["pi_after_us"]
            decision = slew_decision(elapsed, low, high, sign)
            interval = {
                "elapsed_monotonic_us": elapsed,
                "rate_interval_ppm": [str(low), str(high)],
                "width_ppm": str(high - low),
                "decision": decision,
            }
            result["intervals"].append(interval)
            result["rate_interval_ppm"] = interval["rate_interval_ppm"]
            checkpoint()
            print("CURA_SLEW_PROGRESS " + json.dumps(interval), flush=True)
            if decision == "CONTINUE":
                continue
            result["status"] = decision
            checkpoint()
            assert decision == "PASS", interval
            print("CURA_SLEW_RESULT " + json.dumps(interval), flush=True)
            return
    except BaseException as error:
        if result["status"] == "RUNNING":
            result["status"] = "ERROR"
        result["error"] = f"{type(error).__name__}: {error}"
        checkpoint()
        raise


if __name__ == "__main__":
    case, descriptor = sys.argv[1:]
    assert case in ("rtc", "step", "slew", "fault-read", "fault-write")
    descriptor = Path(descriptor)
    assert descriptor.stat().st_uid == 0 and descriptor.stat().st_mode & 0o022 == 0
    execute_component(case, json.loads(descriptor.read_text()))
