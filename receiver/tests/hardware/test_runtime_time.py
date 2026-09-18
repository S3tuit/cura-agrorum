"""Real time adapters and isolated component state on the target Pi.

Safe cases only read deployment clocks. Run as root for read-only device/socket
access on an unprovisioned bench; that does not prove receiver-user privileges.
Destructive cases require the root-owned fixture in test_time_mutations.py.
"""

from dataclasses import asdict, replace
from decimal import Decimal, ROUND_CEILING, ROUND_UP
import ctypes
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest

from cura_receiver.producer_admission import ProducerAdmission
from cura_receiver.generated import receiver_enums_generated as E
from cura_receiver.generated.receiver_entities_generated import RtcProvenanceV1
from cura_receiver.clock_correlation import AnalysisInstance, ClockCorrelation
from cura_receiver.persistence_control_values import (
    CommunicatorStateCommitDisposition as CD,
)
from cura_receiver.platform.linux_clocks import LinuxOsClock
from cura_receiver.platform.linux_chrony import LinuxChronyControl
from cura_receiver.platform.linux_ds3231 import LinuxDs3231Control
from cura_receiver.platform.linux_kernel_clock import LinuxKernelClock, Timex
from cura_receiver.ports.chrony import ChronyQueryStatus as Q
from cura_receiver.ports.ds3231 import Ds3231ReadStatus as R
from cura_receiver.ports.kernel_clock import KernelSampleStatus as K
from cura_receiver.receiver_startup import create_receiver_instance
from cura_receiver.communicator_state_owner import CommunicatorStateOwner
from cura_receiver.runtime_time import RuntimeTime, RuntimeTimeSettings
from cura_receiver.time_policy import TimePolicy
from tests.support.builders.persistence_control import synthetic
from tests.support.coordination.persistence_worker import (
    CheckedPersistenceWorker,
    prepare_worker_files,
)
from tools.check_chrony import check_configuration

pytestmark = pytest.mark.hardware
SOCKET = "/run/chrony/chronyd.sock"
# Bench operation acceptance budget, not a validated hard failure-path bound.
RTC_BOUND_US = 3_000_000


def command(*argv, timeout=10):
    return subprocess.run(
        argv,
        check=True,
        capture_output=True,
        text=True,
        timeout=timeout,
        env={**os.environ, "LC_ALL": "C", "PYTHONPATH": os.pathsep.join(sys.path)},
    )


def record(tmp_path, name, **values):
    (tmp_path / (name + ".json")).write_text(
        json.dumps(values, indent=2, default=str) + "\n"
    )


def rtc_port(clock):
    return LinuxDs3231Control(
        clock, kernel_operation_bound_us=RTC_BOUND_US, device_path="/dev/rtc0"
    )


def tracking_port(clock, socket=SOCKET):
    return LinuxChronyControl(
        clock,
        socket_path=socket,
        deadline_monotonic_us=clock.now_monotonic_us() + 2_000_000,
    )


# Native headers are independent of the Python ABI declaration; target metadata and UTC share one syscall.
def test_real_kernel_abi_and_sample(tmp_path):
    source = tmp_path / "abi.c"
    source.write_text("""#include <sys/timex.h>
#include <stddef.h>
#include <stdio.h>
int main(void) { printf("%zu %zu %zu %zu\\n", sizeof(struct timex),
 offsetof(struct timex,status), offsetof(struct timex,time), sizeof(struct timeval)); }
""")
    binary = tmp_path / "abi"
    command("cc", "-Wall", "-Wextra", "-Werror", str(source), "-o", str(binary))
    assert command(str(binary)).stdout.strip() == "208 40 72 16"
    assert ctypes.sizeof(Timex) == 208
    clock = LinuxOsClock()
    before = clock.now_realtime_us()
    result = LinuxKernelClock(clock).sample(
        deadline_monotonic_us=clock.now_monotonic_us() + 250_000
    )
    after = clock.now_realtime_us()
    record(
        tmp_path,
        "adjtimex",
        result=asdict(result),
        before_utc_us=before,
        after_utc_us=after,
    )
    assert result.status is K.OK
    assert result.adjtimex_return == 5 and result.kernel_status_bits in (0x40, 0x2040)
    assert before <= result.sampled_utc_us <= after
    assert (
        result.operation_finished_at_monotonic_us
        - result.operation_started_at_monotonic_us
        < 250_000
    )


# Independent raw chronyc calls bracket the adapter; same reference epoch makes exact scalar comparison meaningful.
def test_real_chrony_tracking(tmp_path):
    clock = LinuxOsClock()
    adapter = tracking_port(clock)
    captures = []
    for _ in range(3):
        before = (
            command("/usr/bin/chronyc", "-n", "-c", "-h", SOCKET, "tracking")
            .stdout.strip()
            .split(",")
        )
        result = adapter.read_tracking(
            deadline_monotonic_us=clock.now_monotonic_us() + 250_000
        )
        after = (
            command("/usr/bin/chronyc", "-n", "-c", "-h", SOCKET, "tracking")
            .stdout.strip()
            .split(",")
        )
        captures.append([before, asdict(result), after])
        if before[:4] == after[:4]:
            break
    else:
        pytest.fail("no stable independent tracking bracket in three attempts")
    record(tmp_path, "chrony", captures=captures)
    assert result.status is Q.OK and result.synchronized and result.source_selected
    correction = [
        int((Decimal(fields[4]) * 1_000_000).to_integral_value(rounding=ROUND_UP))
        for fields in (before, after)
    ]
    assert (
        min(correction) - 1000
        <= result.remaining_correction_us
        <= max(correction) + 1000
    )
    distances = [
        int(
            (
                (Decimal(fields[10]) / 2 + Decimal(fields[11])) * 1_000_000
            ).to_integral_value(rounding=ROUND_CEILING)
        )
        for fields in (before, after)
    ]
    assert min(distances) - 1 <= result.root_distance_us <= max(distances) + 1
    assert result.estimated_skew_ppb == int(
        (Decimal(before[9]) * 1000).to_integral_value(rounding=ROUND_CEILING)
    )


# Actual driver read, missing configured path, rejected non-device and entry deadline need no physical mutation.
def test_real_rtc_read_and_safe_failures(tmp_path):
    clock = LinuxOsClock()
    rtc = rtc_port(clock)
    result = rtc.read_time(deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000)
    record(
        tmp_path,
        "rtc",
        result=asdict(result),
        driver=Path("/sys/class/rtc/rtc0/name").read_text(),
    )
    assert result.status is R.OK and type(result.rtc_utc_s) is int
    assert (
        result.operation_finished_at_monotonic_us
        - result.operation_started_at_monotonic_us
        < 1_000_000
    )
    rtc.device_path = str(tmp_path / "missing-device")
    assert (
        rtc.read_time(deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000).status
        is R.MISSING
    )
    (tmp_path / "regular-file").touch()
    rtc.device_path = str(tmp_path / "regular-file")
    assert (
        rtc.read_time(deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000).status
        is R.INVALID
    )
    assert (
        rtc.read_time(deadline_monotonic_us=clock.now_monotonic_us()).status
        is R.DEADLINE_EXCEEDED
    )


# The real boot identity survives two new processes, with each durable instance scoped to that boot.
def test_real_process_restart_boot_identity(tmp_path):
    database, config, _ = prepare_worker_files(tmp_path)
    script = """import json,sys
from pathlib import Path
from cura_receiver.platform.linux_clocks import LinuxOsClock
from cura_receiver.receiver_startup import create_receiver_instance
from tests.support.coordination.persistence_worker import CheckedPersistenceWorker
c=LinuxOsClock(); i=create_receiver_instance(c)
w=CheckedPersistenceWorker(instance=i,database_path=Path(sys.argv[1]),configuration_path=Path(sys.argv[2]),clock=c)
w.start()
started=w.wait_started(deadline_monotonic_us=c.now_monotonic_us()+10000000)
assert started.instance_start is not None, started
print(json.dumps({'instance':i.receiver_instance_id.hex(),'monotonic_us':i.started_at_monotonic_us,
'boot':open('/proc/sys/kernel/random/boot_id').read().strip()}),flush=True)
w.finish_test()
"""
    runs = [
        json.loads(
            command(sys.executable, "-c", script, str(database), str(config)).stdout
        )
        for _ in range(2)
    ]
    assert runs[0]["boot"] == runs[1]["boot"]
    assert runs[0]["instance"] != runs[1]["instance"]
    assert runs[0]["monotonic_us"] < runs[1]["monotonic_us"]
    import sqlite3

    with sqlite3.connect(database) as db:
        from uuid import UUID

        rows = db.execute(
            "SELECT receiver_instance_id, linux_boot_id FROM receiver_instances ORDER BY instance_ordinal"
        ).fetchall()
        assert rows == [
            (bytes.fromhex(run["instance"]), UUID(run["boot"]).bytes) for run in runs
        ]
    record(tmp_path, "process-restart", runs=runs)


# Supplied test proof goes through real SQLite loading; offline component startup never invokes Chrony or writes.
@pytest.mark.parametrize("proven", [False, True])
def test_offline_component_startup(tmp_path, proven):
    clock = LinuxOsClock()
    rtc = rtc_port(clock)
    database, config, _ = prepare_worker_files(tmp_path)
    instance = create_receiver_instance(clock)
    owner = CheckedPersistenceWorker(
        instance=instance,
        database_path=database,
        configuration_path=config,
        clock=clock,
    )
    owner.start()
    try:
        started = owner.wait_started(
            deadline_monotonic_us=clock.now_monotonic_us() + 10_000_000
        )
        assert started.instance_start is not None, started
        probe = rtc.read_time(deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000)
        assert probe.status is R.OK
        utc = probe.rtc_utc_s * 1_000_000
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
            rtc_provenance=(
                RtcProvenanceV1(instance.receiver_instance_id, utc, utc, 3_000_000, 10)
                if proven
                else None
            ),
        )
        assert (
            owner.control.commit_communicator_state(
                state, deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000
            ).disposition
            is CD.COMMITTED
        )
        loaded = owner.control.load_communicator_state(
            deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000
        )
        rt = RuntimeTime(
            receiver_instance_id=instance.receiver_instance_id,
            clock=clock,
            kernel=LinuxKernelClock(clock),
            queue=ProducerAdmission(owner.queue),
            policy=TimePolicy(maximum_network_skew_ppb=1000),
            startup_rtc_result=probe,
            state_owner=CommunicatorStateOwner(
                control=owner.control, initial_state=loaded.state
            ),
            settings=RuntimeTimeSettings(rtc_read_budget_us=5_000_000),
        )
        assert rt.state.quality is E.SystemTimeQuality.UNTRUSTED
        update = rt.observe_rtc(rtc, startup=True)
        assert update.observation.system_time_quality is (
            E.SystemTimeQuality.RTC_HOLDOVER
            if proven
            else E.SystemTimeQuality.UNTRUSTED
        )
        assert not any(rt.rtc_write_counts.values()) and not any(
            rt.step_command_counts.values()
        )
        record(
            tmp_path,
            "offline",
            proven_test_input=proven,
            observation=asdict(update.observation),
        )
        owner.request_stop(deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000)
        owner.join(5)
        assert not owner.is_alive() and owner.queue.snapshot().closed_and_drained
    finally:
        owner.finish_test()

    # Inspect the durable history consumed by analysis, including its real instance boundary.
    import sqlite3
    from cura_receiver.generated.receiver_entities_generated import ClockObservationV1

    with sqlite3.connect(database) as db:
        lifecycle = db.execute(
            "SELECT instance_ordinal, receiver_instance_id, linux_boot_id, "
            "started_at_monotonic_us FROM receiver_instances"
        ).fetchone()
        rows = db.execute(
            "SELECT receiver_instance_id, observation_sequence, clock_state_generation, "
            "sampled_at_monotonic_us, sampled_at_utc_us, step_discontinuity_boundary, "
            "system_time_quality_id, rtc_health_id FROM clock_observations"
        ).fetchall()
    observations = [
        ClockObservationV1(
            *row[:5], bool(row[5]), E.SystemTimeQuality(row[6]), E.RtcHealth(row[7])
        )
        for row in rows
    ]
    assert observations == [update.observation]
    assert observations[0].sampled_at_monotonic_us >= instance.started_at_monotonic_us
    correlation = ClockCorrelation([AnalysisInstance(*lifecycle)], observations)
    derived = correlation.correlate(
        instance.receiver_instance_id, observations[0].sampled_at_monotonic_us
    )
    if proven:
        assert derived is not None and derived.utc_us == observations[0].sampled_at_utc_us
    else:
        assert derived is None
    record(tmp_path, "offline-history", lifecycle=lifecycle, observations=rows)


def chrony_process_arguments(pid):
    return Path(f"/proc/{pid}/cmdline").read_bytes().rstrip(b"\0").decode().split("\0")


# Check the trusted procedure's actual launch, expanded config and competing writers without service changes.
def test_deployment_time_writer_audit(tmp_path):
    service = command(
        "systemctl", "show", "chrony.service", "-p", "ExecStart", "-p", "ExecStartPre",
        "-p", "ActiveState", "-p", "MainPID", "-p", "Type", "-p", "Restart",
    ).stdout
    properties = dict(line.split("=", 1) for line in service.splitlines() if "=" in line)
    assert properties["ActiveState"] == "active"
    assert properties["Type"] == "forking" and properties["Restart"] == "on-failure"
    launch = ["/usr/sbin/chronyd", "-F", "1", "-f", "/etc/chrony/chrony.conf"]
    precheck = [
        "/usr/bin/python3", "/usr/libexec/cura-agrorum/check-chrony.py",
        "/etc/chrony/chrony.conf",
    ]
    for name, argv in (("ExecStart", launch), ("ExecStartPre", precheck)):
        configured = properties[name]
        assert configured.count("argv[]=") == 1
        assert configured.startswith(
            "{ path=" + argv[0] + " ; argv[]=" + " ".join(argv) + " ; ignore_errors=no ;"
        ), f"{name} must use the documented launch procedure"
    pid = int(properties["MainPID"])
    assert pid > 0
    process_arguments = chrony_process_arguments(pid)
    assert process_arguments == launch, "running Chrony must use the audited configuration"
    effective = check_configuration("/etc/chrony/chrony.conf")
    units = command("systemctl", "list-unit-files", "--no-pager", "--no-legend").stdout
    active = command(
        "systemctl", "list-units", "--state=active", "--no-pager", "--no-legend"
    ).stdout
    for name in (
        "systemd-timesyncd.service",
        "hwclock.service",
        "ntp.service",
        "ntpd.service",
        "openntpd.service",
    ):
        assert not any(
            row.startswith(name + " ")
            and row.split()[1] in ("enabled", "enabled-runtime")
            for row in units.splitlines()
        )
        assert not any(
            row.lstrip().startswith(name + " ") for row in active.splitlines()
        )
    record(
        tmp_path,
        "writer-audit",
        effective=effective,
        unit_files=units,
        active_units=active,
        chrony_service=service,
        process_arguments=process_arguments,
    )


# A real unprivileged open failure retains errno; no device permissions are changed by this safe case.
def test_rtc_permission_failure(tmp_path):
    if os.geteuid() != 0:
        pytest.fail("this bench permission fixture requires a root test supervisor")
    script = """import json,os
from cura_receiver.platform.linux_clocks import LinuxOsClock
from cura_receiver.platform.linux_ds3231 import LinuxDs3231Control
os.setgroups([]); os.setgid(65534); os.setuid(65534)
c=LinuxOsClock(); r=LinuxDs3231Control(c,kernel_operation_bound_us=3000000,device_path='/dev/rtc0')
v=r.read_time(deadline_monotonic_us=c.now_monotonic_us()+5000000)
print(json.dumps({'status':v.status.name,'errno':v.os_errno}))
"""
    result = command(
        sys.executable,
        "-c",
        script,
    )
    assert json.loads(result.stdout) == {"status": "IO_ERROR", "errno": 13}
    record(tmp_path, "permission-failure", result=json.loads(result.stdout))
