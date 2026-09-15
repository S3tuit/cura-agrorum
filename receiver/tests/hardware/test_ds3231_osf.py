"""Production-adapter OSF capture after the operator's RTC-04 power/cell cycle.

The external operator run owns physical fault creation and RTC-05 recovery.
This case must run before recovery writes; missing confirmation fails explicitly.
"""

from dataclasses import asdict
import errno
import fcntl
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess

import pytest

from cura_receiver.platform.linux_clocks import LinuxOsClock
from cura_receiver.platform.linux_ds3231 import LinuxDs3231Control
from cura_receiver.ports.ds3231 import Ds3231ReadStatus
from tests.hardware.conftest import _validated_destructive_test_root

pytestmark = [pytest.mark.hardware, pytest.mark.destructive]


def test_oscillator_stop_rejected_by_production_adapter(request):
    root = _validated_destructive_test_root(
        request.config.getoption("receiver_test_root")
    )
    assert os.geteuid() == 0, "root is required for read-only bench RTC access"
    inputs = {}
    for name in ("osf-baseline.json", "osf-operator-confirmation.json"):
        path = root / name
        info = path.lstat()
        assert stat.S_ISREG(info.st_mode) and info.st_uid == 0
        assert info.st_mode & 0o022 == 0
        inputs[name] = json.loads(path.read_text())
    baseline = inputs["osf-baseline.json"]
    confirmation = inputs["osf-operator-confirmation.json"]
    assert baseline["rtc"]["status"] == "OK"
    assert baseline["rtc"]["os_errno"] is None
    assert confirmation["baseline_sha256"] == hashlib.sha256(
        (root / "osf-baseline.json").read_bytes()
    ).hexdigest()
    for action in (
        "all_external_power_disconnected",
        "coin_cell_removed_while_unpowered",
        "coin_cell_reinstalled_before_power",
        "no_rtc_write_since_fault",
    ):
        assert confirmation[action] is True, action
    boot = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    assert boot != baseline["boot_id"]
    assert confirmation["after_boot_id"] == boot
    lock = os.open(
        "/run/lock/cura-receiver-time-test.lock",
        os.O_CREAT | os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW,
        0o600,
    )
    try:
        assert os.fstat(lock).st_uid == 0
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        journal = subprocess.run(
            ["journalctl", "-k", "-b", "-o", "short-monotonic", "--no-pager"],
            capture_output=True, text=True, timeout=10, check=True,
            env={**os.environ, "LC_ALL": "C"},
        )
        (root / "osf-kernel.txt").write_text(journal.stdout)
        clock = LinuxOsClock()
        adapter = LinuxDs3231Control(
            clock, kernel_operation_bound_us=3_000_000, device_path="/dev/rtc0"
        )
        result = adapter.read_time(
            deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000
        )
        value = asdict(result)
        value["status"] = result.status.name
        evidence = {"boot_id": boot, "rtc": value, "operator": confirmation}
        (root / "osf-adapter.json").write_text(json.dumps(evidence, indent=2) + "\n")
        assert "SET TIME!" in journal.stdout
        assert "hctosys: unable to read the hardware clock" in journal.stdout
        assert result.status is Ds3231ReadStatus.INVALID
        assert result.os_errno == errno.EINVAL
        assert result.rtc_utc_s is None
        assert (
            result.operation_finished_at_monotonic_us
            - result.operation_started_at_monotonic_us
            < 3_000_000
        )
    finally:
        os.close(lock)
