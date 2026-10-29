"""Prepare the real adapter baseline for the optional boot-3 OSF test."""

from dataclasses import asdict
import json
import os
from pathlib import Path
import sys

from cura_receiver.platform.linux_clocks import LinuxOsClock
from cura_receiver.platform.linux_ds3231 import LinuxDs3231Control
from cura_receiver.ports.ds3231 import Ds3231ReadStatus
from tests.support.receiver_pytest import _validated_destructive_test_root


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: prepare_baseline.py /absolute/marked-test-root")
    if os.geteuid() != 0:
        raise SystemExit("the adapter baseline must be created by root")
    root = _validated_destructive_test_root(sys.argv[1])
    clock = LinuxOsClock()
    adapter = LinuxDs3231Control(
        clock, kernel_operation_bound_us=3_000_000, device_path="/dev/rtc0"
    )
    result = adapter.read_time(
        deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000
    )
    if result.status is not Ds3231ReadStatus.OK:
        raise SystemExit(f"a valid pre-fault RTC is required: {result.status.name}")
    value = asdict(result)
    value["status"] = result.status.name
    baseline = {
        "boot_id": Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
        "rtc": value,
    }
    path = root / "osf-baseline.json"
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(descriptor, "w") as output:
        output.write(json.dumps(baseline, indent=2) + "\n")
    print(path)


if __name__ == "__main__":
    main()
