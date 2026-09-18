"""Shared target clock/SQLite fixture for airtime lifetime and reboot evidence."""

from contextlib import contextmanager
from dataclasses import asdict
import json
import os
import platform
import shutil
import sqlite3
import sys

from cura_receiver.producer_admission import ProducerAdmission
from cura_receiver.communicator_state_owner import CommunicatorStateOwner
from cura_receiver.generated.receiver_entities_generated import (
    TxAirtimeBucketV1 as Bucket,
    communicator_state_v1_parameters,
)
from cura_receiver.platform.linux_boot_identity import read_linux_boot_id
from cura_receiver.platform.linux_chrony import LinuxChronyControl
from cura_receiver.platform.linux_clocks import LinuxOsClock
from cura_receiver.platform.linux_ds3231 import LinuxDs3231Control
from cura_receiver.platform.linux_kernel_clock import LinuxKernelClock
from cura_receiver.receiver_startup import create_receiver_instance
from cura_receiver.runtime_time import RuntimeTime
from cura_receiver.time_policy import TimePolicy
from cura_receiver.tx_airtime import TxAirtimePolicy
from tests.support.builders.persistence_control import state
from tests.support.coordination.persistence_worker import (
    CheckedPersistenceWorker,
    prepare_worker_files,
)

SOCKET = "/run/chrony/chronyd.sock"


def record(root, name, values):
    path = root / (name + ".json")
    path.write_text(json.dumps(values, indent=2, default=str) + "\n")
    return path


def preserve(root, name):
    destination = root / name
    destination.mkdir(exist_ok=True)
    for suffix in ("", "-wal", "-shm"):
        source = root / ("worker.db" + suffix)
        if source.exists():
            shutil.copy2(source, destination / source.name)


@contextmanager
def component(root, *, trusted=True, seed_remaining_us=None):
    if not (root / "worker.db").exists():
        prepare_worker_files(root)
    clock = LinuxOsClock()
    instance = create_receiver_instance(clock)
    worker = CheckedPersistenceWorker(
        instance=instance,
        database_path=root / "worker.db",
        configuration_path=root / "test-group.json",
        clock=clock,
    )
    worker.start()
    try:
        loaded = worker.wait_started(
            deadline_monotonic_us=clock.now_monotonic_us() + 10_000_000
        )
        assert loaded is not None and loaded.instance_start is not None, loaded
        rtc = LinuxDs3231Control(
            clock, kernel_operation_bound_us=3_000_000, device_path="/dev/rtc0"
        )
        probe = rtc.read_time(
            deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000
        )
        runtime = RuntimeTime(
            receiver_instance_id=instance.receiver_instance_id,
            clock=clock,
            kernel=LinuxKernelClock(clock),
            queue=ProducerAdmission(worker.queue),
            policy=TimePolicy(maximum_network_skew_ppb=10_000),
            startup_rtc_result=probe,
        )
        chrony = LinuxChronyControl(
            clock,
            socket_path=SOCKET if trusted else str(root / "unavailable-chrony.sock"),
            deadline_monotonic_us=clock.now_monotonic_us() + 2_000_000,
        )
        update = runtime.poll_chrony(chrony)
        correlation = runtime.airtime_correlation()
        record(
            root,
            "time-" + instance.receiver_instance_id.hex(),
            {
                "instance": asdict(instance),
                "boot": read_linux_boot_id().hex(),
                "rtc_probe": asdict(probe),
                "time_update": asdict(update),
                "correlation": asdict(correlation) if correlation else None,
                "python": sys.version,
                "sqlite": sqlite3.sqlite_version,
                "platform": platform.platform(),
                "uid": os.geteuid(),
                "trusted_source_requested": trusted,
                "fixture_maximum_network_skew_ppb": 10_000,
            },
        )
        assert (correlation is not None) == trusted, update
        seeded = None
        if seed_remaining_us is not None:
            now = clock.now_monotonic_us()
            utc = correlation.sample.utc_us + now - correlation.sample.monotonic_us
            expiration = utc + seed_remaining_us + 3_720_000_000
            initial = state(
                airtime_snapshot_utc_us=utc,
                buckets=(Bucket(1_000_000, expiration),) + (Bucket(0, 0),) * 63,
            )
            # A reviewed existing-history input, not an exemption from missing-state recovery.
            with sqlite3.connect(root / "worker.db") as database:
                database.execute(
                    "INSERT INTO communicator_state VALUES (?,?,?,?,?)",
                    communicator_state_v1_parameters(initial),
                )
            seeded = now, utc, expiration
        state_load = worker.control.load_communicator_state(
            deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000
        )
        owner = CommunicatorStateOwner.from_load(
            control=worker.control, loaded=state_load
        )
        runtime.state_owner = owner
        airtime = TxAirtimePolicy(state_owner=owner, clock=clock)
        airtime.update_time(correlation, rtc_health=runtime.state.rtc_health)
        yield airtime, clock, runtime, instance, seeded
    except BaseException:
        preserve(root, "failure-evidence")
        raise
    finally:
        worker.finish_test()
