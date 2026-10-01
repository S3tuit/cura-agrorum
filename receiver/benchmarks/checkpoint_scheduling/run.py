"""Isolated real-worker checkpoint comparison; no service or device mutations."""

import argparse
from collections import deque
from dataclasses import asdict, replace
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import sqlite3
import sys
from threading import Event, Lock
import time
from uuid import uuid4


def manifest(root):
    files = list((root / "receiver/cura_receiver").rglob("*.py"))
    files += list((root / "receiver/db").glob("*.sql"))
    files += list((root / "receiver/schemas").glob("*.json"))
    files += list((root / "protocol/protocol-v2-lora/python").rglob("*.py"))
    files += [root / "receiver/tests/support/builders/persistence.py",
              root / "receiver/tests/support/coordination/persistence_worker.py"]
    return {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(files)}


def percentiles(values):
    ordered = sorted(values)
    return {str(p): ordered[max(0, math.ceil(len(ordered) * p / 100) - 1)]
            for p in (50, 95, 99, 100)} if ordered else {}


def written_bytes(path):
    if path is None:
        return None
    fields = [int(v) for v in path.read_text().split()]
    if len(fields) < 11:
        raise RuntimeError("block stat does not have the full device field layout")
    return fields[6] * 512


def flush_private_files(root):
    for path in root.iterdir():
        if path.is_file():
            with path.open("rb") as stream:
                os.fsync(stream.fileno())
    descriptor = os.open(root, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", required=True, type=Path)
    parser.add_argument("--source-commit", required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--fixture-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--block-stat", type=Path)
    parser.add_argument("--background-seconds", type=float, default=30)
    parser.add_argument("--warmup-seconds", type=float, default=6)
    parser.add_argument("--idle-seconds", type=float, default=30)
    parser.add_argument("--traffic-seconds", type=int, default=120)
    parser.add_argument("--period-seconds", type=float, default=1)
    parser.add_argument("--reader-seconds", type=float, default=30)
    parser.add_argument("--writeback-seconds", type=float, default=5)
    parser.add_argument("--host-smoke", action="store_true")
    args = parser.parse_args()
    if args.traffic_seconds < 1 or args.period_seconds <= 0 or any(
        getattr(args, name) < 0 for name in (
            "background_seconds", "warmup_seconds", "idle_seconds",
            "reader_seconds", "writeback_seconds"
        )
    ):
        parser.error("durations must be nonnegative; traffic and period must be positive")
    model_path = Path("/proc/device-tree/model")
    model = model_path.read_text().rstrip("\0") if model_path.exists() else platform.platform()
    if not args.host_smoke and "Raspberry Pi" not in model:
        parser.error("target measurement requires a Raspberry Pi; use --host-smoke for plumbing only")
    if args.output.exists() or not args.fixture_root.is_absolute():
        parser.error("output must be new and fixture root must be absolute")
    root = args.source_root.resolve(strict=True)
    source_hashes = manifest(root)
    sys.path[:0] = [str(root / "receiver"), str(root / "protocol/protocol-v2-lora/python")]

    from cura_receiver.generated.receiver_enums_generated import AdmissionResult
    from cura_receiver.ordinary_persistence import OrdinaryPersistence
    from cura_receiver.persist_queue import PersistQueue
    from cura_receiver.persist_queue_entities import PROFILE_ONLY_V1_SPEC, ProfileOnlyUnitV1
    from cura_receiver.persistence_control_values import (
        CommunicatorStateLoadStatus, ReceiverCleanStopV1, ReceiverCleanStopCommitDisposition,
    )
    from cura_receiver.platform.linux_clocks import LinuxOsClock
    from cura_receiver.receiver_startup import ReceiverInstanceStart, insert_receiver_instance_start
    from cura_receiver.sqlite_database import open_receiver_database
    from cura_receiver.sqlite_transactions import SqliteTransactions
    from tests.support.builders.persistence import GROUP, INSTANCE, _profile
    from tests.support.coordination.persistence_worker import CheckedPersistenceWorker, prepare_worker_files

    args.fixture_root.mkdir(parents=False, exist_ok=False)
    path, config, boot = prepare_worker_files(args.fixture_root)
    clock = LinuxOsClock()
    database = open_receiver_database(path, GROUP, minimum_free_bytes=0).database
    insert_receiver_instance_start(database.connection, ReceiverInstanceStart(INSTANCE, 0), b"b" * 16)
    queue = PersistQueue()
    preparer = OrdinaryPersistence(database, queue, instance=ReceiverInstanceStart(INSTANCE, 0), clock=clock)
    preparer.enable_admission()
    sequence = 0
    wal = path.with_name(path.name + "-wal")
    try:
        while wal.stat().st_size < 5 * 1024 * 1024:
            for _ in range(128):
                sequence += 1
                if sequence > 30_000:
                    raise RuntimeError("bounded fixture preparation did not allocate 5 MiB WAL")
                reservation = queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
                if reservation.status is not AdmissionResult.RESERVED:
                    raise RuntimeError("fixture preparation exceeded queue limits")
                reservation.reservation.publish(ProfileOnlyUnitV1(_profile(sequence=sequence)))
            result = preparer.attempt(max_entities=128)
            if result.acknowledged_entities != 128:
                raise RuntimeError("fixture preparation failed to commit its batch")
        checkpoint = preparer.checkpoint()
        if checkpoint.failure or checkpoint.wal_frames != checkpoint.checkpointed_frames:
            raise RuntimeError("fixture preparation did not fully checkpoint")
    finally:
        preparer.close()
    initial_wal_bytes = wal.stat().st_size
    flush_private_files(args.fixture_root)
    time.sleep(args.writeback_seconds)
    background_start = time.monotonic_ns()
    background_before = written_bytes(args.block_stat)
    time.sleep(args.background_seconds)
    background_after = written_bytes(args.block_stat)
    background_us = (time.monotonic_ns() - background_start) // 1000

    checkpoints, commits, control_us, queue_us = [], [], [], []
    arrivals, arrivals_lock, drained = deque(), Lock(), Event()

    class TimedTransactions(SqliteTransactions):
        def commit(self, db):
            start = clock.now_monotonic_us()
            super().commit(db)
            commits.append(clock.now_monotonic_us() - start)

        def checkpoint(self, db):
            start = clock.now_monotonic_us()
            result = super().checkpoint(db)
            end = clock.now_monotonic_us()
            checkpoints.append([start, end - start, *result])
            return result

    class ObservedWorker(CheckedPersistenceWorker):
        def _dispatch_work(self, action):
            before = self._ordinary.counters.batch_entities_committed
            super()._dispatch_work(action)
            acknowledged = self._ordinary.counters.batch_entities_committed - before
            now = clock.now_monotonic_us()
            with arrivals_lock:
                for _ in range(acknowledged):
                    queue_us.append(now - arrivals.popleft())
            if self.queue.snapshot().closed_and_drained:
                drained.set()

    instance = ReceiverInstanceStart(uuid4().bytes, clock.now_monotonic_us())
    owner = ObservedWorker(
        instance=instance, database_path=path, configuration_path=config,
        boot_id_path=boot, clock=clock, transactions=TimedTransactions(),
    )
    owner.start()
    reader = None
    try:
        startup = owner.wait_started(deadline_monotonic_us=clock.now_monotonic_us() + 10_000_000)
        if startup is None or startup.database_failure is not None:
            raise RuntimeError("fixture worker failed startup")
        time.sleep(args.warmup_seconds)
        idle_start = clock.now_monotonic_us()
        idle_calls_before = len(checkpoints)
        idle_bytes_before = written_bytes(args.block_stat)
        time.sleep(args.idle_seconds)
        idle_end = clock.now_monotonic_us()
        idle_calls = len(checkpoints) - idle_calls_before
        idle_bytes_after = written_bytes(args.block_stat)

        reader = sqlite3.connect(path, isolation_level=None)
        reader.execute("BEGIN")
        reader.execute("SELECT count(*) FROM message_profiles").fetchone()
        traffic_start = clock.now_monotonic_us()
        traffic_bytes_before = written_bytes(args.block_stat)
        offered = max(1, math.ceil(args.traffic_seconds / args.period_seconds))
        next_arrival = time.monotonic()
        queue_peak = 0
        for n in range(offered):
            time.sleep(max(0, next_arrival - time.monotonic()))
            next_arrival += args.period_seconds
            if reader is not None and clock.now_monotonic_us() - traffic_start >= args.reader_seconds * 1_000_000:
                reader.close()
                reader = None
            now = clock.now_monotonic_us()
            profile = replace(
                _profile(sequence=n + 1), receiver_instance_id=instance.receiver_instance_id,
                received_at_monotonic_us=now, t1_handler_started_monotonic_us=now + 1,
                t2_packet_copied_monotonic_us=now + 2, t6_set_rx_issued_monotonic_us=now + 4,
            )
            reservation = owner.queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
            if reservation.status is not AdmissionResult.RESERVED:
                raise RuntimeError("fixture traffic was rejected")
            with arrivals_lock:
                arrivals.append(now)
                reservation.reservation.publish(ProfileOnlyUnitV1(profile))
            queue_peak = max(queue_peak, owner.queue.snapshot().published_entities)
            start = clock.now_monotonic_us()
            loaded = owner.control.load_communicator_state(deadline_monotonic_us=start + 5_000_000)
            control_us.append(clock.now_monotonic_us() - start)
            if loaded.status is not CommunicatorStateLoadStatus.STATE_UNAVAILABLE:
                raise RuntimeError("control request did not return the expected missing-state result")
        if reader is not None:
            reader.close()
            reader = None
        owner.queue.close()
        if not owner.queue.snapshot().closed_and_drained and not drained.wait(10):
            raise RuntimeError("fixture drain exceeded its budget")
        stop = owner.control.commit_receiver_clean_stop(
            ReceiverCleanStopV1(instance.receiver_instance_id, clock.now_monotonic_us(), 0),
            deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000,
        )
        if stop.disposition is not ReceiverCleanStopCommitDisposition.COMMITTED:
            raise RuntimeError("fixture clean marker did not commit")
        owner.request_stop(deadline_monotonic_us=clock.now_monotonic_us() + 10_000_000)
        owner.join(15)
        if owner.is_alive() or owner.failure is not None:
            raise RuntimeError(f"fixture worker did not stop successfully: {owner.failure!r}")
        flush_private_files(args.fixture_root)
        time.sleep(args.writeback_seconds)
        traffic_end = clock.now_monotonic_us()
        traffic_bytes_after = written_bytes(args.block_stat)
        with sqlite3.connect(path) as verification:
            if verification.execute("PRAGMA integrity_check").fetchone() != ("ok",):
                raise RuntimeError("fixture integrity failed")
            count = verification.execute(
                "SELECT count(*) FROM message_profiles WHERE receiver_instance_id=?", (instance.receiver_instance_id,)
            ).fetchone()[0]
            if count != offered or len(queue_us) != offered:
                raise RuntimeError("fixture durable count did not match offered traffic")
        if source_hashes != manifest(root):
            raise RuntimeError("source files changed during measurement")
        def byte_delta(before, after):
            if before is None:
                return None
            if after < before:
                raise RuntimeError("block-write counter reset during measurement")
            return after - before
        data = dict(
            label=args.label, mode="host-smoke" if args.host_smoke else "target-Pi",
            parameters={k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
            source_sha256=source_hashes, harness_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            python=sys.version, sqlite=sqlite3.sqlite_version, model=model,
            fixture_preparation_profiles=sequence, initial_wal_bytes=initial_wal_bytes,
            background=dict(duration_us=background_us, write_bytes=byte_delta(background_before, background_after)),
            idle=dict(duration_us=idle_end - idle_start, checkpoint_calls=idle_calls,
                      write_bytes=byte_delta(idle_bytes_before, idle_bytes_after)),
            traffic=dict(duration_us=traffic_end - traffic_start, offered=offered, durable=count,
                         queue_peak_entities=queue_peak, write_bytes=byte_delta(traffic_bytes_before, traffic_bytes_after)),
            checkpoint_columns=["monotonic_us", "duration_us", "busy", "total", "checkpointed"],
            checkpoints=checkpoints, counters=asdict(owner._recovery.counters),
            checkpoint_duration_us=percentiles([row[1] for row in checkpoints]),
            commit_duration_us=percentiles(commits), control_latency_us=percentiles(control_us),
            queue_latency_us=percentiles(queue_us), raw_control_latency_us=control_us,
            raw_queue_latency_us=queue_us,
        )
        with args.output.open("x") as output:
            json.dump(data, output, indent=2)
            output.write("\n")
        print(json.dumps({"label": args.label, "idle_checkpoint_calls": idle_calls, "durable": count}))
    finally:
        if reader is not None:
            reader.close()
        owner.finish_test()


if __name__ == "__main__":
    main()
