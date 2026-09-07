#!/usr/bin/env python3
"""Explicit, non-gating Pi storage characterization; never invoked by tests."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import platform
import sqlite3
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "receiver"), str(ROOT / "protocol/protocol-v2-lora/python")]

from cura_receiver.database_initializer import initialize_database
from cura_receiver.generated.receiver_enums_generated import AdmissionResult
from cura_receiver.ordinary_persistence import OrdinaryPersistence
from cura_receiver.persist_queue import PersistQueue
from cura_receiver.persist_queue_entities import (
    CLOCK_OBSERVATION_V1_SPEC,
    DIAGNOSTIC_V1_SPEC,
    MEASUREMENT_PROFILE_V1_SPEC,
    PROFILE_ONLY_V1_SPEC,
    RECEIVER_HEALTH_REQUEST_V1_SPEC,
    ProfileOnlyUnitV1,
)
from cura_receiver.platform.linux_clocks import LinuxOsClock
from cura_receiver.receiver_startup import (
    ReceiverInstanceStart,
    insert_receiver_instance_start,
)
from cura_receiver.sqlite_database import open_receiver_database
from cura_receiver.sqlite_transactions import SqliteTransactions
from tests.support.builders.persistence import (
    GROUP,
    INSTANCE,
    _diagnostic,
    _health_request,
    _measurement,
    _observation,
    _profile,
)


def _inputs(count):
    units = []
    for index in range(count):
        sequence = index + 1
        phase = index % 20
        if phase < 16:
            # Paired current/backlog messages share the exact application sample;
            # each transport message remains distinct. Construction is untimed.
            units.append(
                (
                    _measurement(
                        sequence=sequence,
                        message=100 + index,
                        sample=200 + index // 2,
                        domain=1 if index % 2 == 0 else 2,
                    ),
                    MEASUREMENT_PROFILE_V1_SPEC,
                )
            )
        elif phase == 16:
            units.append(
                (ProfileOnlyUnitV1(_profile(sequence=sequence)), PROFILE_ONLY_V1_SPEC)
            )
        elif phase == 17:
            units.append((_observation(sequence=sequence), CLOCK_OBSERVATION_V1_SPEC))
        elif phase == 18:
            units.append(
                (_health_request(sequence=sequence), RECEIVER_HEALTH_REQUEST_V1_SPEC)
            )
        else:
            units.append((_diagnostic(sequence=sequence), DIAGNOSTIC_V1_SPEC))
    return units


def _percentiles(values):
    ordered = sorted(values)
    return (
        {
            str(p): ordered[max(0, math.ceil(len(ordered) * p / 100) - 1)]
            for p in (50, 95, 99, 100)
        }
        if ordered
        else {}
    )


def _case(output, units, mode, batch_size, rate, checkpoint_every):
    output.mkdir()
    path = output / "receiver.db"
    initialize_database(path, GROUP)
    connection = open_receiver_database(path, GROUP, minimum_free_bytes=0).connection
    instance = ReceiverInstanceStart(INSTANCE, 0)
    insert_receiver_instance_start(connection, instance, b"b" * 16)
    wake = threading.Event()
    done = threading.Event()
    stop = threading.Event()
    queue = PersistQueue(wake_event=wake)
    commits = []
    arrivals = []
    attempts = []
    checkpoints = []
    errors = []

    class TimedTransactions(SqliteTransactions):
        def commit(self, connection):
            started = time.perf_counter_ns()
            super().commit(connection)
            commits.append((time.perf_counter_ns() - started) / 1000)

    owner = OrdinaryPersistence(
        connection,
        queue,
        instance=instance,
        database_path=path,
        group_id=GROUP,
        clock=LinuxOsClock(),
        transactions=TimedTransactions(),
    )
    # Deliberate benchmark-only experiment on this newly created private DB.
    # Production construction still enforces FULL and receives no NORMAL option.
    # Any storage failure aborts this experiment; recovery under NORMAL is not tested.
    connection.execute(f"PRAGMA synchronous={mode}")
    assert connection.execute("PRAGMA synchronous").fetchone() == (
        (2 if mode == "FULL" else 1),
    )
    owner.enable_admission()
    started_ns = time.perf_counter_ns()
    limit_ns = started_ns + int((len(units) / rate + 60) * 1e9)

    def publish():
        try:
            for index, (entity, spec) in enumerate(units):
                due = started_ns + int(index * 1e9 / rate)
                remaining = (due - time.perf_counter_ns()) / 1e9
                if stop.wait(max(0, remaining)):
                    break
                reservation = queue.try_reserve_one(spec)
                if reservation.status is AdmissionResult.RESERVED:
                    reservation.reservation.publish(entity)
                elif reservation.status is not AdmissionResult.QUEUE_FULL:
                    raise RuntimeError("storage admission unexpectedly unavailable")
                arrivals.append(
                    (
                        (time.perf_counter_ns() - started_ns) / 1000,
                        queue.snapshot().published_entities,
                        reservation.status.name,
                    )
                )
        except BaseException as error:
            errors.append(repr(error))
        finally:
            done.set()
            wake.set()

    producer = threading.Thread(
        target=publish, name="benchmark-entity-publisher", daemon=True
    )
    producer.start()
    oldest_wait_ns = started_ns
    acknowledged = 0
    batch_count = 0
    try:
        while not done.is_set() or queue.snapshot().published_entities:
            if errors:
                raise RuntimeError(errors[0])
            now = time.perf_counter_ns()
            if now > limit_ns:
                raise RuntimeError("bounded benchmark drain deadline exceeded")
            wake.clear()
            queued = queue.snapshot().published_entities
            # Experiment-local batch formation, with a 100 ms flush limit.
            # This is not the receiver's deferred control scheduler.
            if queued == 0:
                oldest_wait_ns = now
                if done.is_set():
                    break
                wake.wait(timeout=0.1)
                continue
            if (
                queued < batch_size
                and not done.is_set()
                and now - oldest_wait_ns < 100_000_000
            ):
                wake.wait(timeout=(100_000_000 - (now - oldest_wait_ns)) / 1e9)
                continue
            before = time.perf_counter_ns()
            result = owner.attempt(max_entities=batch_size)
            after = time.perf_counter_ns()
            if result is None or result.failure is not None:
                raise RuntimeError(f"benchmark transaction failed: {result!r}")
            acknowledged += result.acknowledged_entities
            batch_count += 1
            wal = path.with_name(path.name + "-wal")
            wal_bytes = wal.stat().st_size if wal.exists() else 0
            attempts.append(
                (
                    (after - started_ns) / 1000,
                    (after - before) / 1000,
                    result.acknowledged_entities,
                    queued,
                    queue.snapshot().published_entities,
                    wal_bytes,
                )
            )
            oldest_wait_ns = after
            if batch_count % checkpoint_every == 0 and wal_bytes >= 262144:
                result = owner.checkpoint()
                if result is None or result.failure is not None:
                    raise RuntimeError(f"benchmark checkpoint failed: {result!r}")
                checkpoints.append(
                    (
                        (time.perf_counter_ns() - started_ns) / 1000,
                        result.duration_us,
                        result.wal_frames,
                        result.checkpointed_frames,
                        queue.snapshot().published_entities,
                    )
                )
        producer.join(timeout=5)
        assert not producer.is_alive() and not errors, errors
        elapsed_us = (time.perf_counter_ns() - started_ns) / 1000
        result = owner.checkpoint()
        assert result is not None and result.failure is None
        checkpoints.append(
            (
                (time.perf_counter_ns() - started_ns) / 1000,
                result.duration_us,
                result.wal_frames,
                result.checkpointed_frames,
                queue.snapshot().published_entities,
            )
        )
        accepted = sum(row[2] == "RESERVED" for row in arrivals)
        assert accepted == acknowledged
        counts = {
            table: connection.execute(f"SELECT count(*) FROM {table}").fetchone()[0]
            for table in (
                "clock_observations",
                "message_profiles",
                "reading_messages",
                "diagnostics",
                "receiver_health",
            )
        }
        assert (
            sum(
                counts[t]
                for t in (
                    "clock_observations",
                    "message_profiles",
                    "diagnostics",
                    "receiver_health",
                )
            )
            == acknowledged
        )
        assert connection.execute("PRAGMA integrity_check").fetchall() == [("ok",)]
        assert connection.execute("PRAGMA foreign_key_check").fetchall() == []
        summary = {
            "mode": mode,
            "maximum_batch_entities": batch_size,
            "offered_entities_per_second": rate,
            "offered": len(arrivals),
            "accepted": accepted,
            "queue_full": len(arrivals) - accepted,
            "acknowledged": acknowledged,
            "elapsed_us": elapsed_us,
            "entities_per_second": acknowledged * 1e6 / elapsed_us,
            "transactions_per_second": len(commits) * 1e6 / elapsed_us,
            "commit_latency_us": _percentiles(commits),
            "attempt_duration_us": _percentiles([row[1] for row in attempts]),
            "checkpoint_duration_us": _percentiles([row[1] for row in checkpoints]),
            "checkpoint_count": len(checkpoints),
            "queue_max_entities": max((row[1] for row in arrivals), default=0),
            "wal_max_bytes": max((row[5] for row in attempts), default=0),
            "actual_batch_mean": acknowledged / len(commits),
            "table_counts": counts,
        }
        (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        return summary
    finally:
        stop.set()
        producer.join(timeout=5)
        owner.close()
        assert not producer.is_alive(), "publisher did not stop"
        (output / "raw.json").write_text(
            json.dumps(
                {
                    "format_version": 1,
                    "commit_latency_us": commits,
                    "arrival_columns": [
                        "elapsed_us",
                        "queue_entities",
                        "reservation_result",
                    ],
                    "arrivals": arrivals,
                    "attempt_columns": [
                        "elapsed_us",
                        "duration_us",
                        "acknowledged",
                        "queue_before",
                        "queue_after",
                        "wal_bytes",
                    ],
                    "attempts": attempts,
                    "checkpoint_columns": [
                        "elapsed_us",
                        "duration_us",
                        "wal_frames",
                        "checkpointed_frames",
                        "queue_after",
                    ],
                    "checkpoints": checkpoints,
                    "publisher_errors": errors,
                },
                separators=(",", ":"),
            )
            + "\n"
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--source-manifest", type=Path, required=True)
    parser.add_argument("--seconds", type=int, default=10)
    parser.add_argument("--rate", type=int, default=1000)
    parser.add_argument("--batch-sizes", type=int, nargs="+", default=[1, 16, 64])
    parser.add_argument("--checkpoint-every", type=int, default=32)
    args = parser.parse_args()
    if not (
        1 <= args.seconds <= 60
        and 1 <= args.rate <= 5000
        and args.seconds * args.rate <= 100000
        and all(1 <= size <= 500 for size in args.batch_sizes)
        and args.checkpoint_every >= 1
    ):
        parser.error("workload exceeds bounded experiment limits")
    model = Path("/proc/device-tree/model").read_text().rstrip("\0\n")
    if "Raspberry Pi" not in model:
        parser.error("this benchmark requires the target Raspberry Pi")
    manifest_bytes = args.source_manifest.read_bytes()
    for name, expected in json.loads(manifest_bytes).items():
        assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == expected, name
    args.output.mkdir(parents=True, exist_ok=False)
    filesystem = subprocess.run(
        [
            "findmnt",
            "--target",
            str(args.output),
            "--json",
            "--output",
            "FSTYPE,SOURCE,TARGET",
        ],
        check=True,
        capture_output=True,
        text=True,
        timeout=5,
    )
    metadata = {
        "format_version": 1,
        "source_manifest_sha256": hashlib.sha256(manifest_bytes).hexdigest(),
        "model": model,
        "platform": platform.platform(),
        "python": sys.version,
        "sqlite": sqlite3.sqlite_version,
        "load_start": os.getloadavg(),
        "filesystem": json.loads(filesystem.stdout),
        "seconds": args.seconds,
        "rate": args.rate,
        "batch_sizes": args.batch_sizes,
        "checkpoint_every_batches": args.checkpoint_every,
        "checkpoint_minimum_wal_bytes": 262144,
        "batch_flush_limit_us": 100000,
        "queue_capacity_entities": 500,
        "workload": "80% current/backlog readings; 5% each rejected profile, clock, health, diagnostic",
    }
    (args.output / "source-manifest.json").write_bytes(manifest_bytes)
    (args.output / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    units = _inputs(args.seconds * args.rate)
    summaries = []
    for size in args.batch_sizes:
        for mode in ("FULL", "NORMAL"):
            summary = _case(
                args.output / f"{mode.lower()}-batch-{size}",
                units,
                mode,
                size,
                args.rate,
                args.checkpoint_every,
            )
            summaries.append(summary)
            print(json.dumps(summary), flush=True)
    (args.output / "summary.json").write_text(json.dumps(summaries, indent=2) + "\n")
    metadata["load_end"] = os.getloadavg()
    (args.output / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")


if __name__ == "__main__":
    main()
