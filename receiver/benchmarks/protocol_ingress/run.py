#!/usr/bin/env python3
"""Characterize production protocol ingress on a Raspberry Pi."""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import math
import multiprocessing
import os
import platform
import queue as queue_module
import shutil
import ssl
import subprocess
import sys
import time
import traceback
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, Sequence


REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "receiver"))

from cura_receiver.generated import protocol_v2_lora_generated as protocol
from cura_receiver.generated.receiver_enums_generated import (
    AckSelection,
    AckTxResult,
    PersistenceAdmissionState,
    ProcessingResult,
    RadioState,
)
from cura_receiver.persist_queue import (
    PersistQueue,
    PersistQueueBatchDisposition,
    PersistenceAdmissionSnapshot,
)
from cura_receiver.platform.linux_clocks import LinuxOsClock
from cura_receiver.protocol_ingress import (
    ProtocolIngress,
    ProtocolIngressPacketV1,
    ProtocolIngressTerminalV1,
)
from cura_receiver.protocol_v2_lora_crypto import seal_frame


FORMAT_VERSION = 1
BENCHMARK_NAME = "receiver_protocol_ingress"
GOLDEN_VECTORS_PATH = (
    REPO_ROOT / "protocol/protocol-v2-lora/test-vectors/golden_vectors.json"
)
RELEVANT_SOURCE_PATHS = (
    Path("receiver/benchmarks/protocol_ingress/run.py"),
    Path("receiver/cura_receiver/generated/protocol_v2_lora_generated.py"),
    Path("receiver/cura_receiver/persist_queue.py"),
    Path("receiver/cura_receiver/persist_queue_entities.py"),
    Path("receiver/cura_receiver/platform/linux_clocks.py"),
    Path("receiver/cura_receiver/protocol_ingress.py"),
    Path("receiver/cura_receiver/protocol_v2_lora_crypto.py"),
    Path("protocol/protocol-v2-lora/test-vectors/golden_vectors.json"),
)
REVIEWED_BACKLOG_FRAME = bytes.fromhex(
    "2002010203040506070844332211"
    "0badc99335db74910f378aeac41892874b4259d599b55914dd43ecc75026b743"
    "e36bb78761644e77"
)
CPU_BLOCK_SIZE = 1 << 20
IO_BLOCK_SIZE = 64 << 10
IO_FILE_SIZE_LIMIT = 64 << 20
WORKER_READY_TIMEOUT_SECONDS = 10.0
WORKER_JOIN_TIMEOUT_SECONDS = 5.0
NODE_MINIMUM_RETRY_US = 600_000
ACK_AIRTIME_US = 61_696


def _parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("idle", "load"), required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--warmups", type=int, default=200)
    parser.add_argument("--samples", type=int, default=2_000)
    parser.add_argument("--max-runtime-seconds", type=int, default=60)
    parser.add_argument(
        "--source-commit",
        default="",
        help="Measured commit when the isolated tree has no Git metadata.",
    )
    parser.add_argument(
        "--source-tree-state",
        choices=("auto", "clean", "dirty"),
        default="auto",
        help="Measured tree state when the isolated tree has no Git metadata.",
    )
    args = parser.parse_args(argv)
    if args.warmups < 0:
        parser.error("--warmups must be non-negative")
    if args.samples < 2 or args.samples % 2 != 0:
        parser.error("--samples must be a positive even count of at least two")
    if args.max_runtime_seconds <= 0:
        parser.error("--max-runtime-seconds must be positive")
    if not args.output_dir.is_absolute():
        parser.error("--output-dir must be absolute")
    return args


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _git_output(*arguments: str) -> str | None:
    try:
        completed = subprocess.run(
            ("git", "-C", str(REPO_ROOT), *arguments),
            check=True,
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return completed.stdout.strip()


def _source_manifest(args: argparse.Namespace) -> dict[str, Any]:
    files = []
    combined = hashlib.sha256()
    for relative_path in RELEVANT_SOURCE_PATHS:
        digest = _sha256(REPO_ROOT / relative_path)
        path_text = relative_path.as_posix()
        files.append({"path": path_text, "sha256": digest})
        combined.update(path_text.encode("utf-8"))
        combined.update(b"\0")
        combined.update(digest.encode("ascii"))
        combined.update(b"\n")

    detected_commit = _git_output("rev-parse", "HEAD")
    commit = args.source_commit.strip() or detected_commit
    if args.source_tree_state == "auto":
        status = _git_output("status", "--porcelain", "--untracked-files=all")
        tree_state = None if status is None else ("dirty" if status else "clean")
    else:
        tree_state = args.source_tree_state
    return {
        "commit": commit,
        "tree_state": tree_state,
        "combined_sha256": combined.hexdigest(),
        "files": files,
    }


def _read_os_release() -> dict[str, str]:
    values: dict[str, str] = {}
    try:
        lines = Path("/etc/os-release").read_text(encoding="utf-8").splitlines()
    except OSError:
        return values
    for line in lines:
        if "=" not in line or line.startswith("#"):
            continue
        key, value = line.split("=", 1)
        values[key] = value.strip().strip('"')
    return values


def _read_text(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8").rstrip("\n\0")
    except OSError:
        return None


def _environment(output_parent: Path) -> dict[str, Any]:
    governor_paths = sorted(
        Path("/sys/devices/system/cpu").glob("cpu[0-9]*/cpufreq/scaling_governor")
    )
    governors = sorted(
        {value for path in governor_paths if (value := _read_text(path)) is not None}
    )
    temperatures = []
    for temperature_path in sorted(
        Path("/sys/class/thermal").glob("thermal_zone*/temp")
    ):
        raw = _read_text(temperature_path)
        if raw is None:
            continue
        try:
            milli_c = int(raw)
        except ValueError:
            continue
        temperatures.append(
            {
                "zone": temperature_path.parent.name,
                "type": _read_text(temperature_path.parent / "type"),
                "milli_c": milli_c,
            }
        )
    try:
        affinity = sorted(os.sched_getaffinity(0))
    except AttributeError:
        affinity = None
    try:
        load_average = list(os.getloadavg())
    except OSError:
        load_average = None
    disk = shutil.disk_usage(output_parent)
    try:
        cryptography_version = importlib.metadata.version("cryptography")
    except importlib.metadata.PackageNotFoundError:
        cryptography_version = None
    return {
        "captured_at_utc": datetime.now(UTC).isoformat(),
        "raspberry_pi_model": _read_text(Path("/proc/device-tree/model")),
        "os_release": _read_os_release(),
        "kernel_release": platform.release(),
        "machine": platform.machine(),
        "python": platform.python_version(),
        "python_implementation": platform.python_implementation(),
        "openssl": ssl.OPENSSL_VERSION,
        "cryptography": cryptography_version,
        "logical_cpu_count": os.cpu_count(),
        "cpu_affinity": affinity,
        "cpu_governors": governors,
        "load_average_1m_5m_15m": load_average,
        "temperatures": temperatures,
        "output_filesystem_free_bytes": disk.free,
    }


def _require_raspberry_pi() -> str:
    model = _read_text(Path("/proc/device-tree/model"))
    if model is None or "Raspberry Pi" not in model:
        raise RuntimeError(
            "protocol-ingress characterization must run on a Raspberry Pi target"
        )
    return model


def _load_reviewed_inputs() -> dict[str, bytes]:
    document = json.loads(GOLDEN_VECTORS_PATH.read_text(encoding="utf-8"))
    current_vector = next(
        vector
        for vector in document["reading_vectors"]
        if vector["name"] == "current_reading_all_fields_valid"
    )
    accepted_vector = next(
        vector
        for vector in document["ack_vectors"]
        if vector["name"] == "accepted"
    )
    node_key = bytes.fromhex(document["crypto"]["node_key_hex"])
    node_id = bytes.fromhex(current_vector["clear_header"]["node_id_hex"])
    message_id = current_vector["clear_header"]["message_id"]
    reading_body = bytes.fromhex(current_vector["reading"]["encoded_hex"])
    backlog = seal_frame(
        node_key,
        protocol.ClearHeader(
            control=current_vector["clear_header"]["control"],
            domain=protocol.Domain.BACKLOG_READING_UPLINK,
            node_id=node_id,
            message_id=message_id,
        ),
        reading_body,
    )
    if backlog != REVIEWED_BACKLOG_FRAME:
        raise RuntimeError("constructed backlog frame differs from reviewed literal")
    return {
        "node_key": node_key,
        "node_id": node_id,
        "current": bytes.fromhex(current_vector["encrypted"]["frame_hex"]),
        "backlog": backlog,
        "accepted_ack": bytes.fromhex(accepted_vector["encrypted"]["frame_hex"]),
    }


def _cpu_load_worker(
    worker_id: str,
    stop_event: Any,
    ready_queue: Any,
    result_queue: Any,
    deadline_ns: int,
) -> None:
    iterations = 0
    bytes_processed = 0
    error = None
    termination_reason = "unknown"
    started_ns = time.monotonic_ns()
    seed = b"cura-agrorum-protocol-ingress-cpu-load\n"
    block = (seed * ((CPU_BLOCK_SIZE // len(seed)) + 1))[:CPU_BLOCK_SIZE]
    try:
        if len(block) != CPU_BLOCK_SIZE:
            raise RuntimeError("CPU load block did not reach its fixed size")
        ready_queue.put({"worker_id": worker_id, "kind": "cpu"})
        while (
            not stop_event.is_set()
            and time.monotonic_ns() < deadline_ns
        ):
            hashlib.sha256(block).digest()
            iterations += 1
            bytes_processed += len(block)
        termination_reason = (
            "parent_stop" if stop_event.is_set() else "runtime_deadline"
        )
    except BaseException as exc:  # Preserve child evidence for parent reporting.
        termination_reason = "error"
        error = f"{type(exc).__name__}: {exc}"
    finally:
        finished_ns = time.monotonic_ns()
        result_queue.put(
            {
                "worker_id": worker_id,
                "kind": "cpu",
                "iterations": iterations,
                "bytes_processed": bytes_processed,
                "active_duration_ns": finished_ns - started_ns,
                "termination_reason": termination_reason,
                "error": error,
            }
        )


def _io_load_worker(
    worker_id: str,
    load_path: str,
    stop_event: Any,
    ready_queue: Any,
    result_queue: Any,
    deadline_ns: int,
) -> None:
    bytes_written = 0
    sync_count = 0
    wrap_count = 0
    error = None
    termination_reason = "unknown"
    started_ns = time.monotonic_ns()
    seed = b"cura-agrorum-protocol-ingress-io-load\n"
    block = (seed * ((IO_BLOCK_SIZE // len(seed)) + 1))[:IO_BLOCK_SIZE]
    descriptor: int | None = None
    try:
        if len(block) != IO_BLOCK_SIZE:
            raise RuntimeError("I/O load block did not reach its fixed size")
        descriptor = os.open(load_path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
        ready_queue.put({"worker_id": worker_id, "kind": "io"})
        while (
            not stop_event.is_set()
            and time.monotonic_ns() < deadline_ns
        ):
            view = memoryview(block)
            while view:
                written = os.write(descriptor, view)
                view = view[written:]
            os.fdatasync(descriptor)
            sync_count += 1
            bytes_written += len(block)
            if os.lseek(descriptor, 0, os.SEEK_CUR) >= IO_FILE_SIZE_LIMIT:
                os.lseek(descriptor, 0, os.SEEK_SET)
                wrap_count += 1
        termination_reason = (
            "parent_stop" if stop_event.is_set() else "runtime_deadline"
        )
    except BaseException as exc:  # Preserve child evidence for parent reporting.
        termination_reason = "error"
        error = f"{type(exc).__name__}: {exc}"
    finally:
        finished_ns = time.monotonic_ns()
        maximum_file_size = (
            os.fstat(descriptor).st_size if descriptor is not None else None
        )
        if descriptor is not None:
            os.close(descriptor)
        result_queue.put(
            {
                "worker_id": worker_id,
                "kind": "io",
                "sync_count": sync_count,
                "bytes_written_total": bytes_written,
                "wrap_count": wrap_count,
                "maximum_file_size": maximum_file_size,
                "file_size_limit": IO_FILE_SIZE_LIMIT,
                "active_duration_ns": finished_ns - started_ns,
                "termination_reason": termination_reason,
                "error": error,
            }
        )


class _LoadGroup:
    def __init__(self, output_dir: Path, deadline_ns: int) -> None:
        context = multiprocessing.get_context("spawn")
        self._stop_event = context.Event()
        self._ready_queue = context.Queue()
        self._result_queue = context.Queue()
        self._load_path = output_dir / ".bounded-io-load.bin"
        cpu_count = os.cpu_count() or 1
        self._cpu_workers = min(max(cpu_count - 1, 0), 3)
        self._processes = [
            context.Process(
                name=f"protocol-ingress-cpu-load-{index}",
                target=_cpu_load_worker,
                args=(
                    f"cpu-{index}",
                    self._stop_event,
                    self._ready_queue,
                    self._result_queue,
                    deadline_ns,
                ),
            )
            for index in range(self._cpu_workers)
        ]
        self._processes.append(
            context.Process(
                name="protocol-ingress-io-load",
                target=_io_load_worker,
                args=(
                    "io-0",
                    str(self._load_path),
                    self._stop_event,
                    self._ready_queue,
                    self._result_queue,
                    deadline_ns,
                ),
            )
        )
        self._started_processes: list[Any] = []

    @property
    def recipe(self) -> dict[str, Any]:
        return {
            "cpu_workers": self._cpu_workers,
            "cpu_block_bytes": CPU_BLOCK_SIZE,
            "cpu_operation": "sha256 of fixed public block",
            "io_workers": 1,
            "io_block_bytes": IO_BLOCK_SIZE,
            "io_file_size_limit": IO_FILE_SIZE_LIMIT,
            "io_operation": (
                "write then fdatasync fixed public block, wrapping at file limit"
            ),
        }

    def start(self) -> None:
        for process in self._processes:
            process.start()
            self._started_processes.append(process)
        ready_ids = set()
        deadline = time.monotonic() + WORKER_READY_TIMEOUT_SECONDS
        while len(ready_ids) != len(self._processes):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError("bounded-load workers did not become ready")
            try:
                ready = self._ready_queue.get(timeout=remaining)
            except queue_module.Empty as exc:
                raise RuntimeError(
                    "bounded-load workers did not become ready"
                ) from exc
            ready_ids.add(ready["worker_id"])

    def stop(self) -> list[dict[str, Any]]:
        self._stop_event.set()
        terminated = set()
        for process in self._started_processes:
            process.join(WORKER_JOIN_TIMEOUT_SECONDS)
            if process.is_alive():
                terminated.add(process.name)
                process.terminate()
                process.join(WORKER_JOIN_TIMEOUT_SECONDS)

        results: dict[str, dict[str, Any]] = {}
        expected_results = len(self._started_processes) - len(terminated)
        result_deadline = time.monotonic() + 1.0
        while len(results) < expected_results:
            try:
                result = self._result_queue.get(
                    timeout=max(0.0, result_deadline - time.monotonic())
                )
            except queue_module.Empty:
                break
            results[result["worker_id"]] = result
        for index, process in enumerate(self._processes):
            worker_id = f"cpu-{index}" if index < self._cpu_workers else "io-0"
            result = results.setdefault(
                worker_id,
                {
                    "worker_id": worker_id,
                    "kind": "cpu" if index < self._cpu_workers else "io",
                    "error": "worker exited without reporting a result",
                },
            )
            result["process_exitcode"] = process.exitcode
            result["parent_terminated"] = process.name in terminated
        try:
            self._load_path.unlink()
        except FileNotFoundError:
            pass
        self._ready_queue.close()
        self._result_queue.close()
        return [results[key] for key in sorted(results)]


def _packet(frame: bytes, sequence: int, clock: LinuxOsClock) -> ProtocolIngressPacketV1:
    copied_at = clock.now_monotonic_us()
    return ProtocolIngressPacketV1(
        receiver_instance_id=b"protocol-ingress",
        occurrence_sequence=sequence,
        received_at_monotonic_us=copied_at,
        frame=frame,
        rssi_dbm_x2=-140,
        snr_db_x4=12,
        irq_status=2,
        device_errors=0,
        busy_wait_total_us=0,
        busy_wait_max_us=0,
        busy_wait_count=0,
        busy_timeout_count=0,
        last_busy_timeout_opcode=None,
        t1_handler_started_monotonic_us=copied_at,
        t2_packet_copied_monotonic_us=copied_at,
    )


def _measure_one(
    *,
    ingress: ProtocolIngress,
    persist_queue: PersistQueue,
    clock: LinuxOsClock,
    frame: bytes,
    accepted_ack: bytes,
    sequence: int,
) -> int:
    packet = _packet(frame, sequence, clock)
    started_ns = time.perf_counter_ns()
    decision = ingress.begin(packet)
    elapsed_ns = time.perf_counter_ns() - started_ns
    if (
        decision.pre_tx_profile.processing_result is not ProcessingResult.ACCEPTED
        or decision.pre_tx_profile.ack_selected is not AckSelection.ACCEPTED
        or decision.pre_tx_profile.ack_frame != accepted_ack
    ):
        raise RuntimeError("reviewed frame did not produce the exact ACCEPTED decision")
    ingress.finalize(
        decision,
        ProtocolIngressTerminalV1(
            ack_tx_result=AckTxResult.SUPPRESSED_AIRTIME_BUDGET,
            t4_set_tx_attempted_monotonic_us=None,
            t5_tx_done_monotonic_us=None,
            t6_set_rx_issued_monotonic_us=clock.now_monotonic_us(),
            radio_state=RadioState.RX_SINGLE,
        ),
    )
    lease = persist_queue.claim_batch(max_entities=1)
    if lease is None or len(lease.entries) != 1:
        raise RuntimeError("terminal ingress publication was not claimable")
    lease.acknowledge_durable((PersistQueueBatchDisposition.SQLITE_COMMITTED,))
    return elapsed_ns


def _percentile(values: list[int], percentile: int) -> int:
    ordered = sorted(values)
    rank = max(1, math.ceil((percentile / 100) * len(ordered)))
    return ordered[rank - 1]


def _latency_summary(values: list[int]) -> dict[str, int]:
    return {
        "count": len(values),
        "p50_ns": _percentile(values, 50),
        "p95_ns": _percentile(values, 95),
        "p99_ns": _percentile(values, 99),
        "max_ns": max(values),
    }


def _summaries(samples_ns: list[int]) -> dict[str, dict[str, int]]:
    return {
        "combined": _latency_summary(samples_ns),
        "current": _latency_summary(samples_ns[0::2]),
        "backlog": _latency_summary(samples_ns[1::2]),
    }


def _run_benchmark(
    args: argparse.Namespace,
    evidence: dict[str, Any],
    deadline_ns: int,
) -> None:
    inputs = _load_reviewed_inputs()
    persist_queue = PersistQueue(capacity_entities=1)
    persist_queue.publish_admission_state(
        PersistenceAdmissionSnapshot(
            generation=1,
            state=PersistenceAdmissionState.AVAILABLE,
            changed_at_monotonic_us=1,
        )
    )
    clock = LinuxOsClock()
    ingress = ProtocolIngress(
        queue=persist_queue,
        monotonic_clock=clock,
        auth_node_keys={inputs["node_id"]: inputs["node_key"]},
    )
    load_group = _LoadGroup(args.output_dir, deadline_ns) if args.mode == "load" else None
    evidence["workload"] = (
        load_group.recipe
        if load_group is not None
        else {"cpu_workers": 0, "io_workers": 0, "operation": "no deliberate load"}
    )
    worker_results: list[dict[str, Any]] = []
    try:
        if load_group is not None:
            load_group.start()
        sequence = 1
        cold_ns = _measure_one(
            ingress=ingress,
            persist_queue=persist_queue,
            clock=clock,
            frame=inputs["current"],
            accepted_ack=inputs["accepted_ack"],
            sequence=sequence,
        )
        evidence["cold_sample"] = {"domain": "current", "latency_ns": cold_ns}
        sequence += 1
        for index in range(args.warmups):
            if time.monotonic_ns() >= deadline_ns:
                raise TimeoutError("benchmark exceeded its bounded runtime")
            domain = "current" if index % 2 == 0 else "backlog"
            _measure_one(
                ingress=ingress,
                persist_queue=persist_queue,
                clock=clock,
                frame=inputs[domain],
                accepted_ack=inputs["accepted_ack"],
                sequence=sequence,
            )
            sequence += 1
        samples_ns = []
        for index in range(args.samples):
            if time.monotonic_ns() >= deadline_ns:
                raise TimeoutError("benchmark exceeded its bounded runtime")
            domain = "current" if index % 2 == 0 else "backlog"
            latency_ns = _measure_one(
                ingress=ingress,
                persist_queue=persist_queue,
                clock=clock,
                frame=inputs[domain],
                accepted_ack=inputs["accepted_ack"],
                sequence=sequence,
            )
            samples_ns.append(latency_ns)
            sequence += 1
        evidence["samples"] = {
            "unit": "ns",
            "order": "alternating current then backlog",
            "latency": samples_ns,
        }
        evidence["summary"] = _summaries(samples_ns)
        maximum_ns = evidence["summary"]["combined"]["max_ns"]
        evidence["coarse_margin"] = {
            "node_minimum_retry_us": NODE_MINIMUM_RETRY_US,
            "ack_airtime_us": ACK_AIRTIME_US,
            "observed_maximum_ingress_us": maximum_ns / 1_000,
            "remaining_us": NODE_MINIMUM_RETRY_US
            - ACK_AIRTIME_US
            - (maximum_ns / 1_000),
            "qualification": (
                "Excludes scheduling outside ProtocolIngress.begin(), SX1262 profile "
                "changes, buffer writes, SetTx, IRQ handling and recovery; it is not "
                "an end-to-end guarantee or a test threshold."
            ),
        }
    finally:
        if load_group is not None:
            worker_results = load_group.stop()
        evidence["workload"]["worker_results"] = worker_results
    worker_failures = [
        result
        for result in worker_results
        if result.get("error") is not None
        or result.get("parent_terminated")
        or result.get("process_exitcode") != 0
        or result.get("termination_reason") != "parent_stop"
    ]
    if worker_failures:
        raise RuntimeError("one or more bounded-load workers failed")


def _format_ns(value: int) -> str:
    return f"{value / 1_000:.3f} us"


def _summary_markdown(evidence: dict[str, Any]) -> str:
    source = evidence.get("source", {})
    environment = evidence.get("environment_start", {})
    os_release = environment.get("os_release", {})
    lines = [
        "# Protocol ingress benchmark result",
        "",
        f"- Status: `{evidence.get('status', 'unknown')}`",
        f"- Mode: `{evidence.get('mode', 'unknown')}`",
        f"- Captured: `{evidence.get('generated_at_utc', 'unknown')}`",
        f"- Commit: `{source.get('commit') or 'unavailable'}`",
        f"- Source tree: `{source.get('tree_state') or 'unavailable'}`",
        f"- Source manifest SHA-256: `{source.get('combined_sha256', 'unavailable')}`",
        "",
        "## Environment",
        "",
        f"- Model: {environment.get('raspberry_pi_model') or 'unavailable'}",
        f"- OS: {os_release.get('PRETTY_NAME', 'unavailable')}",
        f"- Kernel/machine: `{environment.get('kernel_release', 'unavailable')}` / "
        f"`{environment.get('machine', 'unavailable')}`",
        f"- Python: `{environment.get('python', 'unavailable')}`; cryptography: "
        f"`{environment.get('cryptography', 'unavailable')}`",
        f"- Logical CPUs / affinity: `{environment.get('logical_cpu_count')}` / "
        f"`{environment.get('cpu_affinity')}`",
        f"- CPU governors: `{environment.get('cpu_governors')}`",
        f"- Start load average: `{environment.get('load_average_1m_5m_15m')}`",
        f"- End load average: "
        f"`{evidence.get('environment_end', {}).get('load_average_1m_5m_15m')}`",
        "",
        "## Load recipe",
        "",
        "```json",
        json.dumps(evidence.get("workload", {}), indent=2, sort_keys=True),
        "```",
        "",
    ]
    cold = evidence.get("cold_sample")
    summary = evidence.get("summary")
    if cold is not None and summary is not None:
        lines.extend(
            [
                "## Latency",
                "",
                f"Cold current sample: {_format_ns(cold['latency_ns'])}",
                "",
                "| Domain | Samples | p50 | p95 | p99 | Maximum |",
                "| --- | ---: | ---: | ---: | ---: | ---: |",
            ]
        )
        for domain in ("combined", "current", "backlog"):
            values = summary[domain]
            lines.append(
                f"| {domain} | {values['count']} | {_format_ns(values['p50_ns'])} | "
                f"{_format_ns(values['p95_ns'])} | {_format_ns(values['p99_ns'])} | "
                f"{_format_ns(values['max_ns'])} |"
            )
        margin = evidence["coarse_margin"]
        lines.extend(
            [
                "",
                "## Coarse timing margin",
                "",
                f"Observed remainder: `{margin['remaining_us']:.3f} us` = "
                f"{margin['node_minimum_retry_us']} us retry interval - "
                f"{margin['ack_airtime_us']} us ACK airtime - "
                f"{margin['observed_maximum_ingress_us']:.3f} us maximum ingress.",
                "",
                margin["qualification"],
            ]
        )
    if "error" in evidence:
        lines.extend(
            [
                "## Failure",
                "",
                f"`{evidence['error']['type']}: {evidence['error']['message']}`",
                "",
                "See `raw.json` for the traceback and partial evidence.",
            ]
        )
    return "\n".join(lines) + "\n"


def _write_results(output_dir: Path, evidence: dict[str, Any]) -> None:
    (output_dir / "raw.json").write_text(
        json.dumps(evidence, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (output_dir / "summary.md").write_text(
        _summary_markdown(evidence),
        encoding="utf-8",
    )


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    try:
        args.output_dir.mkdir(parents=True, exist_ok=False)
    except FileExistsError:
        raise SystemExit(f"output directory already exists: {args.output_dir}")
    evidence: dict[str, Any] = {
        "format_version": FORMAT_VERSION,
        "benchmark": BENCHMARK_NAME,
        "status": "running",
        "mode": args.mode,
        "generated_at_utc": datetime.now(UTC).isoformat(),
        "arguments": {
            "warmups": args.warmups,
            "samples": args.samples,
            "max_runtime_seconds": args.max_runtime_seconds,
        },
        "methodology": {
            "timed_operation": "ProtocolIngress.begin()",
            "timer": "time.perf_counter_ns",
            "percentile_method": "nearest-rank",
            "garbage_collection": "enabled",
            "packet_construction_timed": False,
            "terminal_publication_and_queue_drain_timed": False,
            "domains_alternated": ["current", "backlog"],
        },
    }
    exit_code = 0
    try:
        evidence["source"] = _source_manifest(args)
        evidence["environment_start"] = _environment(args.output_dir.parent)
        _require_raspberry_pi()
        deadline_ns = time.monotonic_ns() + (
            args.max_runtime_seconds * 1_000_000_000
        )
        _run_benchmark(args, evidence, deadline_ns)
        evidence["status"] = "completed"
    except BaseException as exc:  # Persist diagnostic evidence before returning.
        evidence["status"] = "failed"
        evidence["error"] = {
            "type": type(exc).__name__,
            "message": str(exc),
            "traceback": traceback.format_exc(),
        }
        exit_code = 1
    finally:
        try:
            evidence["environment_end"] = _environment(args.output_dir.parent)
        except BaseException as exc:
            evidence["environment_end_error"] = f"{type(exc).__name__}: {exc}"
        _write_results(args.output_dir, evidence)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
