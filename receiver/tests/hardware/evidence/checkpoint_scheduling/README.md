# Checkpoint scheduling comparison — 29 September 2026

**PASS for this bounded storage fixture.** After complete initial coverage,
the changed worker made zero checkpoint calls during 30 seconds of idle time
with a retained 5,244,792-byte WAL; the baseline made six. Both persisted all
120 offered profiles, committed the exact clean-stop marker with generation 0,
stopped without worker/storage failures, and passed integrity and foreign-key
checks. Both returned reader-limited partial results followed by completion.

The paired procedure took 6 minutes 44 seconds of execution, excluding the gap
between runs, and therefore qualifies under the [retention policy](../../../../../EVIDENCE.md).
The [result extract](results.json) retains all checkpoint/control/queue timing
observations, phase write counts, source identity and decisive verification.
Complete source snapshots, unchanged-file manifests, temporary paths and routine
target-test receipts are omitted. Commit-duration percentiles cannot be
recalculated from this extract because the harness did not emit their samples.

Target: `cura-receiver`, Raspberry Pi 3 Model B Rev 1.2, kernel
`6.18.50+rpt-rpi-v8/aarch64`, Python 3.13.5, SQLite 3.46.1. Private databases
were on `/dev/mmcblk0p2`, ext4 `rw,noatime`; results/logs were on RAM storage.
The pilot unit `cura-pilot-bench-8d9782e2.service` remained active/running with
PID 1130 before, between and after measurements. No service, pilot data,
radio, RTC or durability setting was changed.

Baseline: clean commit `0b9e8262ba7226e94828f124de0661a4e8463480`.
Changed: that baseline plus the five runtime file hashes in the result. The
complete runtime/SQL/schema/protocol/fixture manifests were checked unchanged
during execution and matched the local clean baseline/current tree afterward.
Both used the same recorded
[harness](../../../../benchmarks/checkpoint_scheduling/run.py).

Each database retained 8,192 prepared profile rows. After complete preparation
and private-file fsync/writeback, the harness measured 30 seconds of background
activity, started the worker, allowed six seconds of warmup, observed 30 seconds
idle, then offered one profile and one synchronous state read per second for
120 seconds. A real reader held its snapshot for the first 30 traffic seconds.
Drain, clean-stop, final checkpoint/close, private-file fsync and five seconds of
writeback observation were included in the traffic totals. Ordinary defaults
were five-second flushing and 64-entity wake/batch thresholds; the changed
checkpoint interval was five seconds.

Run the same harness sequentially against both source roots with new fixture
directories, using these arguments and their default phase durations:

```bash
python receiver/benchmarks/checkpoint_scheduling/run.py \
  --source-root /path/to/baseline-or-changed \
  --source-commit 0b9e8262ba7226e94828f124de0661a4e8463480 \
  --label baseline-or-changed \
  --fixture-root /sd-parent/new-private-directory \
  --block-stat /sys/block/mmcblk0/stat \
  --output /dev/shm/new-result.json
```

| Observation | Baseline | Changed |
|---|---:|---:|
| Idle checkpoint calls | 6 | 0 |
| All checkpoint calls | 32 | 25 |
| Complete / partial / error | 26 / 6 / 0 | 20 / 5 / 0 |
| Checkpoint p95 / maximum | 24.757 / 166.947 ms | 22.890 / 167.324 ms |
| Control p95 / maximum | 2.384 / 2.539 ms | 2.148 / 2.883 ms |
| Queue p95 / maximum | 4.220 / 4.329 s | 4.290 / 4.382 s |
| Maximum sampled queue length | 5 entities | 5 entities |
| Background writes, about 30 s | 819,200 B | 217,088 B |
| Idle writes, about 30 s | 192,512 B | 110,592 B |
| Traffic plus final writeback, about 124 s | 1,908,736 B | 2,076,672 B |

Percentiles use the nearest-rank calculation over the retained observations.
Checkpoint counts span startup through final shutdown. The last partial and
next complete frame counts were `22/0` then `26/26` for baseline, and `21/0`
then `22/22` for changed. No new writes are required for partial retries in the
separate real-reader host regression; this Pi traffic fixture kept publishing.
All 120 control calls per source met the existing five-second deadline and
0.5-second caller scheduling tolerance. These samples establish no hard I/O
latency bound or replicated performance improvement.

Whole-device bytes use block-stat field 7 in 512-byte sectors. Background
activity differed substantially between runs/phases; subtracting its rate
would yield a negative baseline traffic estimate. The changed raw traffic
total was higher. These observations cannot attribute bytes to the fixture or
establish physical-write or wear savings. Private fsync/writeback does not
prove unrelated filesystem writeback has finished or measure NAND writes.
The five-second default is retained without claiming an optimum.

The service required no restoration; isolated databases and passing results
were left under `/home/cura/cura-checkpoint-20260929-e43RtD` and RAM result
storage for operator inspection. No production deployment, physical power
loss, RF/C6, full-service or field-soak acceptance is claimed. Rerun the
[procedure](../../../../benchmarks/checkpoint_scheduling/README.md) when the
runtime sources, SQLite stack, storage or deployed workload changes.
