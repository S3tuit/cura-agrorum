# Ordinary persistence storage benchmark

This explicitly invoked, non-gating experiment compares WAL `FULL` and `NORMAL`
on the deployed Pi filesystem using the production caller-driven persistence
component and queue. It never runs through a pytest or ordinary Make target.

```sh
.venv/bin/python receiver/benchmarks/ordinary_persistence/run.py \
  --source-manifest ordinary-source-manifest.json \
  --output /absolute/new/experiment-directory
```

The source manifest maps repository-relative files to their SHA-256 values.
The runner verifies every entry before measurement and retains the manifest,
Python/SQLite/kernel/device/filesystem metadata, all raw samples and summaries.
Use an isolated staged checkout and a new output directory on the intended
storage medium. No pilot database, service, mount or configuration is touched.

The default synthetic stress workload offers 1,000 entities per second for ten
seconds per scenario. It contains 80% accepted reading/profile units, pairing
current and backlog transport messages for identical samples, and 5% each
rejected profiles, clock observations, health requests and diagnostics. Health
and diagnostic frequency is deliberately elevated to exercise their storage
cost; this is not a forecast of pilot traffic. Inputs are constructed before
timing from the reviewed test builders; encryption/ingress is excluded.

A single publisher follows monotonic arrival deadlines. The caller drives
transactions with maximum batches of 1, 16 and 64 entities and an experiment-local
100 ms flush limit. Queue capacity remains 500; unsuccessful full-queue offers
are counted and excluded from accepted throughput. Accepted work must drain and
match SQL counts and integrity checks. The drain deadline is offered duration
plus 60 seconds, with a bounded producer join. This loop is a benchmark harness;
it does not implement the receiver persistence thread or control scheduler.

An explicit PASSIVE checkpoint runs every 32 completed batches when the WAL
file occupies at least 256 KiB, plus once after draining. This uses allocated
WAL size, not an estimate of live uncheckpointed frames. Raw checkpoint results
record actual progress and duration; successful PASSIVE calls may retain WAL.
The cadence bounds maintenance frequency after the file reaches its high-water
size. Neither the cadence nor the flush limit is a new receiver runtime policy.

Each scenario initializes a separate database and constructs the component under
its required FULL validation. The NORMAL experiment then changes only that
private connection before its first ordinary transaction. Production exposes
no NORMAL setting and retains FULL enforcement. Any storage error aborts the
experiment; NORMAL recovery and power-loss durability are not being tested.

Raw records include every commit latency, transaction duration, arrival/queue
sample, WAL-size observation and checkpoint result. Summaries use nearest-rank
p50/p95/p99/max values, accepted entity and transaction throughput, full-queue
counts, actual batch size, queue high-water and WAL growth. These are measured
under an offered load and include drain time; they are not universal saturation
capacities. The experiment does not establish RF behavior, control fairness,
worker scheduling, service lifecycle or physical power-loss behavior. FULL
remains the pilot setting regardless of the measured difference.
