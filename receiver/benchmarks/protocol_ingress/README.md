# Protocol ingress benchmark

This benchmark characterizes the radio-independent critical CPU path on the
target Raspberry Pi. It alternates reviewed current and backlog readings and
times only:

```text
ProtocolIngress.begin()
  = fixed-header parsing
  + node-key lookup
  + AES-CCM authentication
  + exact reading decoding and validation
  + deterministic ACCEPTED ACK construction
  + real PersistQueue reservation
```

Packet construction, terminal profile publication and queue draining happen
outside each timing sample. The runner uses the production `LinuxOsClock`,
protocol codec/crypto, ingress component and `PersistQueue`; it performs no
radio, SQLite, configuration or network I/O. The current input comes from the
checked-in protocol golden vector. The backlog input is constructed once before
measurement from the same reviewed header/body values and checked against its
reviewed literal bytes.

The first call is retained separately as a cold sample. The runner then performs
the configured warm-up count and records every subsequent `perf_counter_ns()`
sample in collection order; even entries are current readings and odd entries
are backlog readings. Python garbage collection remains enabled because this
characterizes the long-running production runtime rather than an isolated
cipher primitive.

## Modes

`idle` creates no deliberate load. The operator should run it on an otherwise
idle staged Pi; start/end load averages and target metadata are retained so the
condition is reviewable.

`load` starts bounded standard-library child processes before the cold sample:

- up to `CPU count - 1`, capped at three, repeatedly hash a fixed public 1 MiB
  block until measurement completes; and
- one writer repeatedly writes and `fdatasync()`s a fixed public 64 KiB block
  in a benchmark-private temporary directory, wrapping within a 64 MiB maximum
  file footprint until measurement completes.

Every worker has a monotonic runtime deadline, explicit readiness, cooperative
stop, bounded join and parent-owned termination fallback. The raw result records
worker completion reason and active duration, bytes processed, hash iterations,
sync count, file wraps and errors. A successful load run requires every worker
to report parent-requested termination after measurement; reaching the deadline,
crashing or requiring termination fails the run. The temporary load file is
removed before the result is finalized.

## Invocation and results

From a staged repository tree with receiver dependencies installed:

```sh
make benchmark-receiver-protocol-ingress \
  RECEIVER_BENCHMARK_OUTPUT=/absolute/new/idle-result

make benchmark-receiver-protocol-ingress-load \
  RECEIVER_BENCHMARK_OUTPUT=/absolute/new/load-result
```

`RECEIVER_BENCHMARK_WARMUPS` and `RECEIVER_BENCHMARK_SAMPLES` may override the
reviewed defaults of 200 and 2,000; the sample count must be even and at least
two so both domains have equal populations. The output directory must not
already exist.
Each successful directory contains `raw.json` and `summary.md`; a failed run
retains the available source/environment evidence and traceback in the same
files.

An isolated staged tree without Git metadata can set
`RECEIVER_BENCHMARK_SOURCE_COMMIT` and
`RECEIVER_BENCHMARK_SOURCE_TREE_STATE=clean|dirty`. The per-file and combined
source hashes are always calculated from the tree that actually runs.

The summary reports cold latency, p50, p95, p99 and maximum for the combined
and per-domain samples. Its coarse margin is:

```text
600,000 us minimum node retry interval
  - 61,696 us ACK airtime
  - observed maximum ingress latency
```

That remainder is not an end-to-end guarantee. It excludes Linux scheduling
outside the timed call, SX1262 profile changes, buffer writes, `SetTx`, IRQ
handling and any recovery. No benchmark percentile or margin is a test threshold.
