# Protocol ingress benchmark result

- Status: `completed`
- Mode: `load`
- Captured: `2026-09-04T14:38:00.237777+00:00`
- Commit: `ba44c112d1bf4619e9fdedcb3d99078ec6980ba1`
- Source tree: `dirty`
- Source manifest SHA-256: `d12acce5a4b7d80da6414e56e0fbd13225b1936aadb6e6cf9d73aadd5e6bb7a7`

## Environment

- Model: Raspberry Pi 3 Model B Rev 1.2
- OS: Debian GNU/Linux 13 (trixie)
- Kernel/machine: `6.18.39+rpt-rpi-v8` / `aarch64`
- Python: `3.13.5`; cryptography: `50.0.1`
- Logical CPUs / affinity: `4` / `[0, 1, 2, 3]`
- CPU governors: `['ondemand']`
- Start load average: `[0.3154296875, 0.15966796875, 0.072265625]`
- End load average: `[0.61083984375, 0.2236328125, 0.09375]`

## Load recipe

```json
{
  "cpu_block_bytes": 1048576,
  "cpu_operation": "sha256 of fixed public block",
  "cpu_workers": 3,
  "io_block_bytes": 65536,
  "io_file_size_limit": 67108864,
  "io_operation": "write then fdatasync fixed public block, wrapping at file limit",
  "io_workers": 1,
  "worker_results": [
    {
      "active_duration_ns": 4287077861,
      "bytes_processed": 255852544,
      "error": null,
      "iterations": 244,
      "kind": "cpu",
      "parent_terminated": false,
      "process_exitcode": 0,
      "termination_reason": "parent_stop",
      "worker_id": "cpu-0"
    },
    {
      "active_duration_ns": 4264859997,
      "bytes_processed": 273678336,
      "error": null,
      "iterations": 261,
      "kind": "cpu",
      "parent_terminated": false,
      "process_exitcode": 0,
      "termination_reason": "parent_stop",
      "worker_id": "cpu-1"
    },
    {
      "active_duration_ns": 4260005384,
      "bytes_processed": 265289728,
      "error": null,
      "iterations": 253,
      "kind": "cpu",
      "parent_terminated": false,
      "process_exitcode": 0,
      "termination_reason": "parent_stop",
      "worker_id": "cpu-2"
    },
    {
      "active_duration_ns": 4158432789,
      "bytes_written_total": 2730950656,
      "error": null,
      "file_size_limit": 67108864,
      "kind": "io",
      "maximum_file_size": 67108864,
      "parent_terminated": false,
      "process_exitcode": 0,
      "sync_count": 41671,
      "termination_reason": "parent_stop",
      "worker_id": "io-0",
      "wrap_count": 40
    }
  ]
}
```

## Latency

Cold current sample: 1810.252 us

| Domain | Samples | p50 | p95 | p99 | Maximum |
| --- | ---: | ---: | ---: | ---: | ---: |
| combined | 2000 | 1205.515 us | 5103.101 us | 5539.714 us | 9508.757 us |
| current | 1000 | 1204.682 us | 5464.610 us | 5551.745 us | 5628.827 us |
| backlog | 1000 | 1205.880 us | 1421.712 us | 5531.433 us | 9508.757 us |

## Coarse timing margin

Observed remainder: `528795.243 us` = 600000 us retry interval - 61696 us ACK airtime - 9508.757 us maximum ingress.

Excludes scheduling outside ProtocolIngress.begin(), SX1262 profile changes, buffer writes, SetTx, IRQ handling and recovery; it is not an end-to-end guarantee or a test threshold.
