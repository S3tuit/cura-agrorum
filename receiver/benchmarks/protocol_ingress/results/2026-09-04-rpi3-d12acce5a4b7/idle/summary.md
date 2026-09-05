# Protocol ingress benchmark result

- Status: `completed`
- Mode: `idle`
- Captured: `2026-09-04T14:37:57.391480+00:00`
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
- End load average: `[0.3154296875, 0.15966796875, 0.072265625]`

## Load recipe

```json
{
  "cpu_workers": 0,
  "io_workers": 0,
  "operation": "no deliberate load",
  "worker_results": []
}
```

## Latency

Cold current sample: 1111.453 us

| Domain | Samples | p50 | p95 | p99 | Maximum |
| --- | ---: | ---: | ---: | ---: | ---: |
| combined | 2000 | 531.768 us | 590.258 us | 603.643 us | 677.028 us |
| current | 1000 | 531.872 us | 589.789 us | 603.643 us | 666.194 us |
| backlog | 1000 | 531.716 us | 590.466 us | 603.226 us | 677.028 us |

## Coarse timing margin

Observed remainder: `537626.972 us` = 600000 us retry interval - 61696 us ACK airtime - 677.028 us maximum ingress.

Excludes scheduling outside ProtocolIngress.begin(), SX1262 profile changes, buffer writes, SetTx, IRQ handling and recovery; it is not an end-to-end guarantee or a test threshold.
