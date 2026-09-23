# Raspberry Pi ordinary persistence characterization, 2026-09-07

Raspberry Pi 3 Model B Rev 1.2; Python 3.13.5; SQLite 3.46.1; ext4 on
`/dev/mmcblk0p2`. Verified source manifest:
`fd7238479215f7a55f4994772373c5559061a76deb4d715eef9d9b47119d866a`.

Each scenario offered 10,000 mixed entities over ten seconds with a 500-slot
queue. All accepted entities drained and matched database counts and integrity
checks. The full queue rejected some offers in every scenario. Throughput below
includes drain time; commit latency measures SQLite COMMIT alone.

| Mode | Batch limit | Accepted | Queue-full offers | Entities/s | COMMIT p50 / p99 / max (ms) | Checkpoint max (ms) | WAL max (bytes) |
|---|---:|---:|---:|---:|---:|---:|---:|
| FULL | 1 | 1663 | 8337 | 106.9 | 3.294 / 35.292 / 52.357 | 95.656 | 597432 |
| NORMAL | 1 | 2208 | 7792 | 175.6 | 0.196 / 2.352 / 3.932 | 133.534 | 609792 |
| FULL | 16 | 2916 | 7084 | 251.4 | 6.002 / 39.586 / 87.045 | 58.660 | 2076512 |
| NORMAL | 16 | 3060 | 6940 | 266.1 | 0.396 / 3.234 / 3.311 | 162.052 | 2035312 |
| FULL | 64 | 3131 | 6869 | 278.4 | 11.635 / 15.409 / 15.409 | 75.576 | 3267192 |
| NORMAL | 64 | 3242 | 6758 | 287.1 | 0.712 / 3.032 / 3.032 | 234.550 | 3254832 |

All explicit checkpoints completed their reported frames, and every publisher
joined without an exception. Raw arrivals retain queue growth and admission
outcomes; raw transaction records retain actual batch sizes, duration and WAL
size. Every scenario reached the queue capacity of 500.

These are six short sequential stress scenarios, not a statistical capacity
qualification or a pilot traffic forecast. NORMAL reduced measured COMMIT
latency here, while classification, projection, health sampling and checkpoint
costs also contributed to end-to-end throughput. FULL remains required in
production. No power-loss, service, RF or control-scheduler claim follows from
these results. See [methodology](../../README.md).

`metadata.json`, `source-manifest.json`, `summary.json` and each scenario's
`raw.json` / `summary.json` are immutable retained evidence. Database files and
logs remain in the isolated Pi staging directory
`/home/cura/ordinary-persistence-20260907-iGGocRkv/storage-benchmark`.
