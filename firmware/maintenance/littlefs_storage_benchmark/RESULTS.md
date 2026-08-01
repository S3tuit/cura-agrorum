# Results

All 12 combinations of storage layout, workload, and record size completed
successfully. The results favor a single fixed-record append log for pending
readings.

## 50-byte records

| Workload | Layout | Mean wake | p99 wake | Reads | Writes | Erases | Hottest sector |
|---|---|---:|---:|---:|---:|---:|---:|
| Online 50/50 | Single log | 13.7 ms | 32.2 ms | 62.9 MB | 0.77 MB | 190 | 94 |
| Online 50/50 | One file/reading | 15.6 ms | 49.1 ms | 61.4 MB | 1.15 MB | 546 | 272 |
| Growing 60/40 | Single log | 36.4 ms | 80.3 ms | 83.0 MB | 6.99 MB | 3,170 | 80 |
| Growing 60/40 | One file/reading | 156.4 ms | 790.8 ms | 1,311 MB | 1.86 MB | 570 | 8 |

## 100-byte records

| Workload | Layout | Mean wake | p99 wake | Reads | Writes | Erases | Hottest sector |
|---|---|---:|---:|---:|---:|---:|---:|
| Online 50/50 | Single log | 14.9 ms | 50.3 ms | 63.0 MB | 0.77 MB | 190 | 94 |
| Online 50/50 | One file/reading | 15.1 ms | 50.3 ms | 61.6 MB | 1.17 MB | 288 | 143 |
| Growing 60/40 | Single log | 40.7 ms | 80.3 ms | 83.5 MB | 7.18 MB | 3,219 | 80 |
| Growing 60/40 | One file/reading | 210.4 ms | 800.3 ms | 1,704 MB | 1.89 MB | 591 | 5 |

## 150-byte records

| Workload | Layout | Mean wake | p99 wake | Reads | Writes | Erases | Hottest sector |
|---|---|---:|---:|---:|---:|---:|---:|
| Online 50/50 | Single log | 15.2 ms | 50.3 ms | 66.0 MB | 1.15 MB | 288 | 143 |
| Online 50/50 | One file/reading | 16.9 ms | 50.3 ms | 62.2 MB | 1.54 MB | 378 | 188 |
| Growing 60/40 | Single log | 41.8 ms | 80.3 ms | 81.9 MB | 7.43 MB | 3,275 | 79 |
| Growing 60/40 | One file/reading | 271.7 ms | 870.3 ms | 2,151 MB | 2.49 MB | 765 | 7 |

For 50-byte records with a growing backlog, the single log was 4.3 times faster
on average and almost 10 times faster at p99. It caused more writes and erases,
but LittleFS spread those erases across nearly the entire partition. The
per-file layout reduced erases but caused 15.8 times more reads. Its relative
read and latency costs increased further with larger records.

At a backlog of 1,000 records, reading the newest 50-byte record took 4.55 ms
from the log. Direct access using a known filename took 119 ms with the
per-file layout, while scanning its directory took 782 ms. Including a cold
mount increased those values to 7.61 ms, 203 ms, and 866 ms respectively.
A known filename therefore helps but does not remove LittleFS directory lookup
cost.

The complete campaign erased the hottest sector 279 times: 0.279% of an
assumed 100,000-cycle endurance. Flash wear is not a concern for the pilot,
although the log's greater write amplification should remain visible in future
testing.

## Decision

Use one fixed-size `pending.log`, append new readings, and drain accepted
readings newest-first by truncating the tail. A follow-up benchmark should
model an outage followed by draining several backlog records per wake.
