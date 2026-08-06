# LittleFS storage benchmark

Destructive ESP32-C6 firmware for comparing two pending-reading layouts:

- `single_log`: append a record to `pending.log`; remove the newest record by
  truncating the tail.
- `one_file_per_reading`: create one sample-ID-named file; remove it with
  `unlink`.

Each run uses 3,000 simulated wakes, mounting and unmounting LittleFS on every
wake. It tests 50, 100, and 150-byte records under two deterministic workloads:

- `online_50_50`: every inserted reading is immediately removed, producing an
  equal number of inserts and deletes and no backlog.
- `growing_60_40`: all 3,000 wakes insert and a shuffled 2,000 also delete the
  current reading, producing a 60/40 operation mix and a final backlog of 1,000.

Both layouts benchmark recovery by scanning and normal access using a known
head, with cold mounts and backlog sizes of 0, 100, and 1,000 where applicable.
The known head is held in RAM to model valid RTC state without adding an NVS
write to the measured operation.

The firmware records operation latency and successful raw partition reads,
writes, and erases. UART output lines prefixed with `BENCH_` contain timing
percentiles, phase/run totals, and the complete per-sector erase histogram in
32-sector chunks. A timing row with `backlog=-1` aggregates the whole workload;
peek rows contain the exact backlog size. `life_used_pct_at_100k_cycles` is the
hottest sector's erase count expressed as a percentage of an assumed
100,000-cycle endurance; it is not a measurement of the flash's pre-existing
wear.

This application reuses [`../../partitions.csv`](../../partitions.csv), pins
the relevant LittleFS parameters, and disables file modification timestamps.
It repeatedly formats and destroys only the `storage` partition. A five-second
warning is printed before the first format. Resetting the board repeats the
entire destructive campaign.

Build, flash, and save the complete serial output from this directory:

```bash
source ~/esp/esp-idf/export.sh
idf.py build
idf.py -p PORT flash monitor
```

The campaign is intentionally long-running. Leave the board powered until
`BENCH_CAMPAIGN_END,status=ok` is printed; any failed run stops the campaign.
