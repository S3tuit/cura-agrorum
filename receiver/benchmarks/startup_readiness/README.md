# Pi persistence startup benchmark

The pilot startup budget selected on **2026-10-06 is 12 seconds**:
`ceil(11.109632 s) = 12 s`, leaving 0.890368 s above the largest eligible
measured completion time. Application startup now accepts this explicit budget
through `CURA_RECEIVER_STARTUP_BUDGET_US=12000000`; see the
[deployment procedure](../../deploy/README.md). Installed application qualification
is separate from these worker measurements.

For this pilot, **large database** means the measured synthetic fixture:
21,315,584 main-database bytes (21.316 decimal MB, approximately 20.33 MiB),
100,000 health rows, 100,000 clock observations, 8,000 profiles, and an initially
empty WAL. “21 MB” is its short name, not a guarantee for any database with that
file size. This budget applies to the pilot implementation and fixture envelope.
Future production integrity checks and their budget require a fresh decision.
The benchmark observation cap remains **60 seconds**, independent of the
selected 12-second application budget.

## Findings

Measurements used a Raspberry Pi 3 Model B Rev 1.2, its original microSD/ext4,
Debian 13.6, kernel `6.18.50+rpt-rpi-v8`, Python 3.13.5, SQLite 3.46.1 and
cryptography 43.0.0. Chrony was active; `cura-rtc-bootstrap.service` was absent.
The 60-second extension had both Wi-Fi and Ethernet connected. Keep the initial
campaign and extension separate because the observation cap, harness revision
and environment differed; measured worker/schema and fixture bytes were unchanged.

These are eligible completion times in seconds. The 10-second cohorts ran on
2026-10-05; eligible 60-second reboot/cold cohorts ran on 2026-10-06.

| Fixture / mode | Observation cap | Eligible samples | Median | Maximum |
|---|---:|---:|---:|---:|
| Pilot / warm | 10 s | 30 | 0.870671 | 0.962892 |
| Pilot / orderly reboot | 10 s | 8 | 1.367833 | 1.471062 |
| Pilot / confirmed cold | 10 s | 1 | 1.293443 | 1.293443 |
| Large / warm | 10 s | 5 | 7.888705 | 8.046258 |
| WAL / warm | 10 s | 5 | 0.873771 | 0.875133 |
| Large / orderly reboot | 60 s | 3 | 10.890700 | 10.904618 |
| Large / confirmed cold | 60 s | 5 | 10.965976 | **11.109632** |

The five large cold completions were 10.911303, 11.062963, 10.869173,
11.109632 and 10.965976 seconds. Every eligible attempt completed worker cleanup.
These cold cycles followed orderly shutdown and an operator-confirmed 30 seconds
without power. This is a power reset, not temperature conditioning.

| Large cold stage | Median seconds | Maximum seconds |
|---|---:|---:|
| Configuration load | 0.007051 | 0.016679 |
| Database open / validation | 7.647163 | 7.817020 |
| Instance commit | 0.021493 | 0.035771 |
| Persistence component setup | 3.254297 | 3.265062 |
| Communicator state load | 0.002259 | 0.002314 |

Database validation and component setup dominate. In the measured code, full
`integrity_check` and `foreign_key_check` run on the initial read-only connection,
again on the read/write connection, and again during `OrdinaryPersistence`
construction. Stage measurements do not isolate individual SQL checks. Changing
that validation design is deferred beyond this pilot budget decision.

The initial large reboot reached its 10-second cap without terminal publication
(`PERSISTENCE_STARTUP_INCOMPLETE`, observed at 10.010311 s). Its completion time
is unknown; it remains censored, not a 10.010311-second startup. Three successful
requested-reboot attempts are excluded from orderly-reboot/cold statistics
because their execution included unplanned power cycles: `reboot-01`,
`reboot-06`, and `extended-60s-large-reboot-01`. Five setup smokes are also
excluded. This leaves **57 eligible successes among 61 campaign attempts**,
with one censored result and three mode exclusions.

The original pilot cold target remained 1/5; the original ten-attempt large
reboot expansion stopped at its first observation. The later three reboots and
five cold starts are separate completed cohorts. There was no WAL boot cohort.
The pilot decision accepts this evidence scope; it does not relabel unfinished
targets as completed. These samples establish neither a worst case nor p99,
full production boot-chain timing, RF behavior or abrupt-power-loss durability.

## Retained measurements

[summary.json](results/2026-10-06/summary.json) records the budget calculation,
target, counts and all stage statistics. [measurements.csv](results/2026-10-06/measurements.csv)
retains all 61 campaign attempts, eligibility, timings in microseconds and hashes
of the original request/result/physical-confirmation files. Hardware/network
investigation records remain outside this directory.

The exact [10-second inventory](results/2026-10-06/inventory-10s.json) and
[60-second inventory](results/2026-10-06/inventory-60s.json) bind the fixture bytes
and 94 source files per revision. The baseline was
`4c7ac444bc0f10bcf90742c40b19b8f58061d5e5` **plus uncommitted changes**; that commit
alone cannot reproduce the measured source. These are historical manifests,
not assertions that today's checkout matches. The dated summary records the
decision state before application integration. New runs prepare their own
inventories, including the README's current hash.

The external raw archive is
`cura-startup-benchmark-evidence-1791299560330530411.tar.gz`, SHA-256
`e748e01542bd7aef0bea76a08ddade8ff7fbdb2e6d546691e88d43c41e154319`.
It contains original requests/results, inventories and confirmations. For
example, CSV `result_sha256` addresses `results/<attempt_id>.json` in that
archive. Raw archives, staged source archives and databases stay outside Git.

Recompute the decision from the retained CSV, from the repository root:

```sh
python3 - <<'PY'
import csv
from pathlib import Path

path = Path('receiver/benchmarks/startup_readiness/results/2026-10-06/measurements.csv')
with path.open() as stream:
    rows = list(csv.DictReader(stream))
maximum_us = max(int(row['completion_elapsed_us']) for row in rows
                 if row['eligible'] == 'True')
print(f'Maximum: {maximum_us / 1_000_000:.6f} s')
print(f'Pilot budget: {(maximum_us + 999_999) // 1_000_000} s')
PY
```

## Measurement and fixtures

[run.py](run.py) exercises the real `PersistenceWorker`, SQLite validation,
lifecycle insertion, state load, five stage probes and shared stop/completion
waiter. It constructs no radio and performs no TX, RTC writes or clean-stop
marking. It does not launch the full application.

Timing starts immediately before worker launch and ends at immutable terminal
publication. Imports, fixture preparation and JSON writing are outside it.
Stage durations are successive stage-entry differences, with the final stage
ending at publication. Total time includes the short launch-to-first-stage gap.

The runner observes for at most 60 seconds, returning early on completion or
SIGTERM/SIGINT. Publication must precede the deadline; progress never extends
it. A separate ten-second cleanup follows. The service uses
`TimeoutStartSec=90s`, `TimeoutStopSec=15s`, and `Restart=no`. A nonblocking
`post_cleanup_startup` snapshot retains late publication without changing the
original outcome. Late success is excluded from that request's timing sample.

Offline preparation at `--scale 1` creates:

| Fixture | Health / clock / profile rows | Measured DB bytes | Initial WAL bytes |
|---|---|---:|---:|
| `pilot` | 10,000 / 10,000 / 800 | 2,297,856 | 0 |
| `large` | 100,000 / 100,000 / 8,000 | 21,315,584 | 0 |
| `wal` | Same as `pilot` | 233,472 | 2,105,352 |

All contain valid synthetic communicator state. These are controlled histories,
not production data; never point a production receiver at them. New schemas or
SQLite versions can change fixture bytes/sizes: retain the new inventory.

Arming copies and verifies a private fixture **before** reboot or poweroff;
templates remain intact. SQLite reconstructs SHM at startup. Warm samples are
cache-warm process starts. Reboot/cold samples run automatically once in a new
boot. Do not hash, copy, open or run `verify` against the fixture on that new
boot before service execution: that would warm the storage being measured.

## Installation on an isolated Pi

Use an SD-backed test filesystem, Python 3.12+ with SQLite `setconfig` and
`SQLITE_DBCONFIG_NO_CKPT_ON_CLOSE`, and an interpreter with `cryptography`.
Record exact interpreter/dependency versions. Keep other receiver instances
stopped. Persistent journald setup is in the
[deployment procedure](../../deploy/README.md#persistent-service-journal).

Stage a fixed root-owned copy of the intended checkout at
`/opt/cura-startup-benchmark/source`, preserving repository-relative paths.
Include `receiver/cura_receiver`, `receiver/db`,
`protocol/protocol-v2-lora/python`, this directory, `receiver/deploy/systemd`,
`receiver/tests/__init__.py` and `receiver/tests/support` (fixture builders).
Do not copy only `run.py`. Retain the staged source archive and SHA-256 externally.

These commands assume a **fresh installation**. Preserve an existing campaign
before reusing the fixed paths; never overwrite its source, inventory or results.
Create system account `cura-receiver` with its own group and no login shell if
absent. Replace `BASELINE_COMMIT` below with the staged checkout's commit.

```sh
sudo /usr/bin/python3 /opt/cura-startup-benchmark/source/receiver/benchmarks/startup_readiness/run.py \
  --root /var/lib/cura-startup-benchmark prepare --source-commit BASELINE_COMMIT
sudo chown -R cura-receiver:cura-receiver /var/lib/cura-startup-benchmark
sudo chmod 0600 /var/lib/cura-startup-benchmark/receiver-group.json
sudo install -o root -g root -m 0755 \
  /opt/cura-startup-benchmark/source/receiver/benchmarks/startup_readiness/control.py \
  /usr/local/sbin/cura-startup-benchmark
sudo install -o root -g root -m 0644 \
  /opt/cura-startup-benchmark/source/receiver/benchmarks/startup_readiness/cura-startup-benchmark.service \
  /etc/systemd/system/cura-startup-benchmark.service
sudo systemctl daemon-reload
sudo systemd-analyze verify /etc/systemd/system/cura-startup-benchmark.service
sudo systemctl enable cura-startup-benchmark.service
sudo /usr/local/sbin/cura-startup-benchmark verify
sudo /usr/local/sbin/cura-startup-benchmark status
```

The source manifest binds uncommitted changes. Do not reduce `--scale` on the Pi.
Preparation requires a new canonical absolute root. Check the effective unit
with `systemctl show` and SD storage with `findmnt -T /var/lib/cura-startup-benchmark`.
The source tree must be readable by the service account. Record RTC-bootstrap,
Chrony availability and boot load; do not hide missing-unit diagnostics or
claim equivalent boot environments.

[control.py](control.py) fixes paths, service account and export recipient
(`cura`, `/home/cura`); adapt them explicitly for another target. Normal sudo
access suffices. Optional passwordless delegation uses a root-owned sudoers rule,
validated with `visudo`, allowing only
`cura ALL=(root) NOPASSWD: /usr/local/sbin/cura-startup-benchmark *`.
Use ordinary SSH access; no campaign-specific key or password is needed.
Run a separate `--mode smoke` attempt for each fixture using the warm sequence
below before collecting measurements. Smokes are excluded from cohort statistics.

## Rerun and inspect

Choose sample counts before starting and use new labels. To compare against the
completed large-fixture evidence, collect five warm, three orderly-reboot and
five confirmed cold samples. Historical pilot/WAL counts are above; this
procedure does not silently restart unfinished cohorts. Keep revisions, caps
and environmental cohorts separate. Retain the **60-second observation cap**.

For a warm sample (use `--fixture pilot` or `wal` for those fixtures):

```sh
sudo /usr/local/sbin/cura-startup-benchmark arm --fixture large --mode warm --label new-large-warm-01
sudo /usr/local/sbin/cura-startup-benchmark start
```

For an orderly reboot, enable the unit, arm on the old boot, then reboot:

```sh
sudo systemctl enable cura-startup-benchmark.service
sudo /usr/local/sbin/cura-startup-benchmark arm --fixture large --mode reboot --label new-large-reboot-01
sudo /usr/local/sbin/cura-startup-benchmark reboot
```

Reconnect after automatic execution; do not manually restart the service.
For a cold sample, also ensure the unit is enabled, then:

```sh
sudo /usr/local/sbin/cura-startup-benchmark arm --fixture large --mode cold --label new-large-cold-01
sudo /usr/local/sbin/cura-startup-benchmark poweroff
```

The operator waits for orderly shutdown, disconnects power, waits **30 seconds
with power disconnected and LEDs off**, then reconnects it. After the automatic
run, record the actual operator confirmation with the attempt ID printed by
`arm`; never infer physical actions from SSH loss or a changed boot ID:

```sh
sudo /usr/local/sbin/cura-startup-benchmark attest-cold --attempt ATTEMPT_ID \
  --note 'Operator confirmed orderly shutdown, 30 seconds disconnected with LEDs off, then power restored.'
```

After every attempt:

```sh
sudo /usr/local/sbin/cura-startup-benchmark service
sudo /usr/local/sbin/cura-startup-benchmark status
sudo /usr/local/sbin/cura-startup-benchmark journal
sudo /usr/local/sbin/cura-startup-benchmark verify
sudo /usr/local/sbin/cura-startup-benchmark collect
```

Require `SUCCESS`, `cleanup.worker_stopped=true`, one request/result/boot claim,
matching inventory, and a changed boot for reboot/cold attempts. Warm/smoke
attempts must run in their arming boot: the enabled unit consumes any armed
request at boot, so a warm request left armed across an unexpected restart runs
with a cold cache. `analyze` retains it but reports `boot_eligible=false` and
excludes it from timing statistics. Record power
flags (`vcgencmd get_throttled`) and system/service state before and after each
boot. Copy the printed export to the workstation and verify its SHA-256 before
arming the next boot attempt. Retain current/preceding boot journal readback
separately from JSON persistence: after a subsequent boot inspect
`journalctl -b -1 -u cura-startup-benchmark.service`.

Stop and preserve the attempt on timeout, failure, missing result, incomplete
cleanup, source mismatch or unexpected active receiver. Do not silently retry,
raise the cap, or classify a recovered boot as the requested mode. The CLI
`analyze` groups by requested fixture/mode/cap/inventory and checks cold
attestation, but **does not apply execution-context exclusions**. Review actual
execution before using its group counts; the retained CSV makes exclusions
explicit. Observation/cleanup time never substitutes for missing publication.

At the end, collect and verify a final export, then run
`sudo /usr/local/sbin/cura-startup-benchmark disable` and check `status` has
`armed: null`. Disabling does not cancel an armed request. Keep raw evidence and
staged source outside Git; retain only a reviewed summary and its provenance here.

## Host checks

The reusable code is `run.py`, `control.py` and the systemd unit; installation,
physical actions and summary calculation need no additional program. Focused
regressions are [test_startup_benchmark.py](../../tests/host/test_startup_benchmark.py)
and [test_persistence_startup_evidence.py](../../tests/host/test_persistence_startup_evidence.py).
Run `make test-receiver` for the host suite. Host success does not reproduce Pi
timing, power cycles or journal retention.
