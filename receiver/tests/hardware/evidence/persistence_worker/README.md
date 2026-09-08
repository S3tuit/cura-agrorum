# Persistence worker validation

The 2026-09-08 component run used the exact staged local source on
`cura-receiver`, under `/home/cura/cura-worker-20260908-QoJENP/tree`.
All 323 source-file hashes were verified before and after target execution.
The tested tree hash is
`1b86a33964437780364eec4408a45b40007885e7a39d20c117ff09c6b0ab81e3`,
based on HEAD `6f0f5ef006fd2d93284323b2f1af250cb3d56810` plus the uncommitted
implementation. The manifest records each path and SHA-256. Subsequent edits
only document the completed component and its evidence or organize artifact
retention through `.gitignore`; the manifest still describes the tested tree.

The target ran Python 3.13.5, SQLite 3.46.1, pytest 9.1.1 and Hypothesis 6.167.1
on Linux 6.18.39+rpt-rpi-v8/aarch64. Normal test files were below the isolated
staging directory on `/dev/mmcblk0p2`, not a tmpfs or a pilot database.

| Check | Result |
|---|---|
| Host `make test-receiver-host` | 1,630 passed, 16.20 s |
| Pi `make test-receiver-hardware-all` | 29 passed, 51.47 s |
| Pi dedicated destructive target | 3 passed, 6.14 s |
| Receiver generator validate/check and diff whitespace | Passed |
| Dedicated fixture mounts after teardown | None |

The safe target includes 12 worker cases: three control-latency cases, WAL
threshold/checkpoint stall isolation, seven process-kill cases and a slow
30-second mixed-work soak with one CPU/fsync load subprocess. The destructive
target repeats the existing low-space/full, permission and read-only remount
regressions using only the marked, dedicated 4 MiB fixture. One existing queue
`record_property`/JUnit xunit2 warning is recorded in the safe-run log.

The corrected run measured control calls at 1,482 us while idle, 13,938 us
around a gated commit and 1,333 us during retry backoff. These are individual
samples; limits are a 5-second command deadline plus 0.5 seconds for caller
scheduling. They are not hard real-time guarantees.

The seed-805031 soak accepted and committed 830 entities across all five queue
kinds, rejected 1,175 test admissions during recovery, completed 2,005 control
pairs and injected 52 recoverable BEGIN failures. Each control pair comprises
a load and commit with separate 5-second deadlines. Its largest observed pair
latency was 250,928 us. Raw samples and nearest-rank summaries are retained.
The test verified the exact final clean marker, integrity/foreign keys, and
restart with a different instance and an empty volatile queue.

The initial destructive run exposed a closed-handle access outside ordinary
failure handling. A host regression now covers it. The first failure log is
retained alongside the corrected run; the original source archive and rejected
fixture artifacts remain on the Pi under the isolated staging root.

## Artifacts and commands

This README, the [acceptance timing summary](2026-09-08-summary.json) and the
[tested-source manifest](2026-09-08-source-manifest.json) are retained in Git.
Keep curated summaries and manifests in date-prefixed files beside this README;
dated run directories under `receiver/tests/hardware/evidence/` are ignored.
Curated benchmark results under `receiver/benchmarks/` remain separately
versioned.

The ignored local `2026-09-08/` directory retains the host log, initial and
corrected target logs, JUnit results, per-case raw timing/metadata and initial
source manifest. Paths in the timing summary are relative to this raw run
directory. Target source archives, logs, raw results, database/WAL/SHM crash
evidence and bounded storage-fixture artifacts remain on `cura-receiver` under
`/home/cura/cura-worker-20260908-QoJENP/`. These raw artifacts are not included
in a fresh clone; archive them separately before clearing either location.
Test configuration keys are not copied into these reports.

The receiver Make targets were run with `--basetemp` and `--junitxml` below
that staging root. The destructive target additionally used
`CONFIRM_RECEIVER_DESTRUCTIVE=YES` and its sentinel-validated
`RECEIVER_TEST_ROOT`; credentials were supplied only to the test process.

These results establish persistence-component behavior with a bounded test
caller. They do not establish a production communicator/radio/time integration,
systemd signal/restart policy, reboot behavior or physical power-loss durability.
The existing FULL/NORMAL storage benchmark remains separate evidence.
