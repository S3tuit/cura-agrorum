# Persistence — 8 September 2026

On `cura-receiver` (Raspberry Pi, Linux 6.18.39+rpt-rpi-v8/aarch64,
Python 3.13.5, SQLite 3.46.1), isolated files used `/dev/mmcblk0p2`.
The [original manifest](source-manifest.json) identifies 323 staged files:
base `6f0f5ef006fd2d93284323b2f1af250cb3d56810` plus local changes,
tree SHA-256 `1b86a33964437780364eec4408a45b40007885e7a39d20c117ff09c6b0ab81e3`.

| Test | Recorded outcome |
|---|---|
| Pi component suite | 29 passed in 51.47 s, including seven process-kill cases |
| Dedicated 4 MiB storage fixture | 3 passed in 6.14 s: low/full space, permissions and read-only remount; no mounts left |
| 30-second loaded soak, seed 805031 | 830 accepted/committed, 1,175 rejected during recovery, 52 injected BEGIN failures, 2,005 control pairs |
| Control latency | Idle 1,482 us; gated commit 13,938 us; retry backoff 1,333 us; largest soak pair 250,928 us |

[Results](results.json) contain the selected outcomes, fixture restoration,
original timing summary and its capture provenance; [soak samples](soak.json)
are unchanged original bytes. Percentiles use nearest rank. Each control call
had a 5-second deadline plus 0.5-second caller allowance; one control pair
contains two calls. These observations are not hard real-time guarantees.

The initial read-only remount case exposed `ProgrammingError: receiver database
is closed`: access to a closed handle escaped normal failure handling. The
corrected destructive run passed. Keep
`test_closed_handle_before_attempt_uses_storage_recovery` in the
[host regressions](../../../host/test_ordinary_persistence.py). The obsolete failed session and routine host logs have been removed.

The original source archive and crash databases were never copied here; this
record does not claim to reconstruct them. The retained manifest identifies
the tested source. Component tests do not qualify full service integration,
reboot or physical power-loss behavior. Separate storage benchmarks are unchanged.
Rerun the [persistence procedure](../../../../TESTING.md#persistence) when storage,
SQLite/recovery behavior or the deployed filesystem changes.
