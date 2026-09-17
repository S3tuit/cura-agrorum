# TX airtime — 16 September 2026

This is the latest per-bucket retention qualification, superseding the
15 September implementation runs. Pi 3 Model B Rev 1.2, kernel
6.18.50+rpt-rpi-v8, Python 3.13.5, SQLite 3.46.1; root bench access and
an explicitly approved 10,000-ppb network-skew fixture ceiling.

| Test | Recorded outcome |
|---|---|
| Pi component suite | 17 passed in 18.28 s; existing slow soak deselected |
| Grant lifetime | 1,707 samples; last allowed 632 us before deadline, first denial completed 564 us after; 1,952,962 us remaining after acknowledgement |
| Separate-process restart | Same boot, new instance; loaded 1,067,866-us charge required a new durable increment |
| Real reboot, trusted network time | Unchanged 8,000,000-us charge reconstructed with zero initial allowance |
| Real reboot, unavailable component time | Durable bytes unchanged; reconstruction and TX suppressed |

All four reboot prepare/verify phases passed. Boot and process IDs changed;
SQLite integrity/state digests passed; Chrony was active, synchronized and its
configuration unchanged after restoration. [Results](results.json) retain the
selected original observations and the scope of the pre-deletion database audit;
[lifetime samples](grant-lifetime.json) remain byte-identical.
The [original manifest](source-manifest.json) binds 327 staged files. All match
Git commit `4779361ff04180d34f846ea8942c9387092cc8cc`; the redundant source tarball
was removed. The recorded base commit predates the then-uncommitted redesign.

Lessons: a global retention deadline deferred an ACK for 3,793 s in the steady-time
host scenario. Per-bucket deadlines reduced maximum deferral to 14 s and the
maximum gap to 74 s (480/480 requests). Keep the
[availability regression](../../../host/test_tx_airtime_availability.py).
Earlier separate UTC/monotonic reads could add 30 s under descheduling: use one
paired observation. Fixture failures taught us to use trusted root-owned paths,
reviewed existing-history state and a declared, achievable clock-skew ceiling.

The first reboot controller attempt exceeded its 180-second SSH reconnect limit;
unavailable-mode verification was NOT RUN and restoration unverified then.
Connectivity recovery allowed a separate restoration check and a fresh passing
run. Its repeated SSH errors and failed-run dumps add no further lesson.

From the repository root, run `python3 receiver/tests/hardware/evidence/tx_airtime/verify_evidence.py`.
The verifier checks retained observations and timing; deleted databases cannot be
re-audited. Rerun the [airtime procedures](../../../../TESTING.md#receiver-tx-airtime-policy)
for changes to retention/reconstruction, clocks or durable state handling.
These are component results, not RF, receiver-user permissions or physical
power-loss qualification. The host availability scenario is not RF timing evidence.
