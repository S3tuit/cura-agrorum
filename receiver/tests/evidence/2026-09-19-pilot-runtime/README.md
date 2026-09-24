# Costly pilot qualification — 19 September 2026

The [result record](results.json) contains only the destructive, physical-action
and long-running parts of this historical batch. It is a curation extract with
source/build identities and essential observations; full raw runs are not retained.

| Test | Why retained | Recorded result |
|---|---|---|
| RTC refresh/helper and ±60 s clock steps | Mutates real RTC/system time | 3 PASS; successful restoration in all fixtures |
| Full-space, permissions and read-only storage faults | Destructive mounted fixtures | 3 PASS; all children exited zero and all mounts removed |
| Node identity replacement | Full flash erase and physical removal of all power | Operator confirmed cold cycle before formatter/new image; retired identity replaced |
| Zero-airtime preparation | Same-radio silence longer than one conservative rolling hour | Required 3,613.32 s; observed interval 4,293.509850 s; 8,000,000 µs precharge settled to zero at clean stop |
| RF-019.current.accepted | Three natural wakes, >29 minutes | PASS; current intervals 896.362549 s and 900.244864 s within the approved 855–945 s observation bound |

Pi `00000000e0027211`, kernel `6.18.50+rpt-rpi-v8`, service UID999/GID985;
C6 `cc8da2fc0224`. Sources were staged and verified independently for the time,
storage and RF runs. Shared runtime hashes and additional fixture hashes are
recorded once; they are relevant input identities, not complete source archives.

Time fixtures restored Chrony, RTC ownership/mode and temporary helpers. These
are component tests under the actual service UID, not new full-service sandbox,
drift, slew-rate or physical holdover measurements. The older
[physical time results](../../hardware/evidence/runtime_time/README.md) retain
their own source scope.

## Current persistence and storage qualification

The retained DEP-015 subset is the three destructive storage cases. Root
supervised fixture mounts; children ran as `cura-receiver` with zero effective,
permitted, inherited and ambient capabilities and `no_new_privs`. All temporary
mounts were removed and the receiver remained inactive. Fast contract, process-
kill, backup and 30-second stress checks can be rerun and have no archive here.
This is not deployment soak or physical power-loss evidence.

## Long RF observation

Run `bd0330fc9df14e7fbb1fbaf4f8f97683` used the production node and a controlled
Pi peer, with the historical 900-second sleep configuration. Four messages each
made one attempt: retry-later, accepted current, accepted backlog, retry-later.
Final storage retained sample66 pending, with no diagnostics/quarantine. The
peer shut down safely, the C6 was verified in its loader, and temporary peer
credentials were removed. This is one RF-019 case, not installed-service,
independent sensor-conversion, complete ACK-matrix or 24-hour bench acceptance.

The fast accelerated ACK matrix and RF-020 runs are recorded as historical
outcomes in the [RF catalogue](../../../../tests/rf/test_suite.notes.md#rf-019);
their run archives do not meet the [retention policy](../../../../EVIDENCE.md).
Physical power-cut and Pi post-SetTx uncertainty obligations remain NOT RUN in
the [deployment inventory](../../../../deployment_remaining.notes.md) and
[RF-031](../../../../tests/rf/test_suite.notes.md#rf-031). Rerun affected tests
after source, configuration, fixture or timing changes.
