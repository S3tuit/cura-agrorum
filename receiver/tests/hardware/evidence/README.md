# Hardware tests worth keeping

Historical evidence for tests that are destructive, require physical actions,
or take more than five minutes. Follow the repository [retention policy](../../../../EVIDENCE.md).

| Topic | Retained result |
|---|---|
| [Persistence](persistence_worker/README.md) | Three destructive storage faults and restoration |
| [Runtime time](runtime_time/README.md) | Long slew measurements, physical oscillator/battery sequence, intrusive controller/clock fixtures |
| [TX airtime](tx_airtime/README.md) | Persistent-state reconstruction across two real Pi reboots |

Each topic has a short report and one result extract. Hashes identify historical
inputs; raw sessions and complete source snapshots are not retained. Routine
nominal/host runs can be repeated. Radio failure lessons live in the
[coverage/procedure record](../../RADIO_COVERAGE.md), alongside their limits.
