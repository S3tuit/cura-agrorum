# 24-hour bench plan — DEP-019

Agreed on2026-09-19. This is the acceptance plan for the later bench, not a record that it has run. The selected prerequisite RF tests and tool readiness remain separate.

Run the nominal production node and receiver for at least24 hours. Keep the node powered throughout. Use the actual bench supplies and record what they are; this is not a battery-autonomy test. Record the actual start/end and interventions with approximate wall time and any known clock uncertainty.

| Approximate elapsed time | Operator action |
| --- | --- |
| 0–21 hours | Normal production operation. |
| 21 hours | Remove Pi power; leave the node running. |
| 22 hours | Restore Pi power and observe recovery/backlog delivery. |
| 23 hours | Briefly remove and restore Pi power again. |
| 24 hours or later | Operator ends the bench and preserves final evidence. |

Timing is practical, not a stopwatch requirement. The operator can stop earlier if needed; report actual duration and reason instead of calling a shorter run a completed24-hour bench. Planned check-ins around15/30/60 minutes and at the end remain informal. No extra network outage, artificial clock step or process-crash phase is required by this concrete schedule; separately qualified focused tests retain their original scope.

The desired result is no generated reading missing from both the Pi and the node. Count samples separately from message IDs and retransmissions. A sample already durable on the Pi need not remain on the node. A sample still pending or quarantined on the node is retained, but its delivery is not complete; report that distinction. There is no required reception percentage, RSSI target, permitted-loss quota or fixed extra backlog-drain deadline. Record unresolved gaps and operator interventions rather than concealing them in a percentage.

At the start and end retain consistent receiver SQLite snapshots and read-only full node-storage captures, with source/configuration and device/identity bindings. Start capture precedes the acceptance interval; stop the node before the final capture. Record receiver restarts/boot identities and clean-stop markers without expecting clean-stop markers for the planned power cuts. Preserve actual receiver airtime history across all starts; never restore a zero-use database after transmissions.

Reconcile all samples evidenced by starting retained records, node deliveries, authenticated receiver profiles, durable readings and final retained records. Report samples absent from both final stores, conflicting bodies, missing delivery outcomes, sequence gaps and incomplete captures. A gap in volatile receiver profiling during a power cut is not by itself proof of sample loss. Neither a nominal900-second schedule nor a counter gap proves exactly how many samples were generated. Expose uncertainty; do not invent missing UTC or infer unobserved attempts.

The operator decides whether the observed behavior is acceptable after reviewing these findings. The report must not call unexplained loss, incomplete evidence, unsafe recovery or unaccounted airtime a clean result. There is no automated bench PASS based merely on elapsed time. Preserve evidence and stop/report if identity/state loss or unsafe behavior is found. Device shutdown or explicitly accounted continuation is required at the end.

These Pi cuts are opportunistic recovery observations. They do not qualify the separately deferred controlled post-SetTx physical-cut test, C6 power-loss behavior, or general physical durability. The field week and final field report remain later work.

Governance: owned workplan D-067/D-068; deployment inventory DEP-019/DEP-030; RF-030. This operator-selected schedule supersedes the generic fault-phase list for this bench only.
