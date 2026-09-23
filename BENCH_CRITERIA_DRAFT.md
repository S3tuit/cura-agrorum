# Superseded discussion draft

The operator clarified the quick power cycle as hour23. The agreed plan is [BENCH_PLAN.md](BENCH_PLAN.md); the questions below are historical preparation, not additional gates.

# Bench criteria discussion — T-013 / DEP-019

Status: operator supplied the practical outage plan; the quick power-cycle timing is awaiting clarification. No soak is started by this document.

## Already agreed

- At least 24 hours on the bench; the later field week is separate.
- Use nominal production sleep, configured sensors, isolated/current identities and source-bound captures. RF-019/RF-020's received-current timing tolerance is 900 seconds ±5%; it is not a characterized clock-accuracy guarantee.
- Include normal current readings and controlled backlog/recovery, offline operation and bounded recovery phases. Exact interventions and durations remain to be agreed.
- Keep data locally on the Pi; no uploader or mandatory off-Pi export.
- Reconcile generated samples, distinct transport identities, attempts, receiver observations, node ACK outcomes and durable readings without treating them as interchangeable counts.
- Report observed link performance; no numeric delivery/RSSI target has been selected.
- Require no unexplained identity/state loss, unbounded wait, unsafe recovery, airtime overrun or unresolved failure. Restore controlled faults.
- Verify continuity of the actual selected power supply; do not reuse an inapplicable power-bank auto-off criterion.
- Operator check-ins around15/30/60 minutes after startup and at the end are informal, not mandatory thresholds.
- Physical power-cut proof remains explicitly deferred; an ordinary process stop/reset does not qualify it.

## Operator plan received

Use an approximately24-hour run. Around three hours before the planned end, the operator removes Pi power, leaves it off for about one hour, then restores power (approximately hours21–22). The node continues normally. The operator also requests one fast Pi power-off/on; whether “one hour before the bench” means one hour before its end is awaiting clarification. Record actual intervention times rather than requiring precise scheduled execution.

There is no target loss percentage or observation-gap quota: this is a practical system trial. The desired outcome is no generated reading absent from both the Pi and the node. Reconcile each evidenced sample against durable Pi readings and retained node records; distinguish retries/transport IDs from samples. Report unexplained or unobservable gaps rather than asserting they were retained or never generated. The operator decides when to end the bench; no new rigid monitoring cadence or timed response requirement is imposed.

Pi physical cuts are part of this proposed bench observation. They do not automatically qualify a controlled post-SetTx power-cut test or C6 physical power-loss behavior. Preserve that evidence distinction.

## Inputs still needed before acceptance criteria or dependent tooling

| Topic | Decision needed |
| --- | --- |
| Schedule | Which controlled missed-ACK, receiver-offline, time-disruption and clean/crash-recovery phases are included, and their bounded durations/order? |
| Accounting interval | Where does the 24-hour acceptance interval start/end, and how are planned outage periods identified in observations? |
| Observation gaps | Which gaps or lost volatile records are permitted, what evidence explains them, and what makes the run inconclusive? |
| Completion | How long may pending backlog drain after restoration, and what remaining pending state is allowed at the final capture? |
| Supply | Which actual supplies/connections remain in use and how will continuity/interventions be recorded? |
| Stop/recovery | Who stops the run, how soon after discovering a fault, and which observations require stopping rather than continued recording? |

Do not infer exact generated/attempted totals from a 900-second nominal cadence or from received rows alone. Do not infer a missed sample's UTC time where no valid clock relation exists. Any acceptance target must be agreed before collecting the acceptance dataset.

## Existing building blocks to reuse after agreement

- `tests/rf/service_probe.py`: consistent SQLite snapshot and service observations.
- `tests/rf/production_node.py` / `node_capture.py`: read-only full LittleFS capture and production-format decoding, preserving identity-lifetime counters.
- `tests/rf/verify_service.py` / `verify_ack.py`: existing bounded-case reconciliation examples; do not silently generalize their two/three-wake assumptions into soak policy.
- `receiver/cura_receiver/logical_timestamps.py` and `clock_correlation.py`: existing time interpretation; absent UTC remains explicit.

T-014 will implement only the checks required by the agreed plan. Final field reporting remains deferred until its dataset exists.

Sources: `deployment_remaining.notes.md` DEP-019/DEP-030 and DEC-005; `tests/rf/test_suite.notes.md` RF-030; owned workplan T-013/T-014, D-056/D-060/D-065. Existing governing documents take precedence over this discussion draft.
