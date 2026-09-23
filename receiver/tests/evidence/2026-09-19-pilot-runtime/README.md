# Pilot runtime qualification — 19 September 2026

**DEP-014 verification: PASS within the component/installed-access scope below.**
Current local sources were copied into a new isolated Pi directory and verified
before execution. The installed production package matched all 89 current
runtime files. The receiver stayed stopped/disabled throughout; no C6 was
accessed and no RF episode ran.

| Check | Result |
|---|---|
| Kernel ABI/sample, Chrony brackets, RTC reads/safe failures, process restart identity, offline startup with/without supplied provenance | 6 PASS as `cura-receiver` |
| Effective time-writer audit; explicit unprivileged RTC permission denial | 2 PASS with root fixture supervision |
| RTC refresh/helper privilege and forward/backward clock steps | 3 PASS; root owns fixture changes, component child uses `cura-receiver` UID999/GID985 |
| Current installed helper pin, RTC/Chrony/kernel access and no effective/permitted/inherited/ambient capabilities | PASS as `cura-receiver` |

[Qualification](QUALIFICATION.json) records case names, durations, installed
bindings and final service state. [Mutation captures](TIME_MUTATIONS.json)
retain the three fixtures' public identity, real observations, persisted results
and restoration. All three report successful restoration; Chrony was active,
RTC ownership/mode returned to root:985/0660, and temporary helpers were removed.
No production account gained sudo rights or permanent group membership.

The time fixture previously hardcoded the administrative `cura` account. The
new `--receiver-time-user cura-receiver` option selects the actual service UID
without changing production behavior; `cura` remains the historical default,
and root is rejected as the component identity. Tests ran from the staged
directory, using its fresh Python environment and isolated persistent test root.

[Source identity](SOURCE.json) identifies the Git baseline, exact relevant
files and [test-fixture patch](fixture.patch). Test selection was the existing
`test_runtime_time.py` suite split between service-UID and administrative cases,
plus `test_network_rtc_refresh_and_helper_privilege` and both parameters of
`test_forward_backward_step_component`, using the explicit hardware/destructive
interlocks. Routine raw logs and generated virtual environments are not archived.

Historical [controller/oscillator-stop/slew evidence](../../hardware/evidence/runtime_time/README.md)
is reused only for its unchanged assertions. The native helper, RTC adapter,
process boundary, kernel-clock adapter, elapsed-duration calculations and
reference arithmetic retain their recorded hashes. Git revisions matching the
historical full-file hashes also establish unchanged controller/recovery and
clock-reading implementations. The current writer audit confirms the deployed
Chrony policy; kernel and Chrony versions match. Runtime paths that changed were
exercised by this batch instead of inferred from that older evidence.

The [prior installed-service qualification](../2026-09-18-production-installation/README.md#final-qualification)
retains its sandbox/boot/lifecycle scope: its complete 89-file runtime manifest
still matches the current verified installation. Today's component executions
are not new service-sandbox, physical fault, slew-rate, drift or battery-retention
measurements. Supplied-provenance offline checks are component evidence, not
new physical holdover proof. RF-026/RF-028 and aggregate deployment acceptance
remain separate obligations.

## Current persistence and storage qualification

**DEP-015 verification: PASS within the component scope.** A second fresh source
stage ran 25 Pi hardware cases as `cura-receiver`, three destructive fault cases
with root supervision and capability-free UID999/GID985 children, and 951 focused
contract cases on the Pi as `cura-receiver`. All passed. The contract cases retain
their simulated clocks/radio faults where defined; they exercise real SQLite on
the target, not physical radio behavior. Twenty local supervisor/harness checks
also passed.

[Storage results](STORAGE.json) identify the selected modules, hardware cases,
source stage, public observations and all three successful unmounts. The suite
covers schema/projections, exact ordinary commit reconciliation and poison
boundaries, shared ordinary/control recovery, durable state generations, grant
settlement and composed ACK outcomes. Process-kill cases cover four ordinary
transaction boundaries and seven worker boundaries. The existing 30-second
worker stress case committed 825 accepted entities, rejected 1,018 admissions
during recovery, injected 48 BEGIN failures and validated restart state. It is
not the deployment soak or a physical power-loss test.

[Storage source identity](STORAGE_SOURCE.json) and the
[supervisor patch](storage_fixture.patch) bind the current implementation. Root
owns only the fixture mount/remount/unmount lifecycle; component processes have
zero inherited/permitted/effective/ambient capabilities, empty supplementary
groups and `no_new_privs`. The former child-side sudo requirement is removed;
production account privileges are unchanged. All temporary mounts are gone and
the installed receiver remains inactive. The newer shared conftest adds only
the storage-user option; the earlier time-run source record remains unchanged.
RF-025/RF-027..RF-029, physical power interruption and aggregate acceptance remain
separate. Routine logs, synthetic credentials and damaged database files are not
included in this curated record.

## Local retention workflow

**DEP-021 pilot workflow: PASS within the local component/workflow scope.** As
`cura-receiver`, the [retention check](local_retention_check.py) reopened all three
Pi mixed-workload databases (batch sizes 1/7/32), each containing 24 readings,
48 profiles, 24 clock observations, 24 diagnostics and 24 health rows. The existing
RF service snapshot helper made consistent local SQLite backups. Every table's
complete contents matched after backup and after opening a separate restored
copy through the production database validator. Integrity and foreign keys
passed; original database bytes stayed unchanged. Snapshots remain on the Pi.

[Results](LOCAL_RETENTION.json) bind the driver/helper hashes, all table content
digests, installed state and available capacity: 23,947,653,120 bytes available
against the configured 1,073,741,824-byte low-space threshold. This verifies the
current capacity check and maintenance mechanism; it is not a forecast for an
unspecified fleet, event rate or duration. The architecture retains all rows
within the schema epoch, requires explicit offline maintenance and preserves
rejected originals. This batch's fault, corruption/maintenance and restart tests
verify those existing controls; no automatic pruning or uploader was added.
The earlier current-source offline startup checks and matching installed-service
qualification supply the network-independent startup prerequisite. Physical
node-to-service collection remains RF-020; final supply/placement and soak
acceptance remain separate. The first inspection counted pytest's `current`
symlink twice; the driver now ignores that alias. No original data was modified.

## Installed-service stop verifier

The first T-008 rehearsal exposed an incorrect zero-exit requirement for the
`systemd-inhibit` wrapper; [the discovery](SERVICE_STOP_DISCOVERY.json) retains
the rejected check even though the receiver wrote a clean-stop marker. The
operator-approved shared verifier now checks invocation/instance/boot identity,
matching clean-stop generation, inactive/empty service state, inhibitor release
and only normal-zero or SIGTERM wrapper outcomes. The fresh staged Pi rehearsal
and all 256 RF host checks passed; [verification](SERVICE_STOP_VERIFIED.json)
binds the results and sources. This was receive-only service lifecycle work;
no production-node RF episode is claimed.

## Actual node-image compatibility

The real formatter image exposed a shared incorrect `name_max=64` assumption
in the host reader and synthetic writer. The ESP adapter uses the library
default, 255. [Discovery and image binding](NODE_IMAGE_DISCOVERY.json) retain
the failure; `empty-node-storage.bin.gz` preserves the original dump losslessly.
The approved correction uses the production default in both host configurations.
An independent regression decodes this actual device image, verifies its hashes,
four absent logs and nonmutation. All 257 RF host checks passed. No reformat or
production format change was needed.

## Production startup discoveries and corrections

The first production wake overflowed its default 3584-byte main-task stack.
[Stack discovery](PRODUCTION_STACK_DISCOVERY.json) retains the exact ELF binding,
LittleFS call chain, 56 crashes and preserved pending records. The approved
production default is now 8192 bytes. The next wake completed 33 attempts without
a panic, but all selected Pi ACKs were suppressed; [that separate failure](PRODUCTION_ACK_READINESS_DISCOVERY.json)
retains the source-bound receiver and node outcomes. Compilation or this current
message path alone does not qualify worst-case stack margin/backlog handling.

The receiver never dispatched its initial Chrony poll: a newly sampled deadline
was compared with an earlier clock reading. [The discovery](INITIAL_CHRONY_POLL_DISCOVERY.json)
and [standalone reproducer](reproduce_initial_chrony_poll.py) distinguish advancing
from frozen clocks. The approved correction returns zero for the initial deadline.
All 3,452 receiver host tests passed, including a real scheduler/time-policy
regression that acquires network trust with a clock advancing on every read.
[Installed-service verification](INITIAL_CHRONY_POLL_VERIFIED.json) records the
fresh staged Pi's NETWORK_SYNCED generation-one observation and clean stop with
no received profiles. The one-off driver confused enum 1 (holdover) with enum 2
(network) and timed out; the failed capture remains, and independent inspection
of its SQLite snapshot establishes the actual result without a rerun.
The earlier runtime manifests describe their original qualification snapshots;
`runtime_time.py` subsequently changed by this explicitly qualified correction.

## Deliberate zero-airtime preparation

[Preparation evidence](ZERO_AIRTIME_PREPARATION.json) records the operator's
exclusive-radio confirmation, same-boot quiet interval exceeding the full
conservative rolling window, fresh production-validated zero-charge candidate,
explicit installation with archived prior database, and shared prerequisite check.
The production service reached NETWORK_SYNCED with an 8-second precharge under
the 36-second budget, then stopped cleanly and settled unused charge to zero.
No C6 start or received profiles occurred in this preparation qualification.

All 269 RF host checks passed. The helper cannot overwrite a database and does
not fabricate RTC provenance or clock trust. Production missing-history recovery
is unchanged. The shared prerequisite check is not proof of a live RAM grant;
actual ACK delivery remains RF acceptance. A one-off post-stop snapshot reused
its earlier destination and failed; a separate capture under a new name retained
the stopped database without rerunning the service. Both outcomes are recorded.

## Corrected current and backlog rehearsal

[Current/backlog evidence](CURRENT_BACKLOG_REHEARSAL.json) independently
reconciles authenticated frames, SQLite bodies, successful ACK transmissions and
node delivery records: new sample57/message57 and preserved sample56/message58
each succeeded on its first attempt. Pending and quarantine are empty; no stack
panic occurred. The C6 is stopped in download mode and the Pi's clean stop
settled exactly two ACK charges (135,732 microseconds).

This qualifies the 8192-byte stack for these production paths and the corrected
operational exchange; it is neither worst-case stack analysis nor RF-019's
status matrix nor RF-020's two scheduled wakes. Sensor diagnostics remain:
production BME pins4/5 differ from the carrier's21/22 and both DS18B20 bindings
are unconfigured. Further runs are paused for the carrier configuration and
sensor-selection decisions. T-008 full nominal rehearsal remains open.

## Confirmed nominal sensor configuration

The operator confirmed nominal carrier wiring and physical DS1/DS2 identities.
Production now selects BME SDA21/SCL22, physical DS2 on logical channel0 and DS1
on logical channel1; ROMs remain local configuration inputs. The rebuilt image
was flashed without changing identity, counters or storage. [Nominal rehearsal](NOMINAL_SENSOR_REHEARSAL.json)
records sample58/message59 accepted on attempt1, all sensor-validity bits set,
soil voltages2576/2686mV, no new diagnostic, empty pending/quarantine and clean
stopped endpoints. Previous failures remain retained.

The one-off driver completed capture/cleanup, then raised NameError in its
SQLite reporting step. Independent offline reconciliation of the exact saved
authenticated frames, SQLite body, node logs and stop records passed without
another RF run; its original FAIL is not rewritten. This is one nominal wake,
not RF-020's scheduled wake pair or independent sensor-conversion measurement.

## Operational rehearsal and durability disposition

[Operational rehearsal](OPERATIONAL_REHEARSAL.json) closes DEP-024 within the
isolated bench scope: install/lifecycle, identity replacement and operator cold
power cycle, preserved snapshots/exact restore, nominal collection and stopped
endpoints. The [runbook](../../../../tests/rf/PILOT_RUNBOOK.md) leaves cadence
unspecified, retaining only the operator's informal planned check-ins.

[Durability disposition](DURABILITY_DISPOSITION.json) closes DEP-017's accepted
evidence reconciliation. The retained 43-case bare-C6 ELF/config still match;
relevant firmware changes since that baseline are documentation only. Current
Pi storage fault/process-kill evidence is retained with its limits. The separate
initial Chrony-poll change was qualified afterward. Physical C6/Pi power cuts
remain NOT RUN with their approved deferral, consequences and revisit conditions;
this does not establish RF-031 or physical supply-loss durability.

RF-019 first-case failure is retained in [RF019_WAKE_DISCOVERY.json](RF019_WAKE_DISCOVERY.json). The authenticated next wake arrived after896.734s; the operator subsequently selected900s ±5% as a pragmatic observation bound. Final storage also revealed an extra startup sample/delivery before the peer observation, so rerun remains blocked on startup control. Both endpoints were stopped and temporary peer credentials removed. Neither finding is a passing RF result.

[UART_START_VERIFIED.json](UART_START_VERIFIED.json) records reproduction of the implicit start on the nontransmitting formatter and successful reset-held opening followed by one explicit release. All280 RF host checks pass. Production RF acceptance remains a separate rerun.

`RF019_CURRENT_ACCEPTED.json` records the completed three-wake production-node/controlled-peer accepted case (run `bd0330fc9df14e7fbb1fbaf4f8f97683`), storage reconciliation, interval measurements and confirmed cleanup. This is one of eleven RF-019 cases, not installed-service or bench acceptance. Earlier failed attempts remain preserved.

`RF019_QUALIFICATION.json` indexes all11 passing controlled-peer ACK-policy cases. Ten use the agreed10-second test configuration; the earlier accepted case retains its900-second scope. Per-case compressed captures under `rf019/` retain byte hashes and source/build identity, including LittleFS before/after, UART/peer traces and cleanup. No installed-service acceptance is inferred.

`RF020_QUALIFICATION.json` and `rf020/` retain the passing accelerated installed-service pair, preserved3-reading baseline, two new exact readings/profiles and clean stopped endpoints. `RF031_DISPOSITION.json` closes only the approved deferral record; physical post-SetTx qualification stays NOT RUN.900s cadence and bench acceptance remain later obligations.
