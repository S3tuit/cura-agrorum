# Joint RF verification

This directory is for field-pilot-v2 scenarios involving the C6 node and Pi
receiver. The [RF catalogue](test_suite.notes.md) defines stable scenarios and
their component or full-system scope; the [deployment inventory](../../deployment_remaining.notes.md)
maps coverage, dependencies and proposed pilot gates. Both .notes files are
ignored local planning documents.

Laptop pytest coordinates pytest-embedded through the C6's actual UART
connector and SSH control of a separate Pi process. The C6 Unity app and
ESP-IDF configuration belong in firmware/test_apps/radio/; the Pi component
peer belongs in receiver/test_apps/radio_peer/ and reuses production radio
components. The component apps and guarded runner now implement
RF-001/003/006/008/009/010/012/013. Manual-fixture results are retained in [evidence/](evidence/README.md);
the catalogue owns remaining obligations. Nominal cases are rerun when needed.
A push or host-suite pass does not establish that physical RF tests ran.

Receiver production code and hardware operations run on Pi. Ordinary Pi-local
tests remain in receiver/tests/hardware/; protocol verification remains in
protocol/protocol-v2-lora/tests/. A component peer does not establish full
receiver acceptance.

The [retained destructive firmware results](../../firmware/tests/evidence/README.md)
and [installed Pi pre-radio qualification](../../receiver/tests/evidence/2026-09-18-production-installation/README.md#final-qualification)
record historical execution within their stated limits. Fast firmware checks
and builds have no permanent archive; rerun them when their results are needed.
These records do not close RF-019/RF-020, RF-027/RF-028 or later runtime time/storage
gates. Destructive-test identity handover must precede authenticated TX.

Future runs require explicit cases, fixture readiness, isolated identities and
storage, bounded local scheduling and airtime accounting at both transmitters.
SSH coordinates readiness; independent endpoint clocks are not synchronized.
Keep helpers local until a second real use justifies sharing.

For the first test phase, the operator owns per-transmitter airtime accounting,
admission and pacing using retained manual records. The harness declares
bounded episodes, waits for operator readiness and records attempts; automated
airtime ledgers and cross-run scheduling are not prerequisites. An additional
receiver is a cross-check, not a fresh-allowance test based on silence.
DEC-002 accepts the current firmware wake-budget/sleep heuristic for this pilot
and defers DEP-007 plus RF-029's firmware-ledger assertions. The receiver's
durable airtime policy remains unchanged.

## Run a selected component case

The approved production qualification assembly is the nominal C6 carrier
without JP_REF_ENABLE/R5/R6/R7/R8 and the nominal Pi carrier without
JP_RTC_SCL_FAULT/JP_RTC_SDA_FAULT. These removed fault/reference branches remain
disconnected; all other nominal schematic requirements apply. See
[firmware sequencing and sensor scope](../../firmware/TESTING.md#pilot-production-fixture-and-configuration-sequencing)
and [installed Pi qualification](../../receiver/TESTING.md#pilot-production-fixture-and-installed-qualification).
Final sensor selection and placement are not new pilot gates; technical
configuration and selected nominal assertions remain required. Independent
same-acquisition sensor-to-packet checks belong to the existing sensor-carrier
integration tests; applicable passing evidence is a separate RF-020 prerequisite.
RF-020 checks authenticated frames, received sensor flags/ranges, ACK handling,
SQLite persistence and the ordinary next wake, without adding production
acquisition instrumentation. Removed test circuitry cannot supply new physical-fault proof.

The production build/provisioning sequence permits reviewed sensor, board/layout
and disposable identity inputs before the final image. Built-image, installed
service and RF verification follow their respective prerequisites. This is not
production identity handover or authority to erase existing live identity state.

Read [EPISODES.md](EPISODES.md) for the simple packet/charge table, local timing
and bounded cleanup. The operator handles admission and pacing. Keep the manual
sheet across cases, resets and reruns.

1. Build in the configured ESP-IDF environment with `make test-rf-build`, then
   run `make test-rf-host`. The seal hashes actual compiler dependencies, ELF,
   configuration and flash files. Rebuild/reseal when those inputs change.
2. Copy [fixture.example.json](fixture.example.json) to a run input outside the
   source tree. Confirm each actual device/fixture field; the example's false
   fields intentionally cannot authorize hardware. Use nominal C6, nominal Pi
   and open RTC shunts. Select fault wiring only with power removed; DIO1 open
   needs RN5, and complete module absence needs both specified BUSY/MISO ties.
3. Keep a simple [operator record](operator-record.example.txt) for both physical
   transmitters. Supply its path, the exact cases/parameters, a new run ID and a
   new temporary capture directory and a session file (see below). The runner prompts before each episode. Supplying
   `--ready-run` with the exact run ID explicitly admits the entire selected
   batch; use it only when its complete schedule is already operator-approved.
4. Supply SSH authentication at runtime through your agent or
   `CURA_PI_PASSWORD`. SSH/scp use `-F /dev/null`. The runner stages and verifies
   the current selected source tree in a unique `/var/tmp/cura-rf-<run>` directory.
   It never executes an old Pi checkout. Pi `python3` needs the production radio
   dependencies in `receiver/requirements-radio.txt`; select an isolated environment
   with `--peer-python /absolute/path/to/venv/bin/python`. The peer checks dependency
   versions and required APIs before opening radio devices. When using an IP
   address, `--host-key-alias cura-receiver` retains the existing SSH host identity.

Example first exchange (replace paths/run ID with the actual operator inputs):

```sh
make test-rf-component-nominal RF_ARGS='--fixture /tmp/rf-fixture.json --cases RF-001.exchange --run 0123456789abcdef0123456789abcdef --output /tmp/rf-run-01 --session /tmp/rf-session.json --manual-record /tmp/rf-airtime.txt --confirm-flash'
```

`--confirm-flash` acknowledges replacement of the factory app. The runner checks
the actual flash ranges and binds the factory MAC with a no-reset probe before
requesting the DUT; the identity probe leaves the MCU in its bootloader and cannot
start an old autonomous image. The runner rejects erase-all, NVS erase,
forced/encrypted/alternate-port flashing and stale builds.
See the [app's storage disclosure](../../firmware/test_apps/radio/README.md).

The remaining nominal parameters are `RF-003.silence`, `RF-006.invalid`,
`RF-008.silence`, `RF-008.exchange`, `RF-009.untouched`,
`RF-009.initialized`, `RF-010.wake`. Comma-separated selection is explicit;
there is no automatic retry or implicit “all.” RF-001 must precede dependent
cases, and RF-003 must precede RF-008. They may run earlier in the same
selection or in an earlier successful nominal run in the current bench session.

Pass `--session /absolute/path/to/session.json` on every invocation. Choose a
new file outside the source tree for each bench session, and reuse that path
while changing fixtures. The runner writes a small disposable receipt only for
successful nominal prerequisites. It binds both boards, stable physical
transmitter labels, executable sources and the sealed build. Markdown edits
are excluded; changed executable inputs require fresh nominal tests. There is
no historical archive import or per-file applicability review.

The session expires 12 hours after its first run or at laptop reboot. A failed
or interrupted run clears its prerequisites. A fault run consumes them even
when it passes: restore nominal wiring and pass fresh RF-001 before another
fault. This receipt is local state for one sequential owner of the two devices;
start a new session whenever devices/configuration change or the bench is left
unattended. It is not historical qualification or an airtime ledger.

For RF-012 or RF-013: first pass nominal RF-001, power down and prepare the
selected fault fixture, then use the matching
`test-rf-component-dio1_disconnected` / `test-rf-component-radio_absent` target
with the same session path and a new output directory/run ID. Confirm the fault
fixture explicitly. Afterward restore nominal wiring unpowered and run fresh
RF-001. Do not change the physical transmitter labels between fixture states.
Delete the session file and disposable captures when the bench session ends.

## Production node ACK policy (RF-019)

`make test-rf-node-ack` selects exactly one authenticated case. It uses the
production node and an isolated Pi controlled peer, not the receiver service.
The exact unchanged ACK receive deadline remains a separate firmware host
prerequisite. The runner rebuilds/runs the firmware host suite before access;
RF PASS covers the physical frames and reconciled node records only.

Build with `make test-rf-production-build` in the configured ESP-IDF environment.
This seals actual firmware dependencies, public node ID, binary partition table,
resolved carrier pins and LittleFS settings. Keep the disposable group/key
inputs outside evidence/source archives. Prepare the node separately under the
[identity reset/format procedure](../../firmware/maintenance/erase_storage/README.md),
retaining formatter success and flashing production without overwriting storage.
The runner never flashes, formats, erases or generates credentials. It verifies
the actual installed binaries and an empty, readable LittleFS baseline before
starting production. On the reviewed UART reset circuit, opening the capture
port holds reset (DTR released, RTS asserted); only the explicit start after
peer readiness releases the node. Opening the port must not start an earlier
unobserved wake. Each next case needs explicitly empty logs again; preserve
NVS counters when using the formatter within the same identity lifetime.

Cases are `RF-019.current.accepted`, `.retry_later`, `.unsupported`, `.malformed`,
`.invalid_auth`, `.wrong_message`, `.domain_status`, and
`RF-019.backlog.accepted`, `.retry_later`, `.unsupported`, `.malformed`.
Use the complete prefix for each choice. Setup obtains real pending readings
through RETRY_LATER wakes; there are no fabricated backlog files. Current cases
use one setup wake, the target wake, then a metrics-observation wake. Backlog
cases use two setup wakes, the target wake and a metrics-observation wake.
RF functional runs use the production code configured for10-second deep sleep
and the agreed UART sleep-entry marker. Setup/target/metrics wake counts remain
unchanged; received-current intervals allow9.5..45.5s including unchanged awake
work. Reserve50s per wake plus10s. Production900-second cadence is bench/pilot
coverage; the completed historical long RF case is not a routine prerequisite.
The final current receives RETRY_LATER to preserve remaining records.

The episode declaration reports the full lease and per-transmitter maxima.
Reserve up to 70 attempted C6 transmissions per wake (210/280 for the whole
case, including failed reception). Pi ceilings are case-specific: 3..5 replies
for current cases and 5..6 for backlog cases. The operator owns preceding history,
rolling admission and unexpected-reset/uncertain-interval accounting. A lost
control path does not bound an autonomous node; the operator must retain the
all-power stop capability. No failed scenario is automatically retried.

Single ACKs use Radio at Pi-local RX_DONE + 150 ms. The invalid-auth case uses
the disclosed Sx1262/LinuxRadioIo sequence: corrupted tag at +150 ms and valid
ACK at +350 ms, with the existing 100 ms late-target abort. This is an RF reply
schedule, not proof of the node's exact internal deadline. Other selected invalid
ACKs are sent once, followed by silence for that message's remaining retries.

Example (all paths, identity isolation and the entire declared allowance must
be operator-confirmed; the nominal fixture file follows the component schema):

```sh
make test-rf-node-ack RF_ARGS='--case RF-019.current.accepted --fixture /tmp/nominal.json --run RUN_ID --output /tmp/new-ack-run --manual-record /tmp/airtime.txt --formatter-result /tmp/formatter.txt --build /tmp/accelerated-build --local-group /private/receiver-group.json --remote-group /private/receiver-group.json --peer-python /path/to/pi/venv/bin/python --confirm-isolated'
```

`RUN_ID` must be a new 32-character lowercase hexadecimal ID. SSH staging uses
the current source in a unique directory, with the same authentication and host
options as the component runner. The peer checks its actual UID, board identity,
dependencies and group. Local and remote group/public node IDs must match.
The runner verifies UART MAC and installed images with no restart, reads the
baseline storage, arms the peer, then explicitly starts the node. UART capture
continues across scheduled wakes; loss of UART or peer control fails the run.
After final observation it stops the node into download mode, captures storage
and independently checks ACK SPI bytes/counts, message/sample identities,
delivery outcomes, pending/quarantine transitions and next-wake metrics.
Stop/capture failures retain restoration-required evidence. No automatic
production restart follows. These paths are host-tested; physical RF-019 remains
NOT RUN until an operator-admitted case succeeds.

## Production receiver service (RF-020)

`make test-rf-service` runs the ordinary two-wake exchange against an already
installed isolated production service. Preparation/installing packages and
credentials remains separate. Supply the selected production build, the exact
local copy of the installed disposable group (only the selected node active),
the nominal fixture, and a prerequisite record identifying applicable deployment
and sensor-carrier evidence. Independent same-acquisition conversion is that
sensor prerequisite, not an RF-020 assertion.

The runner stages the current tree, compares the installed package with the
current runtime bundle, checks the exact isolated production unit/environment,
Pi identity, dependencies and actual service UID, and requires the service to
start from stopped state with no prior readings in its isolated database.
RF-020 preserves the existing receiver database and airtime history. Before
start, capture only the selected node's existing message IDs and complete-row
hashes in `reading-baseline.json`, under a read-only transaction as the service
UID. Reuse the required final consistent SQLite capture to prove those rows are
unchanged and exactly two new readings belong to this run's receiver instance.
No second full before-database copy or airtime reset is needed.

Node binaries and an empty readable storage baseline are verified without
flashing. Operator admission precedes service/node start. A new service-instance
health row reporting RX_SINGLE, a fresh same-instance NETWORK_SYNCED observation,
and production-validated airtime history with conservative budget headroom are
required by the shared `InstalledService.wait_for_prerequisites()` check. It waits
at most 45 seconds by default, preserves reason-specific observations in
`service-prerequisites.json`, and fails before starting the node. Rehearsals use
the same method. These are test prerequisites, not proof of a RAM-only grant or
a promise of future ACK delivery. Actual ACKs remain acceptance evidence.
`systemctl active` alone is insufficient. All radio/ACK timing stays on the Pi.

Fresh test databases may be explicitly prepared with zero historical airtime;
production missing/corrupt-history recovery remains conservative. Preparation is
separate from the RF runner and never automatically retried:

1. Stop the isolated service and all other Pi radio transmitters; keep the C6
   stopped. Preserve the old database and its evidence. The operator must attest
   that **all** Pi transmitters stay silent, including component peers.
2. Record the Pi board ID, current Linux boot ID, and the Pi's monotonic timestamp
   at the established silence boundary. Wait the complete conservative physical
   rolling window (3,613.32 monotonic seconds with the current policy). A reboot
   invalidates this receipt; elapsed wall time or an empty database is insufficient.
3. Stage the current tree and invoke the staged `service_probe.py` as the service
   UID with the usual verified `service-config.json`, action `prepare-zero`,
   `--silence-record /path/to/receipt.json` and
   `--output /var/lib/cura-pilot-INSTANCE/data/prepared-RUN_ID.sqlite3`.
   `RUN_ID` is 32 lowercase hexadecimal characters. The receipt has exactly:

   ```json
   {
     "schema": 1,
     "board_id": "ACTUAL_BOARD_ID",
     "boot_id": "ACTUAL_HYPHENATED_LINUX_BOOT_ID",
     "silent_since_monotonic_us": 123456789,
     "all_pi_transmitters_remain_silent": true,
     "operator_record": "Actual operator confirmation and scope"
   }
   ```

   The helper checks the stopped service, board/boot binding and elapsed interval.
   The operator attestation covers other transmitters; the helper cannot infer
   their absence. It creates only a new candidate using the production schema,
   zero-bucket constructor, encoder, validator and classifier. State generation
   is one, historical charge zero, RTC provenance absent and recorded time quality
   UNTRUSTED; neither current clock trust nor a spendable grant is fabricated.
   Retain stdout as the preparation receipt. Existing destinations are rejected.
4. While still stopped and exclusively controlled, explicitly install the
   candidate through the existing offline database replacement procedure, retaining
   the old database/WAL/SHM. The helper never installs its candidate. If preparation
   fails, retain any candidate for inspection and do not install it. Never reuse
   a prepared zero-state image after transmissions; preserve actual history or
   establish a new complete silence interval before another preparation.

Reserve the complete episode: two accelerated wakes, a110-second limit after
node start and conservative maxima of140 C6 uplinks and140 Pi ACKs. Use the
10-second configuration with sleep observations enabled. On the second complete
sleep marker and two observed samples, stop the node in download mode and stop
the service; reconcile consistent database/node captures afterward.
The operator retains the all-power stop capability for lost control; service or
node continuation is never inferred safe from an SSH disconnect. There is no
automatic failed-case rerun, restart, enable, erase or credential replacement.

```sh
make test-rf-service RF_ARGS='--host 10.86.160.140 --host-key-alias cura-receiver --unit cura-pilot-INSTANCE.service --package /opt/cura-pilot-INSTANCE --test-root /var/lib/cura-pilot-INSTANCE --fixture /tmp/nominal.json --run RUN_ID --output /tmp/new-service-run --manual-record /tmp/airtime.txt --prerequisites /tmp/prerequisites.txt --build /tmp/accelerated-build --local-group /private/receiver-group.json --confirm-isolated'
```

Replace the temporary Pi IP, instance/path placeholders and 32-character
hexadecimal run ID with the reviewed inputs. SSH runs as the fixture's
administrative user; database observations/backup run as `cura-receiver`.
Administrative commands use sudo; `CURA_PI_SUDO_PASSWORD`, or the existing
`CURA_PI_PASSWORD`, supplies authentication through stdin without capture.
The production service user and sandbox are unchanged.

Capture includes UART, the service invocation journal, source/configuration
hashes, a consistent SQLite backup, and the full read-only node image. Independent
reconciliation checks authenticated 54/23-byte frames, canonical body/projections,
nominal sensor flags and established ranges, classification/timestamp completeness,
node delivery outcomes, counters, next-wake metrics and the clean service marker.
Missing evidence, changed sources/configuration, service restart, extra wakes,
or failed cleanup cannot pass. Host tests do not establish physical RF-020 PASS.

## Capture and retention

Production-node cases capture the full LittleFS storage image only after the
complete observation sequence, as specified in
[firmware testing](../../firmware/TESTING.md#offline-production-node-evidence-capture).
Decode an already captured image without device access:

```sh
.venv/bin/python tests/rf/node_capture.py /path/to/storage.bin /path/to/new-node-records.json
```

The reader builds the checked-out firmware LittleFS library in `LFS_READONLY`
mode and reuses production record validation. A host C compiler and the fetched
firmware managed component are required. It rejects other image sizes, invalid
files/records and orphan backlog bindings; it never repairs the image. JSON
`null` means a missing file, `[]` means a present empty file. It decodes only
current identity/outcome fields and retains opaque reading/frame/context bytes.
The report identifies the image and decoder sources; run/DUT/build binding is
the responsibility of the capture procedure. An offline decode is not RF PASS.

`--output` is temporary working space outside the repository or under ignored
`tests/rf/raw/`. It contains diagnostic UART/Pi traces, run/JUnit results,
operator readiness, build/source identities and the source transfer snapshot.
The runner does not generate source diffs, archive checksum inventories or
historical-prerequisite review bundles. Check a completed component capture with:

```sh
.venv/bin/python tests/rf/verify.py /path/to/run
```

The same independent assertions run during execution: packet bytes, outcomes,
endpoint-local timestamps, direction-profile commands, cleanup and handle
release. A failed Python/peer/cleanup result cannot be replaced by a passing
Unity subcase. First failure stops the batch. Missing prerequisites, zero
selection, unexpected events/reset and missing completion cannot pass.

Promote only manual-setup, destructive or long tests into [evidence/](evidence/README.md),
with a short result report and essential supporting observations. Keep nominal
runs temporarily for diagnosis, then discard them. Retain a failure only for a
useful problem/cause/fix lesson, not a routine wiring or dependency mistake.
Keep one recoverable source identity per tested tree; retain a source snapshot
only when Git cannot recover it. Clear the remaining temporary captures after
curation. Keep operator airtime history independently for admission/pacing.

The component app enters real short deep sleep after its bounded command and
wakes into a non-transmitting wait. RF-010's second wake requires a fresh host
command. After an interrupted run, issue no new trigger and confirm endpoint
termination/safety or remove all power/back-power paths; retain possible charges.
An ordinary production image has different autonomous-wake requirements.

RF-019 and full-service acceptance remain pending their production dependencies.
No complete-service, durable-airtime-enforcement, electrical timing, output-power
measurement or production 900-second-sleep result follows from this component suite.

RF-002/004/005/007/011/014..018/031 retain their catalogue requirements,
dispositions, consequences and revisit conditions; RF-004 requires negative-IQ
assertions at both endpoints. RF-019 awaits production-node provisioning and
controlled authenticated outcomes. RF-020..030 depend on the real service and
catalogue gates; RF-029 retains its firmware-ledger and Pi physical-uncertainty
deferrals. RF-030 bench and field remain separate. RF-010 uses a short component
sleep; production900-second cadence belongs to the bench/pilot. The
[operating-envelope decision](OPERATING_ENVELOPE.md) does not turn deferred
physical RF-018 measurements into PASS.

Installed-service stop verification binds the observed systemd invocation and
receiver instance before stopping. It requires inactive/dead state, no remaining
service cgroup processes or receiver sleep inhibitor, and that exact instance's
durable clean-stop marker with its matching communicator-state generation.
The verified production unit runs through `systemd-inhibit`: its normal zero
exit or SIGTERM termination are accepted only together with these checks.
Wrapper status alone and clean markers from earlier instances never qualify.
The same verifier owns rehearsal and RF-020 cleanup; failed or missing evidence
retains the original failure and prevents further episodes.

## Accelerated RF execution

RF functional tests use an isolated build/configuration with
`CONFIG_NODE_DEEP_SLEEP_SECONDS=10` and `CONFIG_NODE_RF_SLEEP_OBSERVATION=y`.
Seal it with `production_node.py --seal-build` and pass its directory to the
runner. The default production build remains900s with observation disabled.
No900-second RF prerequisite is required; cadence is validated in the bench/pilot.

Each completed cycle emits `RF_NODE_SLEEP duration_us=10000000` after finalization
and successful timer setup. By operator agreement tests count this as sleep entry.
The observer requires one marker per boot and real deep-sleep resets. RF-019
sends `SLEEP <run> <case> <wake-count>` after the final marker; its peer requires
the final expected current before accepting the notification. RF-020 directly
checks the final marker alongside service progress. This replaces the35-second
wait. Frames, attempts, stored outcomes, bindings and cleanup remain required.
The bound is50s per wake plus10s, with9.5..45.5s received-current intervals for
unchanged awake/retry work. Missing/extra evidence fails; no automatic retry.

Per-case packet ceilings remain unchanged; shorter tests do not grant additional
airtime. Retain operator admission and preceding activity. Bench/pilot endurance,
900-second cadence and real rolling-window observation remain separate long tasks.
