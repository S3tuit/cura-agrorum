# Joint RF verification

This directory verifies radio interactions between the ESP32-C6 node and
Raspberry Pi receiver. It covers component-radio behavior, production-node ACK
handling and installed receiver-service integration, independently of any
particular deployment.

[spec.py](spec.py) is the catalogue of test names, purposes, fixture states and
whole-test C6/Pi airtime reservations. Use its descriptive names directly in
commands. Preparation-only rejection vectors are marked separately; see
[rejection packet preparation](#rejection-packet-preparation) for their build inputs.

Laptop pytest controls the C6 through its UART and a separate Pi process through
SSH. The C6 component app lives in `firmware/test_apps/radio/`; the Pi peer in
`receiver/test_apps/radio_peer/` reuses production radio components.

Receiver production code and hardware operations run on Pi. Ordinary Pi-local
tests remain in receiver/tests/hardware/; protocol verification remains in
protocol/protocol-v2-lora/tests/. A component peer does not establish full
receiver acceptance.

Destructive-test identity handover must precede authenticated TX.

Future runs require explicit cases, fixture readiness, isolated identities and
storage, bounded local scheduling and airtime accounting at both transmitters.
SSH coordinates readiness; independent endpoint clocks are not synchronized.
Keep helpers local until a second real use justifies sharing.

For controlled RF tests, the operator owns per-transmitter airtime accounting,
admission and pacing using retained manual records. The harness declares
bounded episodes, waits for operator readiness and records attempts; automated
airtime ledgers and cross-run scheduling are not prerequisites. An additional
receiver is a cross-check, not a fresh-allowance test based on silence.
These controlled tests do not qualify the deferred rolling airtime firmware-ledger
assertions. The receiver's durable airtime policy remains unchanged.

## Host checks

Prepare the repository [Python test environment](../../receiver/README.md#python-setup),
then run `make test-rf-host` from the repository root. These tests exercise the
runners, verifiers and capture analysis without device access. Offline
LittleFS decoding also needs a host C compiler and the fetched firmware managed
component described under [captures](#captures).

Source staging excludes Markdown under `tests/rf/`, including local planning
notes. Executable inputs, fixtures and the selected contracts elsewhere in the
repository remain in the source manifest.

## Run a selected component case

The approved production qualification assembly is the nominal C6 carrier
without JP_REF_ENABLE/R5/R6/R7/R8 and the nominal Pi carrier without
JP_RTC_SCL_FAULT/JP_RTC_SDA_FAULT. These removed fault/reference branches remain
disconnected; all other nominal schematic requirements apply. See
[firmware sequencing and sensor scope](../../firmware/TESTING.md#pilot-production-fixture-and-configuration-sequencing)
and [installed Pi qualification](../../receiver/TESTING.md#pilot-production-fixture-and-installed-qualification).
Use the declared sensor configuration and selected nominal assertions. Independent
same-acquisition sensor-to-packet checks belong to the existing sensor-carrier
integration tests; contributors rerun them when relevant to the selected hardware/software changes.
service.reading_delivery checks authenticated frames, received sensor flags/ranges, ACK handling,
SQLite persistence and the ordinary next wake, without adding production
acquisition instrumentation. Removed test circuitry cannot supply new physical-fault proof.

The production build/provisioning sequence permits reviewed sensor, board/layout
and disposable identity inputs before the final image. Built-image, installed
service and RF verification follow their respective prerequisites. This is not
production identity handover or authority to erase existing live identity state.

Read [spec.py](spec.py) for each case's packet limits and per-transmitter charge.
The operator handles admission and pacing across cases, resets and reruns.

1. Build in the configured ESP-IDF environment with `make test-rf-build`, then
   run `make test-rf-host`. The seal hashes actual compiler dependencies and
   configured CMake regeneration inputs (including component definitions and
   included scripts), ELF, configuration and flash files. Changed or missing
   inputs fail before device access. Rebuild/reseal when those inputs change;

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
make test-rf-component-nominal RF_ARGS='--fixture /tmp/rf-fixture.json --cases component.ack_exchange --run 0123456789abcdef0123456789abcdef --output /tmp/rf-run-01 --session /tmp/rf-session.json --manual-record /tmp/rf-airtime.txt --confirm-flash'
```

`--confirm-flash` acknowledges replacement of the factory app. The runner checks
the actual flash ranges and binds the factory MAC with a no-reset probe before
requesting the DUT; the identity probe leaves the MCU in its bootloader and cannot
start an old autonomous image. The runner rejects erase-all, NVS erase,
forced/encrypted/alternate-port flashing and stale builds.
See the [app's storage disclosure](../../firmware/test_apps/radio/README.md).

The remaining nominal parameters are `component.ack_timeout`, `component.invalid_downlinks`,
`component.repeat_timeout`, `component.repeat_exchange`, `component.cold_sleep`,
`component.initialized_sleep`, `component.sleep_wake`, `component.header_error_rearm`. Comma-separated selection is explicit;
there is no automatic retry or implicit “all.” ACK exchange must precede dependent
cases, and ACK timeout must precede repeated exchanges. They may run earlier in the same
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
when it passes: restore nominal wiring and pass fresh ACK exchange before another
fault. This receipt is local state for one sequential owner of the two devices;
start a new session whenever devices/configuration change or the bench is left
unattended. It is not historical qualification or an airtime ledger.

For disconnected DIO1 or absent radio: first pass nominal ACK exchange, power down and prepare the
selected fault fixture, then use the matching
`test-rf-component-dio1_disconnected` / `test-rf-component-radio_absent` target
with the same session path and a new output directory/run ID. Confirm the fault
fixture explicitly. Afterward restore nominal wiring unpowered and run fresh
ACK exchange. Do not change the physical transmitter labels between fixture states.
Delete the session file and disposable captures when the bench session ends.

### Expected HeaderErr regression

`component.header_error_rearm` requires nominal wiring at both ends; attached
sensors are not sampled. The existing component image sends a complete known
54-byte packet, interrupts a second transmission at 18 ms from C6 SetTx HAL
entry, and sends the next complete known packet. Starts are at least 12.2 seconds
apart. The whole episode reserves three full C6 charges (338766 us) and no Pi TX;
the fresh ACK-exchange prerequisite has its own reservation. The existing
pytest flow flashes the component image before execution.

The Pi uses production Radio/Sx1262/LinuxRadioIo. Independent verification
requires actual SPI `0x52 / 0x0020 / 0` evidence, explicit `HANDLED_NO_PACKET`,
standby -> exact IRQ clearing -> fresh complete RX profile -> confirmed SetRx,
unchanged counters, no recovery and the next exact complete packet. A missing
HeaderErr fails even when both complete packets arrive. Raw test capture does
not add production telemetry. Host communicator tests establish suppression of
packet occurrences, profiles and diagnostics; this component episode does not
qualify the installed receiver service or identify the historical error cause.

## Component timing and cleanup

For the initial `component.ack_exchange` session, allow at most ten attempts per
transmitter with at least 60 seconds between its starts, as specified by the
[firmware envelope](../../firmware/TESTING.md#sx1262_radio-first-rf-operating-envelope).
Admit every other selected case as a whole, using its declared reservation.

The C6 uses a two-second TX deadline and a three-second RX window. The invalid
downlink burst keeps the original RX deadline. Pi replies target RX_DONE +250 ms;
the invalid burst also uses +750 and +1250 ms. A target missed by more than
100 ms aborts rather than sending late. Repeated uplinks wait for RX completion
and at least 500 ms after the preceding TX_DONE.

The component lease is 45 seconds plus bounded cleanup. Timer deep sleep lasts
two seconds; the next boot waits for a fresh command before transmitting. UART
commands accept LF or CRLF, at most 159 bytes before LF, and two seconds from
first byte to LF (inclusive); idle waiting has no deadline. A malformed or late
command stays rejected until reset.

On lost control, issue no new trigger. Confirm endpoint shutdown and restoration
or remove all power/back-power paths. Account for possible transmissions even
when no packet was received; silence does not refund airtime.

## Production node ACK policy (node)

`make test-rf-node-ack` selects exactly one authenticated case. It uses the
production node and an isolated Pi controlled peer, not the receiver service.
The exact unchanged ACK receive deadline remains a separate firmware host
prerequisite. The runner rebuilds/runs the firmware host suite before access;
RF PASS covers the physical frames and reconciled node records only.

Build with `make test-rf-production-build` in the configured ESP-IDF environment.
This seals actual firmware dependencies, public node ID, binary partition table,
resolved carrier pins and LittleFS settings. Keep the disposable group/key
inputs outside the source tree and captures. Prepare the node separately under the
[identity reset/format procedure](../../firmware/maintenance/erase_storage/README.md),
retaining formatter success and flashing production without overwriting storage.
The runner never flashes, formats, erases or generates credentials. It verifies
the actual installed binaries and an empty, readable LittleFS baseline before
starting production. On the reviewed UART reset circuit, opening the capture
port holds reset (DTR released, RTS asserted); only the explicit start after
peer readiness releases the node. Opening the port must not start an earlier
unobserved wake. Each next case needs explicitly empty logs again; preserve
NVS counters when using the formatter within the same identity lifetime.

Cases are `node.current.accepted`, `.retry_later`, `.unsupported`, `.malformed`,
`.invalid_auth`, `.wrong_message`, `.domain_status`, `.header_error_rearm`,
`.header_error_retry`, and
`node.backlog.accepted`, `.retry_later`, `.unsupported`, `.malformed`.
Use the complete prefix for each choice. Setup obtains real pending readings
through RETRY_LATER wakes; there are no fabricated backlog files. Current cases
use one setup wake, the target wake, then a metrics-observation wake. Backlog
cases use two setup wakes, the target wake and a metrics-observation wake.
RF functional runs use the production code configured for 10-second deep sleep
and the agreed UART sleep-entry marker. Setup/target/metrics wake counts remain
unchanged; received-current intervals allow 9.5..45.5s including unchanged awake
work. Reserve 50s per wake plus 10s. Production 900-second cadence requires
separate validation; a historical long RF case is not a routine prerequisite.
The final current receives RETRY_LATER to preserve remaining records.

The episode declaration reports the full lease and per-transmitter maxima.
Reserve up to 70 attempted C6 transmissions per wake (210/280 for the whole
case, including failed reception). Pi ceilings are case-specific: 3..6 replies
for current cases and 5..6 for backlog cases. The operator owns preceding history,
rolling admission and unexpected-reset/uncertain-interval accounting. A lost
control path does not bound an autonomous node; the operator must retain the
all-power stop capability. No failed scenario is automatically retried.

Single ACKs use Radio at Pi-local RX_DONE + 150 ms. The invalid-auth case uses
the disclosed Sx1262/LinuxRadioIo sequence: corrupted tag at +150 ms and valid
ACK at +350 ms, with the existing 100 ms late-target abort. This is an RF reply
schedule, not proof of the node's exact internal deadline. Other selected invalid
ACKs are sent once, followed by silence for that message's remaining retries.

### Node HeaderErr ACK regressions

`node.current.header_error_rearm` sends two interrupted, otherwise valid
23-byte inverted-IQ ACKs for the target reading, then completes the identical
ACK in the same window. Expect one accepted transmission and one core diagnostic
(error 15, operation RECEIVE, schema 1): attempt 1, header/payload counts 2/0,
valid ACK true. `node.current.header_error_retry` sends only the two interrupted
ACKs in window 1, then completes the same ACK for the retransmitted reading.
Expect two attempts, first ACK_TIMEOUT then ACK_RECEIVED, and exactly one
rejection diagnostic: attempt 1, counts 2/0, valid ACK false. The clean second
window produces no rejection diagnostic. Existing setup, backlog drainage and
next-wake metrics assertions still apply.

Both require `CONFIG_NODE_DEEP_SLEEP_SECONDS=10`,
`CONFIG_NODE_RF_SLEEP_OBSERVATION=y`, and `CONFIG_NODE_RF_PHY_OBSERVATION=y` in
the sealed production build. The last option links passive test wrappers from
`firmware/test_apps/radio/main/node_rf_observe.c`; the runner refuses an ordinary
build for these cases. Firmware policy, cryptography and receive deadlines are
unchanged. At sleep entry the UART includes raw GetIrqStatus data, associated
DIO1 timestamps and SetRx/SetTx HAL brackets, followed by a count/overflow marker.
Every wake's complete bounded trace is required. The independent verifier joins
the two real HeaderErr observations (`0x0020`, or `0x0022` when RX_DONE is also
set) to the saved first timestamp and requires
rearming and the final valid RX_DONE. Saved target TX_DONE timestamps must match
the raw IRQ trace, with each raw SetTx bound to its saved call within 1 ms.
RX_DONE alone does not establish packet validity. Missing HeaderErr, any payload
CRC or unrelated IRQ bit, wrong-window events, overflow or missing capture fails
the case.

The test peer prepares the complete TX profile before each target, then starts
at Pi-local uplink RX_DONE +100/+180 ms. The same-window control ACK starts at
+260 ms. A retry's control ACK, setup, backlog and observation ACKs use +150 ms.
SetTx more than 10 ms late fails rather than sending a catch-up packet. The
proposed interruption is standby at 18 ms after SetTx issue, with a 20 ms upper
bound; the verifier separately checks actual SPI timing (17.5..20 ms) and status
confirmation. Valid ACK completion must fit the original minimum first window
(400 ms), or the second window (300 ms). Pi and node clocks are never directly
subtracted. Every interrupted transmission reserves a full 23-byte ACK charge:
six Pi transmissions total, 407196 us, plus the existing 210-attempt C6 bound.

Before first physical regression, qualify this reverse-direction cutoff with
a declared finite campaign and independent raw observations. The earlier 18 ms
C6-to-Pi result is not a Pi-to-C6 qualification. Require 30/30 actual HeaderErr
observations at the chosen fixed cutoff with valid-ACK controls and controlled
C6 restart coverage; retain no-event/payload-error outcomes as failures, not
HeaderErr successes. Record the fixture, schedule, source/build identities and
actual timing before accepting the cutoff. No failed RF episode is automatically
retried. Host/build validation alone cannot establish RF PASS; execute and record
those physical steps in the active workplan.

Example (all paths, identity isolation and the entire declared allowance must
be operator-confirmed; the nominal fixture file follows the component schema):

```sh
make test-rf-node-ack RF_ARGS='--case node.current.accepted --fixture /tmp/nominal.json --run RUN_ID --output /tmp/new-ack-run --manual-record /tmp/airtime.txt --formatter-result /tmp/formatter.txt --build /tmp/accelerated-build --local-group /private/receiver-group.json --remote-group /private/receiver-group.json --peer-python /path/to/pi/venv/bin/python --confirm-isolated'
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
Stop/capture failures record required restoration. No automatic production
restart follows.

## Production receiver service (service.reading_delivery)

`make test-rf-service` runs the ordinary two-wake exchange against an already
installed isolated production service. Preparation/installing packages and
credentials remains separate. Supply the selected production build, the exact
local copy of the installed disposable group (only the selected node active),
and the nominal fixture. Contributors choose the necessary installation and
sensor-carrier checks before running. Same-acquisition conversion is covered by
the sensor-carrier suite.

The runner stages the current tree, compares the installed package with the
current runtime bundle, checks the exact isolated production unit/environment,
Pi identity, dependencies and actual service UID, and requires the service to
start from stopped state with no prior readings in its isolated database.
service.reading_delivery preserves the existing receiver database and airtime history. Before
start, capture only the selected node's existing message IDs and complete-row
hashes in `reading-baseline.json`, under a read-only transaction as the service
UID. Reuse the required final consistent SQLite capture to prove those rows are
unchanged and exactly two new readings belong to this run's receiver instance.
No second full before-database copy or airtime reset is needed.

Node binaries and an empty readable storage baseline are verified without
flashing. Operator admission precedes service/node start. A new service-instance
health row reporting RX_SINGLE, a fresh same-instance CHRONY_SYNCED observation,
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
   their absence. It creates only a new candidate through the production
   initializer with `known_empty_airtime=True`. The production classifier must
   report missing communicator state and a valid pending commissioning token.
   No runtime state, RTC provenance or clock trust is fabricated. The preparation
   receipt uses schema 2, generation zero and no charged-airtime value because
   startup has not yet installed an airtime snapshot. On first startup the
   receiver atomically installs an empty generation-one ledger and consumes the
   token before TX, including with untrusted UTC. Later startups use ordinary
   recovery. The RF runner still requires its separate fresh network-time and
   validated-history prerequisites; a prepared token alone does not pass them.
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
make test-rf-service RF_ARGS='--host 10.86.160.140 --host-key-alias cura-receiver --unit cura-pilot-INSTANCE.service --package /opt/cura-pilot-INSTANCE --test-root /var/lib/cura-pilot-INSTANCE --fixture /tmp/nominal.json --run RUN_ID --output /tmp/new-service-run --manual-record /tmp/airtime.txt --build /tmp/accelerated-build --local-group /private/receiver-group.json --confirm-isolated'
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
or failed cleanup cannot pass. Host tests do not establish physical service.reading_delivery PASS.

## Rejection packet preparation

The twelve rejection cases have packet builders and host checks, but no integrated
hardware runner. Read [spec.py](spec.py) for names and airtime reservations and
[rejection_vectors.py](rejection_vectors.py) for frames, expected processing
results and ACKs. `make test-rf-host` checks the vectors against production ingress
and compares the C6 packet builder with the independent Python implementation.

From the repository root, prepare fresh disposable credentials and build the
optional C6 image in the configured ESP-IDF environment:

```sh
.venv/bin/python tests/rf/prepare_rejection_inputs.py \
  --output /tmp/new-rejection-inputs --run RUN_ID
CCACHE_DISABLE=1 idf.py -C firmware/test_apps/radio -B /tmp/new-rejection-build \
  -D REJECTION_INPUT_DIR=/tmp/new-rejection-inputs build
```

Replace `RUN_ID` with a fresh 32-character lowercase hexadecimal ID and choose
new private directories. Preparation creates a disposable group and three
distinct identities: active, unknown and revoked. It writes private before/after
receiver allowlists and a C header. The public `rejection-manifest.json` records
source/header hashes, packet/ACK bytes and descriptive command names. Keep the
header, allowlists and enabled image outside the repository and shared captures.
The build rejects changed bound sources or headers; prepare fresh inputs after
such changes. These commands only prepare and build the image.

UART commands require the prepared run ID, the manifest's `command_case`, phase
zero and the current boot nonce. Each command transmits one fixed frame and
observes its expected ACK or silence for 500 ms after TX_DONE. The receiver
allowlist phase is separate from the UART phase: the final vector uses the
post-revocation allowlist. Revocation must preserve the baseline's database rows
so the test distinguishes a revoked node from one that was always unknown.

Use the disposable test image and identities, never ordinary production-node
credentials or counters. Reserve the full twelve-counter range for the matrix;
its pure builder does not allocate or persist counters. Changed authenticated
messages require distinct counters. The bad-tag vector corrupts an already
encrypted frame rather than encrypting another plaintext with the same nonce.

Installed-service orchestration, revocation/restart, physical ACK/silence checks
and database/journal reconciliation remain unimplemented. A matching profile
establishes the selected ACK, not its transmission or reception. Review the
integrated session mechanism before device execution, using
`service.reading_delivery` as the prerequisite exchange.

## Captures

For before/after receiver and node snapshots, use the
[state analysis guide](STATE_ANALYSIS.md). It describes
capture preparation, report commands and the limits of the observed counts.

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
endpoint-local timestamps, effective direction profiles, cleanup and handle
release. A failed Python/peer/cleanup result cannot be replaced by a passing
Unity subcase. First failure stops the batch. Missing prerequisites, zero
selection, unexpected events/reset and missing completion cannot pass.

The Pi profile verifier independently replays the last-written command
parameters and register bytes before every `SetRx`/`SetTx`. Later conflicting
writes, including writes that overlap only part of a register range, fail.
Each operation requires a fresh complete profile; reset, sleep, packet-type
changes and unmodeled commands invalidate known configuration. Required
parameter/workaround dependencies still apply, while independent field order,
split register writes and corrected final values are accepted. This verifies
recorded configuration evidence; physical output power and RF timing retain
their separate qualification requirements.

Keep captures until the run has been checked and any failure diagnosed, then
discard them. Put useful lessons in the owning code or documentation. Operator
airtime history remains necessary for pacing across runs.

The component app enters real short deep sleep after its bounded command and
wakes into a non-transmitting wait. `component.sleep_wake`'s second wake requires a fresh host
command. After an interrupted run, issue no new trigger and confirm endpoint
termination/safety or remove all power/back-power paths; retain possible charges.
An ordinary production image has different autonomous-wake requirements.

Component tests cover their selected interactions. Output-power and waveform
measurements, production 900-second cadence, endurance and rolling-window
behavior require their own procedures and suitable equipment. See the current
[operating envelope](OPERATING_ENVELOPE.md).

Installed-service stop verification binds the observed systemd invocation and
receiver instance before stopping. It requires inactive/dead state, no remaining
service cgroup processes or receiver sleep inhibitor, and that exact instance's
durable clean-stop marker with its matching communicator-state generation.
The verified production unit runs through `systemd-inhibit`: its normal zero
exit or SIGTERM termination are accepted only together with these checks.
Wrapper status alone and clean markers from earlier instances never qualify.
The same verifier owns rehearsal and service.reading_delivery cleanup; failed or missing evidence
retains the original failure and prevents further episodes.

## Accelerated RF execution

RF functional tests use an isolated build/configuration with
`CONFIG_NODE_DEEP_SLEEP_SECONDS=10` and `CONFIG_NODE_RF_SLEEP_OBSERVATION=y`.
Seal it with `production_node.py --seal-build` and pass its directory to the
runner. The default production build remains 900s with observation disabled.
No 900-second RF prerequisite is required; production cadence needs separate validation.

Each completed cycle emits `RF_NODE_SLEEP duration_us=10000000` after finalization
and successful timer setup. By operator agreement tests count this as sleep entry.
The observer requires one marker per boot and real deep-sleep resets. node
sends `SLEEP <run> <case> <wake-count>` after the final marker; its peer requires
the final expected current before accepting the notification. service.reading_delivery directly
checks the final marker alongside service progress. This replaces the 35-second
wait. Frames, attempts, stored outcomes, bindings and cleanup remain required.
The bound is 50s per wake plus 10s, with 9.5..45.5s received-current intervals for
unchanged awake/retry work. Missing/extra evidence fails; no automatic retry.

Per-case packet ceilings remain unchanged; shorter tests do not grant additional
airtime. Retain operator admission and preceding activity. Endurance,
900-second cadence and real rolling-window observation remain separate long tasks.
