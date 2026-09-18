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
configuration, selected nominal assertions and same-acquisition RF oracles
remain required. Removed test circuitry cannot supply new physical-fault proof.

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

## Capture and retention

`--output` is temporary working space outside the repository or under ignored
`tests/rf/raw/`. It contains diagnostic UART/Pi traces, run/JUnit results,
operator readiness, build/source identities and the source transfer snapshot.
The runner does not generate source diffs, archive checksum inventories or
historical-prerequisite review bundles. Check a completed capture with:

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
sleep; production 900-second sleep belongs to RF-020. The
[operating-envelope decision](OPERATING_ENVELOPE.md) does not turn deferred
physical RF-018 measurements into PASS.
