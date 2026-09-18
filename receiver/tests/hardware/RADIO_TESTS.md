# SX1262 component hardware procedure

These cases use the production Linux adapter, command backend and owner. They
do not run receiver end-to-end orchestration, spend durable airtime, or submit
`SetTx`. On 2026-09-17 all three nominal cases passed from a fresh verified
staging directory: device/GPIO/SPI configuration, initialization/readback and
finite RX timeout/rearm. Every teardown confirmed safe shutdown. Required
evidence is retained in the development checkout's
[radio archive](evidence/radio/README.md); Pi storage and `/tmp` are temporary.
See the [coverage record](../RADIO_COVERAGE.md) for archive hashes and limits.
Both earlier failures are summarized as lessons. A subsequent manual held-BUSY run
observed the expected bounded fault with unconfirmed safe shutdown retained;
after operator-confirmed power-off restoration, all three nominal cases passed
in 2.28 s with safe teardown. The coverage record identifies these runs and
their missing standalone command/exit records. RF-peer, gated recovery and
instrument-dependent timing remain unrun.

Follow the [carrier schematic](../../hardware/TEST_CARRIER.md#proposed-sx1262-extension)
for the Waveshare EU868 board without a Pico. The operator must confirm the
actual board/revision, wiring, supply and inactive RESET/NSS levels. Stop the
receiver service and other users of the same GPIO/SPI devices. Run as the
actual non-root receiver service user, with its deployed device permissions.
Do not use root to turn a permission failure into a pass.
The approved supply is one shared rail from Pi physical pin 1 (3.3 V) and one
ground rail from physical pin 6. Both modules use these rails, with one 100 nF
and one 10 uF capacitor in parallel across them. Keep both RTC fault shunts
open. The radio keeps its 10 kohm RESET and CS pull-ups to the same 3.3 V rail.

The approved peer is the real C6 radio application, coordinated from laptop
tests/rf/ with a separate Pi component process in
receiver/test_apps/radio_peer/. Reuse the production Pi radio components where
their fixed profile and state contract applies; deliberate alternative-profile
cases identify the lower layer they exercise. Peer implementation and RF
execution are still pending. Independent waveform/timestamp qualification and
controlled BUSY-gate recovery retain their separate deferred status.

The available multimeter can support supply/static-level checks; it cannot qualify BUSY, DIO1, CS or
RESET waveforms, kernel timestamp accuracy or RF airtime. Those obligations
remain pending. The finite RX-timeout case below is a functional IRQ/rearm
check, not independent timestamp or timing acceptance. TX-adjacent physical
teardown and alternating RF profile acceptance remain with the peer stage.

## Stage current sources

Use a new isolated directory on `cura@cura-receiver`, with SSH/SCP option
`-F /dev/null`. Package the current local `receiver/` and
`protocol/protocol-v2-lora/` trees, including untracked implementation/test
files; a committed-only archive is insufficient during development. Exclude
virtual environments, caches and old test output. Record the local archive's
SHA-256, copy it, verify the identical SHA-256 on the Pi, and only then extract
and run from that new directory. Retain the archive hash and exact command
line beside the results. Do not assume any existing Pi checkout is current.
After execution, copy captures into ignored
`receiver/tests/hardware/evidence/radio/raw/` on the development machine and verify
their hashes against the Pi originals. Curate the useful result and essential
inputs into the [permanent archive](evidence/README.md), then delete raw copies.
A Pi path alone is not retained evidence.

Run these two blocks in the same **local Bash terminal**. Avoid editing the
source trees while packaging. `git ls-files` supplies tracked and nonignored
untracked paths; `tar` reads their current working-tree contents. Git-ignored
local identities/notes and the explicit output exclusions are not packaged.
The radio cases require no receiver-group identity or keys.

```bash
set -euo pipefail
cd $HOME/devspace/cura-agrorum
radio_stage=$(mktemp -d /tmp/cura-radio-stage.XXXXXXXX)

git ls-files -z --cached --others --exclude-standard -- \
  receiver/ protocol/protocol-v2-lora/ > "$radio_stage/candidate-files.nul"

package_command=(tar
  --exclude=results --exclude=evidence --exclude=test-results
  --exclude=.venv --exclude=venv --exclude=.tox
  --exclude=__pycache__ --exclude=.pytest_cache --exclude=.hypothesis
  --exclude=.mypy_cache --exclude=.ruff_cache --exclude=.cache
  --exclude=.fuzz-corpus --exclude=build --exclude=dist
  '--exclude=*.egg-info' '--exclude=*.py[co]' '--exclude=.coverage*'
  --exclude=htmlcov '--exclude=*.log' '--exclude=*junit*.xml'
  --exclude=receiver-group.json --exclude=.env
  -czf "$radio_stage/source.tar.gz"
  --null --no-recursion -T "$radio_stage/candidate-files.nul")
{
  printf 'cd %q\n' "$PWD"
  printf '%q ' "${package_command[@]}"
  printf '\n'
} > "$radio_stage/package-command.txt"
"${package_command[@]}"
(cd "$radio_stage" && sha256sum source.tar.gz > SHA256SUMS)
cat "$radio_stage/SHA256SUMS"
```

The next block creates a fresh directory below the remote user's home, copies
the archive and provenance, checks the hash, and only then extracts it. A
failed command stops the block; an existing remote directory is not reused.
No receiver service, GPIO/SPI operation or hardware test is started.

```bash
remote_stage="receiver-radio-${radio_stage##*.}"
sshpass -p cura ssh -F /dev/null cura@cura-receiver "mkdir -- '$remote_stage'"
sshpass -p cura scp -F /dev/null \
  "$radio_stage/source.tar.gz" "$radio_stage/SHA256SUMS" \
  "$radio_stage/package-command.txt" "$radio_stage/candidate-files.nul" \
  "cura@cura-receiver:$remote_stage/"

sshpass -p cura ssh -F /dev/null cura@cura-receiver bash -s -- "$remote_stage" <<'REMOTE'
set -euo pipefail
cd "$1"
mkdir results
cp SHA256SUMS package-command.txt candidate-files.nul results/
sha256sum --check SHA256SUMS | tee results/archive-verification.log
mkdir src
printf 'cd %q\ntar -xzf source.tar.gz -C src\n' "$PWD" > results/extract-command.txt
tar -xzf source.tar.gz -C src
printf '\nStaged source: %s/src\nEvidence directory: %s/results\n' "$PWD" "$PWD"
REMOTE

printf "\nConnect with: ssh -F /dev/null cura@cura-receiver\nThen: cd ~/%s/src\n" "$remote_stage"
```

Require `source.tar.gz: OK` before using the printed source directory. Keep the
archive and `results/` metadata until the run is checked. Permanent evidence
needs one tested source identity; retain a source archive only when Git cannot
recover it. The candidate list and packaging/extraction logs are temporary
diagnostics. Do not write authentication credentials into command captures.

On the Pi, install the native build prerequisites once. For the recorded
Debian 13 / Python 3.13 target:

```sh
sudo apt update
sudo apt install build-essential python3.13-dev python3-venv
```

`spidev` may build a C extension locally. The
[Python 3.13 development package](https://packages.debian.org/trixie/python3.13-dev)
provides its required headers; a venv alone does not provide `Python.h`.
The headers must match the interpreter used to create the venv.

Enter the printed `src/` directory, create an isolated virtual environment and
install:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r receiver/requirements-test.txt \
  -r receiver/requirements-radio.txt \
  -r protocol/protocol-v2-lora/requirements-test.txt
.venv/bin/python -m pip freeze > ../results/dependencies.txt
```

If an existing venv's `spidev` build fails with `fatal error: Python.h: No such
file or directory`, install the matching development package above and rerun
the same pip install in that venv; rebuilding the source archive or recreating
the venv is unnecessary. An import-only check does not open radio devices:

```sh
.venv/bin/python -c 'import spidev, gpiod; print("Radio dependencies import OK")'
```

The fixture writes a SHA-256 source manifest, exact operator input and target
metadata before any device operation. Capture dependency versions, archive hash,
source manifest, JUnit and stdout/stderr for diagnosis, including failures.
After checking the run, keep its outcome, relevant fixture/target metadata,
restoration and essential measurements; consolidate repeated metadata and
replace resolved failed sessions with lessons. Keep production credentials out
of captures; these tests require no receiver identity or keys.
The [archive README](evidence/radio/README.md) describes the retained files and
offline verification. Preserve the relationship between faults and restoration;
transfer needed inputs before the Pi or temporary files become unavailable.

## Explicit fixture input

Create an absolute-path JSON file outside production configuration. Replace
the board identity and service account, and change confirmations to `true`
only after the operator has performed those checks:

```json
{
  "schema": 2,
  "board_id": "record actual board, revision and oscillator population",
  "service_user": "cura",
  "wiring_checked": false,
  "power_checked": false,
  "no_pico_fitted": false,
  "receiver_service_stopped": false,
  "fixture_state": "radio_nominal",
  "busy_selector_checked": false,
  "configuration": {
    "spi_device": "/dev/spidev0.0",
    "gpio_chip": "/dev/gpiochip0",
    "reset_line": 22,
    "dio1_line": 23,
    "busy_line": 24,
    "spi_speed_hz": 1000000
  }
}
```

`busy_selector_checked: true` confirms exactly one shunt in the position
declared by `fixture_state`, changed and checked with external power removed:

| `fixture_state` | Selector | Connection |
|---|---|---|
| `radio_nominal` | 1-2 | Radio BUSY to Pi GPIO24, physical pin 18 |
| `radio_busy_held` | 2-3 | Pi GPIO24 to shared 3.3 V; radio BUSY disconnected |

Never join the radio's BUSY output to 3.3 V. All other radio wiring remains
fitted. Schema 1 and the former `busy_fault_gate_fitted` field are rejected;
they describe a different selector connection and cannot authorize this one.

From the staged `src/`, save the nominal operator JSON as
`../evidence/fixture-nominal.json`. Run the following in Bash. It creates a fresh
run directory, verifies and retains the archive hash, and saves the expanded
command, working directory, log, JUnit and numeric exit status. The component
evidence directory must not already exist; its parent must exist.

```bash
(
set -euo pipefail
run=$(mktemp -d "$PWD/../evidence/nominal.XXXXXXXX")
printf 'Evidence: %s\n' "$run"
(cd .. && sha256sum --check SHA256SUMS) > "$run/archive-check.log"
cp ../SHA256SUMS "$run/"
command=(env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
  -c receiver/pytest.ini receiver/tests/hardware/test_radio.py
  --receiver-hardware -m 'radio and not destructive'
  --receiver-radio-fixture "$run/../fixture-nominal.json"
  --receiver-radio-evidence "$run/nominal-new"
  --junitxml="$run/nominal-new.junit.xml")
{
  printf 'cd %q\n' "$PWD"
  printf '%q ' "${command[@]}"
  printf '> %q 2>&1\n' "$run/nominal-new.log"
} > "$run/command.txt"
status=0
"${command[@]}" > "$run/nominal-new.log" 2>&1 || status=$?
printf '%s\n' "$status" > "$run/exit-status.txt"
cat "$run/nominal-new.log"
exit "$status"
)
```

The fixture also retains source hashes and target/boot identity. Repeat this
capture pattern after manual restoration, using a fresh run directory and
the newly confirmed nominal operator input. A prior shell exit cannot be
recovered later from `$?` or inferred as a captured value from JUnit. Existing
audited runs retain that limitation; do not overwrite their evidence.

Selected missing/inaccessible hardware fails. There are no runtime skips.
Ordinary receiver hardware targets include these obligations; while the carrier
is absent, explicitly select `hardware and not radio` when testing other
components rather than interpreting unrun radio cases as passing.

## Manual held-BUSY startup case

First establish the nominal baseline above. Shut down Linux, remove external
Pi/module power, move the single shunt to 2-3, and verify the radio BUSY contact
is isolated from both GPIO24 and 3.3 V. Reboot with the receiver service stopped.
Create a fresh operator file selecting `radio_busy_held` and confirming those
checks. Save the JSON as `../evidence_busy_held/fixture-held.json` starting from
`src/`.

Run only this case, using the existing dedicated root bearing
`.cura-receiver-test-root` with exact contents `CURA AGRORUM RECEIVER TEST ROOT`
followed by a newline:

```bash
(
set -euo pipefail
run=$(mktemp -d "$PWD/../evidence_busy_held/held.XXXXXXXX")
printf 'Evidence: %s\n' "$run"
(cd .. && sha256sum --check SHA256SUMS) > "$run/archive-check.log"
cp ../SHA256SUMS "$run/"
printf 'CURA AGRORUM RECEIVER TEST ROOT\n' > "$run/.cura-receiver-test-root"
command=(env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
  -c receiver/pytest.ini receiver/tests/hardware/test_radio.py
  --receiver-hardware -m radio_busy_held
  --confirm-receiver-destructive
  --receiver-test-root "$run"
  --receiver-radio-fixture "$run/../fixture-held.json"
  --receiver-radio-evidence "$run/held-new"
  --junitxml="$run/held-new.junit.xml")
{
  printf 'cd %q\n' "$PWD"
  printf '%q ' "${command[@]}"
  printf '> %q 2>&1\n' "$run/held-new.log"
} > "$run/command.txt"
status=0
"${command[@]}" > "$run/held-new.log" 2>&1 || status=$?
printf '%s\n' "$status" > "$run/exit-status.txt"
cat "$run/held-new.log"
exit "$status"
)
```

Collection rejects any other selected test in this run, including nominal
radio cases. It also rejects a nominal fixture file for a held case and vice
versa. The case requires `INITIALIZATION_FAILED`, fatal INITIALIZE /
BUSY_TIMEOUT and CLEANUP / BUSY_TIMEOUT episodes, only HIGH BUSY samples,
released device handles and no SPI transfer, RX entry or TX attempt. It records elapsed monotonic time and
checks the 2 s startup limit with the existing 50 ms late-only service allowance;
this is functional deadline evidence, not waveform/timing qualification.

`held-busy-observation.json` is written only after those assertions succeed.
**The session still exits nonzero (1) because safe standby is unconfirmed.**
The held input prevents confirmation (`safe_shutdown=false`); terminal-instance
shutdown cannot restart the failed owner. `manual-restoration-required.json` and the teardown
result retain that incomplete status. Capture the complete log, trace and
JUnit for diagnosis: an exit code alone does not distinguish the expected restoration stop
from an earlier assertion/device failure, and a matching fault is not a safe
teardown pass. Once checked, permanent evidence needs the fault observation,
unconfirmed cleanup/session outcome and essential trace, not the full session.

After the run, shut down Linux, remove external power and restore the one shunt
to 1-2. Verify nominal continuity and no rail tie to BUSY. Reboot, confirm the
service remains stopped, and run the nominal command with a fresh nominal
operator file and evidence directory. Retain both outcomes, their boot/source
identities and the operator's restoration confirmation together. Later
success does not relabel the held run's unconfirmed cleanup. These separate
runs establish startup failure and manual restoration, not runtime recovery.

## Deferred controlled fault cases

The existing `radio_fault` soft/hard recovery cases require the unavailable
SN74LVC1G32 gate. Collection rejects their selection before GPIO/SPI access.
No field in the manual fixture input enables them. Adding that gate later
requires a revised schematic and operator input; the current selector pin 3
is directly connected to 3.3 V and must not be joined to a gate output.

The deferred fixture programs a finite receive timeout to create a real IRQ
without an RF peer. It then forces only the Pi's observed BUSY high. One case releases the
gate before soft recovery; another keeps it high through the failed soft wait
and physically releases it when the production reset is asserted. Both require
confirmed RX restoration. The gate is returned LOW in `finally` and released.
This is component fault injection; production idle RX still uses `SetRx(0)`.

Every case captures raw JSONL command/GPIO activity, initialization and teardown
results. Nominal finalization requires confirmed standby, disabled IRQ routing
and handle release. Unconfirmed safe teardown aborts all subsequent hardware
cases; preserve the original failure and restore the fixture before a fresh
run. Software traces do not replace independent waveform measurements.
