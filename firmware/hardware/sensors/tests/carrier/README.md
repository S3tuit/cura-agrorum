# Sensor carrier tests

This ESP32-C6-DEVKITM-1-N4 app runs production sensor drivers through ESP-IDF
Unity and pytest-embedded. The `reading` operation also checks production core,
codec/crypto and real test-partition persistence against the same acquisition.
The [bare-C6 tests](../../../../tests/target/README.md) use fake external I/O.

Read the [fixture and measurement procedure](FIXTURE.md) before running hardware
cases. [firmware/HARDWARE.md](../../../../HARDWARE.md) owns nominal wiring. Use
the UART USB connector, identify the actual board/port, and confirm fixture
readiness for each invocation. Change wiring only with power removed. Include
permanent R12 and the actual I2C pull-up selection in the carrier revision label.

## Destructive test storage

The `reading` operation erases NVS and formats LittleFS before and after each
case. Its test partition labels occupy the **same physical flash ranges** as
production storage; they do not preserve a previously installed node's state:

| Test / production label | Physical range, end exclusive |
|---|---|
| `nvs_test` / `nvs` | `0x9000` through `0xf000` |
| `storage_test` / `storage` | `0x110000` through `0x3f0000` |

Before running `reading`, treat all prior node-local counters, pending readings
and logs as disposable. A warning is printed immediately before the first
erase. Avoiding whole-flash erasure does **not** preserve these partitions.

Each reading case creates a fresh disposable test node ID/key, used only for
local frame validation and never in production. If the board previously held
a production identity, erasing its counters ends that identity lifetime:
**provision a new production node ID and key before any production transmission**.
Follow the existing [identity replacement procedure](../../../../../protocol/protocol-v2-lora/README.md#revocation-and-identity-replacement).
Do not restore the old production image/identity or a counter backup; counter
rollback can reuse a CCM nonce. Changing the test identity does not make the
previous production identity safe to reuse.

## Build and inspect configuration

Run from the repository root in one Bash shell:

```sh
source ~/esp/esp-idf/export.sh
.venv/bin/pip install -r firmware/tests/requirements-hardware.txt
SENSOR_APP=firmware/hardware/sensors/tests/carrier
idf.py -C "$SENSOR_APP" build
.venv/bin/python "$SENSOR_APP/carrier_evidence.py" --record-build "$SENSOR_APP/build"
```

Build/seal commands do not access hardware. The seal checks source, image,
configuration and the compiled [Bosch driver pin](../../../../components/bosch_bme280/source_pin.cmake).
The [driver guide](../../../../components/bosch_bme280/README.md) explains the
optional matching local checkout. Rebuild before sealing after source/configuration
changes; never seal a stale image to bypass a mismatch.

Inspect the resolved configuration against the nominal schematic. The app requires
its fixed sensor pins, 200 ms stabilization and UART0 at 115200. Build defaults
leave both DS ROMs unprovisioned; discovery works, acquisition requires two distinct
attached ROMs. Set them in the app's ignored sdkconfig through `menuconfig`, then
rebuild and seal. Only `reading` accesses persistent storage; no operation loads
a production identity or requires whole-flash erasure.

## Session setup

After identifying the DUT and confirming readiness, enter its actual UART port,
12-hex-digit factory MAC, and assembled fixture revision. This helper creates
ignored session/run directories and prints each result path:

```sh
read -r -p 'UART port: ' SENSOR_PORT
read -r -p 'DUT factory MAC (12 hex digits, no separators): ' SENSOR_DUT
read -r -p 'Carrier revision, R12 and I2C pull-ups: ' SENSOR_CARRIER_REVISION
SENSOR_CARRIER_SESSION=$(mktemp -d "$SENSOR_APP/build/session-$(date +%Y%m%dT%H%M%S%z)-XXXXXX")
sensor_carrier() {
    local run_dir
    if [ ! -d "${SENSOR_CARRIER_SESSION:-}" ]; then
        printf 'Session missing; repeat setup.\n' >&2
        return 2
    fi
    run_dir=$(mktemp -d "$SENSOR_CARRIER_SESSION/run-$(date +%H%M%S)-XXXXXX") || return
    SENSOR_CARRIER_LAST_RUN="$run_dir"
    printf 'Results: %s\n' "$run_dir"
    .venv/bin/python -m pytest "$SENSOR_APP/pytest_sensor_carrier.py" \
        --embedded-services=esp,idf --app-path="$SENSOR_APP" --build-dir=build \
        --target=esp32c6 --port="$SENSOR_PORT" --sensor-dut="$SENSOR_DUT" \
        --carrier-revision="$SENSOR_CARRIER_REVISION" --sensor-fixture-ready \
        --root-logdir="$run_dir" --junitxml="$run_dir/report.xml" \
        -o junit_family=xunit1 -s "$@"
}
```

Each invocation flashes the built image. Commands below are separate operator
steps: select and confirm their fixture before running them. Keep the same
session for paired tests; repeat setup to begin another session.

## Discovery and configuration

```sh
sensor_carrier --sensor-operation discover
```

Discovery reports each CRC-checked DS ROM as the numeric `uint64_t` format used
by configuration: copy its 16 hexadecimal digits exactly, without reversing bus
bytes. Label physical probes, using one connected probe at a time if necessary
and removing power before exchanges. Keep BME connected; discovery also requires
its chip ID `0x60` at I2C address `0x76`. Zero DS devices, overflow beyond eight,
driver failures or incomplete cleanup fail discovery. Unknown ROMs are allowed
here because this is setup.

Enter both labeled identities under **Cura Agrorum sensor board configuration**:

```sh
idf.py -C "$SENSOR_APP" menuconfig
idf.py -C "$SENSOR_APP" build
.venv/bin/python "$SENSOR_APP/carrier_evidence.py" --record-build "$SENSOR_APP/build"
```

Acquisition requires the exact declared inventory: both ROMs for nominal/reference/
missing-BME, only ROM1 for missing-DS0, only ROM0 for missing-DS1. Extra or replaced
probes fail preflight. Connected BME fixtures require its actual chip ID;
missing-BME requires probe NOT_FOUND, not a timeout. Preflight releases resources
and resets before sampling; every boot checks DUT, image and configuration.

## Nominal acquisition and cleanup

With the complete nominal fixture, run the needed operations separately:

```sh
sensor_carrier --sensor-operation acquire --sensor-fixture nominal --sensor-guided
sensor_carrier --sensor-operation repeat --sensor-fixture nominal --sensor-repeat-count 100
sensor_carrier --sensor-operation bme-sleep --sensor-fixture nominal
sensor_carrier --sensor-operation final-cleanup --sensor-fixture nominal --sensor-guided
sensor_carrier --sensor-operation gate-on --sensor-fixture nominal --sensor-guided
sensor_carrier --sensor-operation gate-off --sensor-fixture nominal --sensor-guided
```

`acquire` observes the untouched sample-return hold; `final-cleanup` checks two
force-off calls in a separate boot before its own hold. Omitting `--sensor-guided`
runs software checks and leaves electrical acceptance pending. The fixture guide
owns [measurement limits and hold timing](FIXTURE.md#dc-measurement-procedure).
Enter fresh volts as named, space-separated points, for example:

```text
TP_3V3=3.299 TP_GATE=3.283 TP_SW=0.0003 TP_DQ=0.002
```

These numbers illustrate formatting only. Gate-on additionally requests TP_ADC0
and TP_ADC1. Missing, late or invalid inputs cannot acknowledge the hold.

`repeat` requires the complete requested count. Forwarding observers check the
real stabilization, calibrated ADC acquisition, new bus-wide DS conversion and
addressed reads after at least 750 ms. Equal temperatures are allowed. Each
iteration and `bme-sleep` observe real BME status/mode registers without changing
them or the returned sample. They reject `--sensor-guided` and have no meter hold.
Every acquisition must return within 30 seconds for the declared fixture; a host
timeout is failure, not proof that the target recovered. Sensor values must meet
fixture assertions, without adding plausibility filtering to production sampling.

## Missing devices and restoration

Follow the [fixture states](FIXTURE.md#fixture-states), removing the whole selected
connector with power off. Run one state at a time:

```sh
sensor_carrier --sensor-operation acquire --sensor-fixture missing_ds0 --sensor-guided
sensor_carrier --sensor-operation acquire --sensor-fixture missing_ds1 --sensor-guided
sensor_carrier --sensor-operation acquire --sensor-fixture missing_bme280 --sensor-guided
```

The checks require the exact partial result/diagnostic and preserve unaffected
groups. Missing BME invalidates all three enclosure fields together. After each
fault sequence, restore nominal unpowered, confirm readiness, and rerun guided
nominal acquisition; after BME restoration, also run `bme-sleep`. Software success
alone does not complete the electrical observation.

## Paired identity and ADC tests

### DS identity through a connector exchange

Keep both probes present and predeclare which ROM to warm. Follow the
[identity comparison](FIXTURE.md#adc-and-ds-identity-comparisons) in both positions:

```sh
sensor_carrier --sensor-operation ds-identity --sensor-fixture nominal \
    --sensor-guided --sensor-position A
SENSOR_DS_A="$SENSOR_CARRIER_LAST_RUN/carrier-evidence.json"
```

A successful first position deliberately exits nonzero with
`position_A_complete_sequence_incomplete`. Save its path despite that expected
exit; do not use a shell setup that stops before the assignment. Other failures
require a fresh A run. Remove power, exchange complete physical DS connector
positions, restore power, and keep the same physical ROM warmer:

```sh
sensor_carrier --sensor-operation ds-identity --sensor-fixture nominal \
    --sensor-guided --sensor-position B --sensor-prior-evidence="$SENSOR_DS_A"
```

Keep configured ROM assignments unchanged. B checks the same DUT/image/configuration
and both temperature differences. Restore the desired connectors unpowered.

### ADC reference positions A and B

Assemble [reference position A](FIXTURE.md#test-only-reference-inputs) with both
soil connectors/shunts removed. Keep DS/BME connected. After readiness:

```sh
sensor_carrier --sensor-operation adc-reference --sensor-fixture adc_reference \
    --sensor-guided --sensor-position A
SENSOR_ADC_A="$SENSOR_CARRIER_LAST_RUN/carrier-evidence.json"
```

At the prompt, enter fresh TP_3V3, TP_ADC0 and TP_ADC1 readings within 180 seconds,
following the fixture's settling and comparison rules. Entry starts acquisition.
A retains its sample-return hold and exits incomplete until B. Remove power,
exchange reference leads, confirm position B, and measure all points afresh:

```sh
sensor_carrier --sensor-operation adc-reference --sensor-fixture adc_reference \
    --sensor-guided --sensor-position B --sensor-prior-evidence="$SENSOR_ADC_A"
```

Keep A's JSON unchanged while B uses its ID/hash and measurements. Both positions
must pass on the same DUT/image/configuration. Restore nominal unpowered:
remove injection leads, open JP_REF_ENABLE, and restore both soil shunts/connectors.

## Production core reading mapping

Review [destructive test storage](#destructive-test-storage) first. `reading`
uses a disposable identity and the real persistence path, comparing the sample,
opened frame and pending reading body. Its radio adapter emits no RF; its
returning sleep observer checks the normal sleep request without entering sleep.
It has no electrical sample-return hold.

Run nominal, each needed missing-device fixture, ADC A/B, and restored nominal
as separate wiring steps. The command pattern is:

```sh
sensor_carrier --sensor-operation reading --sensor-fixture nominal
```

Use `missing_ds0`, `missing_ds1` or `missing_bme280` only with that fixture ready.
For reference integration, use the separate pair below after setting up A:

```sh
sensor_carrier --sensor-operation reading --sensor-fixture adc_reference \
    --sensor-guided --sensor-position A
SENSOR_READING_A="$SENSOR_CARRIER_LAST_RUN/carrier-evidence.json"
```

After the expected incomplete A result, remove power and set up B:

```sh
sensor_carrier --sensor-operation reading --sensor-fixture adc_reference \
    --sensor-guided --sensor-position B --sensor-prior-evidence="$SENSOR_READING_A"
```

A component `adc-reference` result cannot substitute for a `reading` A result.
Restore nominal unpowered and repeat nominal reading. The checks in
[carrier_checks.c](main/carrier_checks.c) and [carrier_reading.py](carrier_reading.py)
own exact values, flags and diagnostics; the runner requires the same acquisition
in the frame and persisted body. Its unguided completion label leaves electrical
acceptance pending because this operation has no assigned electrical hold.

## Reset, held reset and deep sleep

With nominal wiring, run each selected case separately:

```sh
sensor_carrier --sensor-operation reset --sensor-fixture nominal --sensor-guided
sensor_carrier --sensor-operation held-reset --sensor-fixture nominal --sensor-guided
sensor_carrier --sensor-operation deep-sleep --sensor-fixture nominal --sensor-guided
```

Each starts with preflight and a fresh boot, then an on-state observation.
`reset` exercises production restart-owned cleanup; `held-reset` and `deep-sleep`
transition with the rail enabled to check default-off. Follow the
[measurement and EN timing](FIXTURE.md#dc-measurement-procedure). Deep sleep has
no timer wake: measure, attest YES while asleep, then press EN only when prompted.

## Exploration and branch isolation

Use `--exploration` instead of `--sensor-guided` to investigate an actual described
circuit. It supports gate-on/off, acquire, final-cleanup, reset, held-reset and
deep-sleep. Acquisition/final-cleanup still require nominal sensor inventory;
pure gate/reset/sleep exploration permits declared branch changes.

```sh
sensor_carrier --sensor-operation held-reset --sensor-fixture nominal --exploration
```

Describe the actual R12/C3, connected branches and any extra load before DUT
access. Enter free-text observations; each is saved immediately with entry time
and phase. `/done` advances the observation. Observation windows are unlimited;
boot/acquisition deadlines remain. Held-reset waits for `/done` before EN release;
deep-sleep waits for `/done`, then prompts manual EN reset without a timer wake.
Unexpected reboot, UART loss, EOF or Ctrl-C leaves an interrupted result. Check
the target's actual state before continuing, and remove power before rewiring.

Completion is `exploration_complete_not_acceptance` with a deliberate nonzero
pytest result. Exploration cannot supply an A/B prerequisite or combine with
guided/position options. Change one branch at a time unpowered, compare like
conditions and restore the nominal circuit before acceptance. A lower voltage
in a modified fixture does not explain or repair an earlier failure.

## Results and host checks

Each run saves `carrier-evidence.json`, `report.xml` and serial logs. Failed and
incomplete runs remain available. Inspect the run status and pytest result as
well as Unity: software completion, A-only incompletion, guided acceptance and
exploration differ. Merged JUnit retains host measurement failures even after
passing Unity preflight. Preserve results while diagnosing or completing A/B;
no permanent archive or committed pass ledger is required. Keep lasting lessons
in the relevant code/procedure, then discard session files when no longer needed.

Runner tests use scripted serial I/O and native checks of real test-app helpers;
they access no board. Run from the repository root:

```sh
.venv/bin/python -m pytest firmware/hardware/sensors/tests/carrier/runner_tests -q
```

See [firmware testing](../../../../tests/README.md) for host builds, test selection
and optional isolated worktrees.
