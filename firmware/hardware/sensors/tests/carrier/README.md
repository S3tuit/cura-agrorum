# Sensor carrier tests

This XIAO ESP32S3 app runs production sensor drivers through ESP-IDF
Unity and pytest-embedded. `acquire` and `adc-reference` also check production
core, codec/crypto and real test-partition persistence against their core acquisition.
The [bare-S3 tests](../../../../tests/target/README.md) use fake external I/O.

Read the [fixture and measurement procedure](FIXTURE.md) before running hardware
cases. [firmware/HARDWARE.md](../../../../HARDWARE.md) owns nominal wiring. Use
the XIAO native USB-C connector, identify the actual board/port, and confirm fixture
readiness for each invocation. Change wiring only with power removed.
The [USB transport](../../../../tests/README.md#native-usb-across-reset-and-sleep)
reconnects across sleep/reset; an unexpected boot still fails an electrical hold.
Establish the [soil input envelope](../../../../HARDWARE.md#soil-inputs) before
connecting either probe output. Previous-board calibration does not qualify S3 ADC readings.

## Destructive test storage

The core stage of `acquire` and `adc-reference` erases NVS and formats LittleFS
before and after its case, including during acquisition exploration. The test
partition labels occupy the **same physical flash ranges** as
production storage; they do not preserve a previously installed node's state:

| Test / production label | Physical range, end exclusive |
|---|---|
| `nvs_test` / `nvs` | `0x9000` through `0xf000` |
| `storage_test` / `storage` | `0x110000` through `0x3f0000` |

Before running either combined operation, treat all prior node-local counters,
pending readings and logs as disposable. A warning is printed immediately before the first
erase. Avoiding whole-flash erasure does **not** preserve these partitions.

Each core case creates a fresh disposable test node ID/key, used only for
local frame validation and never in production. If the board previously held
a production identity, erasing its counters ends that identity lifetime:
**provision a new production node ID and key before any production transmission**.
Follow the existing [identity replacement procedure](../../../../../protocol/protocol-v2-lora/README.md#revocation-and-identity-replacement).
Do not restore the old production image/identity or a counter backup; counter
rollback can reuse a CCM nonce. Changing the test identity does not make the
previous production identity safe to reuse.

## Build and inspect configuration

Run from the repository root in one Bash shell:

If this app was previously configured for another target, move its ignored
`build/`, `sdkconfig` and `sdkconfig.old` to a backup outside the app first.
ESP-IDF rejects a C6 CMake cache even with S3 sources; defaults cannot replace
the resolved target or probe IDs. Build discovery from the empty ROM defaults.

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
its fixed sensor pins, 200 ms stabilization, native USB console, one core and
8 MB flash. Use a fresh configuration when changing target; save only reviewed
probe ROM settings from any previous configuration. Build defaults
leave both DS ROMs unprovisioned; discovery works, acquisition requires two distinct
attached ROMs. Set them in the app's ignored sdkconfig through `menuconfig`, then
rebuild and seal. Only `acquire` and `adc-reference` access persistent storage;
no operation loads a production identity or requires whole-flash erasure.

## Session setup

After identifying the DUT and confirming readiness, enter its actual native USB port
and 12-hex-digit factory MAC. Each run automatically records SHA-256 hashes of
[FIXTURE.md](FIXTURE.md) and [firmware/HARDWARE.md](../../../../HARDWARE.md) as
`fixture_sha256` metadata. These describe the fixture definition; readiness still
confirms the actual assembly. Documentation changes do not require a firmware
rebuild, but paired A/B runs must use identical document contents. This helper
creates ignored session/run directories and prints each result path:

For a first flash, the installed board firmware can expose its own USB CDC
interface instead of native Serial/JTAG. If esptool cannot connect or the runner
rejects the USB VID/PID, unplug USB and hold BOOT while reconnecting it, then
release BOOT. See [Seeed's download-mode procedure](https://wiki.seeedstudio.com/xiao_esp32s3_with_micropython/#connect-the-xiao-esp32s3-on-your-pc).
Select the re-enumerated `303a:1001` port. Read its factory
MAC without leaving download mode, then remove the colons for `SENSOR_DUT`:

```sh
.venv/bin/python -m esptool --chip esp32s3 --port /dev/ttyACM0 \
    --before no-reset --after no-reset --no-stub read-mac
```

If the first flash succeeds but the console stays at `waiting for download`,
restart from ROM with the supported watchdog reset (no erase or flash):

```sh
.venv/bin/python -m esptool --chip esp32s3 --port /dev/ttyACM0 \
    --before no-reset --after watchdog-reset --no-stub read-mac
```

Wait for USB to reappear and reselect the same USB serial number/location;
its ttyACM number may change. Then rerun discovery with the current port.


```sh
read -r -p 'Native USB port: ' SENSOR_PORT
read -r -p 'DUT factory MAC (12 hex digits, no separators): ' SENSOR_DUT
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
        --target=esp32s3 --port="$SENSOR_PORT" --sensor-dut="$SENSOR_DUT" \
        --sensor-fixture-ready \
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

`acquire` observes the untouched sample-return hold, then resets automatically
for the [core-reading stage](#production-core-reading-mapping) on the same fixture.
Both stages must pass. Review [destructive test storage](#destructive-test-storage)
before running it. `final-cleanup` checks two force-off calls in a separate boot
before its own hold. Omitting `--sensor-guided` runs both software stages and
leaves electrical acceptance pending. The fixture guide
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

Both acquisition stages require the exact partial result/diagnostic and preserve
unaffected groups in the sample, frame and stored body. Missing BME invalidates
all three enclosure fields together. After each
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

Each position runs component acquisition and its untouched 60-second sample-return
hold, then resets automatically for the core-reading stage. Before **each** acquisition, enter fresh
TP_3V3, TP_ADC0 and TP_ADC1 readings within 180 seconds, following the fixture's
settling and comparison rules. The first measurements cannot qualify the second
acquisition. Keep the same reference routing across the automatic reset.
Review [destructive test storage](#destructive-test-storage) before running A.

A exits incomplete only after both stages pass; completing the pair requires B.
Remove power, exchange reference leads, confirm position B, and repeat both
fresh measurement sets:

```sh
sensor_carrier --sensor-operation adc-reference --sensor-fixture adc_reference \
    --sensor-guided --sensor-position B --sensor-prior-evidence="$SENSOR_ADC_A"
```

Keep A's JSON unchanged while B uses its ID/hash and both stage comparisons.
Each position must pass both stages on the same DUT/image/configuration;
component-only A receipts cannot qualify B. Restore nominal unpowered:
remove injection leads, open JP_REF_ENABLE, and restore both soil shunts/connectors.

## Production core reading mapping

This is the automatic second stage of `acquire` and `adc-reference`. A fresh boot
keeps core-owned final cleanup after the untouched sample-return observation and
keeps manual hold time outside the core radio deadline. It takes a new real
sample, uses a disposable identity and the real persistence path, then compares
that sample with the opened frame and the pending reading recovered from flash.
The two stages sample independently; their values need not be identical.

The radio adapter captures the frame without RF. A returning sleep observer
checks the normal sleep request without entering sleep. This stage has no
electrical hold. The checks in
[carrier_checks.c](main/carrier_checks.c) and [carrier_reading.py](carrier_reading.py)
own exact values, flags and diagnostics; the runner requires the same core
acquisition in the frame and persisted body. A run completes only after both
ordered stages pass. Unguided acquisition leaves electrical acceptance pending.

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
