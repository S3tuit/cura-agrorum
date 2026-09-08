# Sensor-carrier bring-up

This ESP32-C6-DEVKITM-1-N4 app links the real `node_sensors`, ESP backend,
ADC, RMT 1-Wire and BME280/I2C dependencies. ESP-IDF Unity runs on the C6;
pytest-embedded flashes the image, selects cases and records their results.
The bare-C6 app and its fake sensor adapters are separate.

This first slice implements setup ROM discovery, BME identification, production
gate-on/off holds and one nominal acquisition with a sample-return hold.
Repetition, ADC references, missing-device fixtures, final cleanup, reset/sleep
tests, sensor-to-reading integration and BME hardening remain later work. Their
required coverage is retained in [firmware/TESTING.md](../../TESTING.md).

Use the approved circuit and procedure in
[SENSOR_CARRIER.md](../on_device/SENSOR_CARRIER.md). The operator identifies the
DUT, confirms the fixture ready, changes wiring with power removed and performs
the multimeter observations. Use the connector labeled **UART**, with the
confirmed port (currently `/dev/ttyUSB0`). Every hardware invocation requires
explicit readiness confirmation and a carrier revision label describing the
assembled circuit and its recorded I2C pull-up-jumper choice.

## Build and inspect configuration

All commands run from the repository root. Building does not access the DUT.
Activate ESP-IDF in every new shell; install the runner if needed:

```sh
source ~/esp/esp-idf/export.sh
.venv/bin/pip install -r firmware/tests/requirements-hardware.txt
idf.py -C firmware/test_apps/sensor_carrier build
```

The resolved carrier settings must be:

| Setting | Value |
|---|---:|
| `CONFIG_CURA_SOIL_0_GPIO` / `CONFIG_CURA_SOIL_1_GPIO` | 0 / 1 |
| `CONFIG_CURA_SENSOR_POWER_GATE_GPIO` | 2 |
| `CONFIG_CURA_DS18B20_GPIO` | 3 |
| `CONFIG_CURA_I2C_SDA_GPIO` / `CONFIG_CURA_I2C_SCL_GPIO` | 21 / 22 |
| `CONFIG_CURA_SENSOR_POWER_STABILIZATION_MS` | 200 |
| Console | default UART0, 115200 baud |

CMake prints these settings and both ROM strings. Conflicting configuration
fails compilation; an old sdkconfig cannot silently override the carrier pins.
Use `idf.py -C firmware/test_apps/sensor_carrier menuconfig` to correct values,
then rebuild. Production GPIO defaults are unchanged.

The app's ignored `sdkconfig` owns its local probe ROMs. Both initially contain
`0000000000000000`, which supports discovery but prevents acquisition. The app
has an app-only partition table, opens no persistent storage and uses no radio
identity. Never use whole-flash erasure for these commands.

## Discovery and physical probe labeling

Once the DUT and connected setup are confirmed ready, define this shell
function. Each invocation creates a separate results directory, prints serial
output live and saves a JUnit report plus pytest-embedded logs:

```sh
read -r -p 'Assembled carrier revision and pull-up jumper record: ' SENSOR_CARRIER_REVISION
sensor_carrier() {
    local run_dir
    run_dir=$(mktemp -d firmware/test_apps/sensor_carrier/build/run-XXXXXX) || return
    printf 'Results: %s\n' "$run_dir"
    .venv/bin/python -m pytest \
        firmware/test_apps/sensor_carrier/pytest_sensor_carrier.py \
        --embedded-services=esp,idf \
        --app-path=firmware/test_apps/sensor_carrier --build-dir=build \
        --target=esp32c6 --port=/dev/ttyUSB0 \
        --carrier-revision="$SENSOR_CARRIER_REVISION" --sensor-fixture-ready \
        --root-logdir="$run_dir" --junitxml="$run_dir/report.xml" \
        -o junit_family=xunit1 -s "$@"
}
sensor_carrier --sensor-operation discover
```

The runner flashes the built app before running. For an explicit standalone
flash after fixture confirmation, use:

```sh
idf.py -C firmware/test_apps/sensor_carrier -p /dev/ttyUSB0 flash
```

Discovery prints `CARRIER_ROM value=... family=28 type=DS18B20`. Copy the
**16-digit value exactly as printed**; it is the numeric `uint64_t` format
accepted by the production parser, not bus bytes to reverse. Discovery checks
the driver's CRC-checked addresses through that parser and never assigns
channels from enumeration order. Unknown/unconfigured addresses are reported.
The inventory is limited to eight devices plus an overflow check; overflow,
repeated addresses, driver errors and cleanup failures fail the operation.
Zero DS18B20 devices is an unsuccessful discovery.

One connected probe is enough for labeling discovery; omit `--sensor-fixture`
because this is setup, not nominal acceptance. Keep the BME connected. Its
independent identification result is also printed: address `76`, register `D0`,
ID `60`, status `0`. The ID is specified by
[Bosch's BME280 datasheet, section 5.4.1](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf).
ROM observations remain in the log if BME identification fails, but the overall
operation fails. No BME reset, initialization or measurement is used here.

If probes are not labeled, power down, connect just one and run discovery.
Record its ROM and physical label. Power down before exchanging it for the
other probe and repeat. Choose logical DS0/DS1 explicitly, then restore the
complete `nominal` fixture: both soil probes in air, both DS18B20s and the BME,
soil shunts fitted and reference leads removed.

## Configure the supplied mapping and acquire

Enter both discovered values under **Cura Agrorum sensor board configuration →
DS18B20 channel 0/1 ROM identity**, save and rebuild:

```sh
idf.py -C firmware/test_apps/sensor_carrier menuconfig
idf.py -C firmware/test_apps/sensor_carrier build
```

Check both resolved ROMs and GPIO21/22 in the build output. With nominal wiring
confirmed ready, run each observation separately:

```sh
sensor_carrier --sensor-operation gate-on --sensor-fixture nominal
sensor_carrier --sensor-operation gate-off --sensor-fixture nominal
sensor_carrier --sensor-operation acquire --sensor-fixture nominal
```

Acquisition preflight requires two distinct provisioned ROMs and exactly those
two DS18B20s, plus the expected BME280 at `0x76`. It releases the buses, the
1-Wire pad and switched rail, then the runner resets the C6 and checks the new
boot's DUT identity, ELF hash and configuration. The acquisition case rejects
a boot already used by discovery or gate operations. No hardware preflight
runs in the acquisition boot before the one public sampler call.

The unchanged sampler prints `CARRIER_SAMPLE` with result, duration, all fields
and validity, followed by `CARRIER_DIAGNOSTIC`. Automated nominal success
requires result zero, all five component validity bits, both soil values in
inclusive 2000–2700 mV and empty diagnostics. It imposes no new enclosure or
temperature plausibility ranges. Physical temperature/connector-swap identity
acceptance remains a later operator check.

## Observe and record the holds

Each `CARRIER_HOLD_READY` marker starts a 60-second awake hold. Gate-on readiness
follows at least 200 ms stabilization. Wait at least 5 seconds for on-state
measurements or 10 seconds for off-state measurements, and require three stable
meter updates. Record the prescribed test points and voltage limits from the
carrier procedure beside the run logs.

The sample-return hold begins only after `node_sensors_sample_all` returns.
There is no further sensor or gate call, including no final force-off or
teardown call. A returned failing sample still gets the full observation window
before Unity reports failure. The gate-on case releases the rail after its
hold; the gate-off case exercises an on-to-off transition before readiness.
Hold completion alone is not electrical acceptance.

The host allows 30 seconds per active boot/menu, discovery, preflight or
acquisition-to-readiness phase, and 75 seconds for the 60-second hold and final
Unity result. A timeout is a failed/incomplete operation, with the phase and
serial log reported. It does **not** establish BME boundedness or target cleanup.
The current BME driver has deferred polling/error/low-power risks. Retain the
evidence and resolve a hang or wrong contract result before further acceptance;
do not increase limits or repair the sample-return observation with cleanup.

## Host and build verification

```sh
CCACHE_DISABLE=1 make test-host
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest \
    firmware/test_apps/sensor_carrier/runner_tests -q
source ~/esp/esp-idf/export.sh
idf.py -C firmware/test_apps/sensor_carrier build
idf.py -C firmware/test_apps/on_device build
git diff --check
```

Runner regressions use real pytest-embedded expectation/Unity parsers with a
pipe replacing serial I/O. They check preflight/reset ordering, selection,
missing/duplicate ROM configuration, image mismatch, timeouts and failed,
ignored or empty Unity results. Native sanitized checks compile the actual app
entry point against ESP-IDF dependency stubs to verify the full ELF hash and
six-byte factory MAC record. These checks provide no sensor hardware evidence.

This slice also corrects production DS18B20 pad release: ESP-IDF's
`gpio_reset_pin` enables an internal pull-up. Both production cleanup paths now
explicitly disable input/output and both pulls. The host suite tests the real
primitive and GPIO error propagation. Refresh the bare-C6 fast suite on the
confirmed ready UART DUT with `make test-hardware PORT=/dev/ttyUSB0`. That
flashes the bare-C6 app; the next sensor invocation flashes this app again.
Hardware results and sample-return meter acceptance remain pending actual runs.
