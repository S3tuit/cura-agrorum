# Sensor-carrier bring-up

This ESP32-C6-DEVKITM-1-N4 app links the real `node_sensors`, ESP backend,
ADC, RMT 1-Wire and BME280/I2C dependencies. ESP-IDF Unity runs on the C6;
pytest-embedded flashes the image, selects cases and records their results.
The bare-C6 app and its fake sensor adapters are separate.

Unity console waits yield to the C6 idle task while the operator answers prompts.
The app forwards to the installed Unity parser and UART implementation; the
task watchdog remains enabled with its original timeout.

Missing-device fixtures, BME hardening/low-power and sensor-to-reading integration
remain in their later stages in [firmware/TESTING.md](../../TESTING.md).

Use the approved circuit and procedure in
[SENSOR_CARRIER.md](../on_device/SENSOR_CARRIER.md). The user identifies the
DUT, confirms the fixture ready, changes wiring with power removed and performs
the multimeter observations. Use the connector labeled **UART**, with the
confirmed port (currently `/dev/ttyUSB0`). Every hardware invocation requires
explicit readiness confirmation and a carrier revision label describing the
assembled circuit and its recorded I2C pull-up-jumper choice.

The 2026-09-11 approved carrier includes **permanent R12, 100 kohm from
`+3V3_SW` to GND**, alongside C3, in every fixture. Include `R12=100kohm fitted`
in the carrier revision label, together with the actual I2C pull-up choice.
Use ordinary guided acceptance with R12 fitted; do not select
`--sensor-diagnostic-load` for this permanent component. Earlier no-R12 runs
remain historical evidence. The September 11 nominal/reference results,
including manual-completion deep sleep, are retained in [evidences/](evidences/).
See the [recorded results](../on_device/SENSOR_CARRIER.md#september-11-revised-carrier-results).

## Retained acceptance evidence

Manually retain the latest reviewed successful `carrier-evidence.json` for
each required implemented case in [evidences/](evidences/). Copy the original
bytes without editing metadata, outcomes, timestamps, measurements or hashes.
Use `<operation>-<fixture>-<mode>[-<position>]-evidence.json`, where mode is
`sensor-guided` or `automatic` for unguided repetition. Examples:

```text
gate-on-nominal-sensor-guided-evidence.json
adc-reference-adc_reference-sensor-guided-A-evidence.json
adc-reference-adc_reference-sensor-guided-B-evidence.json
repeat-nominal-automatic-evidence.json
```

Copy matching A/B records together: B binds A's run ID and exact file SHA256.
A retains `position_A_complete_sequence_incomplete` and cannot establish
acceptance alone. Repetition retains `software_passed_operator_acceptance_pending`;
its complete requested count and software completion establish automated
coverage, while guided electrical acceptance belongs to its separate records.
The required cases and links are in the carrier's recorded-results table above;
discovery is setup, and later-stage cases have no placeholder evidence files.

Review the original JSON, JUnit report and UART log before replacing a record.
Keep failed/incomplete/exploratory attempts and raw reports/logs locally; they
cannot replace successful evidence. A later failure remains unresolved even
when an earlier passing file is retained. Missing required evidence means
pending coverage. Update the result summary when replacing evidence; Git
retains earlier committed versions. Documentation cites these tracked JSON
files rather than machine-local run folders. Each record proves only its
recorded build, fixture and identities; later changes require the applicable
reverification in [firmware/TESTING.md](../../TESTING.md).

## Build and inspect configuration

All commands run from the repository root. Building does not access the DUT.
Activate ESP-IDF in every new shell; install the runner if needed:

```sh
source ~/esp/esp-idf/export.sh
.venv/bin/pip install -r firmware/tests/requirements-hardware.txt
idf.py -C firmware/test_apps/sensor_carrier build
.venv/bin/python firmware/test_apps/sensor_carrier/carrier_evidence.py \
    --record-build firmware/test_apps/sensor_carrier/build
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

The app's ignored `sdkconfig` owns its local probe ROMs. Checked-in defaults contain
`0000000000000000`, which supports discovery but prevents acquisition. The
historically accepted pair was DS0=A7000000BF9D1628 and DS1=7E000000540FA728.
The old physical DS0 is now **compromised, cause undetermined**, and retired
from the nominal fixture. No code rejects its ROM; retain its identity and
earlier results as historical evidence.

The replacement is physically labeled **DS2**, ROM **DF00000050F93828**,
confirmed by the operator on September 11. The current ignored configuration
assigns DS2 to logical channel 0 and DS1 to channel 1; both connector
arrangements passed the guided identity check in the session linked above.
The logical interface still has two DS channels. Other builds must explicitly
configure their actual probes, rebuild and record the build; checked-in defaults
remain unprovisioned. The app has an app-only partition table,
opens no persistent storage and uses no radio identity. Never use whole-flash
erasure for these commands.

## Discovery and physical probe labeling

Once the DUT and connected setup are confirmed ready, paste this setup into
your Bash shell (or source a file containing it). It creates one timestamped
session directory and defines `sensor_carrier`. Each subsequent invocation
creates a separate run directory inside that session, prints serial output
live and saves a JUnit report plus pytest-embedded logs:

```sh
read -r -p 'Assembled carrier revision and pull-up jumper record: ' SENSOR_CARRIER_REVISION
SENSOR_CARRIER_SESSION=$(mktemp -d "firmware/test_apps/sensor_carrier/build/session-$(date +%Y%m%dT%H%M%S%z)-XXXXXX")
printf 'Session: %s\n' "$SENSOR_CARRIER_SESSION"
sensor_carrier() {
    local run_dir
    if [ ! -d "${SENSOR_CARRIER_SESSION:-}" ]; then
        printf 'Session directory missing; run the setup again.\n' >&2
        return 2
    fi
    run_dir=$(mktemp -d "$SENSOR_CARRIER_SESSION/run-$(date +%H%M%S)-XXXXXX") || return
    SENSOR_CARRIER_LAST_RUN="$run_dir"
    printf 'Results: %s\n' "$run_dir"
    .venv/bin/python -m pytest \
        firmware/test_apps/sensor_carrier/pytest_sensor_carrier.py \
        --embedded-services=esp,idf \
        --app-path=firmware/test_apps/sensor_carrier --build-dir=build \
        --target=esp32c6 --port=/dev/ttyUSB0 --sensor-dut=cc8da2fc0224 \
        --carrier-revision="$SENSOR_CARRIER_REVISION" --sensor-fixture-ready \
        --root-logdir="$run_dir" --junitxml="$run_dir/report.xml" \
        -o junit_family=xunit1 -s "$@"
}
```

Run the setup again to start a **new session**. The timestamp includes the local
UTC offset; a random suffix prevents collisions. Keep invoking `sensor_carrier`
without repeating the setup to group raw runs together locally.

Each run also contains its own pytest-embedded serial logs. Failed/incomplete
runs remain in the session. `SENSOR_CARRIER_LAST_RUN` still identifies the latest
run, so the A/B prior-evidence commands below work unchanged. This shell helper
is defined by the setup above; it is not an installed command.

For discovery, invoke it separately:

```sh
sensor_carrier --sensor-operation discover
```

The runner flashes the built app before running. The pinned
`pytest-embedded-idf==2.8.1` supplies esptool's deprecated `--after hard_reset`
spelling, even when the build specifies `hard-reset`. Its warning is harmless:
esptool normalizes the accepted alias to `hard-reset`, with unchanged reset
behavior. No dependency patch or warning suppression is applied here.
For an explicit standalone flash after fixture confirmation, use:

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
.venv/bin/python firmware/test_apps/sensor_carrier/carrier_evidence.py \
    --record-build firmware/test_apps/sensor_carrier/build
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
acceptance uses the guided stage-05 commands below.

## Observe and record the holds

Each `CARRIER_HOLD_READY` marker states the observation window. Automatic awake
holds last 60 seconds. With `--sensor-guided`, every awake electrical hold allows
up to **180 seconds**, ending early once complete, valid readings are entered
and acknowledged by the DUT. This includes `final-cleanup`, transition-on before
reset/sleep, and reset-off after restart. Held-reset meter input also allows
180 seconds, then prompts EN release immediately. DS/ADC post-sample holds have
no live meter input and remain automatic 60-second observations.
Gate-on readiness
follows at least 200 ms stabilization. Wait at least 5 seconds for on-state
measurements or 10 seconds for off-state measurements, and require three stable
meter updates. Record the prescribed test points and voltage limits from the
carrier procedure beside the run logs.

The sample-return hold begins only after `node_sensors_sample_all` returns.
There is no further sensor or gate call, including no final force-off or
teardown call. A returned failing sample still enters the observation hold
before Unity reports failure. The gate-on case releases the rail after its
hold, including when acknowledgement is missing or invalid; the gate-off case
exercises an on-to-off transition before readiness.
Hold completion alone is not electrical acceptance.

The host allows 30 seconds per active boot/menu, discovery, preflight or
acquisition-to-readiness phase, 75 seconds for a 60-second hold and final Unity
result, or 195 seconds for the three-minute guided holds. The extra 15 seconds
is transport/result margin, not extra meter time. A timeout is a failed/incomplete operation, with the phase and
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
git diff --check
```

Runner regressions use real pytest-embedded expectation/Unity parsers with a
pipe replacing serial I/O. They check preflight/reset ordering, selection,
missing/duplicate ROM configuration, image mismatch, timeouts and failed,
ignored or empty Unity results. Native sanitized checks compile the actual app
entry point against ESP-IDF dependency stubs to verify the full ELF hash and
six-byte factory MAC record. These checks provide no sensor hardware evidence.

## Acceptance commands

Use the build/record-build commands and define `sensor_carrier` above in this
same shell. Every invocation creates a new run directory below
`SENSOR_CARRIER_SESSION` and records its path in `SENSOR_CARRIER_LAST_RUN`.
Rebuild and record again after any source or local
configuration change. The runner rejects source/configuration/ELF manifest
mismatches before opening the DUT. Do not record a build without first building.
All commands below use the confirmed UART DUT, factory MAC cc8da2fc0224.

First keep the complete `nominal` fixture: soil probes in air, JP_SOIL0/1 fitted,
JP_REF_ENABLE open, reference leads removed, both configured DS probes and BME
connected. Preserve direct SDA/SCL wires and the recorded separate pull-ups.

```sh
sensor_carrier --sensor-operation repeat --sensor-fixture nominal --sensor-repeat-count 100
sensor_carrier --sensor-operation acquire --sensor-fixture nominal --sensor-guided
sensor_carrier --sensor-operation final-cleanup --sensor-fixture nominal --sensor-guided
sensor_carrier --sensor-operation gate-on --sensor-fixture nominal --sensor-guided
sensor_carrier --sensor-operation gate-off --sensor-fixture nominal --sensor-guided
```

`repeat` requires every indexed acquisition and the requested final count, with
all five valid component groups, empty diagnostics and soil readings within
2000..2700 mV. It records durations; each iteration has a 30-second host deadline.
App-only observers forward real operations and check a new CC/44 conversion,
at least 750 ms before addressed scratchpad reads, both configured ROMs and
resource acquisition/release. Equal temperatures are allowed. This does not
claim whole-acquisition BME boundedness, which awaits stage 08.

`acquire` retains the original no-preinitialization guard and untouched
sample-return hold (60 seconds automatically; up to 180 seconds when guided).
`final-cleanup` samples in a separate boot, makes exactly two
public force-off calls, verifies success/empty diagnostics/unchanged sample and
no bus initialization or rail-enable attempt, then holds separately (up to
180 seconds guided, 60 seconds automatic). No sensor
or gate operation occurs during either observation hold. Omitting `--sensor-guided`
for these operations runs software checks only and records electrical acceptance
as pending; it does not supply an electrical pass.

For every guided meter prompt, enter **volts** on one line with **named points
separated by spaces, without commas**. Order does not matter. These are format
examples only; replace every voltage with your fresh measurement.

`acquire`, `gate-off` and `final-cleanup` request four points:

```text
TP_3V3=3.299 TP_GATE=3.283 TP_SW=0.0003 TP_DQ=0.002
```

`gate-on` requests all six points:

```text
TP_3V3=3.299 TP_GATE=0.002 TP_SW=3.290 TP_DQ=3.285 TP_ADC0=2.580 TP_ADC1=2.600
```

Wait at least 5 seconds after ON readiness or 10 seconds after OFF readiness;
record each point only after three stable display updates. Entering the values
attests that procedure. For all guided awake electrical holds, submit the complete
line within three minutes; valid input ends the hold immediately. Invalid or
incomplete input cannot acknowledge the hold. Held-reset release is prompted
immediately after valid readings. DS/ADC post-sample observations retain their
automatic 60-second windows. Deep sleep allows ten minutes for measurements and
YES, then ends early by prompted manual EN/reset.
Missing, nonfinite, late or out-of-limit inputs cannot pass.
The runner stores inputs immediately, including failed observations.

### DS identity through a connector exchange

This checks identity, not thermal response or acquisition speed. Keep both
configured probes present. Position A puts the probe configured for logical
channel 0 in J_DS0 (physical DS2 after its discovery/configuration) and DS1 in
J_DS1. The retired physical DS0 is not the replacement. If necessary, remove
power, arrange the probes and repeat the carrier wiring preflight before
confirming readiness.

```sh
sensor_carrier --sensor-operation ds-identity --sensor-fixture nominal \
    --sensor-guided --sensor-position A
SENSOR_DS_A="$SENSOR_CARRIER_LAST_RUN/carrier-evidence.json"
```

Declare which labeled probe you will warm (logical 0 or 1), then keep that same
physical ROM at least 2 C warmer during acquisition. Both logical temperatures
are retained. A successful position A intentionally exits nonzero with
`position_A_complete_sequence_incomplete`; retain its path. If it failed for
another reason, repeat A and retain that new path instead. Do not wrap paired
commands in a shell that stops before saving this path on the expected A exit.

**Remove carrier power**, exchange the complete physical DS connectors, repeat
wiring preflight, power on and keep the same labeled probe warmer:

```sh
sensor_carrier --sensor-operation ds-identity --sensor-fixture nominal \
    --sensor-guided --sensor-position B --sensor-prior-evidence="$SENSOR_DS_A"
```

Logical identity must still follow the configured ROM. Do not change sdkconfig
ROM assignments or require identical temperatures across boots. B requires the
matching A evidence and both >=2 C separations before full identity acceptance.
Insufficient separation leaves the check incomplete.

### Reset, held reset, deep sleep and back-power

Each command is independent and starts with preflight followed by a fresh boot.
Its first stage enables the rail through production gate control and provides
an on-state hold of up to three minutes, ending as soon as valid readings are
entered. Record TP_3V3, TP_GATE, TP_SW and TP_DQ there. Held-reset and deep-sleep
transition without preparatory force-off. Intentional software restart instead
exercises production restart-owned cleanup.

```sh
sensor_carrier --sensor-operation reset --sensor-fixture nominal --sensor-guided
sensor_carrier --sensor-operation held-reset --sensor-fixture nominal --sensor-guided
sensor_carrier --sensor-operation deep-sleep --sensor-fixture nominal --sensor-guided
```

- `reset`: production `node_platform_esp_restart()` calls unconditional
  `node_sensors_force_power_off()` before `esp_restart()`. A forwarding observer
  requires one successful off call, empty diagnostics and no sensor initialization
  or gate-on attempt. After real software restart, the new boot is checked and stage 2
  provides an untouched off-state hold for all four points, allowing three
  minutes and ending on complete valid input. This establishes intentional
  restart cleanup, not hardware default-off for CPU-only restart. Production
  logs an off-attempt failure and still restarts; such a run cannot pass this test.
- `held-reset`: at the prompt, press and **keep holding EN/reset** while USB
  power remains connected. Confirm while held, measure all four points after
  10 seconds settling, and enter them within three minutes. Release only when
  prompted immediately after valid input; there is no extra wait to fill the
  window. Release confirmation and boot must finish within the 195-second host
  budget counted from the measurement prompt. The next boot must have the C6
  EN/power-on reason.
- `deep-sleep`: the C6 enters real deep sleep with **no timer wakeup**. Within
  ten minutes of entry, measure all four points after 10 seconds settling and
  three stable display updates, enter their named voltages, then send `YES` at
  the attestation prompt. Enter saves the readings immediately; YES confirms
  they were measured while the MCU remained asleep. **Only after the reset
  prompt, press and release EN/reset with USB connected.** The runner checks
  the same image/DUT and the C6 EN/POWERON reset reason, then finishes. No second
  YES is needed after reset, and no ten-minute minimum wait applies. Reset/boot
  must finish within 30 seconds of that prompt and within the 615-second overall
  observation/boot budget. Early boot, UART loss, wrong reset reason or missing
  readings/YES/reset prevents acceptance. The initial awake on-state hold still
  allows three minutes. The test does not initialize sensors or call gate-off
  to prepare the off observation. Timer wakeup remains covered by the separate
  bare-C6 timer test; this case accepts only its measured electrical interval.

Every off observation measures TP_SW with permanent R12 fitted and no additional
test resistor. The **100 mV maximum and 10-second settling interval are
engineering choices** for stable DC bench observation, not sensor-datasheet
reset guarantees or a sleep-current limit. Three stable display updates are
still required. If TP_SW exceeds 0.1 V after that interval, retain the failure;
investigate with separately declared `--exploration` runs. Observations entered
after a window expires remain incomplete and cannot be attached later as
passing measurements of that state.

The runner's off-state failure message instructs you to retain the failure,
keep permanent R12 fitted and investigate using separately declared
`--exploration` runs, with power removed before wiring changes. Do not add a
second 100 kohm resistor in parallel; that would produce 50 kohm.
`--sensor-diagnostic-load` and
`--sensor-backpower-original` retain their legacy meaning: a temporary-load
repeat of a matching no-R12 failure, with neither run becoming revised-carrier
acceptance. They are not required for ordinary acceptance with permanent R12.
The software does not detect R12; declare the actual circuit in the revision
label and confirm its wiring physically. Neither GPIO state nor the AN8008
establishes transients, exact shutdown timing or microamp sleep current; the
discharge resistor can conceal weak back-power in a voltage measurement.

### Exploration and branch isolation

Use `--exploration` instead of `--sensor-guided` to record observations freely.
It supports the existing `gate-on`, `gate-off`, `acquire`, `final-cleanup`,
`reset`, `held-reset` and `deep-sleep` operations. For example:

```sh
sensor_carrier --sensor-operation held-reset --sensor-fixture nominal --exploration
sensor_carrier --sensor-operation deep-sleep --sensor-fixture nominal --exploration
```

Here `nominal` identifies the base circuit. Before accessing the DUT the runner
asks for a description of the **actual** assembled circuit, including C3, R12,
connected/disconnected branches and any additional load, then asks for wiring
and DUT readiness. Pure gate/reset/sleep exploration permits those declared
branch changes and does not require nominal sensor inventory preflight.
Acquisition and final-cleanup exploration still require the complete nominal
fixture, both configured ROMs and BME preflight; they retain every production
sampling assertion. There are no missing-device sampling handlers.

At each observation, type any number of free-text lines. For example:

```text
TP_SW about 150 mV, still decreasing
After one minute: SW 120 mV; DQ 118 mV
Meter disconnected between these observations
/done
```

Every line is immediately retained in `carrier-evidence.json` with its entry
UTC, phase and elapsed seconds. These timestamps describe entry, not an inferred
measurement instant. Input is not parsed as voltages or a stability attestation.
`/done` is the only control line: it advances from the on-state hold to the
transition, or ends the final observation. All observation holds are unlimited;
normal boot/acquisition/command deadlines remain. Notes stay on the host and
do not cause sensor/gate operations. The target yields while waiting awake.

Held-reset prompts you to keep EN pressed until `/done`, then release it.
Deep-sleep exploration configures **no timer wakeup**: enter `/done`, then
press and release EN/reset when prompted. The runner verifies operator reset,
not timer wakeup. Finishing text input alone cannot wake deep sleep. Normal
guided acceptance requires validated readings and YES within 600 seconds before
prompting EN/reset; all voltage/settling rules remain mandatory.
Unexpected reboot or UART/test failure ends the observation. EOF or Ctrl-C
retains an interrupted record. After interruption, the current target state
must be checked; remove power before any wiring change.

Successful software completion plus explicit `/done` produces
`exploration_complete_not_acceptance` and a deliberately nonzero pytest result.
It cannot become acceptance or supply A/B/loaded-acceptance prerequisites.
Do not combine `--exploration` with `--sensor-guided`, position/prior-evidence
or diagnostic-load options. Describe R12's fitted/removed state and any
additional load in the exploration setup. The legacy guided diagnostic flags
above retain their original, non-accepting meaning for the pre-R12 circuit.

For decay investigation, the current reference circuit has C3 (100 nF) and R12
(100 kohm) both fitted from TP_SW to ground. Record elapsed observations, then
remove power and change **one branch at a time**. Describe each change in a new
run and compare at similar elapsed times with the same preceding powered-on
duration and meter setup. A comparison with R12 removed reproduces the earlier
unloaded circuit and is a separately declared, non-accepting exploration. Fit
or remove R12 only with power removed and restore it before acceptance. Preserve
the original failed observations and the retired DS0's identity; a lower
voltage with a changed fixture does not repair an earlier failure.

### ADC reference positions A and B

Remove power. Disconnect both soil connectors and remove JP_SOIL0 and JP_SOIL1.
Fit JP_REF_ENABLE; retain both configured DS probes and BME. Use R5/R6=10k/5.6k
for VREF_A and R7/R8=2.2k/2.2k for VREF_B through R3/R4=1k. Position A connects
J_VREF_A to J_INJECT0 and J_VREF_B to J_INJECT1. Repeat wiring preflight and
confirm the newly selected fixture ready.

```sh
sensor_carrier --sensor-operation adc-reference --sensor-fixture adc_reference \
    --sensor-guided --sensor-position A
SENSOR_ADC_A="$SENSOR_CARRIER_LAST_RUN/carrier-evidence.json"
```

After preflight and its reset, the runner prompts at the fresh acquisition
boot. Allow reference settling, then use the three-minute input window to
measure each point until three stable display updates and enter newly measured
`TP_3V3=... TP_ADC0=... TP_ADC1=...` in volts.
Acquisition starts immediately after entry through the production stabilization
and calibrated averaging path. Each reported ADC value must be within **75 mV**
of its own fresh measurement. The two measured intervals must be distinguishable;
the air-probe range is not applied to reference results. A keeps the untouched
sample-return hold and intentionally exits incomplete until B, as for DS identity.

Remove power and exchange only the reference leads: VREF_B to J_INJECT0 and
VREF_A to J_INJECT1. Repeat wiring preflight and restore power:

```sh
sensor_carrier --sensor-operation adc-reference --sensor-fixture adc_reference \
    --sensor-guided --sensor-position B --sensor-prior-evidence="$SENSOR_ADC_A"
```

Measure all three points again when prompted; historical 1.180/1.641 V divider
observations supply neither the reference values nor ADC accuracy for this run.
B requires both positions to pass the per-channel comparison and reversed mapping
on the same DUT/image/configuration. Missing measurements or a single position
cannot complete the case. Finally remove power, disconnect reference leads,
open JP_REF_ENABLE, reconnect the soil probes and shunts, restore the preferred
DS connector arrangement, repeat wiring preflight and restore `nominal`.

Every run retains `carrier-evidence.json`, `report.xml` and pytest-embedded
`dut.log`. Inspect the evidence status, not just Unity PASS: software success,
position-A incompletion, full guided acceptance and loaded diagnostics have
distinct states. Follow the [retained-evidence policy](#retained-acceptance-evidence)
to copy reviewed records into `evidences/` and update the corresponding carrier
result links; preserve historical accepted stage-03 observations.

The app locally uses merged JUnit reporting: the Python orchestration case and
the Unity cases are both retained, with totals reconciled from their actual
results. This preserves missing measurements and failed electrical checks even
after a passing Unity preflight. Older reports generated in replacement mode
can show only that passing preflight; retain their original files and consult
`carrier-evidence.json` and the pytest failure output for the requested run.
