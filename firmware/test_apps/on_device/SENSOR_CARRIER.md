# ESP32-C6 sensor hardware-test carrier

Status: approved carrier and fixture-state wiring, revised 2026-09-11 to include
permanent R12 (100 kohm, `+3V3_SW` to GND). Replacement-probe configuration and
the implemented nominal/reference cases, including manual-completion deep
sleep, have September 11 evidence retained below. Later fixture cases remain deferred.

This carrier supports the `node_sensors` hardware cases in
[`../../TESTING.md`](../../TESTING.md). It uses one common circuit and five
declared connection states. The device under test is the
ESP32-C6-DEVKITM-1-N4. The carrier is powered from the development board's
3.3 V header while the board is powered and controlled through USB.

The field-pilot record at
`cura-agrorum-logbook/deployments/field-pilot-v2/README.md` identifies the
moisture devices as low-cost capacitive probes and the enclosure device as a
BME280, but does not freeze a moisture-probe model or BME280 breakout revision.
Connector pin numbers in this carrier therefore define electrical functions,
not wire colors or breakout silkscreen order. Before assembly, record the
installed models, verify that each analog probe accepts 3.3 V and never drives
its output above 3.3 V, and complete the BME280 breakout checks stated below.

## Design rules

- Change connectors, shunts and reference leads only while carrier power is
  removed.
- Use one common ground for the development board, sensors, reference dividers
  and multimeter.
- Take `+3V3_DUT` from J1 pin 1 and ground from J1 pin 13. This carrier is a
  load powered by the USB-powered development board; do not connect a second
  supply to its power nets. Leave the development board's J5 jumper fitted.
- The AO3401A is the production-equivalent high-side switch under test. Sensor
  absence is selected with removable connectors rather than additional active
  switching.
- Both soil probes and both DS18B20 probes use the switched rail. The 1-Wire
  pull-up also uses the switched rail. The BME280 uses the always-on 3.3 V rail.
- Fit permanent R12, 100 kohm from `+3V3_SW` to GND, in every fixture state.
  It provides a passive discharge path independent of sensor loading. Keep
  switched-rail capacitance small and observe the settled off voltage. R12 can
  conceal weak back-power in a voltage measurement; it does not establish the
  sleep-current budget. The available multimeter cannot establish switching
  transients or microamp sleep current.
- Do not fit the available 470 uF capacitor. Its inrush and stored charge would
  obscure the rail-off tests. Use the 10 uF capacitor only on the always-on
  carrier input and 100 nF capacitors as shown below.
- Connector families and mechanical footprints may follow the available bench
  hardware, but the pin functions and reference designators below remain
  stable.

## GPIO allocation

| Function | ESP32-C6 GPIO | DevKitM-1 header | Firmware setting |
|---|---:|---|---|
| Soil channel 0 ADC | 0 | J1 pin 7 | `CONFIG_CURA_SOIL_0_GPIO=0` |
| Soil channel 1 ADC | 1 | J1 pin 8 | `CONFIG_CURA_SOIL_1_GPIO=1` |
| Active-low sensor gate | 2 | J1 pin 3 | `CONFIG_CURA_SENSOR_POWER_GATE_GPIO=2` |
| DS18B20 1-Wire | 3 | J1 pin 4 | `CONFIG_CURA_DS18B20_GPIO=3` |
| BME280 SDA | 21 | J3 pin 6 | `CONFIG_CURA_I2C_SDA_GPIO=21` |
| BME280 SCL | 22 | J3 pin 5 | `CONFIG_CURA_I2C_SCL_GPIO=22` |

GPIO4, GPIO5, GPIO8, GPIO9 and GPIO15 are left free because they are strapping
pins on this development board. GPIO12 and GPIO13 remain available for native
USB, while GPIO16 and GPIO17 remain available for the USB-to-UART bridge.
GPIO6, GPIO7, GPIO14, GPIO18, GPIO19, GPIO20 and GPIO23 are left unconnected on
this carrier so the later radio design can allocate seven exposed pins without
rewiring the sensor section. The existing provisional radio GPIO10/GPIO11
defaults cannot be used on the DevKitM-1 because those pins are not exposed.

The sensor test build must override the current provisional BME280 GPIO4/GPIO5
defaults with GPIO21/GPIO22 before the carrier becomes an executable fixture.

## Common carrier schematic

### Power gate and measurement points

```text
 +3V3_DUT ----+---- TP_3V3
              +---- C1 10 uF ---- GND
              +---- C2 100 nF --- GND
              +------------------> BME280 + reference source
              |
              +---- Q1 pin 2, SOURCE
                    AO3401A P-MOSFET
                    Q1 pin 1, GATE ----+---- TP_GATE
                                      +---- R1 100 ohm ---- GPIO2
                                      +---- R2 47 k ------- +3V3_DUT

                    Q1 pin 3, DRAIN ---- +3V3_SW ----+---- TP_SW
                                                      +---- C3 100 nF ---- GND
                                                      +---- R12 100 k ---- GND
                                                      +----> switched sensors
```

The AO3401A source is connected to `+3V3_DUT`, its drain to `+3V3_SW`, and its
gate to the externally pulled-up control node. This orientation makes its body
diode oppose ordinary source-to-load current while the MOSFET is off. R2 creates
the hardware-default-off state. The firmware drives GPIO2 as open drain: low
turns Q1 on and release lets R2 turn it off. R1 limits the brief GPIO/gate
charging current without materially delaying the 200 ms stabilization period.

R12 is a permanent rail-to-ground discharge resistor, not a gate resistor or
a temporary test load. It draws approximately 33 uA and dissipates 0.109 mW
at 3.3 V while the rail is on; its current falls with the off-state voltage.
It remains fitted during sampling, reset, held reset and deep sleep. Historical
unloaded measurements refer to the earlier circuit without R12 and must not
be relabeled as acceptance of this revision.

Do not place 10 uF or 470 uF on `+3V3_SW`. TP_GATE, TP_SW, TP_3V3 and at least
one adjacent ground test point must be accessible to multimeter probes while
the development board remains mounted.

### Soil probes, ADC filtering and reference injection

```text
 J_SOIL0                        JP_SOIL0
  1 +3V3_SW
  2 SOIL0_OUT --------------------o--o----+---- R3 1 k ----+---- GPIO0
  3 GND                                    |                +---- TP_ADC0
 J_INJECT0 --------------------------------+                +---- C4 100 nF ---- GND

 J_SOIL1                        JP_SOIL1
  1 +3V3_SW
  2 SOIL1_OUT --------------------o--o----+---- R4 1 k ----+---- GPIO1
  3 GND                                    |                +---- TP_ADC1
 J_INJECT1 --------------------------------+                +---- C5 100 nF ---- GND
```

JP_SOIL0 and JP_SOIL1 are fitted in every state that uses the real air-exposed
soil probes. They are removed before a reference voltage is connected to the
corresponding injection header. R3/C4 and R4/C5 are placed close to the DevKitM
headers. Espressif recommends 100 nF from each ESP32-C6 ADC input to ground;
the 1 k series resistors give modest input protection while settling long before
the production sampling interval.

The unattended fixture requirement for every expected-valid acquisition from a
connected air-exposed soil probe is 2,000 through 2,700 mV inclusive after the
production power-up, stabilization and averaging path. It does not apply to the
deliberately lower `adc_reference` inputs.

### Reference voltages

```text
 +3V3_DUT ---- JP_REF_ENABLE ----+---- R5 10 k ----+---- VREF_A (~1.185 V)
                                 |                 +---- R6 5.6 k ----- GND
                                 |                 +---- J_VREF_A
                                 |
                                 +---- R7 2.2 k ---+---- VREF_B (~1.650 V)
                                                   +---- R8 2.2 k ----- GND
                                                   +---- J_VREF_B
```

Use 1% resistors. The nominal divider values are setup targets only: measure
TP_ADC0 and TP_ADC1 with the multimeter immediately before acquisition and
retain those measurements as the reference. Each reported ADC value must be
within 75 mV of its measured input. The two references are intentionally well
below the 2,000–2,700 mV air-probe range and are separated far enough that a
swap remains detectable at this tolerance. The 75 mV fixture allowance is
deliberately wider than Espressif's specified 40 mV total error for a calibrated
ADC at the attenuation used by the firmware; it leaves margin for the meter,
wiring and residual noise without accepting a channel swap.

No separate capacitors are fitted to VREF_A or VREF_B. C4 and C5 provide local
filtering at the ADC inputs, after the 1 kOhm series resistors, and both divider
sources settle long before acquisition.

JP_REF_ENABLE remains open outside `adc_reference`. Reference outputs reach an
ADC only through a removable lead from J_VREF_A or J_VREF_B to J_INJECT0 or
J_INJECT1.

### DS18B20 bus

```text
                              R9 4.7 k
 +3V3_SW ----------------------/\/\/----+
                                        |
 GPIO3 ------------------------------- DQ+---- TP_DQ
                                        |
                      +-----------------+-----------------+
                      |                                   |
                 J_DS0 pin 2                        J_DS1 pin 2

 J_DS0                              J_DS1
  1 +3V3_SW                          1 +3V3_SW
  2 DQ                               2 DQ
  3 GND                              3 GND
```

Both probes are externally powered; parasite power is not used. R9 remains on
the carrier when either probe is removed and loses power with the probes when
Q1 turns off. Removing a DS18B20 fixture means unplugging all three of its
power, data and ground contacts. Probe wire colors are not authoritative;
identify their functions before terminating them and record each 64-bit ROM.
No per-probe bypass capacitor is fitted on the carrier; C3 is the shared
switched-rail bypass capacitor. Add local capacitance only if cable testing
provides evidence that it is needed.

### Bench probe status, 2026-09-11

The physical probe historically labeled DS0, ROM `A7000000BF9D1628`, is retired
from the nominal fixture as **compromised, cause undetermined**. It came from
field deployment v1 and showed a much slower off-state decay than DS1 and the
replacement probe. Its sheath is discoloured, but neither moisture ingress nor
corrosion as the electrical failure mechanism has been established. Keep the
probe and its earlier measurements as investigation evidence. This status is
an operator hardware decision, not a firmware ROM rejection rule.

DS1 remains ROM `7E000000540FA728`. The replacement's physical bench label is
DS2, ROM `DF00000050F93828`, confirmed by the operator on September 11. The
operator's current ignored sdkconfig assigns DS2 to logical channel 0 and
retains DS1 on logical channel 1. Both connector arrangements passed the guided
identity check in the session below. The physical label DS2 does not create a
third firmware channel; checked-in defaults remain unprovisioned and there is
no ROM denylist.
The historical configured pair and its acquisition/identity evidence remain
valid records of that earlier fixture, not acceptance of the replacement pair.

### BME280 I2C connection

```text
 +3V3_DUT -------------------------------- J_BME pin 1, 3V3
 GND -------------------------------------- J_BME pin 2, GND
 +3V3_DUT ---- JP_I2C_PULLUPS ----+---- R10 4.7 k ----+---- GPIO21
                                  |                  +---- J_BME pin 3, SDA
                                  |
                                  +---- R11 4.7 k ----+---- GPIO22
                                                     +---- J_BME pin 4, SCL

 TP_SDA on GPIO21; TP_SCL on GPIO22
```

JP_I2C_PULLUPS is normally fitted, including in `missing_bme280`, so that the
bus remains electrically defined when the device is absent. Verify the actual
BME280 breakout accepts 3.3 V directly and inspect its onboard pull-ups before
assembly. If its parallel pull-ups make the combined resistance unsuitable,
open JP_I2C_PULLUPS; record that position as part of the carrier revision. The
four-pin connector assumes the breakout already fixes the BME280 address to
`0x76`, configures it for I2C and provides the local BME280 supply decoupling
recommended by Bosch. No additional BME280 bypass capacitor is fitted on the
carrier; C1 and C2 remain on the always-on carrier rail.

## Annotated fixture-state diagrams

Legend: `[X]` fitted or connected, `[ ]` removed or disconnected, and `-->` a
temporary reference lead.

Permanent R12 and C3 are fitted in **all five states below**, including both
`adc_reference` positions; their common wiring is not repeated in each diagram.

### `nominal`

```text
JP_SOIL0 [X]   J_SOIL0 [X]     air-exposed probe -> GPIO0
JP_SOIL1 [X]   J_SOIL1 [X]     air-exposed probe -> GPIO1
J_DS0    [X]   J_DS1    [X]    both configured ROMs present
J_BME    [X]   JP_I2C_PULLUPS [X]
JP_REF_ENABLE [ ]               J_INJECT0/1 have no leads

Expected on every valid soil acquisition: 2000 <= soil_mV <= 2700.
```

This state runs successful acquisition, the 100-cycle switched-rail case,
idempotent cleanup and the manual gate, rail, reset, deep-sleep, back-power and
DS18B20 identity procedures. For identity mapping, change the probes'
temperatures and later exchange their physical connectors while keeping both
present; logical channels must follow ROM identity.

### `missing_ds0`

```text
JP_SOIL0 [X]   J_SOIL0 [X]     air-exposed probe -> GPIO0
JP_SOIL1 [X]   J_SOIL1 [X]     air-exposed probe -> GPIO1
J_DS0    [ ]   J_DS1    [X]    configured logical channel 0 absent
J_BME    [X]   JP_I2C_PULLUPS [X]
JP_REF_ENABLE [ ]               J_INJECT0/1 have no leads

Expected: soil0/soil1 2000..2700 mV; DS0 invalid and zero; DS1 valid;
          BME280 valid; exact DS0 diagnostic; switched rail off after return.
```

### `missing_ds1`

```text
JP_SOIL0 [X]   J_SOIL0 [X]     air-exposed probe -> GPIO0
JP_SOIL1 [X]   J_SOIL1 [X]     air-exposed probe -> GPIO1
J_DS0    [X]   J_DS1    [ ]    configured logical channel 1 absent
J_BME    [X]   JP_I2C_PULLUPS [X]
JP_REF_ENABLE [ ]               J_INJECT0/1 have no leads

Expected: soil0/soil1 2000..2700 mV; DS1 invalid and zero; DS0 valid;
          BME280 valid; exact DS1 diagnostic; switched rail off after return.
```

### `missing_bme280`

```text
JP_SOIL0 [X]   J_SOIL0 [X]     air-exposed probe -> GPIO0
JP_SOIL1 [X]   J_SOIL1 [X]     air-exposed probe -> GPIO1
J_DS0    [X]   J_DS1    [X]    both configured ROMs present
J_BME    [ ]   JP_I2C_PULLUPS [X]
JP_REF_ENABLE [ ]               J_INJECT0/1 have no leads

Expected: soil0/soil1 2000..2700 mV; both DS18B20 groups valid;
          all three enclosure fields zero and invalid; exact BME diagnostic;
          switched rail remains off through the independent BME failure.
```

### `adc_reference`

Disconnect both soil probes and remove both inline soil shunts before enabling
the dividers.

```text
JP_SOIL0 [ ]   J_SOIL0 [ ]
JP_SOIL1 [ ]   J_SOIL1 [ ]
J_DS0    [X]   J_DS1    [X]
J_BME    [X]   JP_I2C_PULLUPS [X]
JP_REF_ENABLE [X]

Position A: J_VREF_A --> J_INJECT0     J_VREF_B --> J_INJECT1
Position B: J_VREF_B --> J_INJECT0     J_VREF_A --> J_INJECT1

Expected in each position:
  VREF_A is nominally 1185 mV and VREF_B is nominally 1650 mV
  the 2000..2700 mV air-probe range does not apply
  both ADC conversions succeed and both soil validity bits are set
  abs(reported_ADC0 - measured_TP_ADC0) <= 75 mV
  abs(reported_ADC1 - measured_TP_ADC1) <= 75 mV
  each logical channel follows the reference lead connected to that input.
```

Power down before changing from position A to position B. Measure both ADC test
points again after the exchange; do not assume the divider outputs remained
unchanged.

For the nominal identity check, predeclare the physical ROM to warm and keep
both configured probes connected. Its logical temperature must exceed the other
by at least 2 C before and after exchanging physical connectors with power
removed. Insufficient separation leaves the identity demonstration incomplete;
this procedure measures neither thermal response time nor temperature accuracy.

## Electrical observations to record

`--exploration` records declared circuit deviations and successive free-text
observations without electrical acceptance. Describe every fitted/removed part,
branch and temporary load before the run. Each entry is retained with its entry
time and current phase; the timestamp is not an inferred measurement instant.
`/done` advances or ends the observation. Awake observation holds are unlimited;
deep-sleep exploration omits timer wakeup and ends with operator EN/reset after
logging finishes. No sensor/gate call is made during sample-return or separate
final-cleanup observation. Ordinary acceptance retains every limit and settling
rule below and still requires the approved assembled circuit. Exploratory
records cannot supply missing acceptance measurements or complete an A/B pair.

For decay investigation, the current reference circuit has both C3 and R12
fitted. Record voltages versus elapsed time, then remove power and isolate one
branch per new run, describing the actual variation. Compare observations at
similar elapsed times and with the same preceding powered-on duration and
meter setup. Removing R12 to reproduce an earlier unloaded experiment is an
explicit, non-accepting exploration; fit/remove it only with power removed.
Restore the complete approved circuit, including R12, before acceptance. These
diagnostic observations do not alter the voltage limits, settling rules or
stable-display criterion below.

The `reset` operation now measures intentional production restart cleanup:
`node_platform_esp_restart` releases the gate through unconditional
`node_sensors_force_power_off` before `esp_restart`. Its later observation is
untouched after reboot. CPU-only software restart by itself may retain GPIO
state; it is not a hardware-default-off claim. Held EN/reset and deep sleep
retain their enabled-rail transition without a preparatory force-off.

These are stable DC observations for the available AN8008 multimeter, not
oscilloscope measurements. Measure DC voltage relative to an adjacent carrier
ground point. Do not interpret the meter's hold function as transient capture
or a min/max measurement.

Before applying USB power, perform and record this wiring preflight:

- Q1 source has continuity to TP_3V3 and Q1 drain has continuity to TP_SW.
- R1 measures approximately 100 ohm from GPIO2 to TP_GATE.
- R2 measures approximately 47 kohm from TP_GATE to TP_3V3; TP_GATE is not
  shorted to ground.
- R12 is a verified 100 kohm part connected from TP_SW to GND. Verify its value
  before fitting or with one end isolated while unpowered; parallel sensor
  paths mean an in-circuit resistance reading need not equal 100 kohm.
- All carrier and sensor grounds have continuity, and there is no direct short
  from TP_3V3 or TP_SW to ground. Allow the capacitors to charge from the
  resistance meter before judging a low initial resistance.
- Connector pin functions, Q1 pin numbers and the selected fixture state match
  this document. Wire colors are not accepted as evidence.

The test application must provide three distinct observation holds. The
power-on and power-off holds exercise the same production gate-control
implementation used by sampling. The power-on hold waits at least the
production 200 ms stabilization interval before announcing that it is ready.
The sample-return hold runs the unmodified `node_sensors_sample_all` path,
announces readiness only after that call returns, and then makes no further
sensor or gate-control call. It must not call `node_sensors_force_power_off` to
prepare the measurement. Do not pause or extend the production sampling path
to make a meter reading possible.

The sensor-carrier deep-sleep observation allows at most 600 seconds for
TP_3V3, TP_GATE, TP_SW and TP_DQ after 10 seconds settling, followed by a separate
YES attestation that the measurements were taken while the MCU remained asleep.
Enter saves the readings immediately. After valid readings and YES, the runner
prompts the operator to press and release EN/reset with USB connected. No timer
wake is configured; there is no compulsory ten-minute wait. A boot, failure or
UART loss observed before attestation completes prevents acceptance. After the
prompt, reset and boot must finish within 30 seconds and before the overall
615-second observation/boot deadline. Verify the same image/DUT and C6 EN/POWERON
reset reason. Missing readings, YES or reset leave the case incomplete. This
establishes the measured off-state interval; the bare-C6 timer test separately
covers timer-deep-sleep behavior.

Each awake hold lasts for at least 60 seconds or until an explicit host
acknowledgement, so one meter can be moved between points. Every guided awake
electrical hold allows up to 180 seconds and ends early when complete, valid
operator readings cause the host to acknowledge the hold. This includes
`acquire`, `gate-on`, `gate-off`, separate `final-cleanup`, transition-on before
reset/sleep, and reset-off after restart. Their host observation/result ceiling
is 195 seconds. Held-reset measurements allow 180 seconds, then prompt release
immediately after valid input; release and reboot must fit the 195-second host
budget. ADC reference measurements before sampling also allow 180 seconds and
start acquisition on complete input. Automatic post-sample holds without a live
meter prompt (DS identity and ADC reference) keep 60 seconds. Unguided holds keep
their original durations; guided deep sleep ends by the prompted manual reset.
Preparation confirmations are
separate from these observation windows.
Missing measurements or acknowledgement cannot pass. After a ready marker,
wait at least 5 seconds before recording an on-state voltage and at least
10 seconds before recording an off-state voltage. Record a value only after the
display is stable over three consecutive updates.

The **100 mV off-state limit and 10-second settling interval are engineering
choices** for a repeatable stable DC bench observation. They are not a
datasheet-derived sensor reset voltage/time, a guarantee of complete internal
discharge or a sleep-current specification. Keep both values unchanged; a later
decay below the limit does not repair an earlier failed observation. Normal
acceptance measures the approved circuit with R12 fitted and no additional
diagnostic resistor.

The numeric meter values in the table below were recorded on the earlier
carrier without R12. Preserve them as historical observations; revised-carrier
results are recorded separately below.

| Stable condition | Record | Acceptance |
|---|---|---|
| Production gate-on hold in `nominal` | TP_3V3: 3.298V, TP_GATE: 0.0mV, TP_SW: 3.295V, TP_DQ: 3.286V, TP_ADC0: 2.569V, TP_ADC1: 2.581V | TP_GATE <= 0.2 V; `abs(TP_SW - TP_3V3) <= 0.1 V`; `abs(TP_DQ - TP_SW) <= 0.1 V`; both ADC test points are 2.0-2.7 V. |
| Production gate-off hold in `nominal` | TP_3V3: 3.298V, TP_GATE: 3.283V, TP_SW: 0.3mV, TP_DQ: 0.0mV | TP_GATE >= 3.0 V and `abs(TP_GATE - TP_3V3) <= 0.1 V`; TP_SW and TP_DQ are each <= 0.1 V after 10 seconds. |
| Sample-return hold in `nominal` | TP_3V3: 3.298V, TP_GATE: 3.283V, TP_SW: 0.3mV, TP_DQ: 0.0mV | The sample succeeded; the gate and off-rail targets above hold without any post-return cleanup call. |
| Sample-return hold in `missing_ds0`, `missing_ds1` and `missing_bme280` | TP_GATE, TP_SW and TP_DQ | The declared partial result and diagnostic are returned; the gate and off-rail targets above still hold without any post-return cleanup call. |
| Hold after at least two `node_sensors_force_power_off` calls | TP_GATE, TP_SW and TP_DQ | Both calls succeeded and the gate and off-rail targets above still hold. |
| Untouched hold after `node_platform_esp_restart` in `nominal` | TP_3V3, TP_GATE, TP_SW and TP_DQ | The production restart's off attempt succeeded; no sensor initialization/gate-on was observed during cleanup; the gate and off-rail targets above hold after software restart. This is restart-owned cleanup, not CPU-reset hardware defaults. |
| ESP32 held in reset in `nominal` | TP_3V3, TP_GATE, TP_SW and TP_DQ | External R2 releases the gate and R12 provides rail discharge; the gate and off-rail targets above hold without firmware cleanup. |
| ESP32 in a deep-sleep interval long enough to measure all points | TP_3V3, TP_GATE, TP_SW and TP_DQ | The gate and off-rail targets above hold throughout the stable observation window. |
| `adc_reference` position A, then position B | Fresh TP_3V3, TP_ADC0 and TP_ADC1 for each position; both firmware-reported ADC values | VREF_A is nominally 1.185 V and VREF_B 1.650 V; each reported value is within 75 mV of its measured test point and follows the selected reference after the swap. |
| Historical DS0/DS1 ROM identities | Retired compromised DS0: A7000000BF9D1628; retained DS1: 7E000000540FA728 | Physical DS2, DF00000050F93828, now replaces logical channel 0; retain historical records without rewriting their identities. |

Earlier divider observations were TP_3V3=3.298 V, TP_ADC0=1.180 V and
TP_ADC1=1.641 V. These historical observations are retained for context only;
they are not the measured inputs or accuracy evidence for a new A/B run.

Use the [stage-05 commands](../sensor_carrier/README.md#acceptance-commands)
for the implemented nominal/reference matrix. Each guided run retains its
measurements and source/build identity in carrier-evidence.json alongside
report.xml and raw serial logs. Position A alone is incomplete; paired acceptance
requires matching A/B records. The September 11 outcomes, including the approved
manual-completion deep-sleep case, are linked to retained JSON records below.

The historical nominal sample-return values above were supplied by the operator
on 2026-09-08, after the specified settling interval during the 60-second hold. Both identity
preflight and the acquisition from a fresh boot passed; sampling returned
`CURAG_OK`, validity `0x1f` and empty diagnostics. These are sample-return
measurements, separate from the dedicated gate-off observations. The corrected
I2C wiring uses direct SDA/GPIO21 and SCL/GPIO22 connections with separate
pull-ups; the earlier 4.7 kohm series signal connections were removed before
this accepted run. Raw results are local ignored build artifacts.

For every off-state row, measure TP_SW with permanent R12 fitted. If it is
above 0.1 V after 10 seconds, retain the failure and investigate in separately
declared exploratory runs. Do not add a second 100 kohm resistor in parallel
(that would make 50 kohm) or remove R12 to obtain acceptance. The earlier
no-resistor versus temporary-100-kohm comparisons remain diagnostic evidence
and do not repair any original failure.

The runner's off-state failure message retains the failure, keeps permanent
R12 fitted and directs further investigation to separately declared
`--exploration` runs with power removed before wiring changes. The optional
`--sensor-diagnostic-load` workflow retains its historical meaning for a
temporary-load repeat of a pre-R12 failure. Ordinary revised-fixture acceptance
uses `--sensor-guided` without diagnostic-load flags. Record `R12=100kohm fitted`
and the I2C pull-up choice in `--carrier-revision`; this label is operator
evidence, not automatic detection of the resistor.

These observations establish stable on/off levels, sampling-owned cleanup and
hardware-default-off behavior under the fitted discharge load. A low rail
voltage with R12 fitted does not establish absence of weak steady-state
back-power or its energy cost.
They do not establish the 200 ms rail-rise waveform, exact shutdown instant,
brief boot/reset/deep-sleep glitches, MOSFET edge rate or inrush, I2C or 1-Wire
waveform integrity, or the final sleep-current budget. Successful acquisition
through the unchanged production path is only indirect evidence that the rail
is usable after the 200 ms stabilization interval. Those remaining electrical
claims require an oscilloscope or current instrumentation on later hardware.

## September 11 revised-carrier results

The reviewed JSON records are retained in [sensor_carrier/evidences/](../sensor_carrier/evidences/)
under the app's [manual retention policy](../sensor_carrier/README.md#retained-acceptance-evidence).
The recorded carrier revision is `nominal, after fitting R12`. These records use
logical channel 0 = physical DS2 (`DF00000050F93828`), logical channel 1 = DS1
(`7E000000540FA728`), and DUT `cc8da2fc0224` on `/dev/ttyUSB0`.

## Parts used by this carrier

| Reference | Part/value | Purpose |
|---|---|---|
| Q1 | AO3401A, SOT-23 | High-side P-channel sensor-rail switch |
| R1 | 100 ohm | Gate series resistor |
| R2 | 47 kohm | Gate-to-source hardware-off pull-up |
| R3, R4 | 1 kohm | ADC series resistors |
| R5 | 10 kohm, 1% | VREF_A upper resistor |
| R6 | 5.6 kohm, 1% | VREF_A lower resistor |
| R7, R8 | 2.2 kohm, 1% | VREF_B upper and lower resistors |
| R9-R11 | 4.7 kohm | 1-Wire and I2C pull-ups |
| R12 | 100 kohm | Permanent +3V3_SW-to-GND discharge resistor; fitted in every fixture |
| C1 | 10 uF | Always-on carrier input bulk capacitor |
| C2-C5 | 100 nF | Always-on and switched-rail bypass plus ADC filtering |
| JP_SOIL0, JP_SOIL1 | Two-pin headers and shunts | Disconnect soil outputs from ADC inputs |
| JP_REF_ENABLE | Two-pin header and shunt | Power reference dividers only in `adc_reference` |
| JP_I2C_PULLUPS | Two-pin header and shunt | Select carrier I2C pull-ups |
| J_SOIL0, J_SOIL1 | Removable three-pin connectors | Switched power, analog output and ground |
| J_DS0, J_DS1 | Removable three-pin connectors | Switched power, DQ and ground |
| J_BME | Removable four-pin connector | Always-on 3.3 V, ground, SDA and SCL |

## Source references

- [ESP32-C6-DevKitM-1 user guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32c6/esp32-c6-devkitm-1/user_guide.html)
- [ESP32-C6 hardware design guidelines](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32c6/schematic-checklist.html)
- [AO3401A manufacturer data](https://www.aosmd.com/products/mosfets/p-channel-mosfets-8v-60v/ao3401a)
- [DS18B20 data sheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ds18b20.pdf)
- [BME280 data sheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf)
