# ESP32-C6 sensor hardware-test carrier

Status: approved carrier and fixture-state wiring.

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
- Do not fit a switched-rail discharge resistor: it could hide back-power from
  a signal or protection-diode path. Keep switched-rail capacitance small and
  observe the settled off voltage. The available multimeter cannot establish
  the rail's behavior during the switching transient.
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
                                                      +----> switched sensors
```

The AO3401A source is connected to `+3V3_DUT`, its drain to `+3V3_SW`, and its
gate to the externally pulled-up control node. This orientation makes its body
diode oppose ordinary source-to-load current while the MOSFET is off. R2 creates
the hardware-default-off state. The firmware drives GPIO2 as open drain: low
turns Q1 on and release lets R2 turn it off. R1 limits the brief GPIO/gate
charging current without materially delaying the 200 ms stabilization period.

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

## Electrical observations to record

These are stable DC observations for the available AN8008 multimeter, not
oscilloscope measurements. Measure DC voltage relative to an adjacent carrier
ground point. Do not interpret the meter's hold function as transient capture
or a min/max measurement.

Before applying USB power, perform and record this wiring preflight:

- Q1 source has continuity to TP_3V3 and Q1 drain has continuity to TP_SW.
- R1 measures approximately 100 ohm from GPIO2 to TP_GATE.
- R2 measures approximately 47 kohm from TP_GATE to TP_3V3; TP_GATE is not
  shorted to ground.
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

Each awake hold lasts for at least 60 seconds or until an explicit host
acknowledgement, so one meter can be moved between points. After a ready marker,
wait at least 5 seconds before recording an on-state voltage and at least
10 seconds before recording an off-state voltage. Record a value only after the
display is stable over three consecutive updates.

| Stable condition | Record | Acceptance |
|---|---|---|
| Production gate-on hold in `nominal` | TP_3V3: 3.298V, TP_GATE: 0.0mV, TP_SW: 3.295V, TP_DQ: 3.286V, TP_ADC0: 2.569V, TP_ADC1: 2.581V | TP_GATE <= 0.2 V; `abs(TP_SW - TP_3V3) <= 0.1 V`; `abs(TP_DQ - TP_SW) <= 0.1 V`; both ADC test points are 2.0-2.7 V. |
| Production gate-off hold in `nominal` | TP_3V3: 3.298V, TP_GATE: 3.283V, TP_SW: 0.3mV, TP_DQ: 0.0mV | TP_GATE >= 3.0 V and `abs(TP_GATE - TP_3V3) <= 0.1 V`; TP_SW and TP_DQ are each <= 0.1 V after 10 seconds. |
| Sample-return hold in `nominal` | TP_3V3: 3.298V, TP_GATE: 3.283V, TP_SW: 0.3mV, TP_DQ: 0.0mV | The sample succeeded; the gate and off-rail targets above hold without any post-return cleanup call. |
| Sample-return hold in `missing_ds0`, `missing_ds1` and `missing_bme280` | TP_GATE, TP_SW and TP_DQ | The declared partial result and diagnostic are returned; the gate and off-rail targets above still hold without any post-return cleanup call. |
| Hold after at least two `node_sensors_force_power_off` calls | TP_GATE, TP_SW and TP_DQ | Both calls succeeded and the gate and off-rail targets above still hold. |
| ESP32 held in reset in `nominal` | TP_3V3, TP_GATE, TP_SW and TP_DQ | The external R2 default alone satisfies the gate and off-rail targets above. |
| ESP32 in a deep-sleep interval long enough to measure all points | TP_3V3, TP_GATE, TP_SW and TP_DQ | The gate and off-rail targets above hold throughout the stable observation window. |
| `adc_reference` position A, then position B | TP_3V3: 3.298V, TP_ADC0: 1.180V, TP_ADC1: 1.641V; both firmware-reported ADC values | VREF_A is nominally 1.185 V and VREF_B 1.650 V; each reported value is within 75 mV of its measured test point and follows the selected reference after the swap. |
| DS0/DS1 ROM identities | DS0: A7000000BF9D1628, DS1: 7E000000540FA728 | |

The nominal sample-return values above were supplied by the operator on
2026-09-08 for [run-meter-qGe3hI](../sensor_carrier/build/run-meter-qGe3hI/report.xml),
after the specified settling interval during the 60-second hold. Both identity
preflight and the acquisition from a fresh boot passed; sampling returned
`CURAG_OK`, validity `0x1f` and empty diagnostics. These are sample-return
measurements, separate from the dedicated gate-off observations. The corrected
I2C wiring uses direct SDA/GPIO21 and SCL/GPIO22 connections with separate
pull-ups; the earlier 4.7 kohm series signal connections were removed before
this accepted run. Raw results are local ignored build artifacts.

For every off-state row, first measure TP_SW without adding a load. If it is
above 0.1 V after 10 seconds, power down, temporarily connect 100 kohm from
TP_SW to ground, repeat the same state and record both the original open-circuit
and loaded readings. Remove the resistor before normal testing. This is a
diagnostic for weak back-power or retained charge; it does not turn a failed
open-circuit observation into a pass, and it must not become a permanent
bleeder.

These observations establish stable on/off levels, sampling-owned cleanup,
hardware-default-off behavior and absence of obvious steady-state back-power.
They do not establish the 200 ms rail-rise waveform, exact shutdown instant,
brief boot/reset/deep-sleep glitches, MOSFET edge rate or inrush, I2C or 1-Wire
waveform integrity, or the final sleep-current budget. Successful acquisition
through the unchanged production path is only indirect evidence that the rail
is usable after the 200 ms stabilization interval. Those remaining electrical
claims require an oscilloscope or current instrumentation on later hardware.

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
