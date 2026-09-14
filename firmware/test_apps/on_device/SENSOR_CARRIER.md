# ESP32-C6 sensor hardware-test carrier

Status: approved carrier and fixture-state wiring, revised 2026-09-11 to include
permanent R12 (100 kohm, `+3V3_SW` to GND).

This carrier supports the `node_sensors` hardware cases in
[`../../TESTING.md`](../../TESTING.md), which owns test coverage and the
[electrical measurement procedure](../../TESTING.md#node_sensors-manual-electrical-cases).
The device under test is the
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

R12 is a permanent rail-to-ground discharge resistor. It draws approximately
33 uA and dissipates 0.109 mW
at 3.3 V while the rail is on; its current falls with the off-state voltage.
It remains fitted during sampling, reset, held reset and deep sleep.

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

Use 1% resistors. These nominal divider values are setup targets; fresh
measurements and the ADC comparison criteria are defined in
[TESTING.md](../../TESTING.md#node_sensors-declared-fixture-states-and-test-mapping).

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
from the nominal fixture as **compromised, cause undetermined**. This is an
operator hardware decision, not a firmware ROM rejection rule.

DS1 remains ROM `7E000000540FA728`. The replacement's physical bench label is
DS2, ROM `DF00000050F93828`, confirmed by the operator on September 11. The
operator's current ignored sdkconfig assigns DS2 to logical channel 0 and
retains DS1 on logical channel 1. The physical label DS2 does not create a
third firmware channel; checked-in defaults remain unprovisioned and there is
no ROM denylist.

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
```

The [fixture test mapping](../../TESTING.md#node_sensors-declared-fixture-states-and-test-mapping)
defines coverage and expected results for every state. Both configured probes
remain connected during the nominal identity procedure; only their complete
physical connector positions are exchanged with power removed.

### `missing_ds0`

The complete connector to remove is selected by the configured logical ROM.
On the current bench, logical DS0 is the replacement physically labeled DS2,
ROM `DF00000050F93828`; keep DS1, `7E000000540FA728`, connected. Power off before
changing connectors. Use the [missing-probe command and post-return observation](../sensor_carrier/README.md#missing-ds-probes-and-nominal-restoration).

```text
JP_SOIL0 [X]   J_SOIL0 [X]     air-exposed probe -> GPIO0
JP_SOIL1 [X]   J_SOIL1 [X]     air-exposed probe -> GPIO1
J_DS0    [ ]   J_DS1    [X]    configured logical channel 0 absent
J_BME    [X]   JP_I2C_PULLUPS [X]
JP_REF_ENABLE [ ]               J_INJECT0/1 have no leads
```

### `missing_ds1`

Remove the complete connector for logical DS1, ROM `7E000000540FA728`, with
power off. Keep logical DS0 (physical DS2, `DF00000050F93828`) connected. Use the
[symmetric command](../sensor_carrier/README.md#missing-ds-probes-and-nominal-restoration),
then restore both probes with power off and run the nominal restoration check.

```text
JP_SOIL0 [X]   J_SOIL0 [X]     air-exposed probe -> GPIO0
JP_SOIL1 [X]   J_SOIL1 [X]     air-exposed probe -> GPIO1
J_DS0    [X]   J_DS1    [ ]    configured logical channel 1 absent
J_BME    [X]   JP_I2C_PULLUPS [X]
JP_REF_ENABLE [ ]               J_INJECT0/1 have no leads
```

### `missing_bme280`

Remove power before removing the complete four-wire J_BME connector. Preserve
both configured DS identities, both soil probes/shunts and permanent R12. Keep
JP_I2C_PULLUPS fitted as specified above. The implemented fixture preflight
requires a NACK-specific absence result at 0x76; an unpowered/floating or stuck
bus timeout cannot accept this state. Run the
[guided missing-BME acquisition](../sensor_carrier/README.md#missing-bme280-and-nominal-restoration),
then restore nominal wiring with power removed and repeat its acquisition and
read-only BME sleep observation. Wiring and meter observations remain operator-owned.

```text
JP_SOIL0 [X]   J_SOIL0 [X]     air-exposed probe -> GPIO0
JP_SOIL1 [X]   J_SOIL1 [X]     air-exposed probe -> GPIO1
J_DS0    [X]   J_DS1    [X]    both configured ROMs present
J_BME    [ ]   JP_I2C_PULLUPS [X]
JP_REF_ENABLE [ ]               J_INJECT0/1 have no leads
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
```

Power down before changing from position A to position B. Follow the
[ADC reference procedure](../sensor_carrier/README.md#adc-reference-positions-a-and-b)
for fresh measurements and acquisition at each position.

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
