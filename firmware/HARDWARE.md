# Firmware hardware

Nominal wiring for the ESP32-C6-DEVKITM-1-N4 node with two analog soil probes,
two externally powered DS18B20 probes, a BME280, and a Waveshare
Pico-LoRa-SX1262-868M radio. This document owns the production schematic.
[Test fixtures](tests/README.md) describe temporary connections and measurements;
[deployment](deploy/README.md) owns provisioning and flashing.

## Power and configuration

Power the board through its **UART** USB connector, leave J5 fitted, and take
`+3V3_DUT` from J1 pin 1 and GND from J1 pin 13. Use one common ground and no
second supply on these nets. Remove all power/back-power paths before changing
connectors or the antenna. Connector numbers specify functions, not wire colors.
Verify each actual probe/breakout accepts 3.3 V; soil outputs must stay within
0..3.3 V (the selected ADC acquisition assumes at most 3 V).

[Production sdkconfig defaults](sdkconfig.defaults) select this board's exposed
pins. Component Kconfig defaults differ for BME SDA/SCL and radio MISO/CS;
check resolved `sdkconfig` against the assembled board before flashing. DS18B20
ROM identities are local configuration, discovered from the attached probes;
portable defaults are unprovisioned. Logical channels follow ROM, not discovery
order, depth, wire color, or a particular deployment's labels.

## Sensor GPIO allocation

| Function | ESP32-C6 GPIO | DevKitM-1 header | Firmware setting |
|---|---:|---|---|
| Soil channel 0 ADC | 0 | J1 pin 7 | `CONFIG_CURA_SOIL_0_GPIO=0` |
| Soil channel 1 ADC | 1 | J1 pin 8 | `CONFIG_CURA_SOIL_1_GPIO=1` |
| Active-low sensor gate | 2 | J1 pin 3 | `CONFIG_CURA_SENSOR_POWER_GATE_GPIO=2` |
| DS18B20 1-Wire | 3 | J1 pin 4 | `CONFIG_CURA_DS18B20_GPIO=3` |
| BME280 SDA | 21 | J3 pin 6 | `CONFIG_CURA_I2C_SDA_GPIO=21` |
| BME280 SCL | 22 | J3 pin 5 | `CONFIG_CURA_I2C_SCL_GPIO=22` |

GPIO4/5/8/9/15 are reserved strapping pins; GPIO12/13 remain available for native
USB and GPIO16/17 for UART. The remaining seven exposed pins below serve radio.

## Sensor circuit

### Power gate

```text
 +3V3_DUT ----+---- TP_3V3
              +---- C1 10 uF ---- GND
              +---- C2 100 nF --- GND
              +------------------> BME280 + radio
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

Do not place bulk capacitance on `+3V3_SW`. TP_GATE, TP_SW, TP_3V3 and at least
one adjacent ground test point must be accessible to multimeter probes while
the development board remains mounted.

R12 discharges the rail independently of sensor loading. A low settled voltage
can conceal weak back-power; it does not establish the sleep-current budget.

### Soil inputs

```text
J_SOIL0                       R3 1 k
 1 +3V3_SW     2 SOIL0_OUT ----/\/\/----+---- GPIO0 / TP_ADC0
 3 GND                                +---- C4 100 nF ---- GND

J_SOIL1                       R4 1 k
 1 +3V3_SW     2 SOIL1_OUT ----/\/\/----+---- GPIO1 / TP_ADC1
 3 GND                                +---- C5 100 nF ---- GND
```

Place the series resistors and ADC capacitors close to the C6 header. They
provide input filtering and modest protection while settling before sampling.
The [sensor fixture](hardware/sensors/tests/carrier/FIXTURE.md) adds removable
soil links and reference injection; those divider parts are unnecessary for
nominal production wiring.

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
Q1 turns off. Probe wire colors are not authoritative;
identify their functions before terminating them and record each 64-bit ROM.
No per-probe bypass capacitor is fitted on the carrier; C3 is the shared
switched-rail bypass capacitor. Add local capacitance only if cable testing
provides evidence that it is needed.

### BME280

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

JP_I2C_PULLUPS is normally fitted so the bus remains electrically defined. Verify the actual
BME280 breakout accepts 3.3 V directly and inspect its onboard pull-ups before
assembly. If its parallel pull-ups make the combined resistance unsuitable,
open JP_I2C_PULLUPS; record that position as part of the carrier revision. The
four-pin connector assumes the breakout already fixes the BME280 address to
`0x76`, configures it for I2C and provides the local BME280 supply decoupling
recommended by Bosch. No additional BME280 bypass capacitor is fitted on the
carrier; C1 and C2 remain on the always-on carrier rail.

The BME remains powered while the soil/DS rail is off. Firmware uses forced
conversion and verifies sleep afterward. Its board must select I2C at `0x76`.

### Sensor parts

| Reference | Part/value | Purpose |
|---|---|---|
| Q1 | AO3401A, SOT-23 | High-side P-MOSFET |
| R1 / R2 | 100 ohm / 47 kohm | Gate series resistor / default-off pull-up |
| R3 / R4 | 1 kohm | ADC series resistors |
| R9–R11 | 4.7 kohm | Switched 1-Wire / always-on I2C pull-ups |
| R12 | 100 kohm | Permanent switched-rail discharge |
| C1 | 10 uF | Always-on input bulk capacitance |
| C2 / C3 | 100 nF | Always-on / switched-rail bypass |
| C4 / C5 | 100 nF | ADC filtering |
| J_SOIL0/1, J_DS0/1 | Removable three-contact connectors | Sensor power, signal, ground |
| J_BME | Removable four-contact connector | Always-on supply, ground, SDA, SCL |
| JP_I2C_PULLUPS | Header and shunt | Carrier pull-up selection |

R5–R8 are reserved for the optional test reference dividers.

## Radio circuit

Use the Waveshare HF module with its DIO2 RF switch, DIO3-powered 1.7 V TCXO
and DC-DC regulator, as configured by the production backend. Check the
[manufacturer schematic](https://files.waveshare.com/upload/d/d8/Pico-LoRa-SX1262_Sch.pdf)
against the actual module and its `ANT_SW A (3V3)` population. Leave
battery/Pico/VBUS/VSYS connections absent and the battery switch off.

### Radio connectors and pin allocation

| J_RAD_N contact| Signal | Direction relative to C6 | Waveshare U1 physical contact | Pico label on module | C6 GPIO / DevKitM-1 header |
|---|---|---|---:|---|---|
| 1 | Always-on 3.3 V | Supply to radio | 36 | `3V3` / `Pico_3V3` | J1 pin 1, `+3V3_DUT` |
| 2 | GND | Common node ground | 38 | `GND` | J1 pin 13 |
| 3 | SCLK | Output | 14 | `GP10` / CLK | GPIO6, J1 pin 10 |
| 4 | MOSI | Output | 15 | `GP11` | GPIO7, J1 pin 11 |
| 5 | MISO | Input | 16 | `GP12` | GPIO14, J1 pin 12 |
| 6 | CS, active low | Output | 5 | `GP3` | GPIO23, J3 pin 4 |
| 7 | RESET, active low | Output | 20 | `GP15` | GPIO18, J3 pin 9 |
| 8 | BUSY | Input | 4 | `GP2` | GPIO19, J3 pin 8 |
| 9 | DIO1 | Input, rising-edge IRQ | 26 | `GP20` | GPIO20, J3 pin 7 |

`J_HOST_N` contacts 1..7 carry SCLK, MOSI, MISO, CS, RESET, BUSY and DIO1.
Radio power and ground are independent of this removable signal connector.
Pico GP labels identify module contacts, not C6 GPIOs; counting from the module's
other side mirrors the header. SPI uses mode 0 at 8 MHz and 3.3 V logic.

```text
+3V3_DUT ----+---- CN1 10 uF ---- GND
             +---- CN2 100 nF --- GND
             +------------------ J_RAD_N.1 / module U1.36
GND ---------------------------- J_RAD_N.2 / module U1.38

GPIO6  SCLK  ---- J_HOST_N.1 ---- J_RAD_N.3 / U1.14
GPIO7  MOSI  ---- J_HOST_N.2 ---- J_RAD_N.4 / U1.15
GPIO14 MISO  ---- J_HOST_N.3 ---- J_RAD_N.5 / U1.16
GPIO23 CS    ---- J_HOST_N.4 ---- J_RAD_N.6 / U1.5
GPIO18 RESET ---- J_HOST_N.5 ---- J_RAD_N.7 / U1.20
GPIO19 BUSY  ---- J_HOST_N.6 ---- J_RAD_N.8 / U1.4
GPIO20 DIO1  ---- J_HOST_N.7 --+-- J_RAD_N.9 / U1.26
                              +-- RN5 10 k ---- GND
```

The radio uses **always-on `+3V3_DUT`, never `+3V3_SW`**. Fit CN1/CN2 beside
its supply/ground contacts and RN5 on the MCU side of any removable DIO1 link.
No external CS, RESET, BUSY or MISO resistors are fitted. The radio has an
internal reset pull-up; MISO is high impedance with CS inactive. The backend
disables internal BUSY/DIO1 pulls. Keep SPI/control wiring short: DC continuity
does not establish signal integrity at 8 MHz.

Provide accessible `TP_N_3V3`, `TP_N_GND`, `TP_N_CS`, `TP_N_RESET`, `TP_N_BUSY`,
`TP_N_MISO`, and MCU-side `TP_N_DIO1`. Fit CN1/CN2 with at least 6.3 V ratings
and observe CN1 polarity if polarized. RN5 draws about 0.33 mA while DIO1 is high.
Test-only DIO1/absence jumpers and static preflight belong to the
[radio fixture](hardware/radio/tests/component/FIXTURE.md).

Connect the 868 MHz antenna through its IPEX-to-SMA pigtail with power removed,
and secure the cable against strain on the RF socket. Attach the antenna before
powering a connected radio. The coax shield supplies its RF return; never put
a meter or jumper wire in the RF path. The configuration and antenna assumptions
for controlled RF work belong to the [RF operating envelope](../tests/rf/OPERATING_ENVELOPE.md).

## Component references

- [ESP32-C6-DevKitM-1 headers](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32c6/esp32-c6-devkitm-1/user_guide.html#header-block)
- [ESP32-C6 hardware design](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32c6/schematic-checklist.html)
- [AO3401A](https://www.aosmd.com/products/mosfets/p-channel-mosfets-8v-60v/ao3401a)
- [DS18B20](https://www.analog.com/media/en/technical-documentation/data-sheets/ds18b20.pdf)
- [BME280](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf)
- [Waveshare Pico-LoRa-SX1262](https://www.waveshare.com/wiki/Pico-LoRa-SX1262)
