# Firmware hardware

Assembly wiring for the Seeed Studio XIAO ESP32S3 with the kit's B2B-connected
Wio-SX1262 radio, two analog soil probes, two externally powered DS18B20 probes
and a BME280. This document owns the production schematic.

## Power and configuration

Bench power enters through the XIAO's USB-C connector. Take `+3V3_DUT` from its
`3V3` output pad and ground from its `GND` pad. The radio receives the same
always-on supply through B2B. Leave the `5V`/VBUS pad and battery pads unconnected
in this bench assembly; never feed sensors or radio from raw USB/battery voltage.
Use one common ground and no second supply on these nets. Remove every power
and back-power path before changing connectors or the antenna.

Use the **B2B kit radio**, identified by the [kit documentation][kit] and
[Wio schematic][wio-schematic]. Mate its connector without offset and support
the board and antenna cable mechanically. Attach sensor wiring to the **XIAO's
own pads**: the Wio's outer J1/J2 headers expose radio nets, not XIAO GPIO
pass-throughs. Do not join corresponding pads of the two boards with long
through-headers; that would short different GPIOs together. A camera/Sense
expansion board cannot share this occupied B2B connector.

The tables below define the fixed firmware settings implemented by
[sdkconfig defaults](sdkconfig.defaults) and the component Kconfig constraints. Verify each
actual probe/breakout accepts 3.3 V and observe the [soil input limit](#soil-inputs).
DS18B20 ROM identities remain local configuration. Logical channels follow
configured ROMs, not discovery order, depth, wire color or deployment labels.

## Sensor GPIO allocation

| Function | ESP32S3 GPIO | XIAO pad | Required firmware setting |
|---|---:|---|---|
| Soil channel 0 ADC1_CH0 | 1 | D0 | `CONFIG_CURA_SOIL_0_GPIO=1` |
| Soil channel 1 ADC1_CH1 | 2 | D1 | `CONFIG_CURA_SOIL_1_GPIO=2` |
| DS18B20 1-Wire | 4 | D3 | `CONFIG_CURA_DS18B20_GPIO=4` |
| BME280 SDA | 5 | D4 | `CONFIG_CURA_I2C_SDA_GPIO=5` |
| BME280 SCL | 6 | D5 | `CONFIG_CURA_I2C_SCL_GPIO=6` |
| Active-low sensor gate | 44 | D7 | `CONFIG_CURA_SENSOR_POWER_GATE_GPIO=44` |

Viewed from the XIAO's component side with USB-C at the top, D0..D6 form the
left column; D7 is the bottom pad of the right column. Thus both soil inputs and
the shared DS bus stay on one side. Use the [manufacturer pinout][xiao-pinout]
to identify pads before attaching the radio; the view from underneath is mirrored.

Leave D2/GPIO3 unused because it is a strapping pin. D6/GPIO43 is UART0 TX and
can emit boot messages, so it must not drive the active-low gate. D7 is dedicated
to the gate: do not attach UART0 RX or a serial adapter there. Use native USB
for the console, reserving GPIO19/20. D8..D10 are occupied by radio SPI through
B2B; GPIO38..42 serve its other controls. These reservations also exclude a
second peripheral from those pads.

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
                                      +---- R1 100 ohm ---- D7 / GPIO44
                                      +---- R2 47 k ------- +3V3_DUT

                    Q1 pin 3, DRAIN ---- +3V3_SW ----+---- TP_SW
                                                      +---- C3 100 nF ---- GND
                                                      +---- R12 100 k ---- GND
                                                      +----> switched sensors
```

The AO3401A source is connected to `+3V3_DUT`, its drain to `+3V3_SW`, and its
gate to the externally pulled-up control node. This orientation makes its body
diode oppose ordinary source-to-load current while the MOSFET is off. R2 creates
the hardware-default-off state. Firmware must drive GPIO44 as open drain: low
turns Q1 on and release lets R2 turn it off. R1 limits the brief GPIO/gate
charging current without materially delaying the 200 ms stabilization period.
Before sleep/reset, release the gate and leave the pad as an input with both
internal pulls disabled. R2, rather than retained GPIO output state, keeps the
rail off while the MCU sleeps or is held in reset.

R12 is a permanent rail-to-ground discharge resistor. It draws approximately
33 uA and dissipates 0.109 mW at 3.3 V while the rail is on; its current falls
with the off-state voltage.
It remains fitted during sampling, reset, held reset and deep sleep.

Do not place bulk capacitance on `+3V3_SW`. TP_GATE, TP_SW, TP_3V3 and at least
one adjacent ground test point must be accessible to multimeter probes while
the development board remains mounted.

R12 discharges the rail independently of sensor loading. A low settled voltage
can conceal weak back-power; it does not establish the sleep-current budget.

### Soil inputs

```text
J_SOIL0                       R3 1 k
 1 +3V3_SW     2 SOIL0_OUT ----/\/\/----+---- D0 / GPIO1 / TP_ADC0
 3 GND                                +---- C4 100 nF ---- GND

J_SOIL1                       R4 1 k
 1 +3V3_SW     2 SOIL1_OUT ----/\/\/----+---- D1 / GPIO2 / TP_ADC1
 3 GND                                +---- C5 100 nF ---- GND
```

Place R3/C4 and R4/C5 close to the XIAO ADC pads. Each 1 kohm/100 nF pair filters
noise; it neither divides voltage nor clamps an overvoltage.

**Direct soil inputs must remain within 0..2.9 V.** This is the ESP32S3's
documented effective measurement range at the selected 12 dB attenuation, not
the 3.3 V supply voltage; see [Espressif's ADC guidance][s3-hardware]. Leave each
probe's signal disconnected from the ADC until its maximum output has been
checked over the intended supply and measurement conditions. A probe exceeding
this range needs a reviewed input divider and matching voltage conversion before
connection; no divider is fitted in this schematic. The ADC/filter parts can be
wired now, but soil acquisition and calibration still require S3 validation.

### DS18B20 bus

```text
                              R9 4.7 k
 +3V3_SW ----------------------/\/\/----+
                                        |
 D3 / GPIO4 -------------------------- DQ+---- TP_DQ
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
Firmware must release the DQ pad with both internal pulls disabled before
removing sensor power, so it cannot back-power the probes through the bus.
No per-probe bypass capacitor is fitted on the carrier; C3 is the shared
switched-rail bypass capacitor. Add local capacitance only if cable testing
provides evidence that it is needed.

### BME280

```text
 +3V3_DUT -------------------------------- J_BME pin 1, 3V3
 GND -------------------------------------- J_BME pin 2, GND
 +3V3_DUT ---- JP_I2C_PULLUPS ----+---- R10 4.7 k ----+---- D4 / GPIO5
                                  |                  +---- J_BME pin 3, SDA
                                  |
                                  +---- R11 4.7 k ----+---- D5 / GPIO6
                                                     +---- J_BME pin 4, SCL

 TP_SDA on GPIO5; TP_SCL on GPIO6
```

JP_I2C_PULLUPS is normally fitted so the bus remains electrically defined. Verify the actual
BME280 breakout accepts 3.3 V directly and inspect its onboard pull-ups before
assembly. If its parallel pull-ups make the combined resistance unsuitable,
open JP_I2C_PULLUPS; record that jumper position in the test session. The
four-pin connector assumes the breakout already fixes the BME280 address to
`0x76`, configures it for I2C and provides the local BME280 supply decoupling
recommended by Bosch. No additional BME280 bypass capacitor is fitted on the
carrier; C1 and C2 remain on the always-on carrier rail.

The BME remains powered while the soil/DS rail is off. Firmware must use forced
conversion and verify sleep afterward. Its board must select I2C at `0x76`.

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

Use the kit's 862..930 MHz Wio-SX1262 and the 868 MHz antenna for the LoRa v2
profile. The complete radio power/control connection is through B2B; no separate
MCU-to-radio wiring harness is required. The module stays on **always-on
`+3V3_DUT`, never `+3V3_SW`**.

### Radio connectors and pin allocation

| Signal | ESP32S3 GPIO | XIAO pad, where exposed | Wio breakout contact |
|---|---:|---|---|
| SCLK | 7 | D8 | J2.6 |
| MOSI | 9 | D10 | J2.4 |
| MISO | 8 | D9 | J2.5 |
| CS / NSS, active low | 41 | B2B | J1.4 |
| RESET, active low | 42 | B2B | J1.3 |
| BUSY | 40 | B2B | J1.2 |
| DIO1, rising-edge IRQ | 39 | B2B | J1.1 |
| RF enable / RF_SW | 38 | B2B | J1.5 |
| Always-on 3.3 V | — | 3V3 | J2.3 |
| Ground | — | GND | J2.2 |

These connections follow the [Wio schematic][wio-schematic] and the numerical
pinout in [Seeed's BSP][wio-bsp]. J1/J2 contact numbers belong to the Wio board,
not the XIAO's D labels; identify pin 1 from the actual board and schematic
before probing. Wio J2.1 is VIN, not regulated 3.3 V. Keep its breakout headers
free of sensor wiring and use them as probe access without bridging contacts.
SPI requires mode 0 at 8 MHz and 3.3 V logic. The Wio board includes the reset
pull-up; no additional radio signal resistors are specified for the fixed B2B
connection. A removable DIO1 fault link would require a separate test fixture.

### RF path and radio supply

GPIO38 must be high for both TX and RX. It enables the module's RF switch/path;
SX1262 DIO2 separately selects transmit versus receive internally. GPIO38 low
disables the RF path but does not remove module power or replace the radio sleep
command. Firmware must request radio sleep and leave RF enable inactive before
MCU deep sleep, including best-effort cleanup after partial initialization.

Use a DIO3-controlled 3.0 V TCXO with a 5 ms startup allowance, following
[Seeed's BSP][wio-bsp]. DIO2/DIO3 are internal module connections, not extra
XIAO wires. The [module datasheet][wio-datasheet] requires at least 200 mV of
headroom between TCXO voltage and VCC; the selected TCXO setting therefore
requires at least 3.2 V at the radio during operation.

Select DC-DC regulation, supported by the documented module hardware. Seeed's
BSP selects LDO, but does not explain that choice; the hardware specification
and [Semtech regulator guidance][sx1262-datasheet] support selecting DC-DC to
reduce core consumption. The PA setting follows Seeed's expected 14 dBm
entry. Actual RF output, supply behavior and sleep current require measurement;
neither the example's PA table nor an RF-enable level establishes them.

Connect the 868 MHz antenna to the **Wio module's** IPEX socket, directly or
through the matching pigtail, with power removed. The XIAO's separate antenna
socket is for Wi-Fi/Bluetooth. Secure the cable against strain on the RF socket
and attach the LoRa antenna before powering the radio. The coax shield supplies
its RF return; never put a meter or jumper wire in the RF path. The
[RF operating envelope](../tests/rf/OPERATING_ENVELOPE.md) and
[radio fixture](hardware/radio/tests/component/FIXTURE.md) own controlled RF
preflight and fault access. Physical S3 sensor, RF, DC-DC supply and sleep-current
qualification remain required; build and host results do not establish them.

## Component references

- [XIAO ESP32S3 pinout and power][xiao-pinout]
- [ESP32S3 hardware design and ADC limits][s3-hardware]
- [AO3401A](https://www.aosmd.com/products/mosfets/p-channel-mosfets-8v-60v/ao3401a)
- [DS18B20](https://www.analog.com/media/en/technical-documentation/data-sheets/ds18b20.pdf)
- [BME280](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf)
- [XIAO/Wio B2B kit][kit], [Wio schematic][wio-schematic] and [module datasheet][wio-datasheet]
- [Seeed XIAO/Wio BSP][wio-bsp] and [Semtech SX1261/2 datasheet][sx1262-datasheet]

[kit]: https://wiki.seeedstudio.com/xiao_esp32s3_%26_wio_SX1262_kit_for_meshtastic/
[xiao-pinout]: https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/#hardware-overview
[s3-hardware]: https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32s3/schematic-checklist.html#adc
[wio-schematic]: https://files.seeedstudio.com/products/SenseCAP/Wio_SX1262/Schematic_Diagram_Wio-SX1262_for_XIAO.pdf
[wio-datasheet]: https://files.seeedstudio.com/products/SenseCAP/Wio_SX1262/Wio-SX1262_Module_Datasheet.pdf
[wio-bsp]: https://github.com/Seeed-Studio/one_channel_hub/blob/4cc771ac02da1bd18be67509f6b52d21bb0feabd/components/smtc_ral/bsp/sx126x/seeed_xiao_esp32s3_devkit_sx1262.c
[sx1262-datasheet]: https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1198/DS_SX1261_2_V2-2.pdf
