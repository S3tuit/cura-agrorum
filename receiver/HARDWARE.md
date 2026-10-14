# Receiver hardware

The nominal receiver uses a Raspberry Pi 3 Model B, an Adafruit DS3231 with a
CR1220 backup cell, and a Waveshare **Pico-LoRa-SX1262 EU868** module without a
Raspberry Pi Pico installed. This document owns their production connections.
Test-only fault branches are described with the
[RTC controller tests](hardware/ds3231/tests/i2c_faults/README.md#fault-wiring) and
[radio carrier tests](hardware/radio/tests/carrier/README.md#busy-selector).

## Schematic

Pi pin numbers are **physical 40-pin header numbers**; GPIO numbers are BCM.
U1 numbers belong to the Waveshare **Pico-format socket**, which cannot plug into
the Pi header. Identify both connectors from their markings, not wire colors.

```text
Raspberry Pi 3                                  Modules

 1  3.3 V ---- +3V3 rail ----+----------------- DS3231 VIN
                            +----------------- radio U1.36 Pico_3V3
                            +-- pull-ups below
 6  GND ------ GND rail -----+----------------- DS3231 GND
                            +----------------- radio U1.38 GND

 +3V3 rail -----+----------+
                |          |
              100 nF     10 uF
                |          |
 GND rail ------+----------+

 3  GPIO2 / SDA -------------------------------- DS3231 SDA
 5  GPIO3 / SCL -------------------------------- DS3231 SCL
                                                CR1220 in battery holder

23  GPIO11 / SPI0 SCLK ------------------------- U1.14 GP10 / CLK
19  GPIO10 / SPI0 MOSI ------------------------- U1.15 GP11 / MOSI
21  GPIO9  / SPI0 MISO <------------------------ U1.16 GP12 / MISO
24  GPIO8  / SPI0 CE0 ----+---------------------- U1.5  GP3 / CS
                         +-- 10 kohm -- +3V3
15  GPIO22 / RESET ------+---------------------- U1.20 GP15 / RESET
                         +-- 10 kohm -- +3V3
16  GPIO23 / DIO1 <----------------------------- U1.26 GP20 / DIO1
18  GPIO24 / BUSY <----------------------------- U1.4  GP2 / BUSY
```

With the test carrier fitted, the BUSY selector is in its nominal **1–2** position
and both RTC fault shunts are open. These reproduce the direct connections above.
GPIO17/GPIO27 belong only to the disconnected RTC fault branches.

## Electrical choices

- Power the Pi normally. Both modules and all GPIO logic use the common **3.3 V**
  rail, with a short common ground return. The single 100 nF / 10 uF parallel
  capacitor pair sits near the radio takeoff; keep branches and SPI leads short.
  A polarized 10 uF capacitor has its positive terminal on 3.3 V. Check split
  breadboard rails for continuity.
- The DS3231 board's existing I2C pull-ups serve the bus. No extra 5 V connection
  is present. Fit the CR1220 with its positive side visible; leave BAT, SQW, 32K,
  and RST unconnected. Substituting another RTC module requires checking its
  supply, pull-ups, and battery circuitry.
- Kernel SPI owns CE0 on `/dev/spidev0.0`; do not also request GPIO8 through
  libgpiod. RESET is open drain: LOW asserts reset, release lets the external
  10 kohm resistor pull it high. RESET and CS pull-ups keep them inactive when
  software releases its requests. Check GPIO22/23/24 for other consumers.
- The radio socket's pin 36 directly supplies `Pico_3V3`; leave Pico/VSYS/VBUS
  circuitry unconnected. Fit the correct RF cable and antenna before transmission.
- DIO2 controls the onboard RF switch and DIO3 the oscillator control network;
  neither needs another Pi wire. The backend uses the vendor's DIO2 control,
  1.7 V DIO3 setting, and 5 ms startup. The schematic separately supplies Q1 VCC
  from `Pico_3V3`: the DIO3 setting is not a measurement of oscillator supply
  voltage. Check the actual module variant and oscillator population.

The Linux adapter preserves GPIO bias (`AS_IS`). External pull-ups do not prove
that an idle measured voltage reaches a valid logic threshold; investigate a low
RESET level against actual bias and loading rather than inferring a driver fault
from a DC reading alone.

## Assembly checks

Change wiring only after Linux shutdown and removal of all external Pi/module
power. Keep the coin cell installed except for the explicit
[oscillator-stop procedure](hardware/ds3231/tests/boot_cycle/README.md).
Before powering, verify pin continuity, rail isolation, and absence of unintended
battery/VSYS/VBUS connections. At first power check module supply and idle RESET/CS
levels. Device initialization and RF tests follow these electrical checks.

See [deployment](deploy/README.md) for OS/device configuration, the
[DS3231 guide](hardware/ds3231/README.md) for RTC checks, and the
[radio procedure](hardware/radio/tests/carrier/README.md) for carrier checks.

## Electrical references

- [Waveshare schematic, sheet 1](https://files.waveshare.com/upload/d/d8/Pico-LoRa-SX1262_Sch.pdf)
- [Waveshare demo sources](https://files.waveshare.com/upload/0/08/Pico-LoRa-SX1262-868M_Code.zip): `src/boards/rp2040/sx126x-board.c` and `src/include/pico/board-config.h`
- [Pi header and GPIO](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#gpio-and-the-40-pin-header)
- [Adafruit RTC wiring](https://learn.adafruit.com/adding-a-real-time-clock-to-raspberry-pi/wiring-the-rtc)
