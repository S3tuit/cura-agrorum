# Receiver hardware-test carrier

This document owns the common receiver test schematic, physical connections
and fixture states. The current carrier uses the recorded Raspberry Pi 3
Model B and Adafruit DS3231 with a CR1220 battery. The operator has confirmed
assembly of the two removable fault shunts while externally unpowered.
Qualification results and the pilot recovery limitation are recorded in
[DS3231 LIMITATION.md](ds3231/LIMITATION.md).

The [SX1262 extension below](#proposed-sx1262-extension) is a proposed design.
The operator confirmed on 2026-09-16 that no radio is attached, and approved
the shared power rails and manual BUSY selector on 2026-09-17. The operator
subsequently reported unpowered multimeter continuity checks with no apparent
issue. Exact readings and powered preflight are not yet recorded; this report
does not establish electrical, production-adapter or RF acceptance.

[TESTING.md](../TESTING.md) owns required coverage and execution rules.
[The DS3231 installation guide](ds3231/README.md) owns bench OS setup,
[OPERATOR_TESTS.md](ds3231/OPERATOR_TESTS.md) owns the physical RTC acceptance
sequence, and [RTC_FAULT_TESTS.md](../tests/hardware/RTC_FAULT_TESTS.md) owns the
production-adapter fault execution and restoration procedure. Result archives
retain their original carrier/source bindings.

## Design rules

- Power the Pi from its normal supply. Feed the common `+3V3` rail from Pi
  physical pin 1 and the common `GND` rail from physical pin 6. Both DS3231 VIN
  and radio U1.36 use `+3V3`; both module grounds use `GND`. The CR1220 supplies
  RTC backup power.
- Fit one 100 nF capacitor and one 10 uF capacitor in parallel between these
  two rails. Keep their connections and the module power/ground branches short;
  place the pair near the radio's rail takeoff. For a polarized 10 uF part,
  connect its positive terminal to `+3V3` and negative terminal to `GND`.
- Change wiring or shunts only after Linux shutdown and disconnection of all
  external Pi/module power. Keep the coin cell installed except when the
  oscillator-stop procedure explicitly requires its removal.
- Keep the RTC's power and I2C connections assembled. Two removable shunts
  connect the spare fault GPIOs to branches of the existing I2C bus.
- Open both fault shunts for normal hardware tests. With the fault branches
  electrically disconnected, those tests need no checks of GPIO17/GPIO27.
- Fit both shunts for the controller-fault suite. That suite exclusively owns
  the two spare GPIOs and checks their availability before driving them.
- Fault outputs use open drain: LOW pulls the selected bus line to ground;
  RELEASE removes that drive. Never configure a connected fault pin as a
  push-pull high output. Preserve existing pull settings during requests.
- Connector functions and Pi physical pin numbers are authoritative. Wire
  colours and orientation of an unlabelled header are not pin identification.
- Keep branch leads short. The assembled carrier remains the time-test carrier
  until the proposed radio extension has been fitted and checked by the operator.

## Pin and connector allocation

Pi numbers below refer to the **physical 40-pin header**, with BCM GPIO numbers
listed separately. Locate physical pin 1 from the board's header marking.

| Net/function | Pi physical pin | BCM GPIO | Connection/owner |
|---|---:|---:|---|
| `+3V3` | 1 | — | Shared rail: DS3231 VIN, radio U1.36, pull-ups and manual BUSY selector |
| `GND` | 6 | — | Shared rail: DS3231 GND and radio U1.38 |
| `RTC_SDA` | 3 | 2 | DS3231 SDA; kernel I2C controller |
| `RTC_SCL` | 5 | 3 | DS3231 SCL; kernel I2C controller |
| `FAULT_SCL` | 11 | 17 | Through `JP_RTC_SCL_FAULT` to `RTC_SCL` |
| `FAULT_SDA` | 13 | 27 | Through `JP_RTC_SDA_FAULT` to `RTC_SDA` |

`JP_RTC_SCL_FAULT` and `JP_RTC_SDA_FAULT` are two-pin headers with removable
shunts, or equivalent clearly labelled removable links. Their open positions
must break the spare-GPIO branches without breaking the RTC's I2C wiring.
GPIO17 and GPIO27 have no other carrier connections. Their ownership is
reserved for the controller-fault fixture while the shunts are fitted.

Use the Adafruit module's battery holder with CR1220 positive side visible.
Leave BAT, SQW, 32K and RST unconnected. This schematic applies to the recorded
Adafruit board; a different RTC module requires review of its power and pull-up
circuitry before substitution.

## Common schematic

```text
 Raspberry Pi 3             Shared power rails

 pin 1  +3.3 V ---- +3V3 rail ----+------------ DS3231 VIN
                                +------------ radio U1.36 / Pico_3V3
                                +------------ R_CS, R_RESET, BUSY selector pin 3

 pin 6  GND ------ GND rail -----+------------ DS3231 GND
                                +------------ radio U1.38 / GND

 +3V3 rail --------+---------+
                  |         |
             C_RAIL_HF   C_RAIL
               100 nF     10 uF
                  |         |
 GND rail --------+---------+

                                               Adafruit DS3231

 pin 5  GPIO3/SCL --------+-------------------- SCL
                         |
                         +-- JP_RTC_SCL_FAULT -- pin 11 GPIO17

 pin 3  GPIO2/SDA --------+-------------------- SDA
                         |
                         +-- JP_RTC_SDA_FAULT -- pin 13 GPIO27

                                                CR1220 in battery holder
```

The fault shunts are **parallel branches**, not series links in the RTC bus.
The board/module's existing I2C pull-ups serve the common bus. No extra pull-up
or connection to 5 V is added by these fault branches.
The two capacitors above are the single shared rail pair, not separate pairs
for each module. Check rail continuity if the carrier uses split breadboard
rails; a common label does not bridge a physical break.

## Fixture states

`OPEN` means the removable shunt is absent; `FITTED` means its two contacts are
joined. GPIO state is controlled by the test supervisor, not by moving shunts
while powered.

| State | SCL shunt | SDA shunt | GPIO17 | GPIO27 | Use |
|---|---|---|---|---|---|
| `nominal` | OPEN | OPEN | Outside this test's control | Outside this test's control | Ordinary receiver hardware suites and RTC operator tests |
| `rtc_fault_ready` | FITTED | FITTED | Input/released | Input/released | Fault-suite setup, between cases and software restoration |
| `rtc_scl_low` | FITTED | FITTED | Open-drain LOW | Input/released | Held-clock read/write cases |
| `rtc_sda_low` | FITTED | FITTED | Input/released | Open-drain LOW | Held-data read/write cases |

Both shunts stay fitted for all four controller-fault cases. The supervisor
returns to `rtc_fault_ready` between cases. At suite completion the operator
shuts down/disconnects external power and opens both shunts, returning to
`nominal`; the carrier wiring remains assembled.

For physical oscillator-stop creation, begin with the `nominal` shunt state
and follow RTC-04's exact external-power and coin-cell sequence. Battery
retention and recovery follow RTC-03/RTC-05. The state table does not replace
those procedures or authorize additional RTC writes.

## Fault-suite checks and evidence

Only the controller-fault suite checks/claims GPIO17/GPIO27. It requires the
operator's exact wiring confirmation, the current boot ID, destructive opt-in,
a dedicated marked data root and the existing exclusive time-test lock.
The privilege belongs to the test supervisor; production RTC access still
uses the installed kernel driver and the pinned native write helper.

Before fault assertion, both spare lines must be unused inputs and the actual
SDA/SCL pads must read high. The supervisor records the read-only GPIO state,
then verifies the selected bus pad reads low when asserted and records its
level after release. A released injection GPIO does not guarantee a high bus;
the fault procedure checks for high pads after bounded read recovery.
It requests `Bias.AS_IS` throughout, preserving the pull configuration
instead of trying to reconstruct it. On the current Pi, libgpiod reports
`Bias.UNKNOWN`; that value is retained as an observation, not interpreted as
"no pull" or used as a requested bias setting.

Software cleanup releases the open-drain output, restores input direction and
verifies both spare GPIOs are unused inputs before read recovery. Both bus pads
must be high after recovery and before any restoration write. The
RTC-only fixture leaves normal Chrony running with unchanged process identity
and configuration. Read cases verify RTC health without a cleanup write; write
cases explicitly restore and read back the RTC after authorized submission.
Device permissions and helper cleanup are verified even if RTC recovery fails;
failed or uncertain cleanup stops dependent cases and retains its evidence.
Final physical restoration requires confirmation that both shunts are open
and the coin cell/original RTC wiring remain installed, followed by a normal
boot and the safe time suite. Software cannot establish that a shunt has been
physically removed.

Every run records carrier revision, selected state, exact source manifest,
operator confirmations, kernel/controller identity, fault outcomes and
restoration. Existing RTC/OSF evidence predates this connected fault carrier
and must not be relabelled as its controller qualification.

## Proposed SX1262 extension

Use the Waveshare **Pico-LoRa-SX1262, EU868 variant** as an SPI peripheral,
with no Raspberry Pi Pico installed. This is a wired connection to its
Pico-format socket; that socket is not compatible with the Pi 40-pin header.
The proposal preserves BCM GPIO2/3 for the RTC and GPIO17/27 for RTC faults.

The electrical source is the [Waveshare schematic, sheet 1](https://files.waveshare.com/upload/d/d8/Pico-LoRa-SX1262_Sch.pdf).
Its `Pico_3V3` net supplies the radio, RF-switch supply and oscillator supply.
The proposal therefore feeds **3.3 V directly into socket pin 36**, with a
common ground.

### Proposed pin allocation

The first two pin columns identify the **Pi header**; the last two identify
the **Waveshare Pico-format socket U1**. `GPx` labels here are socket labels,
not Pi BCM assignments. Locate both connectors from their markings rather
than assuming the same orientation or numbering.

| Net | Pi physical pin | Pi BCM/function | Module U1 pin | Module label |
|---|---:|---|---:|---|
| `+3V3` | 1, through shared rail | 3.3 V supply | 36 | `3V3 / Pico_3V3` |
| `GND` | 6, through shared rail | Ground | 38 | `GND` |
| `RADIO_SCLK` | 23 | GPIO11 / SPI0 SCLK | 14 | `GP10 / CLK` |
| `RADIO_MOSI` | 19 | GPIO10 / SPI0 MOSI | 15 | `GP11 / MOSI` |
| `RADIO_MISO` | 21 | GPIO9 / SPI0 MISO | 16 | `GP12 / MISO` |
| `RADIO_NSS` | 24 | GPIO8 / SPI0 CE0 | 5 | `GP3 / LoRa_CS` |
| `RADIO_NRESET` | 15 | GPIO22, active-low reset | 20 | `GP15 / LoRa_RESET` |
| `RADIO_BUSY_RAW` | 18, through selector below | GPIO24, input | 4 | `GP2 / LoRa_BUSY` |
| `RADIO_DIO1` | 16 | GPIO23, rising-edge input | 26 | `GP20 / DIO1` |

The [Pi SPI0 mapping](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#spi-hardware)
fixes the four SPI pins. GPIO22/23/24 are the approved local
allocations and must be checked for other consumers on the actual Pi before
requests. Kernel SPI owns CE0; the backend must not also request GPIO8 through
libgpiod. The intended device is `/dev/spidev0.0`.

### Connection schematic

```text
Pi 3 Model B physical header                  Waveshare socket U1

+3V3 rail (from pin 1) ---------------------- 36  Pico_3V3
GND rail  (from pin 6) ---------------------- 38  GND
100 nF + 10 uF across these rails: see common schematic above.

23  GPIO11 / SCLK --------------------------- 14  GP10 / CLK
19  GPIO10 / MOSI --------------------------- 15  GP11 / MOSI
21  GPIO9  / MISO <-------------------------- 16  GP12 / MISO
24  GPIO8  / CE0 ----+-----------------------  5  GP3 / CS
                    +-- R_CS 10 kohm -- +3V3 rail

15  GPIO22 RESET ----+----------------------- 20  GP15 / RESET
                    +-- R_RESET 10 kohm -- +3V3 rail
16  GPIO23 DIO1 <---------------------------- 26  GP20 / DIO1

18  GPIO24 BUSY <--- JP_RADIO_BUSY pin 2
                     pin 1 <----------------  4  GP2 / BUSY (raw)
                     pin 3 ----------------- +3V3 rail

JP_RADIO_BUSY: one shunt only; 1-2 nominal, 2-3 BUSY held high.
```

Use the shared rail capacitor pair, a short common ground return and short
SPI leads. RESET is configured as an
open-drain output: LOW asserts reset and release lets `R_RESET` pull it high.
The external pull-ups keep reset released and chip select inactive after
the software releases its requests. Supply and GPIO logic are 3.3 V.
Investigation of the observed RESET voltage and GPIO bias is deferred to future
hardware work.
Fit the board's correct RF cable and antenna before any transmission.

DIO2 and DIO3 are on-board controls, not extra Pi GPIO wires. The schematic
routes DIO2 to the RF switch and DIO3 into the oscillator control network;
it separately shows Q1 VCC on `Pico_3V3`. The
[Waveshare demo archive](https://files.waveshare.com/upload/0/08/Pico-LoRa-SX1262-868M_Code.zip),
`src/boards/rp2040/sx126x-board.c` and `src/include/pico/board-config.h`,
selects DIO2 RF-switch control, a DIO3 setting of 1.7 V and 5 ms oscillator
startup. These are the proposed module settings with vendor-source provenance,
not a measured claim about the board in hand. Check the actual oscillator
population and control rail when qualifying it; do not describe the 1.7 V
setting as proof that Q1's VCC is 1.7 V.

### Manual BUSY fixture states

`JP_RADIO_BUSY` is a three-pin header with exactly one removable shunt. Pin 2
goes only to Pi BCM GPIO24 (physical pin 18), configured as an input. Pin 1 goes
to module U1.4 / GP2 / BUSY; pin 3 goes to the shared `+3V3` rail.

| Radio fixture state | Single shunt | Receiver observes | Module BUSY |
|---|---|---|---|
| `radio_nominal` | 1-2 | Actual module BUSY | Connected only to Pi input |
| `radio_busy_held` | 2-3 | Constant HIGH | Disconnected from Pi and rail |

Move the shunt only after Linux shutdown and external power removal. Never
fit both positions: that would connect a driven radio output to 3.3 V. Keep
all other radio connections fitted. Neither state uses GPIO5 or an OR gate.
Leave both RTC fault shunts open during radio tests.

The held state tests bounded startup failure. Restore 1-2 unpowered and run
the nominal checks after reboot to establish restoration. These separate runs
do not establish soft/hard recovery in the same radio instance; software
cannot release the manual fault or confirm safe standby while BUSY stays high.

### Deferred controlled BUSY gate

The unavailable [SN74LVC1G32](https://www.ti.com/lit/ds/symlink/sn74lvc1g32.pdf)
and synchronized soft/hard recovery execution remain deferred. The earlier
gate proposal used `observed BUSY = raw BUSY OR GPIO5`, permitting controlled
fault release without driving the module output. It is not wired into this
manual carrier. Adding it later requires a revised selector connection: the
current pin 3 is tied to 3.3 V and must never also receive a gate output.
The retained gate tests cannot be selected with either manual fixture state.

Expose labelled high-impedance measurement points for 3.3 V, GND, raw BUSY,
observed BUSY, DIO1, NSS and RESET. A logic analyzer observes these without
driving them. Raw BUSY is the source for radio assertion/release measurements;
the forced signal is evidence of the fault fixture only.

### Assembly and qualification boundary

The operator will assemble and perform electrical checks. First fit the nominal
connections with `JP_RADIO_BUSY` in 1-2 and leave both RTC fault shunts open.
Change wiring or selector position only with the Pi shut down and external
power removed. Before power, check the pin/net continuity and absence of a
supply short or unintended link to the battery/VSYS/VBUS circuitry. At first
power, measure the radio's 3.3 V rail against its local ground and idle RESET/CS
levels; record the board variant, population and measured values.

For the manual held state, verify unpowered that selector 2-3 joins only the
Pi input to `+3V3`, with selector pin 1 isolated. The raw module BUSY output
must never be driven by the fixture. Restore 1-2 unpowered after that run.
Receiver runtime initialization and RF results require separate production
adapter tests after these electrical checks; a power LED is not acceptance.

An independently controlled SX1262 peer and a waveform capture device remain
to be confirmed. They are required for peer/timing evidence, but are not
required to implement or execute the deterministic host tests.

## References

- [Pi GPIO and header documentation](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#gpio-and-the-40-pin-header)
- [Linux GPIO open-drain behavior](https://docs.kernel.org/driver-api/gpio/driver.html#gpio-lines-with-open-drain-source-support)
- [Linux I2C fault injection](https://docs.kernel.org/i2c/gpio-fault-injection.html)
- [Adafruit RTC wiring](https://learn.adafruit.com/adding-a-real-time-clock-to-raspberry-pi/wiring-the-rtc)

The hardware selection is recorded in the sibling logbook at
`cura-agrorum-logbook/deployments/field-pilot-v2/README.md`; this document owns
the reproducible test connections within the production repository.
