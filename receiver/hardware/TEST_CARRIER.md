# Receiver hardware-test carrier

This document owns the common receiver test schematic, physical connections
and fixture states. The current carrier uses the recorded Raspberry Pi 3
Model B and Adafruit DS3231 with a CR1220 battery. The operator has confirmed
assembly of the two removable fault shunts while externally unpowered.
Qualification results and the pilot recovery limitation are recorded in
[DS3231 LIMITATION.md](ds3231/LIMITATION.md).

[TESTING.md](../TESTING.md) owns required coverage and execution rules.
[The DS3231 installation guide](ds3231/README.md) owns bench OS setup,
[OPERATOR_TESTS.md](ds3231/OPERATOR_TESTS.md) owns the physical RTC acceptance
sequence, and [RTC_FAULT_TESTS.md](../tests/hardware/RTC_FAULT_TESTS.md) owns the
production-adapter fault execution and restoration procedure. Result archives
retain their original carrier/source bindings.

## Design rules

- Power the Pi from its normal supply and the DS3231 VIN from the Pi's 3.3 V
  header. Connect their grounds. The CR1220 supplies RTC backup power.
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
- Keep branch leads short. This is the current time-test carrier; radio
  circuitry and GPIO allocation are added when their implementation establishes
  the actual hardware boundary.

## Pin and connector allocation

Pi numbers below refer to the **physical 40-pin header**, with BCM GPIO numbers
listed separately. Locate physical pin 1 from the board's header marking.

| Net/function | Pi physical pin | BCM GPIO | Connection/owner |
|---|---:|---:|---|
| `+3V3_RTC` | 1 | — | Adafruit DS3231 VIN |
| `GND` | 6 | — | Adafruit DS3231 GND |
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
 Raspberry Pi 3                                Adafruit DS3231

 pin 1  +3.3 V ------------------------------- VIN
 pin 6  GND ---------------------------------- GND

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

## References

- [Pi GPIO and header documentation](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#gpio-and-the-40-pin-header)
- [Linux GPIO open-drain behavior](https://docs.kernel.org/driver-api/gpio/driver.html#gpio-lines-with-open-drain-source-support)
- [Linux I2C fault injection](https://docs.kernel.org/i2c/gpio-fault-injection.html)
- [Adafruit RTC wiring](https://learn.adafruit.com/adding-a-real-time-clock-to-raspberry-pi/wiring-the-rtc)

The hardware selection is recorded in the sibling logbook at
`cura-agrorum-logbook/deployments/field-pilot-v2/README.md`; this document owns
the reproducible test connections within the production repository.
