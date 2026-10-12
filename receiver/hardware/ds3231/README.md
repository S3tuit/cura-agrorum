# DS3231 hardware and tests

The recorded fixture is a Raspberry Pi 3 Model B Rev 1.2 with an Adafruit DS3231
and CR1220 backup cell. The [receiver carrier](../TEST_CARRIER.md) owns wiring,
pin assignments and fault shunts. Change wiring or the battery only after Linux
shutdown and removal of every external power source.

Use [receiver deployment](../../deploy/README.md) to update the OS, enable I2C,
bind the RTC and check device paths. The kernel's `rtc-ds1307` driver serves the
DS3231 at I2C address `0x68`; this Pi exposes it at `/dev/rtc0`. Raw I2C access and
a separate Python DS3231 driver are unnecessary for ordinary RTC reads.

## Test families

| Family | Procedure and code | Findings |
| --- | --- | --- |
| Boot, battery retention, OSF rejection/recovery | [Four boots and optional production-adapter check](tests/boot_cycle/README.md) | [Tested hardware, kernel comparison and outcomes](tests/boot_cycle/RESULTS.md) |
| I2C faults and helper termination | [Physical fixture, four automated cases and restoration](tests/i2c_faults/README.md) | [Controller findings and unresolved recovery limits](tests/i2c_faults/RESULTS.md) |

Each family keeps its procedure, required code and concise maintained findings
together. Detailed dated runs belong in the sibling logbook.

## Time ownership

The boot-cycle check needs no accurate UTC reference. Normal deployment uses
[Chrony configuration and qualified RTC initialization](../../deploy/CHRONY.md).
Receiver runtime trust, privileges, deadlines and durable RTC provenance remain
owned by [ARCHITECTURE.md](../../ARCHITECTURE.md#time-model),
[INTERFACE.md](../../INTERFACE.md#ds3231-control-interface), and
[TESTING.md](../../TESTING.md#time-policy-and-timestamp-analysis).
