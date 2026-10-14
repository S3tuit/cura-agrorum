# DS3231 I2C fault findings

The earlier findings below concern the Pi 3/Adafruit DS3231 carrier,
stock `rtc-ds1307` RTC driver and bcm2835 I2C controller. That kernel was
`6.18.50+rpt-rpi-v8`, package `1:6.18.50-1+rpt1~beta1`, Raspberry Pi Linux commit
`cff533aec2fa601846766b32ff57204e0a61bed7`. This is the retained controller
investigation summary, not the stable-package qualification recorded below.

## Observed behavior

A transaction interrupted by held-low SCL sometimes left subsequent RTC
operations failing after the injecting GPIO was released. Repeated reads could
recover without power cycling, another write or removal of fault leads. One
sequence was `EREMOTEIO → ETIMEDOUT → EIO → OK`; recovery in a fixed number of
reads is not guaranteed.

Diagnostic snapshots showed `DONE`/`ERR` surviving timeout cleanup into the next
transfer. Another timeout left completion interrupts disabled. The investigated
[controller source](https://github.com/raspberrypi/linux/blob/cff533aec2fa601846766b32ff57204e0a61bed7/drivers/i2c/busses/i2c-bcm2835.c)
clears status on interrupt completion but omits that clearing on software timeout.
The original abort/NACK mechanism remains unresolved; clearing stale flags alone
has not been qualified as a complete fix. No custom driver remains deployed.

After a held-SCL write, both bus pads could remain low even with the injecting
GPIOs restored to unused inputs. Allowing read recovery before requiring high
pads recovered one such state in three reads over 1.02 seconds. GPIO release and
bus recovery must therefore be checked separately. `TA` appeared before a
successful read too; it alone does not establish a physically stuck bus.
Multimeter/pad observations do not establish I2C waveforms or transient timing.

## Outcome interpretation and restoration

Held-SCL reads can report transport failures; held-SDA reads can produce invalid
calendars. A held-SCL write can remain inside its ioctl after a kill request:
request submission is not proof of helper termination, so actual reap is required.
A completed held-SDA write reports only the syscall outcome, not verified contents.

Earlier attempts sometimes included an unnecessary restoration write. The
RTC-only fixture reproduced failures while leaving normal Chrony running and
uses reads alone to restore read cases. Fault leads were not proven to cause the
persistent failure merely because access recovered after their removal.

The observation-only diagnostic module was removed. Switching the investigated
RTC controller driver left the HDMI controller unchanged; re-registering rtc0
could set Linux time, so that investigation paused Chrony and checked normal
synchronization afterward. Current fault tests do not switch drivers.

## Current recovery rationale

Use communicator-owned repeated single reads within the contracted three-second
recovery window, with the separate final in-flight-operation bound. Only a
successful read's own bracket contributes UTC/uncertainty; a late valid result
still fails its deadline. Stop ordinary recovery on `INVALID` and use explicit
network-qualified operator recovery. `EINVAL` alone does not identify OSF.

After injection, require normal GPIO-holder exit/reaping and unused input GPIOs.
Reads may attempt recovery while pads remain low; high pads and a valid RTC read
are required before any restoration write, and high pads must be checked again
after verification. Ordinary writes pre-read, recheck trusted time/generation,
invalidate old durable provenance, write once and verify. Never blindly retry an
uncertain write; failed recovery stops dependent tests.

## Unresolved work

Controller timeout/abort and completion-interrupt behavior across reads/writes and
both bus lines remains unresolved. Any driver fix needs new regression and target
qualification. The recent [OSF boot-cycle result](../boot_cycle/RESULTS.md) does
not qualify controller-fault recovery on the stable kernel. A five-second recovery
window was considered but was neither needed nor enabled; longer pacing cannot
waive operation bounds, termination evidence, invalid contents or restoration.

[The family procedure](README.md) owns execution and physical restoration;
[ARCHITECTURE.md](../../../../ARCHITECTURE.md),
[INTERFACE.md](../../../../INTERFACE.md#ds3231-control-interface), and
[TESTING.md](../../../../TESTING.md#time-policy-and-timestamp-analysis) own runtime
behavior, exact bounds and coverage. Original debug and failed-run bundles were
previously removed; this summary preserves the useful findings and their limits.

## Reusing an adopted operation bound

Use `RTC_KERNEL_BOUND_US=3000000` for the applicable stable setup qualified below.
Reuse that result for a matching setup, including another installation of it.
Review applicability and rerun after relevant controller/RTC/wiring, kernel/driver
or helper/adapter build changes. An ordinary read or a passing OSF test does not
qualify this limit. The measured maximum alone is not a universal kernel
worst-case guarantee.

## 2026-10-12 stable-package qualification

Four fault cases passed, followed by operator-confirmed unpowered removal of both
shunts, successful RTC-to-Linux initialization on a new boot without OSF, qualified
network-time restoration, and nine safe time component checks. Adopt the 3-second
complete-operation envelope for this setup.

| Fault | Complete operation (seconds) | Observation |
| --- | ---: | --- |
| SCL-low read | 1.030318 | `IO_ERROR`, errno 110; no trusted UTC |
| SDA-low read | 0.002974 | `INVALID`, errno 22; no trusted UTC |
| SCL-low write | 2.069414 | `OUTCOME_UNKNOWN` / `DEADLINE_EXCEEDED`; pending SIGKILL observed inside the helper's write ioctl; helper reaped before fault release |
| SDA-low write | 0.004914 | `COMPLETED`; contents verified only through restoration read-back |

Maximum complete operation: **2,069,414 us**. All four receipts and restoration
results were independently verified. Read recovery after SCL release required
four attempts for the read case and three for the write case; transient controller
failures remain observable. This run does not qualify a driver repair or guarantee
a fixed recovery count.

Tested configuration: Raspberry Pi 3 Model B Rev 1.2, recorded Adafruit DS3231 and
CR1220, GPIO17→SCL/GPIO3 and GPIO27→SDA/GPIO2 fault wiring, Debian 13.7 Trixie arm64,
stock `rtc-ds1307`/`i2c-bcm2835`, stable kernel/package
`6.18.50+rpt-rpi-v8` / `1:6.18.50-1+rpt1`. GCC 14.2.0, compiler build
`Debian 14.2.0-19`, Chrony 4.6.1. Native helper digest:
`1937c12800cfc8990c55719c087ba0f03c727a2e86809ae84069aba2d87b8096`.

Source baseline was `511fc76e289efb2767f1b27dd41683a3f81cc5ad` plus the current
Chrony drop-in relocation. The exact 288-file current-source snapshot has archive
SHA-256 `482d6f014d3e64e226d014f6e48f2a4173981243e7de28f0b7c80a93ef93c4e5`.
The [logbook record](https://github.com/S3tuit/cura-agrorum-logbook/tree/main/experiments/2026-10-ds3231-operation-bound)
(local sibling `../cura-agrorum-logbook/experiments/2026-10-ds3231-operation-bound`)
retains source, packages/dependencies, operator confirmations, raw receipts and
the verifier. One execution per fault case establishes this specified envelope;
no statistical tail, RTC accuracy/drift, installed-service or RF claim is made.
