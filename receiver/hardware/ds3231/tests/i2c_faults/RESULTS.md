# DS3231 I2C fault findings

These findings concern the previously investigated Pi 3/Adafruit DS3231 carrier,
stock `rtc-ds1307` RTC driver and bcm2835 I2C controller. The kernel was
`6.18.50+rpt-rpi-v8`, package `1:6.18.50-1+rpt1~beta1`, Raspberry Pi Linux commit
`cff533aec2fa601846766b32ff57204e0a61bed7`. This is the retained controller
investigation summary, not a new run on the current stable package.

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

The current stable package's operation bound remains unqualified by this report.
After all four cases and complete restoration pass, record the adopted acceptance
limit, maximum complete-operation durations, hardware/wiring, kernel/packages,
source identity and native helper build/digest. Reuse that result for an applicable
matching setup, including another installation of it. Review applicability and
rerun after relevant controller/RTC/wiring, kernel/driver or helper/adapter build
changes. Three seconds is the tested procedure's candidate envelope; an ordinary
read or a passing OSF test does not qualify it. The measured maximum alone is
not a universal kernel worst-case guarantee.
