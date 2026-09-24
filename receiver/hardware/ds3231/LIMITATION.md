# DS3231 pilot limitation

The pilot uses the stock Raspberry Pi I²C/RTC drivers and bounded read recovery.
Controller driver fixes are deferred. This file owns the investigation findings,
workaround rationale and future work; [ARCHITECTURE.md](../../ARCHITECTURE.md),
[INTERFACE.md](../../INTERFACE.md#ds3231-control-interface) and
[TESTING.md](../../TESTING.md#time-policy-and-timestamp-analysis) retain their
behavior, exact-value and coverage contracts.

## Observed behavior

On the Pi 3/Adafruit DS3231 carrier, a transaction interrupted by held-low SCL
can leave later RTC operations failing after the fault GPIO is released.
A logical read still changes controller/transaction state. Repeated reads
restored communication in the observed runs without a power cycle, a new RTC
write or disconnecting the fault leads. Recovery in a fixed number of reads is
not guaranteed.

The investigated kernel is `6.18.50+rpt-rpi-v8`, package
`1:6.18.50-1+rpt1~beta1`, pinned to Raspberry Pi Linux commit
`cff533aec2fa601846766b32ff57204e0a61bed7`. The earlier read sequence was
`EREMOTEIO → ETIMEDOUT → EIO → OK` after release, counting the fixture's first
read. Some earlier recovery attempts included an unnecessary restoration write;
the dedicated RTC-only fixture removed that write and reproduced the read
failure with normal Chrony left running. Power cycling also recovered access,
but later controlled reads established that it was not always necessary.

Diagnostic register snapshots established that `DONE`/`ERR` survived timeout
cleanup and remained present at the next transfer. That next transfer took the
pending-length error path. Another timeout showed `DONE`/`ERR` with completion
interrupts disabled, consistent with a missed completion interrupt. The
[installed controller source](https://github.com/raspberrypi/linux/blob/cff533aec2fa601846766b32ff57204e0a61bed7/drivers/i2c/busses/i2c-bcm2835.c)
clears status on interrupt completion but omits that clearing on software
timeout. The original abort/NACK mechanism remains unresolved; clearing stale
flags alone has not been qualified as a complete fix.

`TA` also appeared before a successful read, so that bit alone does not prove
a physically stuck bus. Multimeter readings and GPIO pad state cannot establish
I²C waveforms or transient timing. Recovery after removing a jumper did not
establish that the jumper caused the persistent failure: subsequent reads also
recovered with both leads still fitted.

A held-SCL write exposed a second fixture assumption: both bus pads remained
low even after the injection GPIOs were released as unused inputs. The old
fixture's bus-high prerequisite prevented read recovery from running. With
that prerequisite moved after read recovery, the same stuck-low state recovered
in three reads over 1.02 seconds. GPIO release and bus recovery are therefore
checked separately. Physical wiring changes still require external power off.

## Pilot choices

- Keep the installed stock driver. The observation-only diagnostic module was
  removed; no functional custom-driver fix is deployed. Driver switching during
  investigation affected only the RTC controller; the HDMI controller stayed
  on its stock binding. Re-registering rtc0 may set Linux time, so the approved
  setup/rollback paused Chrony and verified synchronization afterward.
- Use communicator-owned repeated single reads within the three-second recovery
  window specified by the interface. A final in-flight call retains its separate
  operation bound; a late completion fails even when its calendar is valid.
  Only the successful attempt's bracket contributes UTC and uncertainty.
- Require a successful pre-read before ordinary writes, recheck fresh trusted
  time/generation, invalidate old durable provenance, derive fresh UTC, write
  once and verify through read recovery. Never blindly retry an uncertain write.
- Stop ordinary recovery on `INVALID`; use explicit network-qualified operator
  recovery. The [installed RTC driver](https://github.com/raspberrypi/linux/blob/cff533aec2fa601846766b32ff57204e0a61bed7/drivers/rtc/rtc-ds1307.c)
  rejects oscillator-stopped RTC reads until time is set. An EINVAL result by
  itself does not distinguish OSF from other invalid calendar/transport results.
- After fault injection, require normal GPIO-holder exit/reaping and unused
  input GPIOs, then allow bounded read recovery even if bus pads remain low.
  Require high pads and a valid RTC read before any restoration write, and
  high pads again after verification. A failed recovery stops dependent tests.

## Qualification on 2026-09-15

All four fault cases passed in 21.26 seconds on the stock driver. The original
three-second operation bound, 500-ms write-fault deadline and three-second
read-recovery window were unchanged.

| Injected operation | Result under fault | Operation duration | Recovery reads | Recovery duration |
|---|---|---:|---:|---:|
| Read, SCL low | IO_ERROR / ETIMEDOUT | 1,013,279 µs | 4 | 1,050,358 µs |
| Read, SDA low | INVALID / EINVAL | 3,022 µs | 1 | 8,622 µs |
| Write, SCL low | OUTCOME_UNKNOWN / DEADLINE_EXCEEDED | 2,069,579 µs | 3 | 1,039,883 µs |
| Write, SDA low | COMPLETED | 4,941 µs | 1 | 8,305 µs |

The held-SCL write included 218 observations of pending SIGKILL inside the
native helper ioctl; the helper was reaped before fault release. A completed
SDA-low write is only the driver's reported operation outcome, not trusted
RTC content. Both write cases performed one restoration write after a valid
recovery read and verified it. All four fixtures restored successfully; normal
Chrony identity/configuration stayed unchanged. Independent postflight verified
RTC reads, high bus pads, released GPIOs, original device permissions and
removed temporary helpers.

After an externally unpowered return to the nominal carrier with both shunts
open, the new boot initialized Linux time from the RTC without OSF. The final
RTC refresh/replacement and both clock-step cases passed (**3 tests, 97.797
seconds**), followed by **8 safe Pi tests in 6.597 seconds**. All restoration
checks passed. The host suite passed **2,300 tests**.

The [consolidated time record](../../tests/hardware/evidence/runtime_time/results.json)
retains controller and intrusive nominal outcomes with relevant source hashes
from manifest `e457c8999da1abbe481743f62386f0e2717348199c8902d46e392d89e5c47aae`.
Earlier failure lessons are summarized here; their raw runs are discarded.

## Deferred work

Investigate and qualify the complete controller timeout/abort and completion
interrupt behavior across combined transfers, both bus lines, and reads/writes.
Any driver fix needs its own regression/deployment qualification before upstream
submission. Consider stronger recovery and invalid-RTC initialization behavior
if field evidence requires it. Do not infer a worst-case guarantee from the
observed retry counts.

A five-second pilot recovery window was considered only as a possible later
response to insufficient recovery time. It was not needed for these four cases
and is not enabled. A longer window cannot waive operation bounds, termination
proof, invalid contents or failed restoration.
