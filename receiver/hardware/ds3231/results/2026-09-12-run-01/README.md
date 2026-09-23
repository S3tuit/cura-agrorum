# DS3231 operator run — 2026-09-12

**RTC-01 through RTC-05: PASS. RTC-06: NOT RUN.**

Fixture: `cura-receiver`, Raspberry Pi 3 Model B Rev 1.2, Adafruit DS3231
with CR1220, `/dev/rtc0` bound to `rtc-ds1307` at `1-0068`. Debian Trixie;
kernel `6.18.50+rpt-rpi-v8` / package `1:6.18.50-1+rpt1~beta1`, Chrony
`4.6.1-3+deb13u2`, util-linux `2.41.5-0+deb13u1`.

Validated against [the operator procedure](../../OPERATOR_TESTS.md);
[procedure-sha256](data/procedure-sha256) record its SHA256.

| Test | Result and evidence |
|---|---|
| RTC-01 | PASS — expected binding, successful bootstrap and [initial RTC read](data/raw/rtc01-read.txt). |
| RTC-02 | PASS — network-qualified step/wait/write succeeded; [baseline](data/raw/rtc02-baseline.txt) is consistent with zero offset. |
| RTC-03 | PASS — new boot, [bootstrap before Chrony started](data/raw/rtc03-after-kernel.txt), no OSF warning, and valid [after-poweroff capture](data/raw/rtc03-after-poweroff.txt). |
| RTC-04 | PASS — new boot reports [OSF and failed bootstrap](data/raw/rtc04-kernel.txt); [RTC read rejects invalid time](data/raw/rtc04-read.txt) with `EINVAL`. |
| RTC-05 | PASS — [qualified recovery](data/raw/rtc05-baseline.txt), another [successful cold boot](data/raw/rtc05-after-kernel.txt), and [valid RTC read](data/raw/rtc05-after-read.txt). |
| RTC-06 | NOT RUN — longer retention remains outstanding. |

The operator confirmed during review: RTC-03 disconnected Pi power for at
least 30 minutes with the battery retained; RTC-04 removed all external power
and the cell, waited about 30 seconds, then reinserted it; RTC-05 included
another power-disconnected boot with the battery installed. No RTC write
occurred between the RTC-02 baseline and RTC-03 after-poweroff capture.
Exact physical-event timestamps were not recorded.

The bracketed captures span about **40 minutes** (including powered time).
Using the procedure's whole-second allowance and the larger Chrony reference
error from each pair of reports, the baseline offset is approximately
`[-0.021094, +1.404549] s`, the later offset `[-0.034828, +1.418428] s`,
and their change `[-1.439376, +1.439522] s` (bounds rounded outward).
**No drift resolved by this measurement.** This is a functional retention
pass, not a datasheet-accuracy or receiver holdover-policy certification.

A read-only follow-up at 17:44 UTC on the
same RTC-05 boot confirmed Chrony running with a selected network source,
normal leap status and 0.000727 seconds remaining correction; timesyncd
remained masked/inactive. The battery and valid RTC were restored.

The only nonzero original command statuses are expected: RTC-04's invalid
read (`1`) and the combined service query (`3`, inactive timesyncd).
Pre-recovery Linux dates are unreliable and remain unchanged in the evidence.
