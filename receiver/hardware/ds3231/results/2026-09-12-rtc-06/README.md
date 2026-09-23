# DS3231 longer retention run — 2026-09-12 to 2026-09-14

**RTC-06: PASS — functional retention; no drift resolved by this measurement.**
The network-qualified captures span approximately **37 hours 42 minutes**,
including powered time before shutdown and after startup. Their offset-change
interval is `[-2.241689, +1.690904] s`, rounded outward. This does not certify
datasheet accuracy or receiver runtime holdover policy.

The original 19 captures were copied from
`cura-receiver:/home/cura/ds3231-runs/2026-09-12-rtc-06` and validated on
2026-09-14 against [RTC-06](../../OPERATOR_TESTS.md#rtc-06-longer-retention-and-best-effort-offset-comparison).
[procedure-sha256](data/procedure-sha256) records the procedure at review time;
it matches the procedure hash in the previous archive. This run did not
contain its own run-time procedure hash, identity/configuration captures,
physical-action notes or checksum manifest.

The [initial](data/raw/rtc01-kernel.txt) and
[after-poweroff](data/raw/rtc06-after-kernel.txt) kernel logs identify
`cura-receiver`, Raspberry Pi 3 Model B Rev 1.2, kernel
`6.18.50+rpt-rpi-v8` / package `1:6.18.50-1+rpt1~beta1`, with `rtc-ds1307`
at `1-0068` registered as `/dev/rtc0`. Captures identify Chrony 4.6.1 and
util-linux 2.41.5. The Adafruit DS3231/CR1220 fixture is documented in
[the preceding run](../2026-09-12-run-01/README.md). Separate
[read-only review evidence](data/review/review-readonly.txt) confirms the
same kernel and driver, Debian 13 Trixie, Chrony package
`4.6.1-3+deb13u2` and util-linux package `2.41.5-0+deb13u1`; this is review-time
identity evidence, not a reconstruction of missing initial captures.

| Test | Result and evidence |
|---|---|
| RTC-01 | PASS — expected binding, successful bootstrap and [valid initial RTC read](data/raw/rtc01-read.txt), preserved before the initialization write. |
| RTC-02 | PASS — [selected network source](data/raw/rtc02-sources.txt), successful [Linux step](data/raw/rtc02-step.txt), [wait](data/raw/rtc02-wait.txt), [RTC write](data/raw/rtc02-write.txt), and [qualified baseline](data/raw/rtc02-baseline.txt) consistent with zero offset. |
| RTC-03 | PASS — retention sequence executed as part of RTC-06: new boot ID, valid RTC bootstrap before Chrony, no fresh OSF warning, and successful reads. The physical outage and unchanged battery are operator-reported. |
| RTC-04 | NOT RUN — no deliberate total-power/battery-loss injection in this run. |
| RTC-05 | NOT RUN — no RTC-04 fault requiring recovery was injected. Normal post-run state was checked separately below. |
| RTC-06 | PASS — operator-confirmed longer retention sequence, [valid first read after reboot](data/raw/rtc06-after-read.txt), and [qualified after-poweroff comparison](data/raw/rtc06-after-poweroff.txt) with uncertainty recorded below. |

During review on 2026-09-14, the operator confirmed that the coin cell stayed
installed and the documented procedure was followed. This confirmation covers
shutdown, removal/restoration of external power and the Linux-only correction
after reboot, without an intervening RTC write. The operator asked to derive
the approximate timeline from the network-synchronized UTC captures. Exact
power-disconnected start/end times were not separately recorded and cannot
be recovered from these reads; the longer outage is operator-reported.
The approximately 37 h 42 min figure describes the sample interval, not an
independently timed outage or a rate measured solely on battery power.

The [before boot ID](data/raw/rtc06-before-boot-id.txt) is
`20a93cf7-dc70-41ff-b00e-9269bb05014b`; the
[after boot ID](data/raw/rtc06-after-boot-id.txt) is
`577740db-60ed-46f8-ab94-042b46381c7f`. The fresh kernel log records successful
RTC bootstrap to `2026-09-14T09:48:16 UTC` at monotonic 16.295494 s. This is an
RTC-derived boot timestamp, not a network-qualified power-on timestamp.
[Chrony starts](data/raw/rtc06-after-chrony.txt) at monotonic 30.458021 s and
first selects a network source at 55.210710 s. The RTC subsequently advances
from the baseline raw second `1789243699` to `1789379448`, with no reset,
frozen value or invalid RTC read in the captures.

The [exact calculation record](data/validation-calculations.json) applies
the procedure's RTC-minus-reference sign and whole-second allowance:

| Input or result | Baseline: `rtc02-baseline.txt` | After: `rtc06-after-poweroff.txt` |
|---|---|---|
| UTC bracket | 2026-09-12 20:08:18.162044384–20:08:19.007947048 | 2026-09-14 09:50:47.363363731–09:50:48.357413267 |
| `B` (epoch s) | 1789243698.162044384 | 1789379447.363363731 |
| `A` (epoch s) | 1789243699.007947048 | 1789379448.357413267 |
| `R` (raw integer epoch s) | 1789243699 | 1789379448 |
| Reference ID | B99DE5FE (`ns3.fibertelecom.it`) | 55C7D663 (`ntp2.leontp.com`) |
| Reference time UTC | 2026-09-12 20:08:04 | 2026-09-14 09:50:12 |
| Before/after reference-error estimates (s) | 0.022086483 / 0.022087400 | 0.0242033835 / 0.0242324485 |
| Selected `E` (s) | 0.022087400 | 0.0242324485 |
| Exact offset interval (s) | [-0.030034448, +1.860043016] | [-0.3816457155, +1.6608687175] |

For each tracking report, the reference-error estimate is
`abs(System time) + Root dispersion + Root delay / 2`. Each capture uses
the larger of its two estimates, without an additional margin. Both paired
reports have normal leap status, a recent reference and the same reference
ID/time within that capture. The two captures use different network sources;
each is qualified separately with its own allowance. No step occurs inside
the documented snapshot sequence, and both UTC brackets advance normally.
These are best-effort allowances conditional on correct upstream references
and stable synchronization during each short bracket.

Subtracting the offset intervals gives the exact change
`[-2.2416887315, +1.6909031655] s`. It contains zero:
**no drift resolved by this measurement**. The interval between reads is
`[135748.3090968345, 135750.2416887315] s`. The optional coarse average-rate
estimate is `[-16.513567, +12.456164] ppm`, rounded outward over all four
change/elapsed endpoint ratios. It includes the entire interval between
reads and carries the same reference assumptions; no ppm pass limit is
applied.

[Raw SHA256SUMS](data/raw/SHA256SUMS) was generated during review from hashes
computed on the Pi and checked against all 19 copied files. The original Pi
directory was not modified. [Review SHA256SUMS](data/review/SHA256SUMS) covers
the two separate follow-up captures. Verify each manifest from its own
directory with `sha256sum -c SHA256SUMS`. The earlier run is preserved
unchanged, including its historical RTC-06 NOT RUN outcome.
