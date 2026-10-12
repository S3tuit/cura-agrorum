# DS3231 boot-cycle findings

The operator's 2026-10-12 four-boot run demonstrated correct OSF rejection,
recovery by setting time, and short battery-backed retention. External power
was removed for 30 seconds between boots; the coin cell was also removed while
unpowered before boot 3. Physical actions are operator-reported; conclusions
use the supplied kernel and `hwclock` output.

## Tested setup

- Raspberry Pi 3 Model B Rev 1.2, recorded Adafruit DS3231 carrier and CR1220.
- Raspberry Pi OS / Debian 13 Trixie arm64; OS reports Debian version 13.7.
- Kernel `6.18.50+rpt-rpi-v8`, package `1:6.18.50-1+rpt1`.
- Driver `rtc-ds1307`, I2C device `1-0068`, RTC `/dev/rtc0`.
- util-linux and util-linux-extra `2.41.5-0+deb13u1`.
- Chrony was not used. No claim of accurate UTC is made for the copied time.

The Pi model, OS, kernel package and util-linux versions were checked over SSH.
Module/battery identity follows the recorded carrier; this run did not include
a new independent inventory of those parts.

## Outcomes

| Boot | Observed behavior | Outcome |
| --- | --- | --- |
| 1 | Initial OSF warning and rejected read; setting time restored successful reads | Initialization/recovery passed |
| 2 | No OSF warning; kernel copied RTC time to Linux; read succeeded | Short battery retention passed |
| 3 | After total power loss, OSF warning and `EINVAL`; setting time restored reads | OSF rejection/recovery passed |
| 4 | No OSF warning; kernel copied RTC time to Linux; read succeeded | Recovery persisted across cold boot |

The portable procedure starts boot 1 with a write because initial OSF state can
vary. The additional initial failed read was observed in this particular run.

## Kernel change

The earlier investigated Pi package `1:6.18.39-1+rpt1` lacked the DS3231 fixes
that reject oscillator-stopped reads and clear OSF when setting time. The
current stable `1:6.18.50-1+rpt1` demonstrated both behaviors on this hardware.
The [upstream driver correction](https://github.com/torvalds/linux/commit/a091e1ba3b68cabc9caedafc6f81d9fe9b3b2200)
adds OSF read rejection and clearing during a time write; a beta package is no
longer required for this tested setup.

This run checks functional behavior and short retention. It does not measure
accuracy/drift, execute the production-adapter test, or requalify the separate
[I2C controller faults](../i2c_faults/RESULTS.md).

## Reuse

Reuse the OSF and short-retention result for hardware and OS/kernel matching the
tested setup above. Repeat the [four-boot procedure](README.md) after a change to
that setup, before time synchronization or another RTC writer can repair the
initial state. A reinstall of the same applicable setup need not repeat the
physical check. The RTC operation bound has its own separate qualification.
