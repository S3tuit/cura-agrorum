# DS3231 installation and configuration

This guide describes the receiver bench setup on a Raspberry Pi 3 Model B,
Raspberry Pi OS Trixie arm64, and an Adafruit DS3231 with a CR1220 battery.
The repeatable acceptance procedure is in [OPERATOR_TESTS.md](OPERATOR_TESTS.md).
The stock-driver recovery limitation and its qualification are documented in
[LIMITATION.md](LIMITATION.md).
[Recorded results](results/README.md) are Git-tracked beside this procedure,
including the dated installation and raw evidence in the
[September 2026 record](results/2026-09-12-run-01/README.md).

These are operator commands for an isolated bench Pi. The production authority
remains [the time model](../../ARCHITECTURE.md#time-model) and
[the Linux backend contract](../../INTERFACE.md#pilot-linux-backend-and-privilege-boundary).
Do not run manual clock steps or RTC writes alongside the receiver service.
No production adapter, privileged helper, stable device alias, or trusted
`RTC_HOLDOVER` state is installed by this guide.

## 1. Wire and enable the device

Shut down Linux, disconnect the Pi's power, and connect the RTC using the
[test carrier's pin table and schematic](../TEST_CARRIER.md#pin-and-connector-allocation).
Use the `nominal` fixture state: both RTC fault shunts open. The common carrier
document owns physical pin assignments and removable fault connections.

Fit the CR1220 with its positive side visible. Leave BAT, SQW, 32K, and RST
unconnected. These instructions apply to the recorded Adafruit board; check
the circuit before substituting another module, particularly one with a
battery-charging circuit. See [Adafruit's wiring guide](https://learn.adafruit.com/adding-a-real-time-clock-to-raspberry-pi/wiring-the-rtc).

Boot the Pi and edit `/boot/firmware/config.txt`. Under the active `[all]`
section, ensure these lines appear once:

```ini
dtparam=i2c_arm=on
dtoverlay=i2c-rtc,ds3231
```

Reboot, then inspect the binding:

```bash
sudo reboot
# Reconnect after boot, then run:
uname -r
cat /sys/class/rtc/rtc0/name
readlink -f /sys/class/rtc/rtc0/device
readlink -f /sys/class/rtc/rtc0/device/driver
sudo journalctl -k -b -o short-monotonic --no-pager |
  grep -E 'rtc-ds1307|hctosys'
```

For this Pi, expect `rtc-ds1307`, I2C device `1-0068` (address `0x68`), and
driver `/sys/bus/i2c/drivers/rtc-ds1307`. `/dev/rtc0` is a verified bench path,
not a universal device identity. The driver is part of the kernel package;
there is no separate DS3231 Python driver to install. Raw `/dev/i2c-1` access
and `i2c-tools` are unnecessary for ordinary RTC reads.

Preserve the first boot log and RTC read using RTC-01 in the operator
procedure **before any time-setting command**. Initial OSF is useful evidence.

## 2. Select a kernel with correct OSF handling

The required behavior is: DS3231 oscillator-stop flag (OSF) set means
`RTC_RD_TIME` fails with `EINVAL`; an explicit `RTC_SET_TIME` clears OSF as
part of setting the clock. Validate this with the operator procedure, not
only a version comparison or the absence of a log message.

On 2026-09-12 the following source/package comparison was established:

| Version | Status in this investigation |
|---|---|
| Pi `1:6.18.39-1+rpt1` | Installed initially; lacked both DS3231 OSF fixes |
| Upstream `6.18.40` | First fixed upstream release in the 6.18 series |
| Pi `1:6.18.50-1+rpt1~beta1` | Downloaded from official `trixie/beta`; installed and exercised |

The [upstream correction](https://github.com/torvalds/linux/commit/a091e1ba3b68cabc9caedafc6f81d9fe9b3b2200)
was backported as `aa33b44f70bfec8d431005c42706104a25062e16` in
[Linux 6.18.40](https://cdn.kernel.org/pub/linux/kernel/v6.x/ChangeLog-6.18.40).
The tested Pi package records Linux source commit
[`cff533aec2fa601846766b32ff57204e0a61bed7`](https://github.com/raspberrypi/linux/blob/cff533aec2fa601846766b32ff57204e0a61bed7/drivers/rtc/rtc-ds1307.c).
The current development branch containing a fix does not establish that a
released distribution package contains it.

Prefer a corrected stable package when one is available. Repository contents
change: the version above is a historical, tested beta selection, not a
permanent recommendation. [Raspberry Pi documents beta packages](https://www.raspberrypi.com/documentation/computers/configuration.html#enable-or-disable-beta-access)
as prerelease software for testing.

### Reproduce the exact beta package installation

The September installation used two authenticated APT downloads and a local
package install. It left the configured repositories on `trixie/main` and did
not opt the whole system into beta updates. On the Pi, download the exact
packages using temporary APT source/index files:

```bash
python3 - <<'PY'
from pathlib import Path
import subprocess
import tempfile

destination = Path.home() / 'ds3231-kernel-6.18.50'
destination.mkdir(exist_ok=True)
with tempfile.TemporaryDirectory(prefix='ds3231-apt-') as directory:
    work = Path(directory)
    (work / 'lists' / 'partial').mkdir(parents=True)
    source = work / 'beta.sources'
    source.write_text(
        'Types: deb\n'
        'URIs: https://archive.raspberrypi.com/debian/\n'
        'Suites: trixie\nComponents: beta\nArchitectures: arm64\n'
        'Signed-By: /usr/share/keyrings/raspberrypi-archive-keyring.pgp\n'
    )
    apt = [
        'apt-get', '-o', 'Dir::Etc::sourcelist=' + str(source),
        '-o', 'Dir::Etc::sourceparts=-',
        '-o', 'Dir::State::lists=' + str(work / 'lists'),
        '-o', 'Dir::Cache::pkgcache=', '-o', 'Dir::Cache::srcpkgcache=',
    ]
    subprocess.run(apt + ['update'], check=True)
    subprocess.run(apt + [
        'download',
        'linux-base-6.18.50+rpt-rpi-v8=1:6.18.50-1+rpt1~beta1',
        'linux-image-6.18.50+rpt-rpi-v8=1:6.18.50-1+rpt1~beta1',
    ], cwd=destination, check=True)
PY
```

Stop if authentication or version selection fails; do not silently substitute
another package. APT verifies downloads against authenticated repository
metadata. The archive may eventually stop publishing this exact version.

Have physical SD-card recovery access available. Preserve the installed old
kernel and make a boot-file backup; use a new backup filename if it exists:

```bash
sudo apt-mark manual linux-image-6.18.39+rpt-rpi-v8
sudo tar -C /boot/firmware -cpf /home/cura/pi-boot-before-6.18.50.tar .
cd ~/ds3231-kernel-6.18.50
dpkg-deb --show ./linux-base-6.18.50+rpt-rpi-v8_*.deb
dpkg-deb --show ./linux-image-6.18.50+rpt-rpi-v8_*.deb
sudo apt-get -s --no-install-recommends install \
  ./linux-base-6.18.50+rpt-rpi-v8_*.deb \
  ./linux-image-6.18.50+rpt-rpi-v8_*.deb
```

The recorded simulation added only those two packages, with no upgrades or
removals. Review any different result before installing. Then:

```bash
sudo apt install --no-install-recommends \
  ./linux-base-6.18.50+rpt-rpi-v8_*.deb \
  ./linux-image-6.18.50+rpt-rpi-v8_*.deb
# Only after successful installation:
sudo reboot
```

After reconnecting, require `uname -r` to report `6.18.50+rpt-rpi-v8` and
`dpkg-query -W linux-image-6.18.50+rpt-rpi-v8` to report the selected version.
The package hooks install the kernel/initramfs into the boot partition; no
manual `kernel=` override was needed on this Pi. Keep the old kernel during
acceptance. If the new kernel cannot boot, recover the boot files from another
machine with the SD card mounted; the backup above is not a full SD image.
Installing a versioned beta package does not subscribe it to future beta
updates. Revisit stable kernel availability and rerun acceptance when changing
the kernel.

## 3. Configure time ownership

The bench uses Chrony as the sole normal Linux clock writer. Capture the
initial RTC state first. Before installing Chrony on a fresh bench, keep both
time services stopped/masked so package defaults cannot start disciplining
clocks before the configuration is in place:

```bash
sudo systemctl mask --now systemd-timesyncd.service chrony.service
sudo apt install chrony
sudo cp -an /etc/chrony/chrony.conf /etc/chrony/chrony.conf.before-ds3231
sudo nano /etc/chrony/chrony.conf
```

Use this effective configuration on this dedicated bench:

```conf
# Use Debian vendor zone.
pool 2.debian.pool.ntp.org iburst

# This directive specifies the file into which chronyd will store the rate
# information.
driftfile /var/lib/chrony/chrony.drift

leapsecmode slew
maxslewrate 3500

# Disable UPD for remote monitoring 
cmdport 0

# Unix domain socket path to which the chronyd daemon binds for listening to
# monitoring commands issued by the chronic utility
bindcmdaddress /run/chrony/chronyd.sock
```

Omit `makestep`, `initstepslew`, `rtcsync`, and `rtcfile`. Audit included
configuration files if retaining a different configuration layout. Inspect
other services, timers, cron jobs and shutdown hooks for system-clock or RTC
writes; removing one competing daemon does not audit every possible writer.
No periodic `hwclock --systohc` service is installed. In particular,
`rtcsync` can enable kernel RTC updates even without an `hwclock` service.

Install the [policy check](../../tools/check_chrony.py) and
[systemd drop-in](chrony-runtime.conf) from the repository root:

```bash
sudo install -D -m 0644 -o root -g root receiver/tools/check_chrony.py /usr/libexec/cura-agrorum/check-chrony.py
sudo install -D -m 0644 -o root -g root receiver/hardware/ds3231/chrony-runtime.conf /etc/systemd/system/chrony.service.d/cura-runtime.conf
sudo systemctl daemon-reload
```

`ExecStartPre` checks the expanded configuration before every manual or
automatic start; failure prevents Chrony from starting. `ExecStart` explicitly
uses that same `/etc/chrony/chrony.conf`, retaining `-F 1` without reading
`DAEMON_OPTS`. `Restart=on-failure` enables crash recovery; an explicit stop
leaves it stopped. This is a trusted-operator procedure: keep configuration
inputs unchanged while running; for later edits, stop the unit, edit, then
start it through systemd. We accept the check/use interval and keep no
configuration snapshots. The receiver process does not run this check.
See [systemd's service contract](https://github.com/systemd/systemd/blob/main/man/systemd.service.xml)
and [Chrony's configuration check](https://chrony-project.org/doc/4.6/chronyd.html).

Enable the configured daemon:

```bash
sudo systemctl unmask chrony.service
sudo systemctl enable --now chrony.service
systemctl --no-pager --full status chrony.service systemd-timesyncd.service
sudo chronyc -h /run/chrony/chronyd.sock sources -v
sudo chronyc -h /run/chrony/chronyd.sock tracking
```

Install `hwclock`:

```bash
sudo apt install util-linux-extra
```

Use `sudo` for these bench commands: with UDP monitoring disabled, the
ordinary SSH account may lack Unix-socket permissions and report
`506 Cannot talk to daemon` even while Chrony is running.

## 4. Initialize only from verified network time

Use RTC-02 in [the operator procedure](OPERATOR_TESTS.md#rtc-02-qualified-write-and-read-back).
That procedure preserves the pre-write result, checks the selected network
source, explicitly corrects Linux time, writes the RTC, and checks read-back.
A selected NTP source or `Leap status: Normal` alone does not mean the Linux
clock has finished correcting a large offset. Without automatic steps, a
failed RTC bootstrap can leave Linux hours behind while Chrony slowly slews.

## Reading the diagnostics

| Observation | Meaning |
|---|---|
| `SET TIME!` | OSF was set at that driver probe; the message is retained in the fixed driver and remains in historical logs after recovery |
| `RTC_RD_TIME` fails with `EINVAL` and OSF boot warning | Expected invalid-time rejection in the tested fault state |
| `hctosys: unable to read the hardware clock` | Kernel declined RTC-to-Linux initialization |
| `RTC_UIE_ON: Invalid argument`, followed by a successful polled read | Update interrupts are unavailable; this is not a failed RTC read |
| `RTC_RD_NAME` in util-linux 2.41.5's error text | A [message typo](https://github.com/util-linux/util-linux/blob/v2.41.5/sys-utils/hwclock-rtc.c#L157); the call is `RTC_RD_TIME` |
| `Calculated Hardware Clock drift is 0.000000 seconds` with `--noadjfile` | Not a measurement establishing zero drift |

OSF is status register `0x0F`, bit 7. It records a stopped oscillator and stays
set until cleared; the oscillator may already be running again. See the
[DS3231 datasheet, page 14](https://www.analog.com/media/en/technical-documentation/data-sheets/ds3231.pdf).
Do not use forced raw I2C access while the driver is bound. The tested driver's
debugfs register dump exposed only register zero, not OSF. Manual unbind/read/
rebind is unnecessary for the acceptance tests below and can reinitialize
Linux time when the RTC driver is reattached.
