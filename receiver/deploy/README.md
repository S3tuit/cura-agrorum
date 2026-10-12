# Fresh Pi receiver deployment

This sequence prepares Raspberry Pi OS Trixie arm64 on the Pi 3 Model B for
the receiver. It ends with the prerequisites for measuring the RTC operation
bound; production service installation follows that qualification.
Use the nominal [carrier wiring](../hardware/TEST_CARRIER.md); shut down and
disconnect all power before changing hardware connections.

## 1. Connect and update the OS

From your laptop:

```bash
ssh cura@cura-receiver
```

Run the remaining commands on the Pi:

```bash
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install --no-install-recommends util-linux-extra
```

`update` refreshes package indexes; `upgrade` installs updates from the configured
repositories. `util-linux-extra` provides `hwclock` for RTC checks. The reboot
below loads any updated kernel.

## 2. Enable RTC and radio interfaces

Keep time synchronization services stopped until their configuration is in place:

```bash
sudo systemctl mask --now systemd-timesyncd.service chrony.service
sudo nano /boot/firmware/config.txt
```

Under the active `[all]` section, ensure these settings appear once. Update
existing entries rather than adding conflicting copies:

```ini
dtparam=i2c_arm=on
dtoverlay=i2c-rtc,ds3231
dtparam=spi=on
```

I2C and the overlay bind the DS3231 to the kernel RTC driver. SPI exposes the
radio interface. The [RTC installation guide](../hardware/ds3231/README.md)
owns module-specific wiring and driver details.

```bash
sudo reboot
```

Reconnect from your laptop after boot:

```bash
ssh cura@cura-receiver
```

## 3. Verify the device bindings

On the Pi:

```bash
uname -r
dpkg-query -W 'linux-image*'
cat /sys/class/rtc/rtc0/name
readlink -f /sys/class/rtc/rtc0/device
readlink -f /sys/class/rtc/rtc0/device/driver
ls -l /dev/rtc0 /dev/spidev0.0 /dev/gpiochip0
systemctl is-enabled systemd-timesyncd.service chrony.service
systemctl is-active systemd-timesyncd.service chrony.service
```

For this Pi, expect RTC name `rtc-ds1307`, I2C device `1-0068`, driver
`/sys/bus/i2c/drivers/rtc-ds1307`, and the listed RTC/SPI/GPIO device nodes.
Both time services should be masked and inactive; a nonzero exit status from
these `systemctl` queries is expected. Resolve missing or unexpected devices
before continuing.

OSF handling and short battery retention have been checked on the setup recorded
in the [boot-cycle findings](../hardware/ds3231/tests/boot_cycle/RESULTS.md#reuse).
Reuse that result for a matching setup. If your hardware or OS/kernel differs,
rerun the [four-boot procedure](../hardware/ds3231/tests/boot_cycle/README.md)
before enabling time synchronization or writing over the initial RTC state.

## 4. Stage the source and prepare the test environment

From the repository root on your laptop, select a commit containing these
deployment files and the component test layout (`HEAD` below). Uncommitted
changes are not included:

```bash
umask 077
cura_source_tmp=$(mktemp -d /tmp/cura-pi-source.XXXXXX)
git rev-parse HEAD > "$cura_source_tmp/source.commit"
git archive --format=tar HEAD > "$cura_source_tmp/source.tar"
(cd "$cura_source_tmp" && sha256sum source.tar > source.tar.sha256)
cura_pi_stage=$(ssh cura@cura-receiver 'mktemp -d /home/cura/cura-stage.XXXXXX')
scp "$cura_source_tmp/source.tar" "$cura_source_tmp/source.tar.sha256" \
  "$cura_source_tmp/source.commit" "cura@cura-receiver:$cura_pi_stage/"
printf 'Pi staging directory: %s\n' "$cura_pi_stage"
ssh cura@cura-receiver
```

On the Pi, replace the directory below with the printed path:

```bash
cd /home/cura/cura-stage.XXXXXX
sha256sum -c source.tar.sha256
mkdir source
tar -xf source.tar -C source
cd source
sudo apt-get install --no-install-recommends \
  python3-venv python3-dev build-essential libcap2-bin \
  libgpiod-dev python3-libgpiod chrony
python3 -m venv .venv
.venv/bin/python -m pip install \
  -r receiver/requirements-runtime.txt -r receiver/requirements-test.txt \
  ./protocol/protocol-v2-lora/python
.venv/bin/python -m pip freeze > ../installed-dependencies.txt
dpkg-query -W 'linux-image*' gcc libc6 libcap2-bin libgpiod3 \
  python3-libgpiod chrony > ../installed-packages.txt
cc --version > ../compiler.txt
```

Require the archive checksum check to pass before extraction. The source stage
and receipts identify what will be measured. The compiler and `setcap` build the
test's temporary native RTC helper. System Python/libgpiod controls the physical
fault GPIOs; the venv supplies receiver and test dependencies. Chrony stays masked
during installation. Keep this stage outside `/tmp` so it survives reboots.

## 5. Create the service identity and configure Chrony

From the staged source root on the Pi:

```bash
sudo adduser --system --group --no-create-home \
  --home /var/lib/cura-agrorum --shell /usr/sbin/nologin cura-receiver
sudo usermod -aG gpio,spi cura-receiver
sudo cp -an /etc/chrony/chrony.conf /etc/chrony/chrony.conf.before-cura
sudo install -o root -g root -m 0644 receiver/deploy/chrony.conf /etc/chrony/chrony.conf
sudo install -D -o root -g root -m 0644 receiver/tools/check_chrony.py \
  /usr/libexec/cura-agrorum/check-chrony.py
sudo install -D -o root -g root -m 0644 receiver/hardware/ds3231/chrony-runtime.conf \
  /etc/systemd/system/chrony.service.d/cura-runtime.conf
sudo systemctl daemon-reload
sudo systemctl unmask chrony.service
sudo systemctl enable --now chrony.service
chronyc -v
sudo chronyc -h /run/chrony/chronyd.sock sources -v
sudo chronyc -h /run/chrony/chronyd.sock tracking
```

The dedicated account owns the eventual receiver process. Create its group before
starting the drop-in, which grants that group Chrony socket access. Chrony is the
normal Linux clock writer; leave `systemd-timesyncd` masked. The configuration
validator runs before each Chrony start. Require `chronyc` version 4.6.1, as used
by the current adapter. [Chrony policy and checks](CHRONY.md) own the configuration
rationale, effective launch, socket permissions and competing-writer audit.

## 6. Initialize the RTC from verified network time

Require a selected network source (`^*`), normal synchronized tracking and a
recent reference consistent with independent UTC. Resolve poor or missing
reference evidence before proceeding. With the receiver stopped, correct Linux
time, wait for a small remaining correction, then write and read the RTC:

```bash
sudo chronyc -h /run/chrony/chronyd.sock makestep && \
  sudo chronyc -h /run/chrony/chronyd.sock waitsync 30 0.1 0 2 && \
  sudo hwclock --systohc --utc --noadjfile --rtc=/dev/rtc0 && \
  sudo hwclock --show --verbose --utc --noadjfile --rtc=/dev/rtc0
```

Require the entire sequence to succeed. The wait checks remaining Linux clock
correction; reference-error limits and runtime trust remain defined by the
[time contract](../ARCHITECTURE.md#time-model). This supplies the valid, advancing
RTC and stable Chrony state required by the controller-fault measurement.

## 7. Prepare to qualify the RTC operation bound

From the staged source root on the Pi:

```bash
cura_rtc_root=$(sudo mktemp -d /var/lib/cura-rtc-qualification.XXXXXX)
sudo chmod 0755 "$cura_rtc_root"
printf 'CURA AGRORUM RECEIVER TEST ROOT\n' | \
  sudo tee "$cura_rtc_root/.cura-receiver-test-root" > /dev/null
sudo chmod 0600 "$cura_rtc_root/.cura-receiver-test-root"
printf 'RTC qualification root: %s\n' "$cura_rtc_root"
pinctrl get 17
pinctrl get 27
pinctrl get 2
pinctrl get 3
```

The root is dedicated and root-owned. Its traversal permission lets the test's
unprivileged `cura` child reach its own session files; it grants no write access
to other users. The sentinel distinguishes it from production data. Require the
spare fault GPIOs to be unused inputs and the normal bus pads to be high.

Stop here before fault injection. Follow the
[I2C fault procedure](../hardware/ds3231/tests/i2c_faults/README.md) for unpowered
shunt installation, operator confirmation, the four read/write cases and complete
restoration. The fixture builds/pins its temporary helper and restores access;
no production helper or receiver service needs to be running for this measurement.

The [interface contract](../INTERFACE.md#pilot-linux-backend-and-privilege-boundary)
requires a finite RTC operation bound for admission, late-result rejection and
timed-out helper reaping. The procedure checks a **3-second acceptance limit**
under its defined faults, including in-flight write termination. It qualifies
that envelope; it does not derive a universal kernel worst-case bound. A successful
normal read or the OSF boot check cannot replace this measurement.

Record outcomes, maximum complete-operation durations, helper/source identity,
hardware/wiring, kernel/packages and successful restoration in the
[owning findings](../hardware/ds3231/tests/i2c_faults/RESULTS.md#reusing-an-adopted-operation-bound).
Once qualified, reuse the recorded limit for an applicable matching setup. That
document owns the conditions for remeasurement. The retained findings concern
the earlier beta package; the current stable package's bound remains to be
qualified before supplying `RTC_KERNEL_BOUND_US=3000000` for production.
