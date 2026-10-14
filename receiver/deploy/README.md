# Fresh Pi receiver deployment

This sequence installs the receiver on a fresh Raspberry Pi OS Trixie arm64
on the Pi 3 Model B, through service startup and receipt of C6 readings.
Use the nominal [carrier wiring](../hardware/TEST_CARRIER.md); shut down and
disconnect all power before changing hardware connections.
Use Bash on each host and resolve errors before continuing. Retain the laptop
and Pi staging paths when reconnecting. Steps 8 onward create a new installation;
an existing group or database requires a separate update procedure.

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

For this Pi, expect RTC name `rtc-ds1307 1-0068`, I2C device `1-0068`, driver
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
sudo install -D -o root -g root -m 0644 receiver/deploy/chrony-runtime.conf \
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

## 7. Select the qualified RTC operation bound

For a setup matching the [recorded qualification](../hardware/ds3231/tests/i2c_faults/RESULTS.md#2026-10-12-stable-package-qualification),
reuse `RTC_KERNEL_BOUND_US=3000000`.
The [findings](../hardware/ds3231/tests/i2c_faults/RESULTS.md#reusing-an-adopted-operation-bound)
own applicability and remeasurement triggers. When they require a new measurement,
stop here and complete the [I2C fault procedure](../hardware/ds3231/tests/i2c_faults/README.md),
including restoration to nominal wiring, before continuing.

The bound covers complete reads, writes and termination of an in-flight write.
The [interface contract](../INTERFACE.md#recovery-and-scheduling) uses it for
operation admission, late-result rejection and helper reaping. A normal read or
the OSF check does not qualify it.

## 8. Package the matching runtime and group

On your laptop, from the repository root, set `$cura_pi_stage` to the path printed
in step 4 and `$cura_group` to the private `receiver-group.json` retained from
[C6 provisioning](../../firmware/deploy/README.md#2-provision-and-install-the-node-identity):

```bash
set -e -o pipefail
umask 077
cura_pi_stage=/home/cura/cura-stage.XXXXXX
cura_group=/absolute/path/to/receiver-group.json
cura_runtime_tmp=$(mktemp -d /tmp/cura-pi-runtime.XXXXXX)
scp "cura@cura-receiver:$cura_pi_stage/source.commit" "$cura_runtime_tmp/"
cura_commit=$(cat "$cura_runtime_tmp/source.commit")
git worktree add --detach "$cura_runtime_tmp/source" "$cura_commit"
python3 "$cura_runtime_tmp/source/receiver/tools/build_runtime_bundle.py" \
  "$cura_runtime_tmp/bundle"
tar -cf "$cura_runtime_tmp/runtime.tar" -C "$cura_runtime_tmp/bundle" .
(cd "$cura_runtime_tmp" && sha256sum runtime.tar > runtime.tar.sha256)
scp "$cura_runtime_tmp/runtime.tar" "$cura_runtime_tmp/runtime.tar.sha256" \
  "cura@cura-receiver:$cura_pi_stage/"
scp "$cura_group" "cura@cura-receiver:$cura_pi_stage/receiver-group.json"
git worktree remove "$cura_runtime_tmp/source"
```

The detached worktree uses the same committed source as step 4. The builder
requires Git metadata and produces the runtime, schema, deployment artifacts and
`SOURCE_MANIFEST.json`; the extracted Pi source has no Git metadata. Transfer the
existing group separately so the Pi accepts the provisioned C6 identities.
`set -e -o pipefail` stops this shell on a failed command or pipeline.

## 9. Install runtime, credentials and storage

On the Pi, set the recorded stage path. These freshness checks must all succeed;
stop if any production path already exists:

```bash
set -e -o pipefail
cura_pi_stage=/home/cura/cura-stage.XXXXXX
cd "$cura_pi_stage"
sha256sum -c runtime.tar.sha256
sudo test ! -e /opt/cura-agrorum
sudo test ! -e /etc/cura-agrorum
sudo test ! -e /var/lib/cura-agrorum
sudo apt-get install --no-install-recommends polkitd sqlite3
sudo install -d -o root -g root -m 0755 /opt/cura-agrorum
sudo tar --no-same-owner -xf runtime.tar -C /opt/cura-agrorum
source/.venv/bin/python -m pip list --format=freeze > runtime-constraints.txt
sudo python3 -m venv /opt/cura-agrorum/venv
sudo /opt/cura-agrorum/venv/bin/python -m pip install \
  -r /opt/cura-agrorum/receiver/requirements-runtime.txt \
  -c "$cura_pi_stage/runtime-constraints.txt"
sudo /opt/cura-agrorum/venv/bin/python -m pip freeze | \
  sudo tee /opt/cura-agrorum/INSTALLED_DEPENDENCIES.txt > /dev/null
sudo chown -R root:root /opt/cura-agrorum
sudo chmod -R a+rX,go-w /opt/cura-agrorum
sudo install -d -o root -g cura-receiver -m 0750 /etc/cura-agrorum
sudo install -o cura-receiver -g cura-receiver -m 0600 receiver-group.json \
  /etc/cura-agrorum/receiver-group.json
sudo install -d -o cura-receiver -g cura-receiver -m 0700 \
  /var/lib/cura-agrorum /var/lib/cura-agrorum/tmp
```

Code and dependencies are root-owned and readable by the service. Runtime
dependencies use the staged versions; test packages are not installed into this
venv. The service owns its private credentials and data. SQLite temporary files
share the data filesystem, which must retain at least 1 GiB of available space.

## 10. Install RTC access and its write helper

On the Pi:

```bash
sudo tee /etc/udev/rules.d/99-cura-ds3231.rules > /dev/null <<'EOF'
SUBSYSTEM=="rtc", KERNELS=="1-0068", DRIVERS=="rtc-ds1307", ATTRS{name}=="ds3231", OWNER="root", GROUP="cura-receiver", MODE="0660", SYMLINK+="rtc-ds3231"
EOF
sudo chmod 0644 /etc/udev/rules.d/99-cura-ds3231.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --action=change /sys/class/rtc/rtc0
sudo udevadm settle
readlink -f /dev/rtc-ds3231
ls -l /dev/rtc0 /dev/rtc-ds3231
sudo -u cura-receiver env TZ=UTC hwclock \
  --show --utc --noadjfile --rtc=/dev/rtc-ds3231
cc -std=c11 -Wall -Wextra -Werror -O2 \
  /opt/cura-agrorum/receiver/native/ds3231-set.c -o "$cura_pi_stage/ds3231-set"
sudo install -d -o root -g root -m 0755 /usr/libexec/cura-agrorum
sudo install -o root -g cura-receiver -m 0750 "$cura_pi_stage/ds3231-set" \
  /usr/libexec/cura-agrorum/ds3231-set
sudo setcap cap_sys_time=ep /usr/libexec/cura-agrorum/ds3231-set
sudo getcap /usr/libexec/cura-agrorum/ds3231-set
sudo sha256sum /usr/libexec/cura-agrorum/ds3231-set
```

The rule selects the DS3231's actual I2C binding and restores its alias and
receiver-group access at boot. Require the alias to resolve to the RTC verified
in step 3, mode `0660` with owner/group `root:cura-receiver`, and a successful
read as the service user.

The native helper receives only `cap_sys_time=ep`; the receiver stays
unprivileged. Require exactly that capability and compare its digest/build with
the [RTC qualification](../hardware/ds3231/tests/i2c_faults/RESULTS.md#2026-10-12-stable-package-qualification)
before reusing the bound. The [helper contract](../INTERFACE.md#pilot-linux-backend-and-privilege-boundary)
owns privilege and protocol checks. Do not execute the helper as a version check;
a valid argument writes the RTC.

## 11. Initialize the group-bound database

Keep the receiver stopped. Run the initializer as the service user:

```bash
sudo -u cura-receiver env \
  PYTHONPATH=/opt/cura-agrorum/receiver:/opt/cura-agrorum/protocol/protocol-v2-lora/python \
  /opt/cura-agrorum/venv/bin/python - <<'PY'
from pathlib import Path
from cura_protocol_v2_lora.receiver_group import load_receiver_group
from cura_receiver.database_initializer import initialize_database

group = load_receiver_group(Path('/etc/cura-agrorum/receiver-group.json'))
result = initialize_database(
    Path('/var/lib/cura-agrorum/receiver.sqlite3'),
    group.group_id,
    known_empty_airtime=False,
)
if not result.cleanup_complete:
    raise SystemExit(f'Initialization cleanup pending: {result.cleanup_pending_path}')
print('Database initialized; group validated.')
PY
rm "$cura_pi_stage/receiver-group.json"
```

The strict loader checks credentials under the eventual service identity; the
initializer verifies the packaged schema and refuses an existing database.
`known_empty_airtime=False` selects conservative recovery for unknown radio
history. Change it to `True` only with independent evidence of known-empty radio
airtime history and exclusive radio control through first startup;
a fresh OS/database alone does not establish that. The
[airtime contract](../INTERFACE.md#initial-airtime-commissioning) owns commissioning
and durable history. With conservative recovery, initial ACK transmission may
remain inhibited while the airtime budget recovers. Resolve any initializer
failure before starting the service; retain the original private group backup.

## 12. Configure and enable the services

Use the bound selected in step 7 (`3000000` for the matching qualification).
On the Pi:

```bash
cura_rtc_bound_us=3000000
cura_helper_sha=$(sudo sha256sum /usr/libexec/cura-agrorum/ds3231-set | cut -d ' ' -f 1)
sudo install -o root -g root -m 0600 /dev/null /etc/cura-agrorum/deployment.env
sudo tee /etc/cura-agrorum/deployment.env > /dev/null <<EOF
RTC_HELPER_SHA256=$cura_helper_sha
RTC_KERNEL_BOUND_US=$cura_rtc_bound_us
CURA_RECEIVER_STARTUP_BUDGET_US=12000000
EOF
sudo install -D -o root -g root -m 0644 \
  /opt/cura-agrorum/receiver/deploy/49-cura-receiver-inhibit.rules \
  /etc/polkit-1/rules.d/49-cura-receiver-inhibit.rules
sudo install -D -o root -g root -m 0644 \
  /opt/cura-agrorum/receiver/deploy/systemd/60-cura-persistent.conf \
  /etc/systemd/journald.conf.d/60-cura-persistent.conf
sudo install -D -o root -g root -m 0644 \
  /opt/cura-agrorum/receiver/deploy/systemd/chrony-bootstrap.conf \
  /etc/systemd/system/chrony.service.d/cura-bootstrap.conf
sudo install -o root -g root -m 0644 \
  /opt/cura-agrorum/receiver/deploy/systemd/cura-rtc-bootstrap.service \
  /opt/cura-agrorum/receiver/deploy/systemd/cura-receiver.service \
  /etc/systemd/system/
sudo systemctl restart systemd-journald.service
sudo journalctl --flush
sudo systemctl daemon-reload
sudo systemctl enable cura-rtc-bootstrap.service cura-receiver.service
```

The environment pins the installed helper and measured RTC bound. The 12-second
persistence startup budget is the selected [pilot benchmark value](../benchmarks/startup_readiness/README.md),
with its stated fixture limits. Production paths use the application defaults.
Polkit authorizes the service's sleep inhibitor; persistent journaling retains
startup/failure records across boots. Bootstrap completes before Chrony, and
the receiver starts after both local service attempts without requiring Internet.

## 13. Boot and verify operation

Require nominal hardware wiring, an attached radio antenna and exclusive access
to the radio. Complete the [Chrony policy checks](CHRONY.md), including the audit
for competing system-clock and RTC writers, before startup. The reboot activates
the RTC rule and the complete service ordering:

```bash
sudo reboot
```

Reconnect from your laptop, then inspect this boot on the Pi:

```bash
ssh cura@cura-receiver
```

```bash
systemctl --no-pager --full status cura-rtc-bootstrap.service chrony.service cura-receiver.service
systemctl show cura-receiver.service -p MainPID -p NRestarts -p ActiveState -p SubState
sudo journalctl -b -u cura-rtc-bootstrap.service -u chrony.service \
  -u cura-receiver.service -o short-monotonic --no-pager
ls -l /dev/rtc0 /dev/rtc-ds3231
systemd-inhibit --list --no-pager
sudo chronyc -h /run/chrony/chronyd.sock tracking
```

Require an active receiver without repeated restarts, its sleep inhibitor, and
a `receiver_startup` record with both `outcome` and `application_outcome` equal
to `SUCCESS`. Check bootstrap completion before Chrony startup; its result may
be `ALREADY_SYNCHRONIZED` when Linux already accepted the RTC. Bootstrap alone
does not establish receiver time trust. Stop and resolve startup, device or
permission failures before continuing. While the receiver runs, let it own time
recovery and RTC writes; stop it before manual clock corrections or RTC writes.

Health is persisted at startup and every minute. After allowing at least one
health interval and a persistence flush, query the latest receiver instance:

```bash
sudo -u cura-receiver sqlite3 -readonly -header -column \
  /var/lib/cura-agrorum/receiver.sqlite3 <<'SQL'
SELECT lower(hex(h.receiver_instance_id)) AS instance, h.health_sequence,
       r.code AS radio, q.code AS time, t.code AS rtc, p.code AS persistence
FROM receiver_health AS h
JOIN radio_state_codes AS r ON r.id = h.radio_state_id
JOIN system_time_quality_codes AS q ON q.id = h.system_time_quality_id
JOIN rtc_health_codes AS t ON t.id = h.rtc_health_id
JOIN persistence_admission_state_codes AS p ON p.id = h.persistence_admission_state_id
WHERE h.receiver_instance_id = (
    SELECT receiver_instance_id FROM receiver_instances ORDER BY instance_ordinal DESC LIMIT 1
)
ORDER BY h.health_sequence DESC LIMIT 1;
SQL
```

Match `instance` to this boot's successful startup record. On the healthy,
network-synchronized setup, expect `RX_SINGLE`, `CHRONY_SYNCED`, `PRESENT` and
`AVAILABLE`; brief radio activity can produce another transient radio state.
Missing/stale rows or persistent degraded states require investigation.

With the matching [C6 firmware](../../firmware/deploy/README.md#5-flash-production-firmware)
running, allow a sample and persistence flush, then inspect recent readings:

```bash
sudo -u cura-receiver sqlite3 -readonly -header -column \
  /var/lib/cura-agrorum/receiver.sqlite3 <<'SQL'
SELECT lower(hex(m.node_id)) AS node_id, m.message_id, m.sample_id,
       printf('0x%04x', m.flags) AS flags, m.soil_0_mv, m.soil_1_mv,
       m.soil_temp_0_centi_c, m.soil_temp_1_centi_c
FROM reading_messages AS m
JOIN message_profiles AS p
  ON p.receiver_instance_id = m.first_receiver_instance_id
 AND p.occurrence_sequence = m.first_occurrence_sequence
WHERE m.first_receiver_instance_id = (
    SELECT receiver_instance_id FROM receiver_instances ORDER BY instance_ordinal DESC LIMIT 1
)
ORDER BY p.received_at_monotonic_us DESC LIMIT 5;
SQL
```

Require the provisioned node ID and advancing samples. `flags` determines which
sensor values are valid; the [reading format](../../protocol/protocol-v2-lora/README.md#reading-payload)
owns its interpretation. Stored readings confirm receiver ingestion, while C6
delivery reports establish whether it received ACKs. Detailed installed-system
and RF qualification remain owned by [TESTING.md](../TESTING.md).
