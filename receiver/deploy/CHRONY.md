# Chrony policy and operator checks

[The deployment sequence](README.md#5-create-the-service-identity-and-configure-chrony)
owns installation commands. [chrony.conf](chrony.conf) is the tracked fresh-OS
configuration input; it replaces the former inline example. Configure services
while Chrony is stopped and start it through systemd afterward.

## Configuration policy

Chrony is the sole normal Linux clock writer. The input selects a network pool,
a persistent drift file, slew-only leap handling, a 3500-ppm slew cap, no UDP
command socket, and `/run/chrony/chronyd.sock` for local control. Audit included
configuration, effective daemon arguments and other services/timers/cron/shutdown
hooks for competing writers. Keep `systemd-timesyncd` masked.

Omit `makestep`, `initstepslew`, `rtcsync` and `rtcfile` from daemon configuration.
An explicit operator `chronyc makestep` during setup is separate from an automatic
step directive. `rtcsync` can enable kernel RTC writes even without an `hwclock`
service; automatic RTC writers must remain absent. The installed
[validator](../tools/check_chrony.py) checks expanded configuration before startup.

## Effective launch and socket permissions

The [drop-in](../hardware/ds3231/chrony-runtime.conf) fixes the checked
`/etc/chrony/chrony.conf` path and launches `/usr/sbin/chronyd -F 1 -f ...` without
reading `DAEMON_OPTS`. Its `!` launch prefix preserves the vendor privilege-drop
behavior with inherited `User=_chrony`. The dedicated `cura-receiver` group must
exist before startup because the post-start command grants its socket access.

Inspect the effective unit, real process arguments and permissions:

```bash
systemctl show chrony.service -p ExecStartEx -p ExecStartPre -p User \
  -p RuntimeDirectory -p RuntimeDirectoryPreserve -p MainPID
systemctl --no-pager --full status chrony.service
sudo ls -ld /run/chrony
sudo ls -l /run/chrony/chronyd.sock
sudo chronyc -h /run/chrony/chronyd.sock tracking
```

Require the documented launch with `flags=no-setuid`, `RuntimeDirectory=chrony`,
`RuntimeDirectoryPreserve=yes`, and successful actual startup. The daemon owns the
sticky directory (01770) and socket (0660), with group `cura-receiver`. `chronyc`
creates a reply socket beside the daemon socket, so directory access is required.
The SSH account may need `sudo` to query it.

Keep the ownership/mode adjustment in one post-start command: systemd reapplies
runtime-directory ownership/mode before each separate command. Reboot clears
`/run`; preserving the directory across separate Chrony stop/start keeps an
already-running receiver's bind mount referring to the same object. Once the
receiver is installed, verify unchanged directory device/inode and recovered
tracking from the same receiver PID/mount namespace across both restart and
separate stop/start. Restart alone does not qualify that lifetime boundary.

Never delete or recreate the directory while the receiver uses it. If cleanup
requires recreation, stop both services, then start Chrony and a new receiver
namespace. Ordinary exits remove their socket entries; killed processes may
leave entries until safe cleanup or reboot. The permission redesign remains
PERM-001 in [the time architecture](../ARCHITECTURE.md#chrony-integration).

## Initialize RTC time before deployment

Follow [deployment step 6](README.md#6-initialize-the-rtc-from-verified-network-time)
with the receiver stopped. Require a selected network source, recent reference
consistent with independent UTC and normal synchronized tracking. Correct Linux
and wait for its remaining correction before copying to the verified RTC path.
A selected source or normal leap status alone does not establish that a large
correction has finished. The wait bounds remaining correction, not total UTC
reference error. Require a successful RTC write and read-back.

The boot-cycle check establishes validity/recovery without establishing accurate
UTC. Production time trust, error bounds and durable RTC provenance remain owned
by [ARCHITECTURE.md](../ARCHITECTURE.md#time-model) and
[INTERFACE.md](../INTERFACE.md#ds3231-control-interface). Trusted operators keep
configuration unchanged while the service runs; the receiver does not manage
Chrony's service lifecycle or make configuration snapshots.
