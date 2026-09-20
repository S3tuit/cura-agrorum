# Pilot bench operator runbook

This runbook covers the isolated pilot rehearsal. It does not authorize field
installation, aggregate deployment acceptance, or an unattended soak. Use the
public identity/path inventory retained with the selected run; never substitute
a default production root for an isolated test root.

## Current rehearsal inventory

- Pi: `cura@10.86.160.140`, board `00000000e0027211`. Reconfirm the address after
  reconnecting; SSH/scp uses `-F /dev/null` and host-key alias `cura-receiver`.
- Service: `cura-pilot-vuh7p781.service`; account `cura-receiver`.
- Installed package: `/opt/cura-pilot-vuh7p781`; data and temporary storage:
  `/var/lib/cura-pilot-vuh7p781/data` and its `tmp` subdirectory.
- Private configuration: `/var/lib/cura-pilot-vuh7p781/config/receiver-group.json`.
  Its matching database is `data/receiver.sqlite3`. Do not copy keys into logs.
- C6: `/dev/ttyUSB0`, verified BASE MAC `cc8da2fc0224`, disposable node
  `fb25c40751b35256`. Verify the MAC again if UART enumeration changes.
- Nominal carrier: soil GPIO0/1, gate GPIO2, DS18B20 GPIO3, BME SDA21/SCL22;
  physical DS2 `DF00000050F93828` is logical channel0, DS1 `7E000000540FA728`
  is channel1. Radio pins and all fixture connections follow
  [SENSOR_CARRIER.md](../../firmware/test_apps/on_device/SENSOR_CARRIER.md).

The current build seal, public configuration hash and staged source manifest
belong to each run's capture. A successful older run is not authority to accept
changed binaries, credentials, device identity or fixture state.

## Prepare and start

1. Establish exclusive C6 UART and Pi radio ownership and operator access to all
   C6 power sources. Confirm nominal wiring/antennas before applying power.
2. Follow the [receiver deployment procedure](../../receiver/deploy/README.md)
   for a source-identified package, pinned dependencies/helper and trusted paths.
   Stage the current local tree in a fresh Pi directory. Use the RF helper's
   package/unit/environment/board/UID checks before starting a test service.
3. Provision test identities through the existing protocol tools. Preserve valid
   node counters during ordinary rebuilds. Full erase requires identity rotation
   and removal of every power/back-power source before reinitialization; never
   restore old node NVS or silently reuse a retired identity.
4. Keep backups bound to their group and schema. Ordinary startup neither creates
   a database nor repairs missing airtime history. Explicit zero-airtime test
   preparation requires the [complete silence procedure](README.md#production-receiver-service-rf-020);
   it is not a recovery shortcut after transmissions.
5. Declare the entire bounded RF episode and record operator airtime admission.
   Use the selected explicit RF runner. It verifies installed firmware/storage,
   starts the receiver, checks its prerequisites, then starts the C6. No manual
   service start alone establishes clock trust, airtime readiness or RF success.

## Observe and respond

No fixed operator monitoring cadence is specified. For the 24-hour bench, the
operator plans informal check-ins 15, 30 and 60 minutes after startup, then at
the end. These are not acceptance criteria or a mandatory schedule.

Use the staged `service_probe.py observe` through `InstalledService.probe` as
`cura-receiver`, and the run's immutable captures. Check exact current instance,
radio health, clock observations, accepted/persisted readings, ACK outcomes and
available disk capacity. `systemctl status cura-pilot-vuh7p781.service` is a
process check only. The configured preventive reserve is 1 GiB; no automatic
pruning or uploading is provided. Missing readings/ACKs require evidence review,
not an automatic failed-case retry.

Stop the bounded episode and preserve its first failure if radio access is lost,
recovery repeats, TX is uncertain, storage is full/corrupt, RTC/time becomes
invalid for the selected test, or the expected sensor validity fails. Do not
change assertions or erase evidence to make a run pass.

## Stop and preserve

The runner stops the C6 in its ROM loader and verifies its MAC, then stops the
receiver and checks the exact instance's clean marker/generation, inactive empty
service and released sleep inhibitor. A wrapper SIGTERM is accepted only with
all those checks. SSH disconnection or `systemctl` output alone proves neither
endpoint safe.

For an operator stop, use `sudo systemctl stop cura-pilot-vuh7p781.service` on the
Pi. If node control is lost, remove **all** C6 power, including USB and external
or signal back-power paths, and confirm the physical stop. Do not assume a
sleeping node is off: production wakes again after 900 seconds. Never reconnect
or restart a failed episode automatically.

Capture the receiver with SQLite's consistent backup API under its service UID;
the staged snapshot helper verifies integrity/foreign keys and retains a hash.
Use a new destination for each snapshot. Do not copy just a live main database
while ignoring its WAL. Capture the node's full reviewed LittleFS partition only
after the complete episode, decode a host copy, preserve the original bytes and
leave the node stopped. This is not permission to restore node flash/counters.

## Restore or replace offline

Follow [last-resort restoration](../../receiver/db/README.md#last-resort-operator-restoration)
with the service stopped. Preserve the original database/WAL/SHM together in an
identified archive. Validate group, schema, source applicability and integrity
before installing a chosen snapshot; preserve ownership and the temporary path.
A historical snapshot cannot silently roll back airtime or identity history.

For a deliberate new test database, keep the old group-bound history archived,
install only the reviewed candidate, and start a new receiver instance. Recheck
all prerequisites before starting the C6. The rehearsal separately exercised
exact-snapshot restoration and explicit fresh-history preparation; neither is
permission for unrestricted rollback. End the bench handover with the service
stopped and C6 stopped, unless a separately admitted episode accounts for their
continued operation. Physical power-loss campaigns remain deferred NOT RUN.
