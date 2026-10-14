# Target-Pi integration tests

These tests exercise receiver software on the deployed Linux stack without a
special physical fault fixture. Follow the shared [test commands and interlocks](../README.md).
Use an isolated current-source stage, dedicated data roots, and preserved service
configuration. Physical wiring/fault procedures live under
[`receiver/hardware/`](../../hardware/).

## Storage faults

For `test_target_bounded_full_recovery` and `test_target_access_recovery` in
[test_ordinary_persistence.py](test_ordinary_persistence.py), run pytest under the
root supervisor with the destructive flags and marked root. The supervisor mounts
only a dedicated 4 MiB tmpfs. `--receiver-storage-user` selects the unprivileged
child (`cura` by default; use the deployed account for installed-UID checks).
The child has no supplementary groups/capabilities and cannot elevate privileges;
only the supervisor can remount its fixture. No service-account sudo grant is needed.

Run other storage cases directly as the unprivileged account. The supervisor reaps
the child, preserves assertion failures, unmounts, and checks restoration. Permission
faults affect only the test database. Tmpfs failures do not measure deployed-storage
latency or physical power-loss durability; the
[persistence benchmark](../../benchmarks/ordinary_persistence/README.md) uses real storage.

## Time mutations and independent elapsed reference

[Nominal time cases](test_runtime_time.py) inspect actual adapters and deployed
configuration. [Mutation cases](test_time_mutations.py) use a root supervisor and
an unprivileged component child. Select the deployed account with
`--receiver-time-user cura-receiver`; the default is `cura`, and root is rejected
as the component identity. Preserve configuration and require independent restoration
of Linux/RTC time, services, permissions, and helper state. Supplied test provenance
exercises policy; it does not calibrate the RTC.

Maximum-slew tests need an independent laptop clock. Run the external bridge's
`--help` from the repository root for its staging, target-interpreter, test-root,
and output options:

```sh
.venv/bin/python receiver/tests/target/time_reference.py --help
```

The bridge verifies its reference before mutation and uses `SSH_PASSWORD` for its
SSH transport. It retains invalid samples too. Each slew direction needs at least
20 minutes and a resolved interval narrower than 200 ppm, with a 40-minute maximum.
A resolved failure cannot be extended to seek a pass; the bounded component run
still requires restoration. Bounds and assertions live in the bridge and tests.
The broader total elapsed-rate assumption remains a
[design limitation](../../ARCHITECTURE.md#limits-that-matter-when-changing-the-design).

RTC power/battery and stuck-bus faults use the
[DS3231 procedures](../../hardware/ds3231/README.md), not these mutation commands.

## Airtime across reboot

Stage current sources with `source-manifest.json` containing the SHA-256 of every
staged source file. From the host, invoke the external controller:

```sh
CURA_PI_PASSWORD=... python receiver/tools/test_airtime_reboot.py \
  --host cura@cura-receiver --remote-source /absolute/isolated/source \
  --python /absolute/target/venv/bin/python \
  --remote-test-root /var/tmp/cura-airtime-unique-run \
  --output /absolute/new/local/captures \
  --confirm-receiver-destructive
```

It creates the marked root, verifies sources before each phase, and performs two
real Pi reboots. One mode supplies an absent component Chrony socket and no RTC
provenance to exercise untrusted recovery without changing time services. Each
reconnect is bounded and requires a changed boot identity. Use trusted ancestry
for root-owned configuration/database files, such as the dedicated `/var/tmp` root.
The read-only fixture's root access does not establish receiver-user permissions.
Keep failed captures through diagnosis and restoration. This component procedure
executes no radio command; [joint RF tests](../../../tests/rf/README.md) own RF runs.
