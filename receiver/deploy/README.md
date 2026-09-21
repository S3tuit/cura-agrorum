# Receiver pilot deployment profile

This directory describes the approved package/configuration inputs. Service
artifacts and their host validation arrive with application implementation.
Nothing here claims the Pi has been installed or qualified.

The production profile uses a dedicated unprivileged `cura-receiver` user and
group. Administrative SSH continues through `cura`. Versioned installed code
lives under `/opt/cura-agrorum`; secrets live at
`/etc/cura-agrorum/receiver-group.json`, owned by the receiver user with mode
0600 beneath trusted directories. Data lives in
`/var/lib/cura-agrorum/receiver.sqlite3`; SQLite temporary files use
`/var/lib/cura-agrorum/tmp` on the same mounted persistent filesystem.

Package the receiver, the protocol-owned `cura_protocol_v2_lora` loader,
cryptography, Linux SPI/GPIO dependencies and the exact generated schema.
Production must work without test modules or a development checkout. The
offline initializer verifies the schema fingerprint and creates a fresh
database for the supplied group. Archive incompatible epochs; never silently
migrate, reinitialize or manufacture group credentials at service startup.

[ApplicationSettings](../cura_receiver/application_settings.py) supplies the
explicit pilot runtime profile using existing validated component settings:

- 1-GiB preventive free-space reserve; WAL/FULL, foreign keys and the existing
  250-ms SQLite busy timeout. Required configuration/data/temp mounts and
  available capacity must be verified before radio use.
- Queue capacity 500, 5-second worker flush, 64-entity wake and batch thresholds,
  4-MiB checkpoint threshold, as already defined by the persistence contract.
- One-minute health period; 10-second internal shutdown deadline, with a
  15-second supervisor stop timeout. Restart delay 5 seconds, at most five
  starts in 60 seconds.
- Documented time/airtime defaults, including 3500-ppm Chrony slew,
  3700-ppm elapsed bound and 10-ppm RTC drift assumption. Network admission
  uses complete error plus observation age, with no separate skew cutoff;
  RTC writes require the five-second source bound through their projected
  operation and actual-stage checks. Full qualification of the elapsed-rate
  envelope beyond the slew cap is [deferred](../ARCHITECTURE.md#chrony-integration).
- RTC bootstrap makes at most three reads within five seconds, supervised by
  a six-second timeout, once per Linux boot before Chrony. Its failure permits
  untrusted offline startup; it never reads receiver secrets or SQLite.

Use the [documented fixed helper](../INTERFACE.md#pilot-linux-backend-and-privilege-boundary)
at `/usr/libexec/cura-agrorum/ds3231-set` and stable `/dev/rtc-ds3231` device.
The helper is root:receiver 0750 with exactly `cap_sys_time=ep`, no setuid;
the receiver parent has no permitted/effective/ambient capabilities. Its
bounding set retains CAP_SYS_TIME and NoNewPrivileges is disabled so the helper
can acquire its file capability. Pin the built helper's digest during
installation; no fabricated digest or fallback privileged shell is allowed.

Chrony uses `/run/chrony/chronyd.sock` with receiver-group access. Install the
[existing validator and drop-in](../hardware/ds3231/README.md#3-configure-time-ownership)
and audit competing writers. The service must require local mounts and bounded
bootstrap completion, not Internet connectivity, and prevent suspend while
active. GPIO/SPI/RTC permissions belong to the dedicated service identity.

Before installation/bench acceptance, measure package/dependency identity,
effective daemon arguments, helper/device permissions and database configuration.
Verify capacity for the declared capture schedule, including health, retries,
profiles, WAL and operating-system headroom; free space alone is not a retention
forecast. No automatic deletion or forwarding is authorized. Test deployments
use dedicated identities, paths and service names. Current-source staging and
source manifests precede target tests; the final installed-service, boot,
storage/time and RF checks remain separate deployment obligations.

Production artifacts now include the receiver entry point, source-bundle builder,
service units, sleep-inhibition authorization and a separate boot-scoped RTC copy.
Build with `python receiver/tools/build_runtime_bundle.py <new-directory>`.
The bundle preserves the runtime package/schema layout and includes a SHA-256
manifest of its actual files plus the baseline Git commit. It contains no tests,
credentials, database, workplan or pre-existing virtual environment. Install that
reviewed tree beneath `/opt/cura-agrorum`, create its `venv`, and install
`receiver/requirements-runtime.txt` with target-compatible dependencies. The
crypto version is the version exercised by the host implementation; Linux
GPIO/SPI and complete target dependency qualification remain deployment gates.
Record installed dependency versions as part of the installation manifest.

Install the two units from `systemd/`, and install `chrony-bootstrap.conf` as a
Chrony unit drop-in alongside the existing Chrony validation drop-in. Install
`49-cura-receiver-inhibit.rules` in the system polkit rules directory. The main
service obtains a blocking sleep inhibitor; failure to obtain it prevents normal
startup. Preserve the dedicated identity/device/socket permissions described
above. No unit depends on `network-online.target`.

Create the trusted deployment environment file
`/etc/cura-agrorum/deployment.env` with `RTC_HELPER_SHA256` equal to the digest of
the installed, reviewed native helper (64 lowercase hexadecimal characters,
as emitted by `sha256sum`) and `RTC_KERNEL_BOUND_US` equal to the
validated target kernel/device operation bound. Both are required inputs; no
sample digest or unqualified bound is substituted. Bootstrap uses the same bound,
a maximum of three reads and a five-second total episode, supervised at six
seconds. Its root-owned `/run/cura-agrorum-rtc-bootstrap/<linux-boot-id>` claim is
created before attempting the copy and survives unit restarts, including a failed
or interrupted attempt. RuntimeDirectory storage disappears at reboot. Bootstrap
never accesses receiver configuration or SQLite, and its provisional clock copy
never establishes receiver time trust or RTC provenance.

Install/order bootstrap before starting Chrony. An already-synchronized kernel
clock makes bootstrap skip copying. Receiver restarts do not rerun the copy.
A failed bootstrap outcome still permits Chrony and receiver startup; runtime
RTC probing and the normal time policy determine health and trust independently.
The service's storage preflight verifies writable data/temp directories on the
same filesystem and the preventive reserve. It does not initialize or repair a
database; use the existing offline database initializer with the supplied group.

Installation, enabling units, changing privileges/devices/Chrony and actual
boot/restart/RF qualification are separate later phases. These files are not
installed by the bundle builder or by host tests.

## Shared paths for isolated installation

Runtime and storage preflight both read the same path environment. The shared
loader provides production defaults when the entire path set is absent. The
unit does not prepopulate path variables, so a partial environment-file override
cannot silently combine test and production paths. To install a test instance,
set all of these in its trusted deployment environment file:

```ini
CURA_RECEIVER_TEST_ROOT=/var/lib/cura-pilot-test
CURA_RECEIVER_CONFIGURATION=/var/lib/cura-pilot-test/config/receiver-group.json
CURA_RECEIVER_DATABASE=/var/lib/cura-pilot-test/data/receiver.sqlite3
SQLITE_TMPDIR=/var/lib/cura-pilot-test/data/tmp
```

Use a new dedicated root/service name for the actual run. The example is not
authorization to reuse an existing directory. Partial overrides, empty or
relative paths, parent traversal, and test roots/paths overlapping the default
production directories fail before device access. The optional test-root guard
is lexical: before installation verify each real directory/file is nonsymlink,
has the intended trusted ownership and cannot alias production storage. Group
file validation still uses the strict protocol loader under the service UID.

Both unit commands inherit these values; SQLite receives the same temporary
directory checked by preflight. Keep the initializer's group/database arguments
identical to those values. Update `EnvironmentFile`, `PYTHONPATH`, executable
paths, `RequiresMountsFor` and `ReadWritePaths` in the dedicated installed unit
to match the staged package and test roots. A path override does not update
systemd mount/permission directives. Keep the production unit unchanged on the
host; never replace its group or database for a test.

Defaults are available for direct invocations only when all three path variables
are absent; `CURA_RECEIVER_TEST_ROOT` requires explicit paths. RTC helper/time
inputs, radio settings, persistence policy and service ownership remain unchanged.


## Pilot Chrony socket permissions

Pinned chronyc creates its reply socket beside `/run/chrony/chronyd.sock`.
The Chrony drop-in starts with a private runtime directory, then grants
`cura-receiver` group access: directory01770 and daemon socket0660, both still
owned by the daemon user. The sticky bit protects daemon-owned entries against
receiver unlink/rename; receiver-created reply sockets can be removed normally.
The receiver unit permits `/run/chrony` writes and orders startup after the local
Chrony start attempt, without requiring synchronization or Internet access.
A missing directory is tolerated by the sandbox; unavailable Chrony remains an
ordinary untrusted runtime input. No membership in the daemon's group is needed.
Verify real tracking, reply-socket cleanup, rejection of daemon-socket removal
and unrelated writes, and permission restoration after daemon restart.

## Deferred post-pilot permissions review

**PERM-001 — deferred, not qualified:** review permissions generally after the
pilot, including replacing direct Chrony socket access with a narrow local
coordinator. The pilot uses the approved shared runtime directory; this leaves
broader Chrony command authority with the receiver identity. Revisit before
post-pilot deployment or earlier if permission requirements change. No coordinator
or command-level privilege isolation is implemented or claimed.
