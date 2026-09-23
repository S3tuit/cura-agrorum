# Receiver application host verification — 2026-09-18

PASS for the receiver application implementation and host-verification batch.
This is not Pi installation, installed privilege/systemd qualification, RF
acceptance or electrical power-loss proof. No device/clock mutation or service
installation occurred during this host run.

Source is the uncommitted working tree based on
`33a1b2fbb9ae2d9d6e3905fcb5c0f362b25244b5`, identified by actual file hashes in
[SOURCE_MANIFEST.json](SOURCE_MANIFEST.json). It includes production code,
contracts, schemas, generators, receiver host/support tests and protocol/C codec
inputs. Python/kernel and relevant Python package versions are recorded there.
[PACKAGE_MANIFEST.json](PACKAGE_MANIFEST.json) independently identifies the 88
files in the production source bundle; the baseline commit alone is not its
source identity.

| Check | Result |
|---|---|
| `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q -c receiver/pytest.ini receiver/tests/host` | **3386 passed in 31.64 s** |
| `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q protocol/protocol-v2-lora/tests` | **258 passed in 5.34 s**, including generated codec/crypto, sanitized and bounded fuzz checks |
| `.venv/bin/python receiver/tools/generate.py --validate-only` | PASS |
| `.venv/bin/python receiver/tools/generate.py --check` | PASS |
| Python compilation of runtime and migrated hardware/peer harnesses | PASS for syntax only; hardware harness execution NOT RUN |
| Production bundle verification | 88 file hashes verified; no tests or receiver-group credentials. Both module entry points load with bundle-only PYTHONPATH outside the checkout; offline initializer creates a temporary database using the packaged schema. |
| `systemd-analyze verify receiver/deploy/systemd/cura-receiver.service receiver/deploy/systemd/cura-rtc-bootstrap.service` | Incomplete: exactly two missing-executable reports for the uninstalled `/opt/cura-agrorum/venv/bin/python`. No installed-unit PASS is claimed. |
| `git diff --check` | PASS |

The isolated bundle used `/tmp/cura-application-final-f8u3_b8g/runtime`; it is a
rebuildable temporary artifact, not an installed receiver. Its manifest SHA-256
is `e76f6fc094a7ae8d41a3457a65e0f10e8403a2930b965977dea3787d72fcef77`.
The retained manifests and test sources are the durable verification inputs;
transient pytest databases/build output are not promoted as deployment evidence.

RTC regression timing uses fake ports and a manual monotonic clock. A stop during
a 2.9-second pre-read returns before invalidation/write, leaving 7.1 seconds of
the original ten-second budget. Separate tests retain conservative provenance
and actual dispositions on post-write/read-retry/unknown-commit cancellation.
Application lifecycle tests use real SQLite and actual SIGKILL at named process
boundaries; they do not establish hardware timing or physical power durability.

Intermediate failures included stale immediate-radio-state expectations,
bootstrap's misplaced `time` import, incorrect new test fixture SQL/UUID/state
inputs and an unbounded manual-clock test that waited before its scheduled RTC
retry. These were corrected; no timeout or acceptance threshold was relaxed.
The original synchronous RTC overrun and chained radio-poll reproductions were
host defect demonstrations, not passing deployment evidence. Their approved
production regressions are included in the final receiver run.

See [APPLICATION_COVERAGE.md](../../APPLICATION_COVERAGE.md) for owner/assertion
mapping. Installed helper digest/kernel bound, dedicated UID permissions,
Chrony arguments, boot/restart/suspend behavior, current Pi storage/time tests
and the selected RF scenarios remain outstanding deployment obligations.
