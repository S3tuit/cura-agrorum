# Receiver tests

The repository must pass its implemented tests. After software or hardware changes,
contributors rerun the necessary tests using their judgment. Keep regression reasons
beside the relevant test or component; no pass ledger or test-selection specification
is required. See [receiver setup](../README.md#python-setup) for dependencies.

## Run tests

From the repository root:

| Command | Selection |
| --- | --- |
| `make test-receiver` or `make test-receiver-host` | Complete deterministic host suite |
| `make test-receiver-hardware` | Fast, non-destructive Pi/component cases |
| `make test-receiver-hardware-slow` | Slow, non-destructive Pi/component cases |
| `make test-receiver-hardware-all` | Both safe sets |
| `make test-receiver-hardware-destructive` | Explicitly confirmed invasive cases |
| `make test-rf-host` | Host checks for joint RF tooling |

Hardware targets exclude `rf_peer`. Fixture-specific options still apply; use an
explicit path/marker selection for a single family. For example:

```sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest \
  -c receiver/pytest.ini receiver/tests/host/test_persist_queue.py
.venv/bin/python receiver/tools/generate.py --check
```

Make disables unrelated pytest plugin autoload. Direct discovery from `receiver/`
is host-only; from the repository root, `-c receiver/pytest.ini` alone does not
constrain discovery, so supply a test path. Hypothesis is explicitly loaded;
ordinary receiver tests do not load pytest-embedded.

## Where tests belong

- [`host/`](host/): software behavior with deterministic clocks, real temporary
  SQLite files, and narrow fault seams. Process-kill tests check process crashes;
  they do not establish physical power-loss durability.
- [`target/`](target/): receiver software and deployed-Pi integration. Its
  [procedure](target/README.md) covers storage, time mutation, and reboot fixtures.
- [`../hardware/`](../hardware/): physical component tests, with preparation and
  restoration under `<component>/tests/<family>/`. Start with the
  [DS3231](../hardware/ds3231/README.md) or
  [radio carrier](../hardware/radio/tests/carrier/README.md) procedure.
- [`support/`](support/README.md): shared builders, independent models, fakes, and
  coordination helpers. Production code never imports test support.

[Joint RF tests](../../tests/rf/README.md) coordinate Pi/S3 behavior; protocol
codec tests remain in [`protocol-v2-lora/tests/`](../../protocol/protocol-v2-lora/tests/).
Test source and assertions describe individual cases; this guide does not duplicate
that inventory. Use reviewed vectors/independent oracles, not the production
implementation itself, to calculate expected values.

## Hardware and destructive runs

All Pi/component cases carry `hardware` and run serially on a Pi with
`--receiver-hardware`. `slow`, `destructive`, `rf_peer`, and the radio fixture
markers refine selection. Shared policy in
[`support/receiver_pytest.py`](support/receiver_pytest.py), registered by
[`../conftest.py`](../conftest.py), applies to both target and component trees.
Selected missing hardware fails; runtime skips cannot turn it into success.

Use isolated configuration, service names, databases, and storage. Stage current
sources and install the required target dependencies before running; installed
service assertions must use its actual account and sandbox. Physical actions and
fixture-specific confirmations belong to the selected procedure.

Destructive runs require an absolute dedicated directory containing the exact
sentinel line and the explicit opt-in:

```sh
receiver_test_root=/absolute/path/to/dedicated/receiver-test-root
mkdir -p "$receiver_test_root"
printf '%s\n' 'CURA AGRORUM RECEIVER TEST ROOT' \
  > "$receiver_test_root/.cura-receiver-test-root"
make test-receiver-hardware-destructive \
  CONFIRM_RECEIVER_DESTRUCTIVE=YES \
  RECEIVER_TEST_ROOT="$receiver_test_root"
```

Use the ownership required by the family (root-owned for root-supervised fixtures).
This shared opt-in does not replace radio selector, RTC wiring, or other specific
interlocks. Follow the family command when tests must run alone.

Restore changed host/device state and independently check restoration. Uncertain
cleanup fails and stops dependent cases. Keep failed outputs and database/WAL/SHM
sets together while investigating; discard temporary captures after diagnosis and
restoration, retaining useful lessons beside their code or procedure.
