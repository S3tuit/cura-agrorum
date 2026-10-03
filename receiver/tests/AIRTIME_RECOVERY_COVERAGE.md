# Airtime recovery coverage

This note maps airtime recovery scenarios to executable tests and explains how
to rerun them. The authoritative behavior and encoding are documented in
[Receiver TX-airtime budget](../ARCHITECTURE.md#receiver-tx-airtime-budget) and
[Fixed-charge airtime snapshots and recovery](../INTERFACE.md#fixed-charge-airtime-snapshots-and-recovery).
The wider validation requirements are in
[Receiver TX-airtime policy](../TESTING.md#receiver-tx-airtime-policy).

## Test coverage

- [Independent models](host/test_tx_airtime_model.py): rational-time and
  integer-time models under `support/models/` check finite recovery envelopes,
  exact clock edges, generated histories and targeted scenarios. Deliberately
  broken controls check that the oracles detect the corresponding defects.
- [Production histories](host/test_tx_airtime_properties.py): the production
  policy and real SQLite are checked against a separate physical transmission
  history oracle at state changes and crossed expiration edges. Cases include
  fresh monotonic origins, rate changes, bounded UTC steps/errors, refunds,
  unknown/failed commits and restarts.
- [Regression requests](support/data/airtime_regression_requests.json): a fixed
  sequence of lost threshold saves, trusted restarts and untrusted fallback
  exposed an over-budget rolling hour in an unsafe predecessor policy. The
  rational model and production test replay the same requests; the saved
  predecessor result supplies context, while independent oracles check current
  behavior.
- [Process crashes](host/test_tx_airtime_crash.py): child processes receive
  SIGKILL at recovery/save boundaries; repeated no-TX startups exercise budget
  exhaustion through recovery.
- [Ledger](host/test_airtime_ledger.py),
  [recovery](host/test_tx_airtime_recovery.py) and
  [admission](host/test_tx_airtime_admission.py): occupancy, expiration,
  rounding/overflow, the 28/29/30 ACK threshold and recovery boundaries.
- [Settlement](host/test_tx_airtime_settlement.py),
  [integration](host/test_tx_airtime_integration.py),
  [runtime time](host/test_runtime_time.py) and
  [communicator](host/test_communicator.py): partial/duplicate receipts, RTC
  completion, correlated snapshot preemption, RX rearming before saves and
  continuous packet arrivals with paced retries.
- [Encoding](host/test_receiver_interface_generation.py) and
  [state persistence](host/test_communicator_state_persistence.py): canonical
  serialization, validation and incompatible-state handling.

## Availability scenarios

[Availability tests](host/test_tx_airtime_availability.py) use the production
policy with nominal-rate virtual time and fresh eligible observations:

- One ACK request per minute for eight hours measures sparse-traffic overhead.
- One request per second for two hours measures suppression under sustained
  demand.
- One request per minute with an RTC save after every ACK for two hours exposes
  the cost of closing partial groups. This is a stress profile, not the deployed
  RTC refresh frequency.
- Twenty startups spaced ten seconds apart, with no TX, check exhaustion and
  the first subsequent capacity release.
- Long idle periods and repeated save/load cycles check retained unsaved usage
  and conservative lifetime growth.

Traffic and restart tests write measurements into pytest's temporary directory.
Traffic measurements include admitted/suppressed requests, save count, maximum
ACK gap and occupancy. Calculated pauses at two seconds per save are hypothetical;
they do not measure disk latency or physical missed packets.

## Running the checks

From the repository root, run the normal host checks:

```sh
.venv/bin/python receiver/tools/generate.py --check
make test-receiver-host
make test-rf-host
```

For a focused model, production-history, crash and availability run:

```sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -c receiver/pytest.ini receiver/tests/host/test_tx_airtime_model.py receiver/tests/host/test_tx_airtime_properties.py receiver/tests/host/test_tx_airtime_crash.py receiver/tests/host/test_tx_airtime_availability.py
```

Larger independent model runs are available without changing CI defaults:

```sh
PYTHONPATH=receiver .venv/bin/python -m tests.support.models.tx_airtime --trials 1000 --steps 500
PYTHONPATH=receiver .venv/bin/python -m tests.support.models.tx_airtime_adversarial --trials 1000 --hours 48
```

## Coverage limits

Models assume the contract's single active owner, conservative packet charges,
bounded monotonic rate and normal TX-completion envelope. The finite enumerations
cover their stated domains; generated histories are bounded samples, not an
exhaustive proof. Production tests separately exercise encoding and integration.

Host process crashes do not establish physical power-loss durability, field RTC
holdover, arbitrary late TX after stalled I/O or actual persistence-induced RX
loss. Hardware procedures and their prerequisites are described in
[TESTING.md](../TESTING.md#receiver-tx-airtime-policy). Collecting their cases
checks discoverability only and does not execute or qualify hardware:

```sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -c receiver/pytest.ini receiver/tests/hardware --collect-only -q
```
