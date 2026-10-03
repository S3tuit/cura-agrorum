# Receiver benchmarks

Receiver benchmarks are explicitly invoked, non-gating target
characterizations. They are not pytest tests and are never dependencies of
`test-receiver`, any receiver hardware-test target or another ordinary
validation command.

Each child directory owns the benchmark inputs, methodology and runner. Runners
write to a new caller-selected directory outside the source tree. Review the
measurements, keep lessons that inform design in the owning documentation, and
discard run outputs. Machine-specific samples do not become golden thresholds.

The [ordinary persistence benchmark](ordinary_persistence/README.md) records
FULL/NORMAL commit and checkpoint latency, accepted throughput, queue pressure
and WAL growth on isolated target databases.
