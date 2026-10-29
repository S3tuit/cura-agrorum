# Receiver benchmarks

Receiver benchmarks are explicitly invoked, non-gating target
characterizations. They are not pytest tests and are never dependencies of
`test-receiver`, any receiver hardware-test target or another ordinary
validation command.

Each child directory owns the benchmark inputs, methodology and runner. Runners
write to a new caller-selected directory outside the source tree. Review the
measurements and keep lessons that inform design in the owning documentation.
Raw runs stay outside the repository; a reviewed summary may retain the source,
fixture and sample provenance needed for a recorded decision. Machine-specific
samples do not become golden test thresholds.

The [ordinary persistence benchmark](ordinary_persistence/README.md) records
FULL/NORMAL commit and checkpoint latency, accepted throughput, queue pressure
and WAL growth on isolated target databases.

The [startup readiness benchmark](startup_readiness/README.md) retains the Pi
worker-startup measurements, synthetic fixture definitions, rerun procedure and
the pilot-only 12-second budget decision for the measured 21 MB large fixture.
