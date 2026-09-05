# Receiver benchmarks

Receiver benchmarks are explicitly invoked, non-gating target
characterizations. They are not pytest tests and are never dependencies of
`test-receiver`, any receiver hardware-test target or another ordinary
validation command.

Each child directory owns one benchmark's reviewed inputs, concise methodology,
standalone runner and curated results. A runner writes to a new caller-selected
output directory. Ordinary runs remain outside the tracked result tree; a run
is copied under that benchmark's `results/` directory only after its source,
environment, workload completion and output have been reviewed.

Curated result directories are immutable evidence. A correction or rerun gets
a new directory rather than changing an existing run. Raw samples are retained
with their units, a versioned result format, source-file hashes, platform
metadata and a human-readable summary. Machine-specific observations do not
become protocol requirements or golden test thresholds merely by being checked
in.
