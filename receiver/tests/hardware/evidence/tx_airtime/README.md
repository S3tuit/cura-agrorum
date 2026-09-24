# Airtime reboot qualification — 16 September 2026

**PASS across two real Pi reboots.** This disrupts the dedicated target and its
time configuration. The [result](results.json) preserves before/after boot and
instance identities, state digests, charges, allowance and restoration.

| Mode after reboot | Observation |
|---|---|
| Trusted network time | Same 8,000,000 µs charge reconstructed; initial allowance zero |
| Unavailable component time | Durable state unchanged; reconstruction/TX suppressed |

Both prepare/verify pairs passed. Chrony was active and synchronized after
restoration, with unchanged configuration. Source: all 327 recorded files were
verified against commit `4779361ff04180d34f846ea8942c9387092cc8cc` during the original
curation. Target: Pi 3 Model B Rev 1.2, kernel `6.18.50+rpt-rpi-v8`, Python 3.13.5,
SQLite 3.46.1; root fixture with approved 10,000-ppb network-skew ceiling.

The original verifier passed before this consolidation. Database integrity and
state hashes had been checked on disposable copies before the earlier deletion
of DB/WAL/SHM files; that database audit cannot be replayed from this record.
Short grant-lifetime/process-restart tests and virtual-clock availability runs
can be rerun. These results establish neither RF behavior, service-user access
nor physical power-loss durability. Rerun the
[airtime procedure](../../../../TESTING.md#receiver-tx-airtime-policy) after relevant
clock, retention, reconstruction or durable-state changes.
