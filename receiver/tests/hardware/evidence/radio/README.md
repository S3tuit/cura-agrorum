# SX1262 — 17 September 2026

Operator-declared Pi 3B+, Waveshare Pico-LoRa-SX1262 and DS3231 carrier;
Linux/Python/configuration are in [results](results.json). GPIO/SPI ran as
service user `cura`. All retained runs used the same source snapshot.

| Run | Recorded outcome |
|---|---|
| Initial nominal | 3 passed in 2.09 s; three safe teardowns; captured exit 0 |
| Manual held BUSY | Expected INITIALIZE and CLEANUP/BUSY_TIMEOUT in 277,477 us; 809 HIGH samples; no SPI/RX/TX; handles released |
| Restored nominal | 3 passed in 2.28 s; three safe teardowns; no SetTx |

Held-run assertions passed, but `safe_shutdown=false` caused a session abort.
Later restoration does not turn that into a safe-teardown pass. Boot IDs changed;
power removal and selector restoration are [operator-confirmed](operator-observations.json).
Held/restored exact commands and numeric process exits were not captured.

Useful failures: reset status `0x2A` initially rejected initialization; validation
needed to account for that observed post-reset state and confirm a fresh command.
Later `0x26` with IRQ `0x0200` exposed finite-RX timeout handling: evaluate the
correlated status/IRQ evidence and preserve subsequent strict command checks.
The [backend regressions](../../../host/test_sx1262.py) retain those cases.
A missing-sentinel attempt ran zero cases; keep the fixture interlock.
Old failed-session bundles and repeated nominal traces have been removed.

[Results](results.json) are a curation extract of outcomes, initialization/teardown,
fixture/target metadata and dependency versions, with original paths/hashes.
Four unchanged traces retain held BUSY and restored nominal command activity.
[Source manifest](source-manifest.json) has 189 entries; [source snapshot](source.tar.gz)
has 235 files, SHA-256 `058630a6ecf23de93af500e3678212a571e606f9f503001c24f2008af2ae49b1`.
This snapshot preserves then-uncommitted implementation changes.

## Verification

From the repository root: `python3 receiver/tests/hardware/evidence/radio/verify_evidence.py`.
This checks the source snapshot, retained outcomes/traces, boot change and safe
restoration without Pi access. The root checksum list covers all archive files.

These results cover non-peer component behavior. RF interoperability, independent
waveform/timestamp/timing and gate-driven runtime recovery remain NOT RUN.
RESET-bias investigation remains deferred; operator DC readings are not proof
of a defect or physical timing. Rerun the [component procedure](../../RADIO_TESTS.md)
when the radio backend, GPIO/kernel environment or fixture changes; collect the
missing instruments/peer evidence before claiming the deferred coverage.
