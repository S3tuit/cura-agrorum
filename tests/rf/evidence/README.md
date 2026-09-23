# RF results worth keeping

Historical component results from 2026-09-18. Both require manual fault wiring;
nominal RF tests are inexpensive to repeat and have no permanent run archive.

| Case | Fixture and observed result | Restoration |
|---|---|---|
| RF-012 | Radio DIO1 disconnected; MCU-side RN5 retained. C6 attempted one TX, missed TxDone, returned TRANSMIT / WAIT_IRQ / HARDWARE_TOUCHED 6,060 us after its deadline (50,000 us allowance). Pi copied exact A, sent nothing. PASS. | Operator reconnected DIO1 unpowered; fresh RF-001 PASS. |
| RF-013 | Radio absent; GPIO19/BUSY tied high, GPIO14/MISO low, RN5 retained. INITIALIZE / WAIT_BUSY error `0x20004` after 21,631 us, no SetTx. Pi observed no packets or TX. PASS. | Operator removed both ties unpowered and reconnected the radio; fresh RF-001 PASS. |

Both C6 runs entered real timer deep sleep and woke into non-transmitting idle.
Pi standby/IRQ cleanup, handle release and process exit passed. RF-013's C6
cleanup BUSY error is permitted with the module absent; it proves no physical
radio sleep. The tested devices were C6 `cc8da2fc0224` and Pi
`00000000e0027211`; the shared firmware ELF begins `6e8acd2deb2a2eef`.
Full identities, fixtures, selected assertions and restoration records are in
[results.json](results.json). Operator confirmation and observed results remain
labelled separately. These results do not qualify a changed image, full service,
physical timing, output power, durable airtime enforcement or production sleep.

The two C6 captures, two Pi traces and source manifests retain original bytes.
`results.json` is a curation product. Before deleting the session bundles, the
original checksums and verifier passed for both faults and both restorations,
including JUnit and prerequisite checks. Routine setup/wiring failures and
nominal raw captures were discarded. Their removal does not change their
historical outcomes. Old paths/hashes in provenance identify discarded inputs.

`source.tar.gz` preserves the otherwise uncommitted tested tree: `rf012/` is the
first manifest's complete source set; overlay the three `rf013/` Markdown files
for the second. Executable sources were identical. The source manifests and
build file hashes identify what ran; the external SDK/compiler dependency list
and binaries are not retained. This snapshot is historical and is never used
as a prerequisite for new hardware work.

From this directory run `sha256sum -c SHA256SUMS`. From the repository root run
`.venv/bin/python tests/rf/verify.py tests/rf/evidence` to recheck the retained
endpoint assertions. Neither operation accesses hardware. Rerun these manual
cases when relevant firmware, driver, fixture or test expectations change.

Keep only manual-setup, destructive or long-run results here: a short report,
source/build identity, essential observations, limits and restoration. A failure
merits retention only when it teaches a useful lesson. Temporary diagnostics go
outside the repository or in ignored `raw/`; curate what matters, then delete
the rest. Operator airtime history remains separately owned and must survive
for as long as admission/pacing needs it; clearing diagnostics grants no refund.

One useful harness lesson survives from the initial attempts: ESP-IDF basic UART
reads can deliver fragments of a command. Parsing each successful `fgets` as a
complete command rejected valid input; the subsequent parser also rejected a
valid framed command. The bounded line accumulator and exact ASCII parser now
have [host regressions](../host/test_command.py). Keep that lesson and regression,
not the failed session bundles. The [operating-envelope approval](../OPERATING_ENVELOPE.md)
is maintained separately; physical RF-018 measurement remains NOT RUN.
