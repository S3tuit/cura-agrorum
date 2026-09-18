# Bounded component episodes

The operator handles the suite's airtime admission and pacing. A manual batch
sheet is enough: identify both physical transmitters, preceding activity, the
selected cases, reservation, start/latest possible end and result. The runner
retains that sheet without parsing or scheduling its history. A spare receiver
is an observation cross-check. Silence does not reset either transmitter's history.

The harness presents these maxima and waits for operator readiness before an
episode. One run selects explicit parameters; it has no automatic retries.
The following limits describe potential attempts, including failures.

| Selection | C6 maximum 54-byte attempts | Pi payload lengths | C6 charge (us) | Pi charge (us) |
|---|---:|---|---:|---:|
| RF-001.exchange | 1 | 23 | 112922 | 67866 |
| RF-003.silence | 1 | none | 112922 | 0 |
| RF-006.invalid | 1 | 1, 4, 23 | 112922 | 130382 |
| RF-008.silence | 2 | none | 225844 | 0 |
| RF-008.exchange | 2 | 23, 23 | 225844 | 135732 |
| RF-009.untouched | 1 | none | 112922 | 0 |
| RF-009.initialized | 1 setup; later call must fail before SetTx | none | 112922 | 0 |
| RF-010.wake | 2, across two wakes | 23, 23 | 225844 | 135732 |
| RF-012.disconnected | 1 uncertain at the C6 | none | 112922 | 0 |
| RF-013.absent | 1 conservatively reserved; expected 0 SetTx | none | 112922 | 0 |
| All ten parameters once | 13 reserved attempts | 8 packets | 1467986 | 469712 |

Charges are independently calculated from the fixed pilot PHY, with 10% margin:
54/23/4/1-byte modeled airtimes are 102656/61696/30976/25856 us and charges are
112922/67866/34074/28442 us. Setup is included; nominal restoration and explicit
reruns are additional episodes. Retain uncertainty; no refund based on nonreceipt.
The suite total is a description, not an automated allowance or a compliance claim.

RF-001 retains the initial envelope's ten-attempt session limit and 60-second
spacing between successive starts at the same transmitter. The operator owns
spacing across episodes/reruns. The expanded cases explicitly require their
whole episode to be admitted: downlinks start at Pi-local targets 250 ms after
uplink RX_DONE; RF-006's next two targets are 750 and 1250 ms. Each C6 receive
window is 3 seconds from its local call; RF-006 retains the first deadline
through all three rejected packets. A second same-boot uplink follows the first
receive operation and at least 500 ms after the first C6 TX_DONE. These are
component scheduling guards, not production ACK or retry timing requirements.
If a target is missed by more than 100 ms, the peer stops instead of catching up
with overlapping packets. Capture the actual starts/completions independently.

RF-010 uses a two-second timer deep sleep. After waking the app waits for a
fresh host command bound to the new boot nonce before the second exchange.
No boot, reset, terminal cleanup or abandoned command can initiate another TX.
Other cases also prove final deep-sleep entry by observing a timer wake into
the non-transmitting command wait. This is not production 900-second behavior.

The C6 control channel accumulates a complete LF-terminated command, accepting
CRLF, before validating its run/case/boot selection. It permits at most 159 bytes
before LF. Idle readiness has no timeout; after the first byte the complete line
must arrive within two seconds, including the boundary. Fragmented reads do not
restart that deadline. Invalid bytes, overflow, incomplete input or invalid
selection emit a structured `RF_REJECT` before any radio operation and latch
rejection until reset. The host retains rejection and fails immediately without
resending. Successful commands retain `RF_COMMAND` with the byte count and local
first-byte-to-newline duration. This contract was approved on 2026-09-18; the
operator permits an evidence-supported later relaxation of the two-second bound.

Each armed peer is limited to a 45-second episode, plus bounded cleanup. The
C6 operation has a two-second TX deadline, three-second RX windows and finite
packet counts. If the harness loses control it issues no further trigger;
retain the possible active packet/wake interval, peer timeout and teardown
result. Failed cleanup requires operator power removal. An autonomous production
image is outside this component runner's assumptions.

Readiness binds the run, exact parameter, source/build identities, fixture and
operator records. C6 records additionally bind a fresh boot nonce and factory
MAC; Pi records bind boot ID and process ID. Completion includes observed packet
counts, return/diagnostic facts, independent endpoint timestamps, test outcome
and cleanup. Missing completion or evidence is failure, never a skipped pass.

Nominal prerequisites are fresh results from the current bench session. A small
temporary receipt binds executable sources/build and physical devices, and is
cleared by a failed/interrupted or fault run. Historical archive reuse and
per-file applicability reviews have been retired. See the [session procedure](README.md).

On 2026-09-18 the operator approved the documented firmware/receiver hardware
and configured +14 dBm pilot envelope, including autonomous wakes, retries and
resets; see the [retained DEC-003/DEP-023 decision](OPERATING_ENVELOPE.md).
RF-018 disposition is closed; its physical measurement remains deferred NOT RUN.
Numerical output uncertainty remains unmeasured. This approval does not change
this component runner's finite episodes or non-transmitting idle behavior, the
operator's airtime ownership, or the separate production enforcement contracts.
