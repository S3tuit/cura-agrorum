# Per-bucket retention redesign evidence

[Summary](summary.json) and [source manifest](source-manifest.json) identify the
redesign. The manifest binds 327 staged files
from the reviewed commit plus the uncommitted redesign. Raw outputs, source
archive and the original regression failure are retained under the ignored
[2026-09-16 directory](2026-09-16/).

The complete host suite passes **2,484 tests**. The independent eight-hour
availability regression begins with valid empty history, supplies fresh trusted
time at an unchanged UTC-minus-monotonic offset, and requests one ACK each
minute with one-second retries. Its virtual clocks advance at the physical
reference rate. The same regression fails on the reviewed commit: 417 requests
succeed and the maximum gap is 3,794 seconds. With the redesign all 480 requests
succeed; maximum deferral is 14 seconds and maximum gap is 74 seconds. The
baseline database/WAL/SHM failure files and both event traces are retained.
Separate generated properties retain the physical-window safety checks at
both monotonic-rate extremes and after reconstruction.

All **17 selected Pi component tests passed** in 18.28 seconds, using the same
327-file staged tree verified before and after execution. They cover airtime
lifetime/process restart and 15 existing worker, SQLite and time regressions.
The existing slow soak was intentionally deselected.

**Reboot requalification passed in a fresh controller run.** Both modes completed
an actual reboot and passed all four prepare/verify phases. With fresh trusted
network time, the policy reconstructed the unchanged 8,000,000-us charge with
zero initial allowance. With the component time source unavailable, it
preserved the durable bytes and suppressed reconstruction and TX. Linux boot
and receiver-process identities changed; SQLite integrity and state digests
passed. The controller verified unchanged Chrony configuration, active service
and restored synchronization. All phases used the same 327-file staged source.

The first controller attempt
passed the trusted prepare/verify phases across an actual reboot, and passed
unavailable-mode preparation. After the second reboot SSH reported `No route
to host`, and the unchanged 180-second reconnect bound failed. Unavailable-mode
verification was not run in that attempt. After connectivity recovered, the
original target artifacts were retrieved and Chrony restoration was verified
separately, before the fresh qualification. The complete failed attempt remains
labeled as a failure, including reconnect and final-inspection attempts.

The [evidence audit](2026-09-16/evidence-audit.json) checks all target JUnit
outcomes, ten clock captures, and 1,707 lifetime samples. The last allowed sample
preceded the deadline by 632 us; the first denied sample finished 564 us after
it. Remaining lifetime after acknowledgement was 1,952,962 us, within the
original two-second interval. The separate-process restart retained the
1,067,866-us baseline. Retrieved files match their raw archives byte-for-byte;
pytest's absolute convenience symlinks remain archived but are not followed
during local extraction. The raw checksum manifest also covers the fresh
controller transcript, recovered target artifacts and audit inputs.

The fixture retains the approved 10,000-ppb network-skew ceiling. No radio
operation is included. These results do not establish physical power-loss
durability or receiver-user permissions. The preceding qualification remains
in the [parent evidence directory](../README.md), bound to its original sources.
