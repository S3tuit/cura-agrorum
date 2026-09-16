# Receiver TX-airtime component evidence

[Summary](summary.json) and [source manifest](source-manifest.json) identify the
current implementation and its validation. Raw runs are retained under the
ignored [2026-09-15 directory](2026-09-15/); its `SHA256SUMS` covers the captures,
including source archives, pytest output, controller transcripts and target
database/WAL/SHM sets.

The corrected implementation passes **2,471 host tests**, generator validation,
generation checks and whitespace checks. Target qualification is complete:

| Coverage | Result |
|---|---|
| Pi grant lifetime and separate-process restart | 2 passed; 8.28 seconds |
| Existing Pi worker, SQLite and time regressions | 15 passed |
| Real reboot with fresh trusted network time | Passed; unchanged 8,000,000-us charge reconstructed with zero initial allowance |
| Real reboot with unavailable component time source | Passed; reconstruction and TX suppressed |

The two airtime cases and both final reboot modes use the user's approved
10 ppm fixture ceiling; production policy and airtime assertions are
unchanged. The last allowed timing sample preceded the deadline by 472 us;
the first denied sample finished 763 us after it. The measured remaining
grant lifetime after acknowledgement was 1,954,799 us, within the original
two-second bucket interval. All 1,653 samples are retained.

The accepted source directory is
`/home/cura/cura-airtime-accepted-ex9hiQLw`; its 326-file manifest was checked
before and after execution. The 15 regression passes used the same production
sources in `/home/cura/cura-airtime-qualified-O3hbjMyd`; only `TESTING.md` and
the airtime fixture's approved input/metadata differ. Both reboot identities
changed, and the controller verified the unchanged Chrony configuration,
active service and restored network synchronization after the final reboot.

Earlier lifetime/process-restart cases and two actual reboot modes passed.
Final review then found that descheduling between separate UTC and monotonic
reads could extend the original grant deadline. The retained failing host
regression demonstrates a 30-second extension; the fix uses one captured clock
sample and passes that regression. Earlier Pi passes remain labeled as passes
on their original sources. The accepted runs above independently qualify the
corrected source.

Other retained failures are explicit fixture failures: root-owned configuration
beneath an untrusted user-owned ancestor, and an arbitrary generation-one input
correctly rejected by conservative missing-state recovery. The corrected
fixture uses a reviewed existing-history row and the production loader. The
later time-readiness failure under the original 1 ppm fixture ceiling is also
retained, along with bounded readiness samples and finite NTP measurement-burst
requests. Those requests change no
Chrony configuration and issue no time-step command.

Procedures and target applicability live in
[TESTING.md](../../../../TESTING.md#receiver-tx-airtime-policy). This evidence
covers the isolated policy, clocks, complete-state owner and SQLite worker.
It does not establish RF behavior, physical power-loss durability, or
receiver-user permissions: the bench clock interfaces required root access.
