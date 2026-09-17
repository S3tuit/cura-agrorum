# Hardware results worth keeping

Start here to see what expensive tests already established. These are historical,
source-bound results; later code changes do not inherit their qualification.

| Topic | Result worth remembering | Report |
|---|---|---|
| Persistence | Storage faults recovered; process-kill cases and loaded soak passed | [Persistence](persistence_worker/README.md) |
| Runtime time | Clock slew, RTC oscillator-stop/cold boot and controller faults checked | [Time](runtime_time/README.md) |
| TX airtime | Grant lifetime, process restart and both real reboot modes passed | [Airtime](tx_airtime/README.md) |
| Radio | Nominal non-peer cases passed; held BUSY fault observed, then fixture restored | [Radio](radio/README.md) |

Keep a short report of the result, tested source/fixture, limits, restoration and
reason to rerun. Retain supporting measurements only when needed to check that
result. A useful failure becomes a problem, cause/fix and lesson; keep a minimal
capture only if it still explains an unresolved issue or supports a claim.
Routine host logs, setup transcripts and superseded runs are disposable.

Capture diagnostics during a run in ignored `raw/` or outside this directory.
Before clearing them, check the result, promote its essential evidence and
update the relevant report. Delete the rest after curation; ignored directories
are temporary working space, not a second archive. Keep one source identity per
tested tree, and a source snapshot only where Git cannot recover that tree.
Do not accumulate complete session bundles or add a new report for every rerun.

The JSON result records are explicitly labelled curation products; selected
original captures retain their bytes. Original paths in their provenance identify
where data came from, not files required today. Missing historical evidence stays
missing. Operator confirmation, incomplete sessions and machine observations
remain distinct. Reboot/process-kill results do not prove power-loss durability.

From this directory, `sha256sum -c SHA256SUMS` checks the retained files. The
checksum list was generated during curation. Topic reports give any additional
offline verification commands; neither a Pi nor temporary storage is required.
