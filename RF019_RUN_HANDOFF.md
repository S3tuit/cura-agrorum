# RF execution status

RF-019: all11 cases PASS. RF-020: PASS with10-second configuration and minimal
before/after reading baseline. No runner remains active; C6 stopped in loader,
Pi service stopped. The C6 still contains the accelerated image, not the900s
bench/pilot image. No bench has started.

Results: `receiver/tests/evidence/2026-09-19-pilot-runtime/RF019_QUALIFICATION.json`
and `RF020_QUALIFICATION.json`. Compressed captures retain exact bindings.
RF-031 physical scenario remains deferred NOT RUN; its disposition is complete.
RF-023 waits for workplan D-092 group/database preparation choice before further
implementation/device execution. Consult the owned workplan for current tasks.

---

# RF-019 active run and completion checks

Run: `bd0330fc9df14e7fbb1fbaf4f8f97683`  
Case: `RF-019.current.accepted`  
Status: PASS, completion records checked on2026-09-19. Three natural wakes; received-current intervals896.362549s and900.244864s. Peer safely shut down; C6 stopped in loader; temporary credentials removed. Only RF-019.current.accepted is complete. Concise evidence: `receiver/tests/evidence/2026-09-19-pilot-runtime/RF019_CURRENT_ACCEPTED.json`.  
Capture directory: `/tmp/cura-production-preparation-9b1tykup/rf019-current-accepted-bd0330fc9df14e7fbb1fbaf4f8f97683`

## Authority and limits

The operator explicitly requested this restart after confirming a temporary Wi-Fi outage interrupted run `53ac7f51c1604ca2adad3b501dbfefd0`. Preserve that FAIL and the earlier timing/startup failure. There is no automatic retry.

Three natural wakes, nominal 900-second sleep, inclusive 855–945-second received-current interval. Peer lease: 1950 seconds. Maximum 210 C6 uplinks and four Pi replies. Allow about 35 minutes after node start for the episode and final storage capture. Leave power, USB and network available. The existing runner owns both endpoints and its cleanup; perform no competing Pi/C6 access while it runs.

## Files to inspect first, locally

- `/tmp/cura-production-preparation-9b1tykup/rf019-current-accepted-bd0330fc9df14e7fbb1fbaf4f8f97683/job.json`: wrapper status, runner exit, copied run result and temporary credential cleanup exit.
- `/tmp/cura-production-preparation-9b1tykup/rf019-current-accepted-bd0330fc9df14e7fbb1fbaf4f8f97683/runner-output.txt`: runner progress and any traceback.
- `/tmp/cura-production-preparation-9b1tykup/rf019-current-accepted-bd0330fc9df14e7fbb1fbaf4f8f97683/run/run.json`: authoritative case status, failures and independently verified results.
- `/tmp/cura-production-preparation-9b1tykup/rf019-current-accepted-bd0330fc9df14e7fbb1fbaf4f8f97683/run/pi.jsonl` and `pi.stderr`: ready/armed/complete identities, radio trace, cleanup and transport errors.
- `/tmp/cura-production-preparation-9b1tykup/rf019-current-accepted-bd0330fc9df14e7fbb1fbaf4f8f97683/run/c6-uart.bin`: exactly one initial application boot, then two SLEEP_WAKEUP boots; no panic or unexpected reset.

A RUNNING job is not a result. A file's presence, elapsed wall time, SSH exit or endpoint silence is not PASS. A missing completion/cleanup record remains unresolved even if the peer process has exited.

## Acceptance checks after completion

1. Require job status PASS, runner exit0, run status PASS, no failures and one verified result for this exact case/run/node identity. Do not replace a failed original status with an inferred pass.
2. Verify current-source host exact-deadline prerequisite passed separately. Check source-manifest/build/MAC/group/run bindings; compare later edits for applicability rather than silently treating a changed tree as the tested tree.
3. Require a matching peer complete event with no failure, safe_shutdown=true and confirmed TX trace. For this case expect four authenticated uplinks and four replies: setup current RETRY_LATER; next current ACCEPTED; its seeded backlog ACCEPTED; final current RETRY_LATER. Both accepted deliveries must terminate on their first attempt.
4. Verify three consecutive current sample IDs, increasing message IDs, genuine deep-sleep reset/previous metrics, and both received-current intervals within855..945s. Backlog uses a distinct transport ID/domain with the exact seeded reading body. Previous metrics in the final wake must report the accepted current plus accepted backlog.
5. Require empty before logs and complete after image/binding/decoded records. The independent verifier must reconcile all delivery starts/outcomes, packet attempts, pending/removal and body/frame bindings. Expect only the final observation reading pending, no quarantine and no diagnostics. No extra initial sample/delivery may exist.
6. Require final-stop-mac.json success for the correct C6 with no-reset/remaining in loader, peer shutdown confirmed, and credential cleanup exit0. Inspect restoration-required.json if present; absence of the original peer completion cannot be repaired by assuming cleanup.
7. Preserve concise source-bound evidence and update the owned workplan immediately. A pass completes only this one case; T-010 still contains ten other RF-019 cases. RF-020 and soak acceptance remain separate.

## Failure handling

Do not restart, erase, reformat, reflash or launch another case. Preserve run.json, failure/cleanup records and raw captures. If the after-image was not captured, first establish that the node was stopped; preserve it read-only during an explicitly resumed cleanup step. If radio shutdown was unconfirmed, re-establish it without transmission and keep that restoration distinct from the missing original receipt. Report any newly exposed contract gap before changing behavior.

## Local work while waiting

The operator permits independent local work only. Keep running-test sources unchanged. T-013 needs the operator's soak ideas before criteria are fixed; T-014 depends on those agreed criteria. T-011 requires hardware, and T-012 follows T-011. Prepare local evidence/procedure reconciliation as possible without claiming unfinished acceptance. No soak starts in this batch. Informal bench check-ins at15/30/60 minutes and at the end are not mandatory acceptance thresholds.
