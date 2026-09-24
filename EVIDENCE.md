# Test evidence

Evidence tells our future selves that a hard-to-run test was actually run.
Keep it only for destructive tests, tests requiring physical actions, or tests
taking **more than five minutes**. Hardware access alone does not qualify a
quick, automatically repeatable test. Rerun easy tests when their result matters.

Keep one short report per topic and, only when necessary, one supporting result
file. Record the date, test/command, outcome, why it qualifies, tested source or
build, fixture, decisive observations, restoration and acceptance limits. Share
repeated identity data. Retain only measurements needed to understand the claim;
do not archive whole sessions, source trees, dependency inventories, routine
host/build receipts, duplicate manifests or one-off verification scripts.

Capture raw diagnostics in temporary/ignored working space while executing and
investigating. Validate the result before curation, then remove redundant inputs.
Checksums identify bytes; they are not execution proof. A curated extract must
say it is an extract and disclose any checks that can no longer be replayed.
Do not replace many files with one opaque archive of the same clutter.

Do not keep failed runs. Preserve a useful lesson in a nearby code comment,
regression test or owning documentation. Evidence is the fallback only when
there is no better home, and then keeps the lesson and minimum necessary facts,
not the failed session. Routine wiring/setup mistakes need no lasting record.
An expected fault in a passing destructive test is still a valid test result.

Preserve distinctions between PASS, incomplete/operator-pending acceptance and
NOT RUN. Removing a capture does not change the historical outcome. Keep test
requirements and approved deferrals in coverage/procedure documents with their
reason, consequence and revisit condition. Historical source-bound evidence is
not automatically current qualification; process crashes and orderly reboots
are not physical power-loss proof.

Regression inputs belong beside tests, not in evidence. Evidence must be
readable without a device, temporary paths or a separate tool to unpack it.
