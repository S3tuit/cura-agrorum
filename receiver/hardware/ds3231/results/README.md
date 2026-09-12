# DS3231 test results

These dated records and their raw evidence are tracked in Git alongside the
[operator procedure](../OPERATOR_TESTS.md) and
[installation guide](../README.md). They document the tested hardware/kernel
combination and the limits of each observation; a past pass does not establish
acceptance for another fixture, kernel or receiver implementation.

| Run | Outcome |
|---|---|
| [2026-09-12-run-01 — Raspberry Pi 3](2026-09-12-run-01/README.md) | RTC-01–RTC-05 PASS; RTC-06 NOT RUN. No drift resolved in the short retention check. |

## Adding a run

Create a new directory named for its UTC date, fixture and, when needed, a
unique run suffix. Keep a `README.md` summary with hardware/software identity,
procedure version, operator actions, acceptance outcomes, evidence links,
uncertainties and recovery status. Put original captures in `data/raw/`. Do not
modify raw observations to correct timestamps or errors; explain corrections
separately in the summary or derived data.
