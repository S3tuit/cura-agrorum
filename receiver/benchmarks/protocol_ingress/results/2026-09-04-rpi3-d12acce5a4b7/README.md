# Raspberry Pi 3 protocol-ingress characterization

This immutable evidence pair was captured on 2026-09-04 from one isolated
staging tree. Both modes name base commit
`ba44c112d1bf4619e9fdedcb3d99078ec6980ba1`, record the tree as dirty because
the ingress work was not yet committed, and share measured-source manifest
SHA-256 `d12acce5a4b7d80da6414e56e0fbd13225b1936aadb6e6cf9d73aadd5e6bb7a7`.

Before measurement, the staged Raspberry Pi runtime passed all three fast
protocol-ingress target-compatibility cases. The curated pair contains:

- `idle/`: no deliberate load; and
- `load/`: three sustained SHA-256 workers and one sustained circular
  write-plus-`fdatasync` worker bounded to a 64 MiB file and the shared runtime
  deadline.

Every load worker exited on the parent's post-measurement stop request with
exit code zero. The writer performed 41,671 syncs and 40 wraps without growing
beyond 64 MiB. See each mode's `summary.md` for environment, percentiles,
maximum and qualified coarse margin, and `raw.json` for every nanosecond sample
and the complete recorded evidence.

Raw evidence SHA-256:

- idle: `fe3d3362201a935b56479ece341fd9eb94ae3a8c0a932d9b664f9387b432a0fd`
- load: `3a2fed8d89053be06f0cfe77f344c21b6c8559f3312c3b448dfc43c95c9f4ba7`
