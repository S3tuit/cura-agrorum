# Real LittleFS regression input

`empty-node-storage.bin.gz` is the unchanged empty 2944 KiB storage partition
captured from C6 `cc8da2fc0224` on 2026-09-19 after the production formatter.
`empty-node-storage.json` binds its compressed/raw hashes, device, formatter and
partition identity. This is a test input, not an execution-evidence archive.

The offline reader and synthetic writer once shared `name_max=64`, so they
agreed with each other but rejected the real image's library default (255).
`test_actual_c6_formatter_image` independently requires successful nonmutating
decode and all four logs absent. Keep the actual image to prevent that shared
assumption from returning; both configurations use the production default.
