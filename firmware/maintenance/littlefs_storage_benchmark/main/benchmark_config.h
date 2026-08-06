#pragma once

#include <stdint.h>

#define BENCHMARK_STORAGE_LABEL "storage"
#define BENCHMARK_MOUNT_PATH "/storage"
#define BENCHMARK_DIRECTORY_PATH BENCHMARK_MOUNT_PATH "/bench"
#define BENCHMARK_LOG_PATH BENCHMARK_DIRECTORY_PATH "/pending.log"

#define BENCHMARK_STORAGE_BYTES (2944u * 1024u)
#define BENCHMARK_SECTOR_SIZE 4096u
#define STORAGE_SECTOR_COUNT (BENCHMARK_STORAGE_BYTES / BENCHMARK_SECTOR_SIZE)

#define BENCHMARK_WAKE_CYCLES 3000u
#define BENCHMARK_GROWING_DELETE_CYCLES 2000u
#define BENCHMARK_PEEK_REPETITIONS 100u
#define BENCHMARK_MAX_RECORD_SIZE 150u
#define BENCHMARK_WARNING_DELAY_MS 5000u

_Static_assert(BENCHMARK_STORAGE_BYTES % BENCHMARK_SECTOR_SIZE == 0,
               "storage partition must contain complete erase sectors");
_Static_assert(STORAGE_SECTOR_COUNT == 736u,
               "update the benchmark if partitions.csv changes");
