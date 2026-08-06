#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "node_sensors_backend.h"

/*
 * Resolves the two configured textual identities. Returns true only when at
 * least one valid, non-duplicated identity requires 1-Wire bus access.
 */
bool node_sensors_ds18b20_resolve_identities(
    const char *const identities[2], uint64_t out_roms[2],
    node_sensors_backend_result_t out_channel[2]);
