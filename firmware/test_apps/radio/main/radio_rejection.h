#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RF023_CASE_COUNT 12u
#define RF023_CASE_NAMES \
  "RF-023.implausible", "RF-023.control", "RF-023.domain", \
  "RF-023.body-length", "RF-023.flags", "RF-023.direction", \
  "RF-023.control-direction", "RF-023.bad-tag", "RF-023.unknown", \
  "RF-023.short-header", "RF-023.before-revoke", "RF-023.revoked"

typedef struct {
  char run[33];
  uint8_t node_ids[3][8], node_keys[3][16]; /* active, unknown, revoked */
  uint32_t first_message, first_sample;
} rf023_config_t;

typedef struct {
  uint8_t frame[54], ack[23];
  size_t frame_length, ack_length; /* ack_length==0 requires observed silence */
} rf023_packet_t;

const char *rf023_case_name(unsigned index);
bool rf023_authorized(const rf023_config_t *config, const char *run,
                      unsigned index, unsigned phase);
/* No I/O, counters, credentials loading or radio ownership. Output cleared on failure. */
bool rf023_build(const rf023_config_t *config, unsigned index, rf023_packet_t *out);
