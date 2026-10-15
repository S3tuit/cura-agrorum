#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define REJECTION_CASE_COUNT 12u
#define REJECTION_CASE_NAMES \
  "rejection.implausible", "rejection.control", "rejection.domain", \
  "rejection.body_length", "rejection.flags", "rejection.direction", \
  "rejection.control_direction", "rejection.bad_tag", "rejection.unknown", \
  "rejection.short_header", "rejection.before_revoke", "rejection.revoked"

typedef struct {
  char run[33];
  uint8_t node_ids[3][8], node_keys[3][16]; /* active, unknown, revoked */
  uint32_t first_message, first_sample;
} rejection_config_t;

typedef struct {
  uint8_t frame[54], ack[23];
  size_t frame_length, ack_length; /* ack_length==0 requires observed silence */
} rejection_packet_t;

const char *rejection_case_name(unsigned index);
bool rejection_authorized(const rejection_config_t *config, const char *run,
                      unsigned index, unsigned phase);
/* No I/O, counters, credentials loading or radio ownership. Output cleared on failure. */
bool rejection_build(const rejection_config_t *config, unsigned index, rejection_packet_t *out);
