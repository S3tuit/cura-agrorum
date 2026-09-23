#include "radio_rejection.h"
#include <string.h>
#include "protocol_v2_lora_crypto.h"

static const char *const names[] = {RF023_CASE_NAMES};
const char *rf023_case_name(unsigned index) {
  return index < RF023_CASE_COUNT ? names[index] : NULL;
}
static bool valid(const rf023_config_t *config) {
  if (!config || config->run[32] != '\0' ||
      config->first_message > UINT32_MAX - (RF023_CASE_COUNT - 1) ||
      config->first_sample > UINT32_MAX - (RF023_CASE_COUNT - 1)) return false;
  for (unsigned i = 0; i < 32; ++i)
    if (!((config->run[i] >= '0' && config->run[i] <= '9') ||
          (config->run[i] >= 'a' && config->run[i] <= 'f'))) return false;
  for (unsigned i = 0; i < 3; ++i)
    for (unsigned j = i + 1; j < 3; ++j)
      if (!memcmp(config->node_ids[i], config->node_ids[j], 8)) return false;
  return true;
}
bool rf023_authorized(const rf023_config_t *config, const char *run,
                      unsigned index, unsigned phase) {
  return valid(config) && run && index < RF023_CASE_COUNT && phase == 0 &&
         !strcmp(config->run, run);
}
bool rf023_build(const rf023_config_t *config, unsigned index, rf023_packet_t *out) {
  if (!out) return false;
  memset(out, 0, sizeof(*out));
  if (!valid(config) || index >= RF023_CASE_COUNT) return false;
  const unsigned identity = index == 8 ? 1 : index >= 10 ? 2 : 0;
  cura_lora_v2_clear_header_t header = {
    .control = 0x20, .domain = 1,
    .message_id = config->first_message + index
  };
  memcpy(header.node_id, config->node_ids[identity], 8);
  const uint8_t *key = config->node_keys[identity];
  cura_lora_v2_reading_t reading = {
    .sample_id = config->first_sample + index, .run_ms = 1000,
    .soil_0_mv = 65000, .soil_1_mv = 64000,
    .soil_temp_0_centi_c = 20000, .soil_temp_1_centi_c = -20000,
    .enclosure_centi_c = 25000, .enclosure_pressure_pa = UINT32_C(4000000000),
    .enclosure_humidity_centi_pct = 60000, .reset_reason = 1, .flags = 0xfe
  };
  uint8_t body[32];
  if (cura_lora_v2_encode_reading(body, sizeof(body), &reading) != CURA_LORA_V2_CODEC_OK) return false;
  size_t body_length = sizeof(body);
  int status = 0;
  if (index == 1 || index == 6) { header.control = 0x30; status = 2; }
  if (index == 2) { header.domain = 0x7f; status = 2; }
  if (index == 3) { body_length--; status = 3; }
  if (index == 4) { body[30] &= (uint8_t)~2u; status = 3; }
  if (index == 5 || index == 6) {
    header.domain = 3; body[0] = 0; body_length = 1;
    if (index == 5) status = -1;
  }
  if (index == 7 || index == 8 || index == 9 || index == 11) status = -1;
  rf023_packet_t packet = {0};
  if (cura_lora_v2_seal_frame(packet.frame, sizeof(packet.frame), &packet.frame_length,
                            key, &header, body, body_length) != CURA_LORA_V2_CRYPTO_OK) return false;
  if (index == 7) packet.frame[packet.frame_length - 1] ^= 1;
  if (index == 9) packet.frame_length = 13;
  if (status >= 0) {
    header.control = 0x20; header.domain = (uint8_t)(status + 3);
    const uint8_t ack_status = (uint8_t)status;
    if (cura_lora_v2_seal_frame(packet.ack, sizeof(packet.ack), &packet.ack_length,
                              key, &header, &ack_status, 1) != CURA_LORA_V2_CRYPTO_OK) return false;
  }
  *out = packet;
  return true;
}
