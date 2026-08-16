#include "esp_attr.h"

#include "node_core.h"
#include "node_platform_esp.h"
#include "protocol_v2_lora_identity.h"

static const node_identity_t NODE_IDENTITY = {
    .node_id = CURA_LORA_V2_NODE_ID_BYTES,
    .node_key = CURA_LORA_V2_NODE_KEY_BYTES,
};

RTC_DATA_ATTR static node_rtc_record_t rtc_record;

void app_main(void) {
  const node_platform_ports_t *const platform = node_platform_esp_ports();
  node_cycle_run(platform, &NODE_IDENTITY, &rtc_record);
}
