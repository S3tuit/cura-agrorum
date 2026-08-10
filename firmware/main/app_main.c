#include "node_platform_esp.h"

/*
 * Fresh composition root for the LoRa v2 firmware. Wake-cycle orchestration
 * and node_cycle_run will be implemented from ARCHITECTURE.md; until then this
 * resolves the concrete platform dependency without introducing wake policy.
 */
void app_main(void) {
  const node_platform_ports_t *const platform = node_platform_esp_ports();
  (void)platform;
}
