/* App-local passive interposition. No logging or substituted hardware results
 * inside a production operation; fixed storage and no public singleton API. */
#include "radio_observe.h"
#include "sx1262_radio_backend.h"
#include "esp_timer.h"
#include <inttypes.h>
#include <stdio.h>

typedef struct {
  const char *name;
  uint64_t before, after;
  unsigned argument, result;
} event_t;
static event_t events[128];
static radio_observation_t observed;

static void record(const char *name, uint64_t before, unsigned argument,
                   unsigned result) {
  unsigned index = observed.calls++;
  if (index >= 128) { observed.overflows++; return; }
  events[index] = (event_t){name, before, (uint64_t)esp_timer_get_time(), argument, result};
}
radio_observation_t radio_observation(void) { return observed; }
void radio_observation_dump(void) {
  for (unsigned i = 0; i < observed.calls && i < 128; ++i) {
    const event_t *e = &events[i];
    printf("RF_TRACE {\"operation\":\"%s\",\"before\":%" PRIu64
           ",\"after\":%" PRIu64 ",\"argument\":%u,\"result\":%u}\n",
           e->name, e->before, e->after, e->argument, e->result);
  }
}

bool __real_sx1262_radio_backend_initialize(const sx1262_radio_profile_t *, sx1262_radio_backend_error_t *);
bool __wrap_sx1262_radio_backend_initialize(const sx1262_radio_profile_t *p, sx1262_radio_backend_error_t *e) {
  uint64_t before = esp_timer_get_time();
  observed.initializes++;
  bool result = __real_sx1262_radio_backend_initialize(p, e);
  record("initialize", before, p->frequency_hz, result);
  return result;
}

#define SIMPLE(name, counter) \
  sx1262_radio_backend_call_result_t __real_sx1262_radio_backend_##name(sx1262_radio_backend_error_t *); \
  sx1262_radio_backend_call_result_t __wrap_sx1262_radio_backend_##name(sx1262_radio_backend_error_t *e) { \
    uint64_t before = esp_timer_get_time(); counter; \
    sx1262_radio_backend_call_result_t result = __real_sx1262_radio_backend_##name(e); \
    record(#name, before, 0, result); return result; \
  }
SIMPLE(set_standby, (void)0)
SIMPLE(start_single_rx, (void)0)
SIMPLE(set_sleep_cold, observed.sleeps++)

#define ARGUMENT(name, type, counter) \
  sx1262_radio_backend_call_result_t __real_sx1262_radio_backend_##name(type, sx1262_radio_backend_error_t *); \
  sx1262_radio_backend_call_result_t __wrap_sx1262_radio_backend_##name(type value, sx1262_radio_backend_error_t *e) { \
    uint64_t before = esp_timer_get_time(); counter; \
    sx1262_radio_backend_call_result_t result = __real_sx1262_radio_backend_##name(value, e); \
    record(#name, before, value, result); return result; \
  }
ARGUMENT(start_tx, uint32_t, observed.starts++)
ARGUMENT(configure_irq, uint16_t, (void)0)
ARGUMENT(clear_irq, uint16_t, (void)0)

sx1262_radio_backend_call_result_t __real_sx1262_radio_backend_set_packet_params(uint8_t, bool, sx1262_radio_backend_error_t *);
sx1262_radio_backend_call_result_t __wrap_sx1262_radio_backend_set_packet_params(uint8_t length, bool inverted, sx1262_radio_backend_error_t *e) {
  uint64_t before = esp_timer_get_time();
  sx1262_radio_backend_call_result_t result = __real_sx1262_radio_backend_set_packet_params(length, inverted, e);
  record("set_packet_params", before, length | ((unsigned)inverted << 8), result);
  return result;
}
