/* Raw component tests. No identity, NVS, LittleFS or authenticated packets. */
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "esp_app_desc.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "node_platform_esp.h"
#include "radio_observe.h"
#include "radio_command.h"
#include "sdkconfig.h"
#include "sx1262_radio.h"
#include "unity.h"

#if !CONFIG_IDF_TARGET_ESP32C6 || CONFIG_CURA_SX1262_SCLK_GPIO != 6 || \
    CONFIG_CURA_SX1262_MOSI_GPIO != 7 || CONFIG_CURA_SX1262_MISO_GPIO != 14 || \
    CONFIG_CURA_SX1262_CS_GPIO != 23 || CONFIG_CURA_SX1262_RESET_GPIO != 18 || \
    CONFIG_CURA_SX1262_BUSY_GPIO != 19 || CONFIG_CURA_SX1262_DIO1_GPIO != 20
#error "Radio app requires the declared C6 carrier GPIO allocation"
#endif
#if !defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || CONFIG_ESP_CONSOLE_UART_NUM != 0 || CONFIG_ESP_CONSOLE_UART_BAUDRATE != 115200
#error "Radio app requires UART0 at 115200 baud"
#endif

static const char *const cases[] = {
  "RF-001.exchange", "RF-003.silence", "RF-006.invalid", "RF-008.silence",
  "RF-008.exchange", "RF-009.untouched", "RF-009.initialized", "RF-010.wake",
  "RF-012.disconnected", "RF-013.absent"
};
static unsigned selected, phase;
static char run_id[33];
static uint32_t boot_nonce;
static const node_platform_ports_t *ports;
/* Only a host-selected continuation can use this record. It never triggers TX. */
RTC_DATA_ATTR static struct {
  uint32_t magic, previous_boot;
  char run[33], selection[32];
  unsigned continuation;
} retained;

typedef struct {
  uint64_t before, after, deadline;
  err_curag_t error;
  diagn_context_t diagnostic;
  sx1262_radio_tx_result_t tx;
  sx1262_radio_rx_result_t rx;
  uint8_t payload[54];
  bool transmit;
} result_t;
static result_t results[8];
static unsigned result_count;
static uint64_t last_tx_done;

static uint64_t now(void) { return ports->clock.monotonic_us(ports->clock.context); }
static void hex(const uint8_t *p, size_t length) {
  for (size_t i = 0; i < length; ++i) printf("%02x", p[i]);
}
static result_t *new_result(bool tx) {
  TEST_ASSERT_LESS_THAN(8, result_count);
  result_t *r = &results[result_count++];
  memset(r, 0, sizeof(*r));
  r->transmit = tx;
  return r;
}
static void no_diagnostic(const diagn_context_t *d) {
  TEST_ASSERT_EQUAL_UINT16(0, d->operation);
  TEST_ASSERT_EQUAL_UINT8(0, d->context_schema);
  TEST_ASSERT_EQUAL_UINT8(0, d->context_length);
}
static result_t *transmit(uint8_t start, bool success) {
  result_t *r = new_result(true);
  for (unsigned i = 0; i < 54; ++i) r->payload[i] = start + i;
  r->before = now();
  r->deadline = r->before + 2000000;
  r->error = sx1262_radio_transmit_uplink(r->payload, 54, r->deadline, &r->tx, &r->diagnostic);
  r->after = now();
  TEST_ASSERT_TRUE(r->after <= r->deadline + 50000);
  if (success) {
    TEST_ASSERT_EQUAL_UINT32(CURAG_OK, r->error);
    no_diagnostic(&r->diagnostic);
    TEST_ASSERT_TRUE(r->tx.tx_started && r->tx.tx_done);
    TEST_ASSERT_TRUE(r->before <= r->tx.set_tx_at_us && r->tx.set_tx_at_us > 0);
    TEST_ASSERT_TRUE(r->tx.tx_done_at_us <= r->after);
    uint64_t elapsed = r->tx.tx_done_at_us - r->tx.set_tx_at_us;
    TEST_ASSERT_TRUE(elapsed >= 102656 && elapsed <= 112922);
    last_tx_done = r->tx.tx_done_at_us;
  }
  return r;
}
static void receive_packet(uint64_t deadline, const uint8_t *expected, unsigned length) {
  result_t *r = new_result(false);
  r->before = now(); r->deadline = deadline;
  r->error = sx1262_radio_receive_downlink_until(deadline, &r->rx, &r->diagnostic);
  r->after = now();
  TEST_ASSERT_EQUAL_UINT32(CURAG_OK, r->error);
  no_diagnostic(&r->diagnostic);
  if (expected) {
    TEST_ASSERT_EQUAL(SX1262_RADIO_RX_PACKET, r->rx.outcome);
    TEST_ASSERT_EQUAL_UINT8(length, r->rx.payload_length);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, r->rx.payload, length);
    TEST_ASSERT_TRUE(r->before <= r->rx.rx_done_at_us && r->rx.rx_done_at_us <= deadline);
    TEST_ASSERT_TRUE(r->rx.rx_done_at_us <= r->after);
    TEST_ASSERT_TRUE(r->rx.rssi_dbm_x2 >= -255 && r->rx.rssi_dbm_x2 <= 0);
    TEST_ASSERT_TRUE(r->rx.snr_db_x4 >= -128 && r->rx.snr_db_x4 <= 127);
  } else {
    TEST_ASSERT_EQUAL(SX1262_RADIO_RX_DEADLINE, r->rx.outcome);
    TEST_ASSERT_TRUE(r->after >= deadline);
    TEST_ASSERT_TRUE(r->after <= deadline + ((deadline - r->before) * 15 + 99) / 100);
    TEST_ASSERT_EQUAL_UINT64(0, r->rx.rx_done_at_us);
    TEST_ASSERT_EQUAL_INT(0, r->rx.rssi_dbm_x2);
    TEST_ASSERT_EQUAL_INT(0, r->rx.snr_db_x4);
    TEST_ASSERT_EQUAL_UINT8(0, r->rx.payload_length);
    uint8_t zero[255] = {0};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(zero, r->rx.payload, 255);
  }
}
static void downlink(uint8_t start) {
  uint8_t expected[23];
  for (unsigned i = 0; i < 23; ++i) expected[i] = start + i;
  receive_packet(now() + 3000000, expected, 23);
}
static void cold_sleep(void) {
  diagn_context_t d;
  TEST_ASSERT_EQUAL_UINT32(CURAG_OK, sx1262_radio_sleep(&d));
  no_diagnostic(&d);
}
static void test_episode(void) {
  if (selected == 5) {
    cold_sleep(); cold_sleep();
    TEST_ASSERT_EQUAL_UINT(0, radio_observation().calls);
    transmit(0, true);
  } else if (selected == 6) {
    transmit(0, true);
    cold_sleep();
    unsigned calls = radio_observation().calls;
    cold_sleep();
    TEST_ASSERT_EQUAL_UINT(calls, radio_observation().calls);
    result_t *r = transmit(0, false);
    TEST_ASSERT_EQUAL_UINT32(curag_error_make(CURAG_EDOM_RADIO, CURAG_ERADIO_EINVALID_STATE), r->error);
    TEST_ASSERT_FALSE(r->tx.tx_started);
    TEST_ASSERT_EQUAL_UINT(calls, radio_observation().calls);
  } else if (selected == 8 || selected == 9) {
    result_t *r = transmit(0, false);
    TEST_ASSERT_NOT_EQUAL(CURAG_OK, r->error);
    TEST_ASSERT_EQUAL_UINT16(CURAG_EDOM_RADIO, curag_error_domain(r->error));
    TEST_ASSERT_EQUAL_UINT8(1, r->diagnostic.context_schema);
    TEST_ASSERT_EQUAL_UINT8(14, r->diagnostic.context_length);
    TEST_ASSERT_FALSE(r->tx.tx_done);
    if (selected == 8) {
      TEST_ASSERT_TRUE(r->tx.tx_started);
      TEST_ASSERT_EQUAL_UINT16(CURAG_OP_TRANSMIT, r->diagnostic.operation);
      TEST_ASSERT_EQUAL_UINT8(CURAG_RADIO_STAGE_WAIT_IRQ, r->diagnostic.context[2]);
      TEST_ASSERT_TRUE(r->diagnostic.context[3] & CURAG_RADIO_CONTEXT_HARDWARE_TOUCHED);
      TEST_ASSERT_EQUAL_UINT(1, radio_observation().starts);
    } else {
      TEST_ASSERT_FALSE(r->tx.tx_started);
      TEST_ASSERT_EQUAL_UINT16(CURAG_OP_INITIALIZE, r->diagnostic.operation);
      TEST_ASSERT_EQUAL_UINT16(CURAG_ERADIO_EBUSY_TIMEOUT, curag_error_code(r->error));
      TEST_ASSERT_EQUAL_UINT(0, radio_observation().starts);
    }
  } else {
    transmit(phase ? 0x80 : 0, true);
    if (selected == 1 || selected == 3) receive_packet(now() + 3000000, NULL, 0);
    else if (selected == 2) {
      uint64_t deadline = now() + 3000000;
      const uint8_t first[] = {0}, second[] = {0xde, 0xad, 0xbe, 0xef};
      uint8_t third[23];
      for (unsigned i = 0; i < 23; ++i) third[i] = 0xe0 + i;
      receive_packet(deadline, first, 1);
      receive_packet(deadline, second, 4);
      receive_packet(deadline, third, 23);
      receive_packet(deadline, NULL, 0);
    } else downlink(phase ? 0xc0 : 0xa0);
    if (selected == 3 || selected == 4) {
      while (now() < last_tx_done + 500000) vTaskDelay(1);
      transmit(0x80, true);
      if (selected == 4) downlink(0xc0);
    }
  }
  TEST_ASSERT_EQUAL_UINT(1, radio_observation().initializes);
  TEST_ASSERT_EQUAL_UINT(0, radio_observation().overflows);
}
void setUp(void) {}
void tearDown(void) {}

static void dump_results(void) {
  for (unsigned i = 0; i < result_count; ++i) {
    result_t *r = &results[i];
    printf("RF_RESULT {\"tx\":%s,\"before\":%" PRIu64 ",\"after\":%" PRIu64
           ",\"deadline\":%" PRIu64 ",\"error\":%" PRIu32 ",\"operation\":%u,\"diagnostic\":\"",
           r->transmit ? "true" : "false", r->before, r->after, r->deadline, r->error, r->diagnostic.operation);
    hex(r->diagnostic.context, r->diagnostic.context_length);
    printf("\",\"started\":%s,\"done\":%s,\"set_tx\":%" PRIu64 ",\"tx_done\":%" PRIu64
           ",\"rx_outcome\":%u,\"rx_done\":%" PRIu64 ",\"rssi\":%d,\"snr\":%d,\"payload\":\"",
           r->tx.tx_started ? "true" : "false", r->tx.tx_done ? "true" : "false",
           r->tx.set_tx_at_us, r->tx.tx_done_at_us, r->rx.outcome, r->rx.rx_done_at_us,
           r->rx.rssi_dbm_x2, r->rx.snr_db_x4);
    hex(r->transmit ? r->payload : r->rx.payload, r->transmit ? 54 : r->rx.payload_length);
    puts("\"}");
  }
}
static void reject_command(const char *reason, const rf_line_t *line) {
  printf("RF_REJECT {\"boot\":%" PRIu32 ",\"reason\":\"%s\",\"bytes\":%u,\"elapsed_us\":%" PRIu64 "}\n",
         boot_nonce, reason, (unsigned)line->length,
         line->started ? now() - line->first_byte_us : 0);
  fflush(stdout);
  /* Rejection is terminal for this boot, including any buffered command tail. */
  for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}

void app_main(void) {
  ports = node_platform_esp_ports();
  uint8_t mac[6]; char elf[65];
  ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_EFUSE_FACTORY));
  const esp_app_desc_t *description = esp_app_get_description();
  for (unsigned i = 0; i < 32; ++i) snprintf(elf + 2*i, 3, "%02x", description->app_elf_sha256[i]);
  boot_nonce = esp_random();
  bool sleep_wake = esp_reset_reason() == ESP_RST_DEEPSLEEP && retained.magic == 0x52464336;
  if (!sleep_wake) memset(&retained, 0, sizeof(retained));
  printf("RF_BOOT {\"dut\":\"%02x%02x%02x%02x%02x%02x\",\"elf\":\"%s\",\"boot\":%" PRIu32
         ",\"reset\":%d,\"previous_boot\":%" PRIu32 ",\"continuation\":%u,\"previous_run\":\"%s\",\"previous_case\":\"%s\"}\n",
         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], elf, boot_nonce, esp_reset_reason(),
         retained.previous_boot, retained.continuation, retained.run, retained.selection);
  puts("RF_READY"); fflush(stdout);
  rf_line_t line = {0};
  uint64_t received_at = 0;
  while (line.status == RF_LINE_PENDING) {
    int byte = getchar();
    received_at = now();
    rf_line_feed(&line, byte == EOF ? -1 : byte, received_at);
    if (byte == EOF) { clearerr(stdin); vTaskDelay(1); }
  }
  if (line.status != RF_LINE_COMPLETE) {
    const char *reason = line.status == RF_LINE_EXPIRED ? "incomplete_timeout" :
                         line.status == RF_LINE_OVERSIZED ? "oversized" : "invalid_byte";
    reject_command(reason, &line);
  }
  rf_command_t command;
  if (!rf_command_parse(line.text, &command)) {
    reject_command("invalid_command", &line);
  }
  if (command.boot != boot_nonce) {
    reject_command("wrong_boot", &line);
  }
  strcpy(run_id, command.run);
  phase = command.phase;
  for (selected = 0; selected < sizeof(cases)/sizeof(cases[0]); ++selected)
    if (strcmp(command.selection, cases[selected]) == 0) break;
  if (selected == sizeof(cases)/sizeof(cases[0]) ||
      (phase && !(selected == 7 && sleep_wake && retained.continuation == 1 &&
                  strcmp(retained.run, run_id) == 0 && strcmp(retained.selection, command.selection) == 0))) {
    reject_command("unsupported_case_or_continuation", &line);
  }
  /* A boot executes at most one command, then sleeps even when Unity fails. */
  memset(&retained, 0, sizeof(retained));
  printf("RF_BEGIN {\"run\":\"%s\",\"case\":\"%s\",\"boot\":%" PRIu32 ",\"phase\":%u}\n",
         run_id, cases[selected], boot_nonce, phase);
  printf("RF_COMMAND {\"boot\":%" PRIu32 ",\"bytes\":%u,\"elapsed_us\":%" PRIu64 "}\n",
         boot_nonce, (unsigned)line.length, received_at - line.first_byte_us);
  UNITY_BEGIN();
  UnityDefaultTestRun(test_episode, cases[selected], __LINE__);
  int failed = UNITY_END();
  diagn_context_t cleanup;
  err_curag_t cleanup_error = sx1262_radio_sleep(&cleanup);
  dump_results(); radio_observation_dump();
  /* Absent hardware may refuse radio cleanup; the MCU must still sleep. */
  if (cleanup_error && selected != 9) failed = 1;
  printf("RF_END {\"run\":\"%s\",\"case\":\"%s\",\"boot\":%" PRIu32
         ",\"phase\":%u,\"failed\":%d,\"cleanup_error\":%" PRIu32
         ",\"cleanup_operation\":%u,\"cleanup_diagnostic\":\"",
         run_id, cases[selected], boot_nonce, phase, failed, cleanup_error, cleanup.operation);
  hex(cleanup.context, cleanup.context_length); puts("\"}");
  retained.magic = 0x52464336; retained.previous_boot = boot_nonce;
  strcpy(retained.run, run_id); strcpy(retained.selection, cases[selected]);
  retained.continuation = !failed && selected == 7 && phase == 0;
  puts("RF_SLEEP"); fflush(stdout);
  ports->system.enter_deep_sleep_for(ports->system.context, 2000000);
}
