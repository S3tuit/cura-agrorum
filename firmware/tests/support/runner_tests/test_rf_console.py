"""Run the real RF writer and platform terminal boundary with bounded fake time."""
import os
from pathlib import Path
import subprocess

import pytest

SUPPORT = Path(__file__).resolve().parents[1]
FIRMWARE = SUPPORT.parents[1]


@pytest.fixture(scope="module", params=["production", "component"])
def boundary(tmp_path_factory, request):
    root = tmp_path_factory.mktemp("rf-boundary")
    (root / "esp_rom_serial_output.h").write_text(
        '#include <stdint.h>\nint esp_rom_output_rx_one_char(uint8_t *);\n'
        'int esp_rom_output_tx_one_char(uint8_t);\n')
    (root / "unity_config.h").write_text('void unity_putc(int);\n')
    (root / "main.c").write_text(r'''
#include <assert.h>
#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "esp_err.h"
#include "esp_system.h"
#include "node_platform_esp.h"
#include "node_sensors.h"
#include "rf_console.h"
static jmp_buf terminal;
static int64_t now;
static const char *input;
static size_t input_size, read_at, used;
static char output[16384];
static unsigned timer_calls, sleep_calls;
static bool sink_failure;
int64_t esp_timer_get_time(void) { return now; }
uint32_t esp_random(void) { return 0x12345678; }
esp_reset_reason_t esp_reset_reason(void) { return ESP_RST_POWERON; }
const char *esp_err_to_name(esp_err_t status) { (void)status; return "fake"; }
void fake_esp_log_error(const char *tag, const char *format, ...) {
    (void)tag; (void)format; assert(!"unexpected platform failure");
}
err_curag_t node_sensors_force_power_off(diagn_context_t *diag) { (void)diag; return 0; }
void esp_restart(void) { assert(!"must not restart parked image"); }
esp_err_t esp_sleep_enable_timer_wakeup(uint64_t duration) {
    assert(duration == 10000000); ++timer_calls; return ESP_OK;
}
void esp_deep_sleep_start(void) { ++sleep_calls; longjmp(terminal, 1); }
int esp_rom_output_tx_one_char(uint8_t byte) {
    if (sink_failure) return 1;
    assert(used + 1 < sizeof(output)); output[used++] = byte; return 0;
}
void unity_putc(int byte) { esp_rom_output_tx_one_char((uint8_t)byte); }
int esp_rom_output_rx_one_char(uint8_t *byte) {
    if (read_at == input_size) return -1;
    *byte = input[read_at++]; return 0;
}
void vTaskDelay(unsigned ticks) {
    if (strstr(output, "RF_PARKED ") || strstr(output, "RF_FINALIZE_FAILED ") ||
        (sink_failure && now >= 30000000)) longjmp(terminal, 2);
    now += ticks * INT64_C(1000);
    assert(now < 32000000);
}
int main(int argc, char **argv) {
    assert(argc == 2);
    input = argv[1]; input_size = strlen(input);
    if (strcmp(input, "NUL") == 0) {
        input = "RF_FINALIZE_ACK 12345678 SLEEP\0wrong\n";
        input_size = sizeof("RF_FINALIZE_ACK 12345678 SLEEP\0wrong\n") - 1;
    }
    assert(rf_console_puts("RF_END {}") == 10);
#ifdef CURA_RF_COMPONENT_TEST
    rf_console_puts("RF_SLEEP");
#endif
    if (strcmp(input, "oversized_output") == 0) {
        char large[513]; memset(large, 'x', 512); large[512] = 0;
        size_t before = used;
        assert(rf_console_printf("%s", large) == -1 && used == before);
    }
    if (strcmp(input, "sink_failure") == 0) sink_failure = true;
    int result = setjmp(terminal);
    if (result == 0) {
        const node_platform_ports_t *ports = node_platform_esp_ports();
        ports->system.enter_deep_sleep_for(ports->system.context, 10000000);
        assert(!"terminal platform call returned");
    }
    assert(timer_calls == 1 && (sleep_calls == 1) == (result == 1));
    fwrite(output, 1, used, stdout);
    printf("terminal=%s elapsed=%lld\n", result == 1 ? "sleep" : "park", (long long)now);
}
''')
    binary = root / "boundary"
    components = FIRMWARE / "components"
    command = [os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Wextra", "-Werror",
               "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
               "-D" + ("CONFIG_NODE_RF_SLEEP_OBSERVATION=1" if request.param == "production" else "CURA_RF_COMPONENT_TEST=1"),
               "-I" + str(root), "-I" + str(SUPPORT), "-I" + str(FIRMWARE / "tests/host/fakes")]
    for name in ("node_platform_esp", "node_core", "node_sensors", "node_common", "node_persistence", "sx1262_radio", "protocol_v2_lora"):
        command += ["-I" + str(components / name / "include")]
    if request.param == "component":
        command += ["-DCURA_RF_UNITY_OUTPUT=1"]
    command += [str(root / "main.c"), str(SUPPORT / "rf_console.c"),
                str(components / "node_platform_esp/node_platform_esp.c"), "-o", str(binary)]
    subprocess.run(command, check=True, capture_output=True, text=True)
    return binary, request.param


@pytest.mark.parametrize("message,terminal", [
    ("RF_FINALIZE_ACK 12345678 SLEEP\r\n", "sleep"),
    ("RF_FINALIZE_ACK 12345678 STOP\n", "park"),
    ("", "park"), ("RF_FINALIZE_ACK deadbeef SLEEP\n", "park"),
    ("RF_FINALIZE_ACK 12345678 SLEEP", "park"),
    ("RF_FINALIZE_ACK 12345678 SLEEP extra\n", "park"),
    ("x" * 70 + "RF_FINALIZE_ACK 12345678 SLEEP\n", "park"),
    ("NUL", "park"), ("oversized_output", "park"), ("sink_failure", "park"),
])
def test_receipt_is_required_for_actual_platform_sleep(boundary, message, terminal):
    binary, image = boundary
    result = subprocess.run([str(binary), message], check=True, capture_output=True, text=True)
    output = result.stdout
    assert f"terminal={terminal}" in output
    if message != "sink_failure":
        marker = "RF_SLEEP" if image == "component" else "RF_NODE_SLEEP duration_us=10000000"
        assert marker in output
        if message != "oversized_output":
            assert output.index(marker) < output.index("RF_FINALIZE 12345678")
    if "STOP" in message:
        assert "RF_PARKED 12345678" in output
    elif terminal == "park" and message != "sink_failure":
        assert "RF_FINALIZE_FAILED 12345678" in output
    if not message or "deadbeef" in message:
        assert "elapsed=30000000" in output
