"""Reproduce the stale USB VFS prefix loss against real IDF console code."""
import os
from pathlib import Path
import subprocess

import pytest

from carrier_runner import APP


@pytest.mark.parametrize('tick_ms', [1, 10])
def test_carrier_records_share_paced_unity_output(tmp_path, tick_ms):
    idf = Path(os.environ.get('IDF_PATH', Path.home() / 'esp/esp-idf'))
    headers = {
        'sdkconfig.h': '#define CONFIG_ESP_CONSOLE_ROM_SERIAL_PORT_NUM 4\n',
        'esp_err.h': '#include <stdint.h>\n#define ESP_OK 0\n',
        'esp_cpu.h': '#include <stdint.h>\nstatic inline uint32_t esp_cpu_get_cycle_count(void) { return 0; }\n',
        'esp_private/esp_clk.h': 'static inline unsigned esp_clk_cpu_freq(void) { return 160000000; }\n',
        'esp_system_console.h': 'void esp_system_console_put_char(char);\n',
        'esp_rom_serial_output.h': '#include <stdint.h>\nint esp_rom_output_rx_one_char(uint8_t *);\nvoid esp_rom_output_tx_one_char(char);\nvoid esp_rom_output_tx_wait_idle(int);\n',
        'esp_timer.h': '#include <stdint.h>\nint64_t esp_timer_get_time(void);\n',
        'freertos/FreeRTOS.h': f'typedef unsigned TickType_t;\n#define pdMS_TO_TICKS(ms) ((ms)/{tick_ms})\n',
        'freertos/task.h': 'void vTaskDelay(unsigned);\n',
    }
    for name, contents in headers.items():
        path = tmp_path / name
        path.parent.mkdir(exist_ok=True)
        path.write_text(contents)

    # Keep the installed IDF implementation as the negative control. Its stale
    # last_tx_ts skips waiting for the FIFO and still reports successful writes.
    vfs = (idf / 'components/esp_driver_usb_serial_jtag/src/usb_serial_jtag_vfs.c').read_text()
    start = vfs.index('static void usb_serial_jtag_tx_char_no_driver(int fd, int c)\n{')
    end = vfs.index('\nstatic int usb_serial_jtag_rx_char_no_driver', start)
    (tmp_path / 'vfs.c').write_text('''#include <stdint.h>
#define TX_FLUSH_TIMEOUT_US 50000
static struct { int64_t last_tx_ts; } s_ctx;
int64_t esp_timer_get_time(void);
int usb_serial_jtag_ll_txfifo_writable(void);
void usb_serial_jtag_ll_write_txfifo(const uint8_t *, unsigned);
void usb_serial_jtag_ll_txfifo_flush(void);
''' + vfs[start:end] + '''
void stale_vfs_char(int c) { usb_serial_jtag_tx_char_no_driver(0, c); }
''')
    # Redirect only the writer's sink for the control, leaving the actual hold
    # and formatter intact. The corrected build links the real IDF unity_putc.
    (tmp_path / 'legacy_output.c').write_text(
        '#define unity_putc stale_vfs_char\n'
        f'#include "{APP / "main/carrier_output.c"}"\n')
    (tmp_path / 'boundary.c').write_text(r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "carrier_hold.h"
#include "carrier_output.h"
static int64_t now = 1000000;
static char captured[4096];
static unsigned used, rx;
static const char *operation;
int64_t esp_timer_get_time(void) { return now++; }
void vTaskDelay(unsigned ticks) { now += ticks * TICK_MS * INT64_C(1000); }
int usb_serial_jtag_ll_txfifo_writable(void) { return now++ >= 1000015; }
void usb_serial_jtag_ll_write_txfifo(const uint8_t *data, unsigned length) {
    assert(used + length < sizeof(captured));
    memcpy(captured + used, data, length); used += length;
}
void usb_serial_jtag_ll_txfifo_flush(void) {}
void esp_system_console_put_char(char c) {
    while (!usb_serial_jtag_ll_txfifo_writable()) {}
    usb_serial_jtag_ll_write_txfifo((const uint8_t *)&c, 1);
}
void esp_rom_output_tx_one_char(char c) { esp_system_console_put_char(c); }
void esp_rom_output_tx_wait_idle(int port) { (void)port; }
int esp_rom_output_rx_one_char(uint8_t *byte) {
    const bool selecting = strcmp(operation, "select") == 0;
    const char *command = selecting ? "guided\n" : "CARRIER_HOLD_ACK gate-on\n";
    if (selecting && !strstr(captured, "CARRIER_HOLD_MODE\r\n") &&
                     !strstr(captured, "CARRIER_HOLD_MODE\n")) return -1;
    if (!selecting && (!strstr(captured, "CARRIER_HOLD_READY gate-on seconds=180\r\n") ||
                       now < 6000000)) return -1;
    if (!command[rx]) return -1;
    *byte = (uint8_t)command[rx++]; return 0;
}
int main(int argc, char **argv) {
    assert(argc == 3); operation = argv[1];
    if (strcmp(operation, "select") == 0) {
        int selected = carrier_hold_select();
        assert(selected == (strcmp(argv[2], "legacy") == 0 ? -1 : 1));
    } else if (strcmp(operation, "hold") == 0) {
        assert(carrier_hold_wait("gate-on", CARRIER_HOLD_GUIDED));
        assert(now >= 6000000 && now < 181000000);
    } else if (strcmp(operation, "format") == 0) {
        assert(carrier_output_printf("CARRIER_SAMPLE status=%08x temp=%.9g\n", 0x1234, 21.125) == 43);
        assert(carrier_output_puts("") == 1);
    } else {
        char limit[512]; memset(limit, 'x', sizeof(limit)); limit[511] = 0;
        assert(carrier_output_printf("%s", limit) == 511);
        unsigned before = used;
        assert(carrier_output_puts(limit) == -1);
        assert(carrier_output_printf("%s!", limit) == -1);
        assert(used == before);
    }
    fwrite(captured, 1, used, stdout);
}
''')
    common = [os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra', '-Werror',
              '-Wno-unused-parameter', '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
              '-DUNITY_INCLUDE_CONFIG_H', f'-DTICK_MS={tick_ms}', '-I', str(tmp_path),
              '-I', str(APP / 'main'), '-I', str(idf / 'components/unity/include'),
              '-I', str(idf / 'components/unity/unity/src'), str(tmp_path / 'boundary.c'),
              str(tmp_path / 'vfs.c'), str(APP / 'main/carrier_hold.c'),
              str(idf / 'components/unity/unity_port_esp32.c'),
              str(APP.parents[3] / 'tests/support/unity_console.c'),
              '-Wl,--wrap=unity_putc', '-Wl,--wrap=unity_gets',
              '-Wl,--wrap=esp_rom_output_rx_one_char']
    for legacy in (True, False):
        binary = tmp_path / ('legacy' if legacy else 'unified')
        writer = tmp_path / 'legacy_output.c' if legacy else APP / 'main/carrier_output.c'
        compiled = subprocess.run(common + [str(writer), '-o', str(binary)], capture_output=True)
        assert compiled.returncode == 0, compiled.stderr.decode()
        def run(operation):
            result = subprocess.run([str(binary), operation, 'legacy' if legacy else 'unified'],
                                    capture_output=True)
            assert result.returncode == 0, result.stderr.decode()
            return result.stdout
        if legacy:
            assert run('select') == b'HOLD_MODE\n'
        else:
            assert run('select') == b'CARRIER_HOLD_MODE\r\n'
            held = run('hold')
            assert held.startswith(b'CARRIER_HOLD_READY gate-on seconds=180\r\n')
            assert b'CARRIER_HOLD_ACKED gate-on elapsed_us=' in held
            assert held.endswith(b'CARRIER_HOLD_END gate-on\r\n')
            assert run('format') == b'CARRIER_SAMPLE status=00001234 temp=21.125\r\n\r\n'
            assert run('overflow') == b'x' * 511
