"""Compile the test-image boot barrier; no ACK must never reach hardware tests."""
import os
from pathlib import Path
import subprocess

import pytest

SUPPORT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def boot(tmp_path_factory):
    root = tmp_path_factory.mktemp('usb-boot')
    headers = {
        'esp_random.h': '#include <stdint.h>\nuint32_t esp_random(void);\n',
        'esp_timer.h': '#include <stdint.h>\nint64_t esp_timer_get_time(void);\n',
        'esp_rom_serial_output.h': '#include <stdint.h>\nint esp_rom_output_rx_one_char(uint8_t *);\n',
        'freertos/FreeRTOS.h': '', 'freertos/task.h': 'void vTaskDelay(unsigned);\n',
    }
    for name, text in headers.items():
        p = root / name
        p.parent.mkdir(exist_ok=True)
        p.write_text(text)
    (root / 'main.c').write_text(r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "usb_console_test_support.h"
static int64_t now;
static const char *input;
uint32_t esp_random(void) { return 0x12345678; }
int64_t esp_timer_get_time(void) { return now; }
void vTaskDelay(unsigned ticks) { now += 1000 * ticks; }
int esp_rom_output_rx_one_char(uint8_t *byte) {
    if (!*input) return -1;
    *byte = (uint8_t)*input++;
    return 0;
}
int main(int argc, char **argv) {
    if (argc != 2 && argc != 3) return 2;
    input = argv[1];
    const bool sleep = argc == 3 && strcmp(argv[2], "sleep") == 0;
    if (sleep ? hwtest_usb_console_prepare_sleep() : hwtest_usb_console_ready())
        puts("HARDWARE_TEST_ENTRY");
    printf("elapsed=%lld\n", (long long)now);
    return 0;
}
''')
    exe = root / 'probe'
    subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=address,undefined', '-I', str(root), '-I', str(SUPPORT),
                    str(SUPPORT/'usb_console_test_support.c'), str(root/'main.c'),
                    '-o', str(exe)], check=True, capture_output=True)
    return exe


@pytest.mark.parametrize('message,admitted', [('', False),
    ('CURAG_USB_ACK deadbeef\n', False),
    ('CURAG_USB_ACK 12345678\r\n', True),
    ('stale command\nCURAG_USB_ACK 12345678\n', True),
    ('x'*80+'\nCURAG_USB_ACK 12345678\n', False)])
def test_only_current_complete_ack_admits_tests(boot, message, admitted):
    result = subprocess.run([str(boot), message], check=True, capture_output=True, text=True)
    assert ('HARDWARE_TEST_ENTRY' in result.stdout) is admitted
    assert 'CURAG_USB_READY 12345678\n' in result.stdout
    elapsed = int(result.stdout.rsplit('elapsed=', 1)[1])
    assert elapsed <= 30_000_000
    if not message or 'deadbeef' in message:
        assert elapsed == 30_000_000


@pytest.mark.parametrize('message,admitted', [('', False),
    ('CURAG_USB_ACK 12345678\n', False),
    ('CURAG_USB_SLEEP_ACK deadbeef\n', False),
    ('CURAG_USB_SLEEP_ACK 12345678\r\n', True)])
def test_sleep_requires_its_own_current_receipt(boot, message, admitted):
    result = subprocess.run([str(boot), message, 'sleep'], check=True,
                            capture_output=True, text=True)
    assert ('HARDWARE_TEST_ENTRY' in result.stdout) is admitted
    assert 'CURAG_USB_SLEEP_READY 12345678\n' in result.stdout
    if not admitted:
        assert result.stdout.endswith('elapsed=30000000\n')


@pytest.mark.parametrize('change', [None, 'IDF_TARGET', 'CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG',
    'CONFIG_ESP_CONSOLE_SECONDARY_NONE', 'CONFIG_FREERTOS_UNICORE', 'CONFIG_ESPTOOLPY_FLASHSIZE'])
def test_shared_board_guard_rejects_stale_configuration(tmp_path, change):
    settings = dict(IDF_TARGET='esp32s3', CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG='ON',
                    CONFIG_ESP_CONSOLE_SECONDARY_NONE='ON', CONFIG_FREERTOS_UNICORE='ON',
                    CONFIG_ESPTOOLPY_FLASHSIZE='8MB')
    if change: settings[change] = 'OFF'
    script = tmp_path/'guard.cmake'
    script.write_text(''.join(f'set({k} "{v}")\n' for k, v in settings.items()) +
                      f'include("{SUPPORT.parents[1]}/cmake/xiao_board.cmake")\n')
    result = subprocess.run(['cmake', '-P', str(script)], capture_output=True, text=True)
    assert (result.returncode == 0) is (change is None)
    if change: assert 'XIAO assembly requires' in result.stderr
