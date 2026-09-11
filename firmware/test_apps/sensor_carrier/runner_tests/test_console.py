"""Run the actual installed Unity input loop with UART/RTOS boundary stubs."""
import os
from pathlib import Path
import subprocess

import pytest

from carrier_runner import APP


@pytest.fixture(scope='module')
def console_binary(tmp_path_factory):
    root = tmp_path_factory.mktemp('unity-console')
    headers = {
        'unity.h': '#include <stddef.h>\nvoid unity_gets(char *, size_t);\n',
        'sdkconfig.h': '#define CONFIG_ESP_CONSOLE_ROM_SERIAL_PORT_NUM 0\n',
        'esp_cpu.h': '#include <stdint.h>\nstatic inline uint32_t esp_cpu_get_cycle_count(void) { return 0; }\n',
        'esp_private/esp_clk.h': 'static inline unsigned esp_clk_cpu_freq(void) { return 160000000; }\n',
        'esp_rom_serial_output.h': '''#include <stdint.h>
int esp_rom_output_rx_one_char(uint8_t *byte);
void esp_rom_output_tx_one_char(char byte);
void esp_rom_output_tx_wait_idle(int port);
''',
        'esp_system_console.h': 'void esp_system_console_put_char(char byte);\n',
        'freertos/FreeRTOS.h': '',
        'freertos/task.h': 'void vTaskDelay(unsigned ticks);\n',
    }
    for name, contents in headers.items():
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents)
    harness = root / 'boundary.c'
    harness.write_text('''#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "unity.h"
#include "esp_rom_serial_output.h"
static unsigned now_ms, idle_ticks, polls;
static const char *input;
void vTaskDelay(unsigned ticks) { assert(ticks == 1); now_ms += 10; ++idle_ticks; polls = 0; }
int esp_rom_output_rx_one_char(uint8_t *byte) {
  /* Without the fix, Unity loops indefinitely and starves IDLE. Fail rather
   * than hanging this regression if the yielding wrapper is absent/bypassed. */
  assert(++polls < 100);
  if (now_ms < 12000 || !*input) return 7;
  *byte = (uint8_t)*input++;
  return 0;
}
void esp_rom_output_tx_one_char(char byte) { (void)byte; }
void esp_system_console_put_char(char byte) { (void)byte; }
void esp_rom_output_tx_wait_idle(int port) { (void)port; }
int main(int argc, char **argv) {
  assert(argc == 2);
  input = atoi(argv[1]) == 0 ? "YES\\n" : "YEX\\bS\\r";
  char line[16];
  unity_gets(line, sizeof(line));
  assert(strcmp(line, "YES") == 0);
  assert(now_ms == 12000 && idle_ticks == 1200);
  /* The wrapper must not add a delay or alter status outside Unity waiting. */
  uint8_t byte = 123;
  assert(esp_rom_output_rx_one_char(&byte) == 7);
  assert(byte == 123 && idle_ticks == 1200);
}
''')
    # The boundary implementation must be a separate object so GNU --wrap
    # interposes calls just as it does for the target's ROM UART symbol.
    idf = Path(os.environ.get('IDF_PATH', str(Path.home() / 'esp/esp-idf')))
    unity = idf / 'components/unity/unity_port_esp32.c'
    binary = root / 'console'
    # Put the UART definition apart from its call sites to exercise interposition.
    source = harness.read_text()
    begin = source.index('int esp_rom_output_rx_one_char(uint8_t *byte) {')
    end = source.index('void esp_rom_output_tx_one_char', begin)
    boundary = root / 'uart.c'
    boundary.write_text('#include <assert.h>\n#include <stdint.h>\n'
                        'extern unsigned now_ms, polls;\nextern const char *input;\n' + source[begin:end])
    harness.write_text((source[:begin] + source[end:]).replace('static unsigned', 'unsigned')
                       .replace('static const char *input;', 'const char *input;'))
    subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-g',
                    '-I', str(root), str(harness), str(boundary), str(unity),
                    str(APP/'main/carrier_console.c'), '-Wl,--wrap=unity_gets',
                    '-Wl,--wrap=esp_rom_output_rx_one_char', '-o', str(binary)],
                   check=True, capture_output=True, text=True)
    return binary


@pytest.mark.parametrize('scenario', [0, 1], ids=['long_wait', 'stock_backspace_parser'])
def test_unity_wait_yields_without_changing_parser_or_other_uart(console_binary, scenario):
    subprocess.run([str(console_binary), str(scenario)], check=True, capture_output=True, text=True)
