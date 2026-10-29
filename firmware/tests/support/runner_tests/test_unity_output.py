"""Exercise stock Unity output against a bounded tty drained every 50 ms."""
import os
from pathlib import Path
import subprocess

import pytest

SUPPORT = Path(__file__).resolve().parents[1]
LINE = b'An independently registered long Unity test name and its reset/deep-sleep attributes; repeated to exceed the native USB tty buffering capacity.'


@pytest.mark.parametrize('tick_ms', [1, 10])
def test_long_unity_output_survives_normal_reader_pacing(tmp_path, tick_ms):
    idf = Path(os.environ.get('IDF_PATH', Path.home()/'esp/esp-idf'))
    headers = {
        'sdkconfig.h': '#define CONFIG_ESP_CONSOLE_ROM_SERIAL_PORT_NUM 4\n',
        'esp_err.h': '#include <stdint.h>\n#define ESP_OK 0\n',
        'esp_cpu.h': '#include <stdint.h>\nstatic inline uint32_t esp_cpu_get_cycle_count(void) { return 0; }\n',
        'esp_private/esp_clk.h': 'static inline unsigned esp_clk_cpu_freq(void) { return 160000000; }\n',
        'esp_system_console.h': 'void esp_system_console_put_char(char);\n',
        'esp_rom_serial_output.h': '#include <stdint.h>\nint esp_rom_output_rx_one_char(uint8_t *);\nvoid esp_rom_output_tx_one_char(char);\nvoid esp_rom_output_tx_wait_idle(int);\n',
        'freertos/FreeRTOS.h': f'typedef unsigned TickType_t;\n#define pdMS_TO_TICKS(ms) ((ms)/{tick_ms})\n',
        'freertos/task.h': 'void vTaskDelay(unsigned);\n',
    }
    for name, contents in headers.items():
        p=tmp_path/name; p.parent.mkdir(exist_ok=True); p.write_text(contents)
    source=tmp_path/'boundary.c'
    source.write_text(r'''
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "unity.h"
static unsigned now, queued, dropped, used;
static char captured[20000];
void setUp(void) {}
void tearDown(void) {}
void vTaskDelay(unsigned ticks) {
    unsigned next=now+ticks*TICK_MS;
    if (next/50 != now/50) queued=0;
    now=next;
}
void esp_system_console_put_char(char c) {
    if (queued == 4096) { ++dropped; return; }
    if (used == sizeof(captured)) abort();
    captured[used++]=c; ++queued;
}
void esp_rom_output_tx_one_char(char c) { esp_system_console_put_char(c); }
void esp_rom_output_tx_wait_idle(int port) { (void)port; }
int esp_rom_output_rx_one_char(uint8_t *byte) { (void)byte; return -1; }
int main(void) {
    UnityPrint("Press ENTER to see the list of tests."); UNITY_PRINT_EOL();
    assert(now == 0);
    for (unsigned i=0;i<80;++i) { UnityPrint(LINE); UNITY_PRINT_EOL(); }
    UnityPrint("Enter test for running.");
    unsigned prompt_finished=now;
    UNITY_PRINT_EOL();
    assert(now == prompt_finished);
    fwrite(captured,1,used,stdout);
    fprintf(stderr,"dropped=%u\n",dropped);
    return 0;
}
''')
    common=[os.environ.get('CC','cc'),'-std=c11','-Wall','-Wextra','-Werror',
        '-fsanitize=address,undefined','-fno-omit-frame-pointer','-DUNITY_INCLUDE_CONFIG_H',
        f'-DTICK_MS={tick_ms}', '-DLINE="'+LINE.decode()+'"',
        '-I',str(tmp_path),'-I',str(idf/'components/unity/include'),
        '-I',str(idf/'components/unity/unity/src'), str(source),
        str(idf/'components/unity/unity/src/unity.c'),
        str(idf/'components/unity/unity_port_esp32.c')]
    results=[]
    for paced in [False,True]:
        exe=tmp_path/('paced' if paced else 'unpaced')
        extra=([str(SUPPORT/'unity_console.c'),'-Wl,--wrap=unity_putc',
                '-Wl,--wrap=unity_gets','-Wl,--wrap=esp_rom_output_rx_one_char'] if paced else [])
        compiled=subprocess.run(common+extra+['-o',str(exe)],capture_output=True)
        assert compiled.returncode == 0, compiled.stderr.decode()
        results.append(subprocess.run([str(exe)],check=True,capture_output=True))
    control,paced=results
    assert b'Enter test for running.' not in control.stdout
    assert paced.stdout == (b'Press ENTER to see the list of tests.\r\n'+
                            (LINE+b'\r\n')*80+b'Enter test for running.\r\n')
    assert paced.stderr == b'dropped=0\n'
