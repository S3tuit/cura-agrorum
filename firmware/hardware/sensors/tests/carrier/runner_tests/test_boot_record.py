"""Compile the actual app entry point against its ESP-IDF dependencies."""

import os
import subprocess

import pytest

from carrier_runner import APP


@pytest.fixture(scope="module")
def boot_record(tmp_path_factory):
    root = tmp_path_factory.mktemp("carrier-boot-record")
    headers = {
        "esp_err.h": """#pragma once
#include <assert.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERROR_CHECK(value) assert((value) == ESP_OK)
""",
        "esp_mac.h": """#pragma once
#include <stdint.h>
#include "esp_err.h"
typedef enum { ESP_MAC_EFUSE_FACTORY } esp_mac_type_t;
esp_err_t esp_read_mac(uint8_t *mac, esp_mac_type_t type);
esp_err_t esp_efuse_mac_get_default(uint8_t *mac);
""",
        "esp_app_desc.h": """#pragma once
#include <stddef.h>
#include <stdint.h>
typedef struct { uint8_t app_elf_sha256[32]; } esp_app_desc_t;
const esp_app_desc_t *esp_app_get_description(void);
int esp_app_get_elf_sha256(char *dst, size_t size);
""",
        "unity_test_runner.h": "void unity_run_menu(void);\n",
        "sdkconfig.h": """#define CONFIG_IDF_TARGET_ESP32C6 1
#define CONFIG_CURA_SOIL_0_GPIO 0
#define CONFIG_CURA_SOIL_1_GPIO 1
#define CONFIG_CURA_SENSOR_POWER_GATE_GPIO 2
#define CONFIG_CURA_DS18B20_GPIO 3
#define CONFIG_CURA_I2C_SDA_GPIO 21
#define CONFIG_CURA_I2C_SCL_GPIO 22
#define CONFIG_CURA_SENSOR_POWER_STABILIZATION_MS 200
#define CONFIG_ESP_CONSOLE_UART_DEFAULT 1
#define CONFIG_ESP_CONSOLE_UART_NUM 0
#define CONFIG_ESP_CONSOLE_UART_BAUDRATE 115200
#define CONFIG_CURA_DS18B20_0_ROM "0000000000000000"
#define CONFIG_CURA_DS18B20_1_ROM "0000000000000000"
""",
    }
    for name, content in headers.items():
        (root / name).write_text(content)
    harness = root / "idf_boundary.c"
    harness.write_text("""#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "esp_mac.h"
#include "esp_app_desc.h"
static const esp_app_desc_t description = {.app_elf_sha256 = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f
}};
const esp_app_desc_t *esp_app_get_description(void) { return &description; }
int esp_app_get_elf_sha256(char *dst, size_t size) {
    /* ESP-IDF defaults CONFIG_APP_RETRIEVE_LEN_ELF_SHA to nine. */
    int count = snprintf(dst, size, "000102030");
    return count + 1;
}
esp_err_t esp_efuse_mac_get_default(uint8_t *mac) {
    /* The C6 default API returns EUI-64, not the six-byte base MAC. */
    const uint8_t eui64[8] = {0x10,0x20,0x30,0xff,0xfe,0x40,0x50,0x60};
    memcpy(mac, eui64, sizeof(eui64));
    return ESP_OK;
}
esp_err_t esp_read_mac(uint8_t *mac, esp_mac_type_t type) {
    assert(type == ESP_MAC_EFUSE_FACTORY);
    const uint8_t factory[6] = {0x10,0x20,0x30,0x40,0x50,0x60};
    memcpy(mac, factory, sizeof(factory));
    return ESP_OK;
}
void unity_run_menu(void) { puts("UNITY_MENU_ENTERED"); }
void app_main(void);
int main(void) { app_main(); return 0; }
""")
    executable = root / "boot-record"
    build = subprocess.run([
        os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-g",
        "-I", str(root), str(APP / "main/test_app_main.c"), str(harness),
        "-o", str(executable),
    ], capture_output=True, text=True)
    assert build.returncode == 0, build.stderr
    result = subprocess.run([str(executable)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert result.stdout.endswith("UNITY_MENU_ENTERED\n")
    return dict(field.split("=", 1) for field in result.stdout.splitlines()[0].split()[1:])


def test_boot_prints_full_hash_independent_of_idf_log_prefix(boot_record):
    assert boot_record["elf"] == (
        "000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f"
    )


def test_boot_reports_factory_mac_without_eui64_overflow(boot_record):
    assert boot_record["dut"] == "102030405060"
