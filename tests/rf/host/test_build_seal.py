"""F-001: exercise source/build guards with real compiler and CMake records."""
import os
import shutil
import struct
import subprocess
from types import SimpleNamespace

import pytest

import inputs


@pytest.fixture
def build(tmp_path, monkeypatch):
    # Spaces exercise Ninja's displayed paths without interpreting shell syntax.
    app = tmp_path / "radio source"
    main = app / "main"
    component = app / "component"
    main.mkdir(parents=True)
    component.mkdir()
    (app / "CMakeLists.txt").write_text(
        'cmake_minimum_required(VERSION 3.22)\nproject(seal_guard C)\n'
        'add_subdirectory(component)\nadd_subdirectory(main)\n')
    (component / "CMakeLists.txt").write_text(
        'add_library(build_flags INTERFACE)\n'
        'include("${CMAKE_CURRENT_LIST_DIR}/flags.cmake")\n')
    (component / "flags.cmake").write_text(
        'target_compile_definitions(build_flags INTERFACE ORIGINAL_BUILD=1)\n')
    (main / "CMakeLists.txt").write_text(
        'add_executable(cura_radio_component radio_app.c)\n'
        'target_link_libraries(cura_radio_component PRIVATE build_flags)\n')
    # The guard requires substantial actual compiler dependencies, as in IDF.
    for index in range(110):
        (main / f"header{index}.h").write_text(f"#define VALUE_{index} {index}\n")
    (main / "radio_app.c").write_text(
        "".join(f'#include "header{index}.h"\n' for index in range(110)) +
        'int main(void) { return 0; }\n')
    for name in ("sdkconfig", "sdkconfig.defaults", "partitions.csv", "dependencies.lock"):
        (app / name).write_text("test input\n")
    build = tmp_path / "radio build"
    for command in (["cmake", "-S", str(app), "-B", str(build), "-G", "Ninja",
                     "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"],
                    ["cmake", "--build", str(build)]):
        subprocess.run(command, check=True, capture_output=True, text=True, timeout=30,
                       env=os.environ | {"CCACHE_DISABLE": "1"})
    shutil.copyfile(build / "main/cura_radio_component", build / "cura_radio_component.elf")
    (build / "cura_radio_component.bin").write_bytes(b"application fixture")
    (build / "bootloader").mkdir()
    (build / "bootloader/bootloader.bin").write_bytes(b"bootloader fixture")
    (build / "partition_table").mkdir()
    partitions = [(1, 2, 0x9000, 0x6000, "nvs"),
                  (1, 1, 0xf000, 0x1000, "phy_init"),
                  (0, 0, 0x10000, 0x100000, "factory"),
                  (1, 131, 0x110000, 0x2e0000, "storage")]
    (build / "partition_table/partition-table.bin").write_bytes(b"".join(
        struct.pack("<HBBII16sI", 0x50aa, kind, subtype, start, size, label.encode(), 0)
        for kind, subtype, start, size, label in partitions))
    inputs.write_json(build / "flasher_args.json", dict(
        flash_files={"0x0": "bootloader/bootloader.bin",
                     "0x8000": "partition_table/partition-table.bin",
                     "0x10000": "cura_radio_component.bin"},
        extra_esptool_args={"chip": "esp32c6"}))
    (build / "config").mkdir()
    pins = dict(SCLK=6, MOSI=7, MISO=14, CS=23, RESET=18, BUSY=19, DIO1=20)
    config = {f"CURA_SX1262_{name}_GPIO": pin for name, pin in pins.items()}
    config.update(ESP_CONSOLE_UART_NUM=0, ESP_CONSOLE_UART_BAUDRATE=115200)
    inputs.write_json(build / "config/sdkconfig.json", config)
    inputs.write_json(build / "project_description.json",
                      dict(config_file=str(app / "sdkconfig"), config_environment={}))
    monkeypatch.setattr(inputs, "APP", app)
    monkeypatch.setattr(inputs.firmware_sources, "__defaults__", (app,))
    sealed = inputs.seal_build(build)
    assert inputs.verify_build(build) == sealed
    return app, build, sealed


# Component definitions and included CMake scripts must invalidate an old ELF.
@pytest.mark.parametrize("name", ["CMakeLists.txt", "flags.cmake"])
def test_changed_build_definition_rejected_without_rebuild(build, name):
    app, root, sealed = build
    definition = app / "component" / name
    definition.write_text(definition.read_text() +
                          'target_compile_options(build_flags INTERFACE -O0)\n')
    with pytest.raises(ValueError, match="source/build seal mismatch"):
        inputs.verify_build(root)
    assert inputs.digest(root / "cura_radio_component.elf") == sealed["files"]["cura_radio_component.elf"]
    assert inputs.digest(root / "cura_radio_component.bin") == sealed["files"]["cura_radio_component.bin"]


# Deleted inputs cannot silently disappear from the configured dependency graph.
def test_missing_configured_build_definition_rejected(build):
    app, root, _ = build
    (app / "component/flags.cmake").unlink()
    with pytest.raises(ValueError, match="missing CMake regeneration input"):
        inputs.verify_build(root)


# Missing/invalid graph records fail closed rather than omitting build definitions.
@pytest.mark.parametrize("record", ["", "build.ninja:\n  input: phony\n  outputs:\n",
                                   "build.ninja:\n  input: RERUN_CMAKE\n  outputs:\n"])
def test_missing_cmake_regeneration_records_rejected(build, monkeypatch, record):
    _, root, _ = build
    original = subprocess.run

    def query(command, **kwargs):
        if command[-3:] == ["-t", "query", "build.ninja"]:
            return SimpleNamespace(stdout=record)
        return original(command, **kwargs)

    monkeypatch.setattr(inputs.subprocess, "run", query)
    with pytest.raises(ValueError, match="missing actual CMake regeneration records"):
        inputs.verify_build(root)
