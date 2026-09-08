import json
from pathlib import Path

import pytest

from carrier_runner import APP, load_build, run_operation


def test_sensor_carrier(request, record_property):
    config = request.config
    operation = config.getoption("sensor_operation")
    # Validate before requesting dut: its fixture opens and flashes hardware.
    if Path(config.getoption("app_path") or "").resolve() != APP:
        raise pytest.UsageError(f"--app-path must be {APP}")
    if config.getoption("target") != "esp32c6":
        raise pytest.UsageError("--target=esp32c6 is required")
    if set((config.getoption("embedded_services") or "").split(",")) != {"esp", "idf"}:
        raise pytest.UsageError("--embedded-services=esp,idf is required")
    if config.getoption("erase_all") or config.getoption("skip_autoflash"):
        raise pytest.UsageError("use the default app flash; do not erase all or skip autoflash")
    if config.getoption("count") != 1:
        raise pytest.UsageError("this slice uses exactly one DUT")
    port = config.getoption("port")
    if not port or not Path(port).is_char_device():
        raise pytest.UsageError("--port must identify the confirmed, available UART DUT")
    build_dir = Path(config.getoption("build_dir") or "build")
    if not build_dir.is_absolute():
        build_dir = APP / build_dir
    build = load_build(build_dir, operation)
    metadata = {
        "operation": operation,
        "fixture": config.getoption("sensor_fixture") or "discovery_setup",
        "carrier_revision": config.getoption("carrier_revision"),
        "port": port,
        "elf_sha256": build.elf_sha256,
        "config_sha256": build.config_sha256,
        "source_version": build.source_version,
        "rom0": build.boot_values["rom0"],
        "rom1": build.boot_values["rom1"],
    }
    for key, value in metadata.items():
        record_property(key, value)
    print("CARRIER_RUN " + json.dumps(metadata, sort_keys=True), flush=True)
    dut = request.getfixturevalue("dut")
    run_operation(dut, build, operation, record_property)
