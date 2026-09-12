import json
from pathlib import Path

import pytest

from carrier_runner import APP, MISSING_FIXTURES, load_build, run_operation
from carrier_evidence import Evidence, verify_build
from carrier_guided import Guided, IncompleteCase, prior_position
from carrier_exploration import Exploration, OPERATIONS as EXPLORATION_OPERATIONS


def test_sensor_carrier(request, record_property):
    config = request.config
    operation = config.getoption("sensor_operation")
    fixture = config.getoption("sensor_fixture")
    exploration = config.getoption("exploration")
    if exploration and (operation not in EXPLORATION_OPERATIONS or
                        fixture in MISSING_FIXTURES or
                        config.getoption("sensor_guided") or
                        config.getoption("sensor_position") or config.getoption("sensor_prior_evidence")):
        raise pytest.UsageError('--exploration requires an electrical operation and cannot combine with '
                                'guided acceptance or A/B options')
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
    expected_dut = config.getoption("sensor_dut")
    if not expected_dut or len(expected_dut) != 12 or any(c not in "0123456789abcdef" for c in expected_dut):
        raise pytest.UsageError("--sensor-dut must be the confirmed 12-digit factory MAC")
    count = config.getoption("sensor_repeat_count")
    if not isinstance(count, int) or not 100 <= count <= 0xffffffff:
        raise pytest.UsageError("--sensor-repeat-count must be 100..4294967295")
    manifest = verify_build(build_dir, build)
    metadata = {
        "operation": operation,
        "exploration": bool(exploration),
        "expected_dut": expected_dut,
        "requested_count": count if operation == "repeat" else 1,
        "fixture": fixture or "discovery_setup",
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
    root_logdir = config.getoption("root_logdir")
    if not root_logdir:
        raise pytest.UsageError("--root-logdir must identify a new run directory")
    paired = operation in {"ds-identity", "adc-reference"}
    position = config.getoption("sensor_position")
    guided_requested = config.getoption("sensor_guided")
    if (paired or operation in {"reset", "held-reset", "deep-sleep"}) and not (guided_requested or exploration):
        raise pytest.UsageError("this operation requires --sensor-guided and live operator input")
    if not paired and (position or config.getoption("sensor_prior_evidence")):
        raise pytest.UsageError("position/prior evidence apply only to paired identity/reference cases")
    if paired and position not in {"A", "B"}:
        raise pytest.UsageError("select --sensor-position A or B")
    if guided_requested and operation in {"repeat", "discover"}:
        raise pytest.UsageError("guided measurements are not assigned to repeat/discover")
    prior = prior_position(config.getoption("sensor_prior_evidence"), metadata, position) if paired else None
    evidence = Evidence(Path(root_logdir) / "carrier-evidence.json", metadata, manifest)
    record_property("carrier_evidence", str(evidence.path))
    guided = Guided(evidence, operation, position, prior) if guided_requested else None
    if exploration:
        guided = Exploration(evidence, operation)
    try:
        if guided:
            guided.wiring()
        dut = request.getfixturevalue("dut")
        run_operation(dut, build, operation, record_property, expected_dut=expected_dut,
                      fixture=fixture, repeat_count=count, evidence=evidence, guided=guided)
        if guided:
            guided.complete()
        else:
            evidence.finish("software_passed_operator_acceptance_pending")
    except IncompleteCase as exc:
        if exploration and evidence.data['status'] != 'exploration_complete_not_acceptance':
            evidence.finish('exploration_interrupted')
        evidence.add("incomplete", message=str(exc))
        raise
    except KeyboardInterrupt:
        evidence.finish('exploration_interrupted' if exploration else 'incomplete')
        evidence.add('interrupted', message='operator interrupted the run')
        raise
    except BaseException as exc:
        evidence.finish("failed")
        evidence.add("failure", message=str(exc))
        raise
