from __future__ import annotations

import json
import os
import shlex
import subprocess
from pathlib import Path


GOLDEN_HEADER_NAME = "protocol_v2_lora_golden_vectors_generated.h"


def _u8_array(hex_value: str) -> str:
    return ", ".join(
        f"UINT8_C(0x{byte:02x})" for byte in bytes.fromhex(hex_value)
    )


def _u(value: int, bits: int) -> str:
    return f"UINT{bits}_C({value})"


def _i16(value: int) -> str:
    if value < 0:
        return f"-INT16_C({-value})"
    return f"INT16_C({value})"


def _render_golden_header(vectors: dict[str, object]) -> str:
    vector = vectors["reading_vectors"][0]
    header = vector["clear_header"]
    reading = vector["reading"]

    return f"""\
/* Translated from test-vectors/golden_vectors.json for the native test.
 * Expected bytes are copied, not calculated by the protocol generator. */
#pragma once

#include "protocol_v2_lora_schema_generated.h"

static const cura_lora_v2_clear_header_t TEST_GOLDEN_HEADER = {{
  .control = {_u(header["control"], 8)},
  .domain = {_u(header["domain"], 8)},
  .node_id = {{{_u8_array(header["node_id_hex"])}}},
  .sample_id = {_u(header["sample_id"], 32)},
}};

static const uint8_t
    TEST_GOLDEN_HEADER_BYTES[CURA_LORA_V2_CLEAR_HEADER_SIZE] = {{
  {_u8_array(header["encoded_hex"])}
}};

static const uint8_t TEST_GOLDEN_NONCE[CURA_LORA_V2_NONCE_SIZE] = {{
  {_u8_array(vector["nonce_hex"])}
}};

static const cura_lora_v2_reading_t TEST_GOLDEN_READING = {{
  .run_ms = {_u(reading["run_ms"], 16)},
  .soil_0_mv = {_u(reading["soil_0_mv"], 16)},
  .soil_1_mv = {_u(reading["soil_1_mv"], 16)},
  .soil_temp_0_centi_c = {_i16(reading["soil_temp_0_centi_c"])},
  .soil_temp_1_centi_c = {_i16(reading["soil_temp_1_centi_c"])},
  .enclosure_centi_c = {_i16(reading["enclosure_centi_c"])},
  .enclosure_pressure_pa = {_u(reading["enclosure_pressure_pa"], 32)},
  .enclosure_humidity_centi_pct =
      {_u(reading["enclosure_humidity_centi_pct"], 16)},
  .reset_reason = {_u(reading["reset_reason"], 8)},
  .previous_current_tx_attempts =
      {_u(reading["previous_current_tx_attempts"], 8)},
  .previous_awake_ms = {_u(reading["previous_awake_ms"], 16)},
  .previous_current_delivery_ms =
      {_u(reading["previous_current_delivery_ms"], 16)},
  .previous_cycle_tx_attempts =
      {_u(reading["previous_cycle_tx_attempts"], 8)},
  .previous_cycle_accepted_readings =
      {_u(reading["previous_cycle_accepted_readings"], 8)},
  .flags = {_u(reading["flags"], 16)},
}};

static const uint8_t
    TEST_GOLDEN_READING_BYTES[CURA_LORA_V2_READING_BODY_SIZE] = {{
  {_u8_array(reading["encoded_hex"])}
}};
"""


def test_generated_c_codec_with_asan_and_ubsan(
    repo_root: Path,
    tmp_path: Path,
) -> None:
    protocol_root = repo_root / "protocol" / "protocol-v2-lora"
    vectors_path = protocol_root / "test-vectors" / "golden_vectors.json"
    vectors = json.loads(vectors_path.read_text(encoding="utf-8"))

    golden_header = tmp_path / GOLDEN_HEADER_NAME
    golden_header.write_text(
        _render_golden_header(vectors),
        encoding="utf-8",
    )

    generated_component = (
        repo_root / "firmware" / "components" / "protocol_v2_lora"
    )
    harness = protocol_root / "tests" / "c" / "test_codec_sanitized.c"
    executable = tmp_path / "test_codec_sanitized"

    compiler = shlex.split(os.environ.get("CC", "cc"))
    compile_command = [
        *compiler,
        "-std=c11",
        "-g",
        "-O1",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-pedantic",
        "-fsanitize=address,undefined",
        "-fno-sanitize-recover=all",
        "-fno-omit-frame-pointer",
        f"-I{generated_component / 'include'}",
        f"-I{tmp_path}",
        str(harness),
        str(generated_component / "protocol_v2_lora_schema_generated.c"),
        "-o",
        str(executable),
    ]

    environment = os.environ.copy()
    environment["CCACHE_DISABLE"] = "1"
    compilation = subprocess.run(
        compile_command,
        cwd=repo_root,
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )
    assert compilation.returncode == 0, (
        compilation.stdout + compilation.stderr
    )

    environment["ASAN_OPTIONS"] = (
        "detect_leaks=1:"
        "abort_on_error=1:"
        "halt_on_error=1:"
        "fast_unwind_on_malloc=0"
    )
    environment["UBSAN_OPTIONS"] = "halt_on_error=1:print_stacktrace=1"
    execution = subprocess.run(
        [str(executable)],
        cwd=repo_root,
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )
    assert execution.returncode == 0, execution.stdout + execution.stderr
