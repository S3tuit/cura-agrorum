from __future__ import annotations

import json
import os
import shlex
import subprocess
from pathlib import Path
from typing import Any


GOLDEN_HEADER_NAME = "protocol_v2_lora_crypto_vectors_generated.h"


def _u8_array(hex_value: str) -> str:
    return ", ".join(
        f"UINT8_C(0x{byte:02x})" for byte in bytes.fromhex(hex_value)
    )


def _render_golden_header(vectors: dict[str, Any]) -> str:
    vector = vectors["reading_vectors"][0]
    header = vector["clear_header"]
    body_hex = vector["reading"]["encoded_hex"]
    frame_hex = vector["encrypted"]["frame_hex"]

    return f"""\
/* Translated from test-vectors/golden_vectors.json for the native test. */
#pragma once

#include "protocol_v2_lora_schema_generated.h"

static const uint8_t TEST_CRYPTO_NODE_KEY[CURA_LORA_V2_KEY_SIZE] = {{
  {_u8_array(vectors["crypto"]["node_key_hex"])}
}};

static const cura_lora_v2_clear_header_t TEST_CRYPTO_HEADER = {{
  .control = UINT8_C({header["control"]}),
  .domain = UINT8_C({header["domain"]}),
  .node_id = {{{_u8_array(header["node_id_hex"])}}},
  .message_id = UINT32_C({header["message_id"]}),
}};

static const uint8_t
    TEST_CRYPTO_READING_BODY[CURA_LORA_V2_READING_BODY_SIZE] = {{
  {_u8_array(body_hex)}
}};

static const uint8_t
    TEST_CRYPTO_READING_FRAME[CURA_LORA_V2_READING_FRAME_SIZE] = {{
  {_u8_array(frame_hex)}
}};
"""


def test_firmware_crypto_with_asan_and_ubsan(
    repo_root: Path,
    tmp_path: Path,
) -> None:
    protocol_root = repo_root / "protocol" / "protocol-v2-lora"
    generated_component = (
        repo_root / "firmware" / "components" / "protocol_v2_lora"
    )
    vectors = json.loads(
        (
            protocol_root / "test-vectors" / "golden_vectors.json"
        ).read_text(encoding="utf-8")
    )

    golden_header = tmp_path / GOLDEN_HEADER_NAME
    golden_header.write_text(
        _render_golden_header(vectors),
        encoding="utf-8",
    )
    executable = tmp_path / "test_crypto_sanitized"
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
        "-DCURA_LORA_V2_CRYPTO_USE_OPENSSL",
        f"-I{generated_component / 'include'}",
        f"-I{tmp_path}",
        str(protocol_root / "tests" / "c" / "test_crypto_sanitized.c"),
        str(generated_component / "protocol_v2_lora_schema_generated.c"),
        str(generated_component / "protocol_v2_lora_crypto.c"),
        "-o",
        str(executable),
        "-lcrypto",
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
