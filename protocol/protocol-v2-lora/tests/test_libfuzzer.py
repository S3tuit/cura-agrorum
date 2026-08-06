from __future__ import annotations

import json
import os
import secrets
import shlex
import subprocess
from pathlib import Path
from typing import Any


DECODE_HEADER = 0
DECODE_READING = 1
DECODE_ACK = 2
ENCODE_HEADER = 3
ENCODE_READING = 4
ENCODE_ACK = 5
BUILD_NONCE = 6
MAX_LIBFUZZER_SEED = (1 << 32) - 1


def _seed_corpus(
    corpus: Path,
    vectors: dict[str, Any],
) -> None:
    corpus.mkdir(parents=True, exist_ok=True)
    reading_vector = vectors["reading_vectors"][0]
    header = bytes.fromhex(reading_vector["clear_header"]["encoded_hex"])
    reading = bytes.fromhex(reading_vector["reading"]["encoded_hex"])

    seeds = {
        "decode-header-golden": bytes((DECODE_HEADER,)) + header,
        "decode-header-short": bytes((DECODE_HEADER,)) + header[:-1],
        "decode-header-long": bytes((DECODE_HEADER,)) + header + b"\x00",
        "decode-reading-golden": bytes((DECODE_READING,)) + reading,
        "decode-reading-short": bytes((DECODE_READING,)) + reading[:-3],
        "decode-reading-long": bytes((DECODE_READING,)) + reading + b"\x00",
        "encode-header-golden": (
            bytes((ENCODE_HEADER, len(header))) + header
        ),
        "encode-reading-golden": (
            bytes((ENCODE_READING, len(reading))) + reading
        ),
        "build-nonce-golden": bytes((BUILD_NONCE, 13)) + header,
    }

    for vector in vectors["ack_vectors"]:
        status = bytes.fromhex(vector["ack"]["encoded_hex"])
        name = vector["name"]
        seeds[f"decode-ack-{name}"] = bytes((DECODE_ACK,)) + status
        seeds[f"encode-ack-{name}"] = bytes((ENCODE_ACK, 1)) + status

    for name, seed in seeds.items():
        (corpus / name).write_bytes(seed)


def _libfuzzer_seed() -> int:
    configured_seed = os.environ.get("FUZZ_SEED")
    if configured_seed is None:
        return secrets.randbelow(MAX_LIBFUZZER_SEED) + 1

    seed = int(configured_seed, 0)
    if not 1 <= seed <= MAX_LIBFUZZER_SEED:
        raise ValueError(
            f"FUZZ_SEED must be between 1 and {MAX_LIBFUZZER_SEED}"
        )
    return seed


def test_libfuzzer_codec_smoke(
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

    corpus = protocol_root / ".fuzz-corpus" / "codec"
    _seed_corpus(corpus, vectors)
    executable = tmp_path / "fuzz_codec"

    compiler = shlex.split(os.environ.get("FUZZ_CC", "clang"))
    compile_command = [
        *compiler,
        "-std=c11",
        "-g",
        "-O1",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-pedantic",
        "-fsanitize=fuzzer,address,undefined",
        "-fno-sanitize-recover=all",
        "-fno-omit-frame-pointer",
        f"-I{generated_component / 'include'}",
        str(protocol_root / "tests" / "c" / "fuzz_codec.c"),
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
    fuzz_seed = _libfuzzer_seed()
    execution = subprocess.run(
        [
            str(executable),
            str(corpus),
            "-runs=30000",
            f"-seed={fuzz_seed}",
            "-max_len=64",
            "-use_value_profile=1",
            "-print_final_stats=1",
        ],
        cwd=tmp_path,
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )
    assert execution.returncode == 0, (
        f"libFuzzer seed: {fuzz_seed}\n"
        f"Reproduce with FUZZ_SEED={fuzz_seed}\n"
        + execution.stdout
        + execution.stderr
    )
