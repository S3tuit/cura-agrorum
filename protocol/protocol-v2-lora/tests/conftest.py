from __future__ import annotations

import ctypes
import importlib.util
import json
import os
import shlex
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType
from typing import Any, Mapping, Protocol

import pytest


PROTOCOL_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
GOLDEN_VECTORS_PATH = PROTOCOL_ROOT / "test-vectors" / "golden_vectors.json"
GENERATED_C_HEADER_DIR = (
    REPO_ROOT / "firmware" / "components" / "protocol_v2_lora" / "include"
)
GENERATED_C_SOURCE = (
    REPO_ROOT
    / "firmware"
    / "components"
    / "protocol_v2_lora"
    / "protocol_v2_lora_schema_generated.c"
)
GENERATED_PYTHON = (
    REPO_ROOT
    / "receiver"
    / "cura_receiver"
    / "generated"
    / "protocol_v2_lora_generated.py"
)


class CodecRejected(ValueError):
    """A generated codec rejected a value or byte sequence."""


class Codec(Protocol):
    name: str

    def is_supported_control(self, control: int) -> bool: ...

    def domain_is_reading(self, domain: int) -> bool: ...

    def domain_is_ack(self, domain: int) -> bool: ...

    def encode_header(self, fields: Mapping[str, Any]) -> bytes: ...

    def decode_header(self, data: bytes) -> dict[str, Any]: ...

    def build_nonce(self, fields: Mapping[str, Any]) -> bytes: ...

    def encode_reading(self, fields: Mapping[str, Any]) -> bytes: ...

    def decode_reading(self, data: bytes) -> dict[str, Any]: ...

    def encode_ack(self, fields: Mapping[str, Any]) -> bytes: ...

    def decode_ack(self, data: bytes) -> dict[str, Any]: ...

    def ack_status_matches_domain(self, domain: int, status: int) -> bool: ...


def _without_encoded_hex(fields: Mapping[str, Any]) -> dict[str, Any]:
    return {key: value for key, value in fields.items() if key != "encoded_hex"}


@dataclass(frozen=True)
class PythonCodec:
    module: ModuleType
    name: str = "python"

    def is_supported_control(self, control: int) -> bool:
        return self.module.is_supported_control(control)

    def domain_is_reading(self, domain: int) -> bool:
        return self.module.domain_is_reading(domain)

    def domain_is_ack(self, domain: int) -> bool:
        return self.module.domain_is_ack(domain)

    def _header(self, fields: Mapping[str, Any]) -> Any:
        return self.module.ClearHeader(
            control=fields["control"],
            domain=fields["domain"],
            node_id=bytes.fromhex(fields["node_id_hex"]),
            sample_id=fields["sample_id"],
        )

    def encode_header(self, fields: Mapping[str, Any]) -> bytes:
        try:
            return self.module.encode_clear_header(self._header(fields))
        except self.module.CodecError as exc:
            raise CodecRejected(str(exc)) from exc

    def decode_header(self, data: bytes) -> dict[str, Any]:
        try:
            header = self.module.decode_clear_header(data)
        except self.module.CodecError as exc:
            raise CodecRejected(str(exc)) from exc
        return {
            "control": header.control,
            "domain": header.domain,
            "node_id_hex": header.node_id.hex(),
            "sample_id": header.sample_id,
        }

    def build_nonce(self, fields: Mapping[str, Any]) -> bytes:
        try:
            return self.module.build_nonce(self._header(fields))
        except self.module.CodecError as exc:
            raise CodecRejected(str(exc)) from exc

    def encode_reading(self, fields: Mapping[str, Any]) -> bytes:
        try:
            reading = self.module.Reading(**_without_encoded_hex(fields))
            return self.module.encode_reading(reading)
        except self.module.CodecError as exc:
            raise CodecRejected(str(exc)) from exc

    def decode_reading(self, data: bytes) -> dict[str, Any]:
        try:
            reading = self.module.decode_reading(data)
        except self.module.CodecError as exc:
            raise CodecRejected(str(exc)) from exc
        return {
            field: getattr(reading, field)
            for field in reading.__dataclass_fields__
        }

    def encode_ack(self, fields: Mapping[str, Any]) -> bytes:
        try:
            ack = self.module.Ack(**_without_encoded_hex(fields))
            return self.module.encode_ack(ack)
        except self.module.CodecError as exc:
            raise CodecRejected(str(exc)) from exc

    def decode_ack(self, data: bytes) -> dict[str, Any]:
        try:
            ack = self.module.decode_ack(data)
        except self.module.CodecError as exc:
            raise CodecRejected(str(exc)) from exc
        return {"status": ack.status}

    def ack_status_matches_domain(self, domain: int, status: int) -> bool:
        return self.module.ack_status_matches_domain(domain, status)


class CHeader(ctypes.Structure):
    _fields_ = [
        ("control", ctypes.c_uint8),
        ("domain", ctypes.c_uint8),
        ("node_id", ctypes.c_uint8 * 8),
        ("sample_id", ctypes.c_uint32),
    ]


class CReading(ctypes.Structure):
    _fields_ = [
        ("run_ms", ctypes.c_uint16),
        ("soil_0_mv", ctypes.c_uint16),
        ("soil_1_mv", ctypes.c_uint16),
        ("soil_temp_0_centi_c", ctypes.c_int16),
        ("soil_temp_1_centi_c", ctypes.c_int16),
        ("enclosure_centi_c", ctypes.c_int16),
        ("enclosure_pressure_pa", ctypes.c_uint32),
        ("enclosure_humidity_centi_pct", ctypes.c_uint16),
        ("reset_reason", ctypes.c_uint8),
        ("previous_current_tx_attempts", ctypes.c_uint8),
        ("previous_awake_ms", ctypes.c_uint16),
        ("previous_current_delivery_ms", ctypes.c_uint16),
        ("previous_cycle_tx_attempts", ctypes.c_uint8),
        ("previous_cycle_accepted_readings", ctypes.c_uint8),
        ("flags", ctypes.c_uint16),
    ]


class CAck(ctypes.Structure):
    _fields_ = [("status", ctypes.c_uint8)]


@dataclass(frozen=True)
class CCodec:
    library: ctypes.CDLL
    name: str = "c"

    def is_supported_control(self, control: int) -> bool:
        return self.library.cura_lora_v2_is_supported_control(control)

    def domain_is_reading(self, domain: int) -> bool:
        return self.library.cura_lora_v2_domain_is_reading(domain)

    def domain_is_ack(self, domain: int) -> bool:
        return self.library.cura_lora_v2_domain_is_ack(domain)

    @staticmethod
    def _check(result: int) -> None:
        if result != 0:
            raise CodecRejected(f"C codec result: {result}")

    @staticmethod
    def _header(fields: Mapping[str, Any]) -> CHeader:
        node_id = bytes.fromhex(fields["node_id_hex"])
        if len(node_id) != 8:
            raise CodecRejected("node_id must be 8 bytes")
        return CHeader(
            fields["control"],
            fields["domain"],
            (ctypes.c_uint8 * 8)(*node_id),
            fields["sample_id"],
        )

    @classmethod
    def _encode(
        cls,
        function: Any,
        output_size: int,
        value: ctypes.Structure,
    ) -> bytes:
        output = (ctypes.c_uint8 * output_size)()
        cls._check(function(output, output_size, ctypes.byref(value)))
        return bytes(output)

    @classmethod
    def _decode(
        cls,
        function: Any,
        structure_type: type[ctypes.Structure],
        data: bytes,
    ) -> ctypes.Structure:
        value = structure_type()
        input_buffer = (ctypes.c_uint8 * len(data)).from_buffer_copy(data)
        cls._check(function(ctypes.byref(value), input_buffer, len(data)))
        return value

    def encode_header(self, fields: Mapping[str, Any]) -> bytes:
        return self._encode(
            self.library.cura_lora_v2_encode_clear_header,
            14,
            self._header(fields),
        )

    def decode_header(self, data: bytes) -> dict[str, Any]:
        header = self._decode(
            self.library.cura_lora_v2_decode_clear_header,
            CHeader,
            data,
        )
        return {
            "control": header.control,
            "domain": header.domain,
            "node_id_hex": bytes(header.node_id).hex(),
            "sample_id": header.sample_id,
        }

    def build_nonce(self, fields: Mapping[str, Any]) -> bytes:
        return self._encode(
            self.library.cura_lora_v2_build_nonce,
            13,
            self._header(fields),
        )

    def encode_reading(self, fields: Mapping[str, Any]) -> bytes:
        values = _without_encoded_hex(fields)
        reading = CReading(
            *(values[field_name] for field_name, _ in CReading._fields_)
        )
        return self._encode(
            self.library.cura_lora_v2_encode_reading,
            28,
            reading,
        )

    def decode_reading(self, data: bytes) -> dict[str, Any]:
        reading = self._decode(
            self.library.cura_lora_v2_decode_reading,
            CReading,
            data,
        )
        return {
            field_name: getattr(reading, field_name)
            for field_name, _ in CReading._fields_
        }

    def encode_ack(self, fields: Mapping[str, Any]) -> bytes:
        ack = CAck(fields["status"])
        return self._encode(
            self.library.cura_lora_v2_encode_ack,
            1,
            ack,
        )

    def decode_ack(self, data: bytes) -> dict[str, Any]:
        ack = self._decode(
            self.library.cura_lora_v2_decode_ack,
            CAck,
            data,
        )
        return {"status": ack.status}

    def ack_status_matches_domain(self, domain: int, status: int) -> bool:
        return self.library.cura_lora_v2_ack_status_matches_domain(
            domain,
            status,
        )


def _configure_c_library(library: ctypes.CDLL) -> None:
    byte_pointer = ctypes.POINTER(ctypes.c_uint8)

    library.cura_lora_v2_is_supported_control.argtypes = [ctypes.c_uint8]
    library.cura_lora_v2_is_supported_control.restype = ctypes.c_bool
    library.cura_lora_v2_domain_is_reading.argtypes = [ctypes.c_uint8]
    library.cura_lora_v2_domain_is_reading.restype = ctypes.c_bool
    library.cura_lora_v2_domain_is_ack.argtypes = [ctypes.c_uint8]
    library.cura_lora_v2_domain_is_ack.restype = ctypes.c_bool
    library.cura_lora_v2_encode_clear_header.argtypes = [
        byte_pointer,
        ctypes.c_size_t,
        ctypes.POINTER(CHeader),
    ]
    library.cura_lora_v2_encode_clear_header.restype = ctypes.c_int
    library.cura_lora_v2_decode_clear_header.argtypes = [
        ctypes.POINTER(CHeader),
        byte_pointer,
        ctypes.c_size_t,
    ]
    library.cura_lora_v2_decode_clear_header.restype = ctypes.c_int
    library.cura_lora_v2_build_nonce.argtypes = [
        byte_pointer,
        ctypes.c_size_t,
        ctypes.POINTER(CHeader),
    ]
    library.cura_lora_v2_build_nonce.restype = ctypes.c_int
    library.cura_lora_v2_encode_reading.argtypes = [
        byte_pointer,
        ctypes.c_size_t,
        ctypes.POINTER(CReading),
    ]
    library.cura_lora_v2_encode_reading.restype = ctypes.c_int
    library.cura_lora_v2_decode_reading.argtypes = [
        ctypes.POINTER(CReading),
        byte_pointer,
        ctypes.c_size_t,
    ]
    library.cura_lora_v2_decode_reading.restype = ctypes.c_int
    library.cura_lora_v2_encode_ack.argtypes = [
        byte_pointer,
        ctypes.c_size_t,
        ctypes.POINTER(CAck),
    ]
    library.cura_lora_v2_encode_ack.restype = ctypes.c_int
    library.cura_lora_v2_decode_ack.argtypes = [
        ctypes.POINTER(CAck),
        byte_pointer,
        ctypes.c_size_t,
    ]
    library.cura_lora_v2_decode_ack.restype = ctypes.c_int
    library.cura_lora_v2_ack_status_matches_domain.argtypes = [
        ctypes.c_uint8,
        ctypes.c_uint8,
    ]
    library.cura_lora_v2_ack_status_matches_domain.restype = ctypes.c_bool


def _load_golden_vectors() -> dict[str, Any]:
    vectors = json.loads(GOLDEN_VECTORS_PATH.read_text(encoding="utf-8"))
    if vectors["format_version"] != 1:
        raise ValueError("unsupported golden-vector format")
    if vectors["protocol_version"] != 2:
        raise ValueError("golden vectors are not for protocol v2")
    return vectors


GOLDEN_VECTORS = _load_golden_vectors()


@pytest.fixture(scope="session")
def repo_root() -> Path:
    return REPO_ROOT


@pytest.fixture(scope="session")
def python_codec() -> PythonCodec:
    module_name = "cura_receiver_protocol_v2_lora_generated_for_tests"
    spec = importlib.util.spec_from_file_location(module_name, GENERATED_PYTHON)
    if spec is None or spec.loader is None:
        pytest.fail(f"cannot import generated Python codec: {GENERATED_PYTHON}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return PythonCodec(module)


@pytest.fixture(scope="session")
def c_codec(
    tmp_path_factory: pytest.TempPathFactory,
) -> CCodec:
    output = tmp_path_factory.mktemp("protocol_v2_lora_c") / "codec.so"
    compiler = shlex.split(os.environ.get("CC", "cc"))
    command = [
        *compiler,
        "-std=c11",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-pedantic",
        "-fPIC",
        "-shared",
        f"-I{GENERATED_C_HEADER_DIR}",
        str(GENERATED_C_SOURCE),
        "-o",
        str(output),
    ]
    environment = os.environ.copy()
    environment["CCACHE_DISABLE"] = "1"
    result = subprocess.run(
        command,
        cwd=REPO_ROOT,
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        pytest.fail(
            "failed to compile generated C codec:\n"
            f"{result.stdout}{result.stderr}"
        )

    library = ctypes.CDLL(str(output))
    _configure_c_library(library)
    return CCodec(library)


@pytest.fixture(scope="session")
def codec_pair(
    python_codec: PythonCodec,
    c_codec: CCodec,
) -> tuple[Codec, Codec]:
    return python_codec, c_codec


@pytest.fixture(params=("python_codec", "c_codec"), ids=("python", "c"))
def codec(request: pytest.FixtureRequest) -> Codec:
    return request.getfixturevalue(request.param)


@pytest.fixture(
    params=GOLDEN_VECTORS["reading_vectors"],
    ids=lambda vector: vector["name"],
)
def reading_vector(request: pytest.FixtureRequest) -> dict[str, Any]:
    return request.param


@pytest.fixture(
    params=GOLDEN_VECTORS["ack_vectors"],
    ids=lambda vector: vector["name"],
)
def ack_vector(request: pytest.FixtureRequest) -> dict[str, Any]:
    return request.param
