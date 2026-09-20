"""Compile actual C packet construction with production crypto and compare independent vectors."""
import ctypes as C
import json
import os
from pathlib import Path
import subprocess

import pytest

from evidence import REPO, digest
from prepare_rejection_inputs import prepare, BOUND_SOURCES
from rejection_vectors import matrix, C6_CASES
from cura_protocol_v2_lora.receiver_group import load_receiver_group
from provisioning_common import derive_node_key

MAIN = REPO / 'firmware/test_apps/radio/main'
PROTOCOL = REPO / 'firmware/components/protocol_v2_lora'


class Config(C.Structure):
    _fields_ = [('run', C.c_char * 33), ('node_ids', (C.c_uint8 * 8) * 3),
                ('node_keys', (C.c_uint8 * 16) * 3), ('first_message', C.c_uint32), ('first_sample', C.c_uint32)]


class Packet(C.Structure):
    _fields_ = [('frame', C.c_uint8 * 54), ('ack', C.c_uint8 * 23),
                ('frame_length', C.c_size_t), ('ack_length', C.c_size_t)]


@pytest.fixture(scope='module')
def library(tmp_path_factory):
    root = tmp_path_factory.mktemp('c6-rejection')
    lib = root / 'packets.so'
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-shared', '-fPIC',
                    '-DCURA_LORA_V2_CRYPTO_USE_OPENSSL', '-I' + str(PROTOCOL / 'include'),
                    str(MAIN / 'radio_rejection.c'), str(PROTOCOL / 'protocol_v2_lora_crypto.c'),
                    str(PROTOCOL / 'protocol_v2_lora_schema_generated.c'), str(MAIN / 'radio_command.c'), '-lcrypto', '-o', str(lib)],
                   check=True, capture_output=True, text=True)
    loaded = C.CDLL(str(lib))
    loaded.rf023_build.argtypes = [C.POINTER(Config), C.c_uint, C.POINTER(Packet)]
    loaded.rf023_build.restype = C.c_bool
    loaded.rf023_authorized.argtypes = [C.POINTER(Config), C.c_char_p, C.c_uint, C.c_uint]
    loaded.rf023_authorized.restype = C.c_bool
    loaded.rf023_case_name.argtypes = [C.c_uint]
    loaded.rf023_case_name.restype = C.c_char_p
    return loaded


def config_from(identities, first=0):
    c = Config(); c.run = b'a' * 32; c.first_message = first; c.first_sample = first
    for i, (node, key) in enumerate(identities):
        c.node_ids[i][:] = node; c.node_keys[i][:] = key
    return c


@pytest.mark.parametrize('first', [0, 0x12345678, 0xffffffff - 11])
def test_actual_c_frames_and_acks_match_independent_python(library, first):
    identities = [(bytes([i + 1]) * 8, bytes([i + 4]) * 16) for i in range(3)]
    c = config_from(identities, first)
    expected = matrix(active=identities[0], unknown=identities[1], revoked=identities[2], first_message=first, first_sample=first)
    for i, vector in enumerate(expected):
        packet = Packet()
        assert library.rf023_build(C.byref(c), i, C.byref(packet))
        assert bytes(packet.frame[:packet.frame_length]) == vector.frame
        assert bytes(packet.ack[:packet.ack_length]) == (vector.ack or b'')
        name = library.rf023_case_name(i)
        assert name.decode() == C6_CASES[i] and len(name) <= 31


@pytest.mark.parametrize('damage', ['counter', 'alias', 'bad_run', 'index'])
def test_c_invalid_input_clears_output_without_crypto_or_tx(library, damage):
    c = config_from([(bytes([i + 1]) * 8, bytes([i + 4]) * 16) for i in range(3)])
    index = 0
    if damage == 'counter': c.first_message = 0xffffffff
    elif damage == 'alias': c.node_ids[1][:] = c.node_ids[0][:]
    elif damage == 'bad_run': c.run = b'wrong'
    else: index = 12
    packet = Packet(); C.memset(C.byref(packet), 0xff, C.sizeof(packet))
    assert not library.rf023_build(C.byref(c), index, C.byref(packet))
    assert bytes(packet) == bytes(C.sizeof(packet))


def test_c_run_phase_and_case_binding(library):
    c = config_from([(bytes([i + 1]) * 8, bytes([i + 4]) * 16) for i in range(3)])
    assert library.rf023_authorized(C.byref(c), b'a' * 32, 11, 0)
    assert not library.rf023_authorized(C.byref(c), b'b' * 32, 11, 0)
    assert not library.rf023_authorized(C.byref(c), b'a' * 32, 11, 1)
    assert not library.rf023_authorized(C.byref(c), b'a' * 32, 12, 0)


def test_every_alias_uses_the_existing_real_command_parser(library):
    from host.test_command import Command
    parser = library.rf_command_parse
    parser.argtypes = [C.c_char_p, C.POINTER(Command)]
    parser.restype = C.c_bool
    for case in C6_CASES:
        command = Command()
        assert parser(('RUN ' + 'a' * 32 + ' ' + case + ' 123 0').encode(), C.byref(command))
        assert command.selection.decode() == case and command.boot == 123 and command.phase == 0


def test_fresh_private_preparation_and_revocation(tmp_path, library):
    root = tmp_path / 'private'; value = prepare(root, 'a' * 32)
    assert os.stat(root).st_mode & 0o777 == 0o700
    before = load_receiver_group(root / 'receiver-group-before.json')
    after = load_receiver_group(root / 'receiver-group-after.json')
    identities = []
    for role in ('active', 'unknown', 'revoked'):
        node = bytes.fromhex(value['nodes'][role]); identities.append((node, derive_node_key(before.group_master_key, node)))
    assert before.active_node_ids == {identities[0][0], identities[2][0]}
    assert after.active_node_ids == {identities[0][0]} and after.retired_node_ids == {identities[2][0]}
    assert identities[1][0] not in before.active_node_ids | before.retired_node_ids
    assert value['sources'] == {name: digest(REPO / name) for name in BOUND_SOURCES}
    for name in ('rf023_inputs.h', 'receiver-group-before.json', 'receiver-group-after.json'):
        assert os.stat(root / name).st_mode & 0o777 == 0o600
    public = (root / 'rf023-manifest.json').read_text()
    assert before.group_master_key.hex() not in public
    assert all(key.hex() not in public for _, key in identities)
    with pytest.raises(FileExistsError): prepare(root, 'b' * 32)
    # Compile the generated private C initializer; compare its bytes without exposing keys.
    wrapper = root / 'wrapper.c'
    wrapper.write_text('#include "rf023_inputs.h"\nconst rf023_config_t *get_config(void) {return &rf023_config;}\n')
    so = root / 'input.so'
    subprocess.run(['cc', '-shared', '-fPIC', '-Wall', '-Wextra', '-Werror', '-I' + str(MAIN),
                    str(wrapper), '-o', str(so)], check=True, capture_output=True)
    loaded = C.CDLL(str(so)); loaded.get_config.restype = C.POINTER(Config)
    config = loaded.get_config()
    for i, expected in enumerate(value['cases']):
        packet = Packet()
        assert library.rf023_build(config, i, C.byref(packet))
        assert bytes(packet.frame[:packet.frame_length]).hex() == expected['frame']
        assert (bytes(packet.ack[:packet.ack_length]).hex() if packet.ack_length else None) == expected['ack']


@pytest.mark.parametrize('damage', [None, 'header', 'source'])
def test_actual_cmake_rejects_changed_bundle(tmp_path, damage):
    root = tmp_path / 'private'; prepare(root, 'a' * 32)
    if damage == 'header':
        with (root / 'rf023_inputs.h').open('a') as stream: stream.write('\n/* altered */\n')
    if damage == 'source':
        p = root / 'rf023-manifest.json'; value = json.loads(p.read_text())
        value['sources'][BOUND_SOURCES[0]] = '0' * 64; p.write_text(json.dumps(value))
    # Execute the real component CMake checks; only IDF target registration is stubbed.
    script = tmp_path / 'check.cmake'
    script.write_text('''cmake_minimum_required(VERSION 3.22)
function(idf_component_register)
endfunction()
function(target_compile_options)
endfunction()
function(target_include_directories)
endfunction()
function(target_link_libraries)
endfunction()
function(target_compile_definitions)
endfunction()
set(COMPONENT_LIB ignored)
''' + f'set(RF023_INPUT_DIR "{root}")\ninclude("{MAIN / "CMakeLists.txt"}")\n')
    result = subprocess.run(['cmake', '-P', str(script)], capture_output=True, text=True)
    assert (result.returncode == 0) == (damage is None), result.stderr
    if damage: assert 'prepare a fresh bundle' in result.stderr
