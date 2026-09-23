"""Portable driver provenance and rejection before requesting a hardware fixture."""
import hashlib
import json
from pathlib import Path
import subprocess
import pytest
import carrier_evidence as evidence


@pytest.fixture
def source_build(tmp_path, monkeypatch):
    source = tmp_path / 'fork'
    source.mkdir()
    names = ('bme280.c', 'bme280.h', 'bme280_defs.h', 'LICENSE')
    for name in names:
        (source / name).write_text('test dependency boundary: ' + name)
    def git(*args):
        return subprocess.run(['git', '-C', str(source), *args], check=True,
                              capture_output=True, text=True).stdout.strip()
    git('init', '-q')
    git('add', '.')
    git('-c', 'user.name=Seal Test', '-c', 'user.email=seal@example.invalid', 'commit', '-qm', 'fixture')
    commit = git('rev-parse', 'HEAD')
    pin = tmp_path / 'pin.cmake'
    values = dict(COMMIT=commit, REPOSITORY='https://example.invalid/test-fork.git')
    values.update({'SHA256_' + name.replace('.', '_'): hashlib.sha256((source/name).read_bytes()).hexdigest()
                   for name in names})
    pin.write_text(''.join(f'set(CURA_BME280_{key} "{value}")\n' for key, value in values.items()))
    monkeypatch.setattr(evidence, 'BOSCH_PIN', pin)
    build = tmp_path / 'build'
    build.mkdir()
    (build/'bosch-bme280-source.txt').write_text(f'{source}\n{commit}\nexplicit_clean_checkout\n')
    (build/'compile_commands.json').write_text(json.dumps([
        dict(file=str(source/'bme280.c'), command='cc -DBME280_DOUBLE_ENABLE -c bme280.c')]))
    return source, build, pin, git


def test_portable_pin_matches_compiled_source(source_build):
    source, build, _, _ = source_build
    manifest = evidence.bosch_manifest(build)
    assert str(source) not in json.dumps(manifest)
    assert manifest['selection'] == 'explicit_clean_checkout'
    assert set(manifest['sources']) == {'bme280.c', 'bme280.h', 'bme280_defs.h', 'LICENSE'}


@pytest.mark.parametrize('mutation', ['dirty', 'wrong_head', 'wrong_pin', 'wrong_hash',
                                     'missing_record', 'wrong_selection', 'wrong_source',
                                     'missing_double', 'duplicate_source'])
def test_driver_mismatch_cannot_be_sealed(source_build, mutation):
    source, build, pin, git = source_build
    record = build/'bosch-bme280-source.txt'
    commands = build/'compile_commands.json'
    if mutation == 'dirty':
        (source/'bme280.c').write_text('modified')
    elif mutation == 'wrong_head':
        git('-c', 'user.name=Seal Test', '-c', 'user.email=seal@example.invalid',
            'commit', '--allow-empty', '-qm', 'changed identity')
    elif mutation == 'wrong_pin':
        record.write_text(record.read_text().replace(git('rev-parse', 'HEAD'), '0'*40))
    elif mutation == 'wrong_hash':
        pin.write_text(pin.read_text().replace(hashlib.sha256((source/'bme280.c').read_bytes()).hexdigest(), '0'*64))
    elif mutation == 'missing_record':
        record.unlink()
    elif mutation == 'wrong_selection':
        record.write_text(record.read_text().replace('explicit_clean_checkout', 'unknown'))
    else:
        entries = json.loads(commands.read_text())
        if mutation == 'wrong_source':
            entries[0]['file'] = str(source/'copy.c')
        elif mutation == 'missing_double':
            entries[0]['command'] = 'cc -c bme280.c'
        else:
            entries.append(entries[0].copy())
        commands.write_text(json.dumps(entries))
    with pytest.raises((ValueError, FileNotFoundError)):
        evidence.bosch_manifest(build)
