"""Fixture provenance is run-local and binds physical A/B comparisons."""
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import carrier_evidence as evidence
import carrier_guided as guided
import carrier_runner as runner
import pytest_sensor_carrier as entry
from test_runner import build_dir, scripted_dut


@pytest.fixture
def carrier_request(build_dir, tmp_path, monkeypatch):
    options = dict(sensor_operation='acquire', sensor_fixture='nominal',
                   app_path=str(runner.APP), target='esp32s3', embedded_services='esp,idf',
                   count=1, port='/dev/null', build_dir=str(build_dir), sensor_dut='001122334455',
                   sensor_repeat_count=100, root_logdir=str(tmp_path/'run'))
    request = SimpleNamespace(config=SimpleNamespace(getoption=options.get),
        getfixturevalue=lambda name: pytest.fail('DUT must not be requested'))
    monkeypatch.setattr(entry, 'verify_build', lambda *args: {})
    return request, options


def test_entry_records_document_bytes_without_a_manual_revision(
        carrier_source_tree, carrier_request, scripted_dut, monkeypatch):
    repo, app = carrier_source_tree
    (repo/'firmware/HARDWARE.md').write_bytes(b'abc')
    (app/'FIXTURE.md').write_bytes(b'reference wiring\r\n')
    expected = {
        'firmware/HARDWARE.md': 'ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad',
        'firmware/hardware/sensors/tests/carrier/FIXTURE.md':
            hashlib.sha256(b'reference wiring\r\n').hexdigest(),
    }
    request, options = carrier_request
    dut, _, _ = scripted_dut
    request.getfixturevalue = lambda name: dut
    monkeypatch.setattr(entry, 'usb_identity', lambda port: None)
    monkeypatch.setattr(entry, 'attach_usb_console', lambda *args, **kwargs: None)
    properties = {}
    entry.test_sensor_carrier(request, properties.__setitem__)
    saved = json.loads((Path(options['root_logdir'])/'carrier-evidence.json').read_text())
    assert saved['metadata']['fixture_sha256'] == expected
    assert 'carrier_revision' not in saved['metadata']
    assert json.loads(properties['fixture_sha256']) == expected
    assert saved['status'] == 'software_passed_operator_acceptance_pending'
    runner.validate_selection('discover', None, True)


@pytest.mark.parametrize('document', ['firmware/HARDWARE.md',
    'firmware/hardware/sensors/tests/carrier/FIXTURE.md'])
def test_document_edits_change_provenance_without_invalidating_build(
        carrier_source_tree, tmp_path, monkeypatch, document):
    repo, _ = carrier_source_tree
    original = evidence.fixture_manifest()
    sources = evidence.source_manifest()
    manifest = dict(sources=sources, bosch_bme280={}, elf_sha256='elf', config_sha256='config')
    (tmp_path/'carrier-build.json').write_text(json.dumps(manifest))
    monkeypatch.setattr(evidence, 'bosch_manifest', lambda build: {})
    (repo/document).write_bytes(b'changed fixture definition\n')
    updated = evidence.fixture_manifest()
    assert updated[document] != original[document]
    assert {key for key in updated if updated[key] != original[key]} == {document}
    assert evidence.verify_build(tmp_path, SimpleNamespace(elf_sha256='elf', config_sha256='config')) == manifest


@pytest.mark.parametrize('document', ['firmware/HARDWARE.md',
    'firmware/hardware/sensors/tests/carrier/FIXTURE.md'])
def test_missing_document_fails_before_requesting_dut(carrier_source_tree, carrier_request, document):
    repo, _ = carrier_source_tree
    (repo/document).unlink()
    request, _ = carrier_request
    with pytest.raises(FileNotFoundError):
        entry.test_sensor_carrier(request, lambda *args: None)


@pytest.mark.parametrize('change', ['firmware/HARDWARE.md',
    'firmware/hardware/sensors/tests/carrier/FIXTURE.md', 'legacy_receipt'])
def test_changed_or_unrecorded_fixture_cannot_supply_position_a(
        carrier_source_tree, carrier_request, build_dir, tmp_path, change):
    repo, _ = carrier_source_tree
    request, options = carrier_request
    options.update(sensor_operation='adc-reference', sensor_fixture='adc_reference',
                   sensor_guided=True, sensor_position='B')
    build = runner.load_build(build_dir, 'adc-reference')
    metadata = dict(operation='adc-reference', fixture='adc_reference',
                    fixture_sha256=evidence.fixture_manifest(), expected_dut='001122334455',
                    elf_sha256=build.elf_sha256, config_sha256=build.config_sha256,
                    rom0=build.boot_values['rom0'], rom1=build.boot_values['rom1'])
    if change == 'legacy_receipt':
        metadata.pop('fixture_sha256')
        metadata['carrier_revision'] = 'old assembled carrier label'
    prior = evidence.Evidence(tmp_path/'a.json', metadata, {})
    prior.add('position_result', position='A')
    prior.add('software_completed')
    prior.finish('position_A_complete_sequence_incomplete')
    options['sensor_prior_evidence'] = str(prior.path)
    if change != 'legacy_receipt':
        (repo/change).write_bytes(b'changed reference fixture\n')
    original_receipt = prior.path.read_bytes()
    with pytest.raises(ValueError, match='fixture_sha256'):
        entry.test_sensor_carrier(request, lambda *args: None)
    assert prior.path.read_bytes() == original_receipt
    if change == 'legacy_receipt':
        with pytest.raises(ValueError, match='fixture_sha256'):
            guided.prior_position(prior.path, metadata, 'B')
