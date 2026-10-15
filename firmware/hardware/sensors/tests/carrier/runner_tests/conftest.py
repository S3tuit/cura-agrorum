"""Isolated source inputs for collector and build-seal host regressions."""
import pytest

import carrier_evidence as evidence


@pytest.fixture
def carrier_source_tree(tmp_path, monkeypatch):
    repo = tmp_path / 'checkout.notes'
    app = repo / 'firmware/hardware/sensors/tests/carrier'
    paths = [app / name for name in ('CMakeLists.txt', 'dependencies.lock',
             'sdkconfig.defaults', 'sdkconfig', 'partitions.csv', 'main/carrier_core.c',
             'managed_components/joltwallet__littlefs/src/littlefs/lfs.c')]
    paths += [repo / name for name in (
        'firmware/tests/support/persistence_test_support.c',
        'firmware/tests/support/persistence_test_support.h',
        'firmware/components/node_core/node_core.c',
        'firmware/components/node_persistence/node_persistence_backend_esp.c',
        'firmware/components/protocol_v2_lora/protocol_v2_lora_crypto.c',
        'firmware/components/sx1262_radio/CMakeLists.txt',
        'firmware/components/sx1262_radio/sx1262_radio_timing.c',
        'firmware/components/sx1262_radio/include/sx1262_radio.h',
        'protocol/protocol-v2-lora/schemas/protocol_v2_lora.json')]
    # These bytes exercise real file selection/hashing, not ESP-IDF compilation.
    for path in paths:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('fixture input')
    monkeypatch.setattr(evidence, 'REPO', repo)
    monkeypatch.setattr(evidence, 'APP', app)
    return repo, app
