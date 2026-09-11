"""Evidence for this carrier only; never a substitute for DUT or meter results."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import uuid

APP = Path(__file__).resolve().parent
REPO = APP.parents[2]


def source_manifest():
    roots = [APP / 'main', APP / 'runner_tests', REPO / 'firmware/components/node_sensors',
             REPO / 'firmware/components/node_common', REPO / 'firmware/components/node_platform_esp']
    roots += [APP / 'managed_components' / ('espressif__' + name)
              for name in ('ds18b20', 'onewire_bus', 'bme280', 'i2c_bus')]
    paths = [p for root in roots for p in root.rglob('*')
             if p.is_file() and p.suffix in {'.c', '.h', '.py', '.txt', '.yml'}]
    paths += list(APP.glob('*.py'))
    paths += [REPO / 'firmware/components/node_core/include/node_platform_ports.h',
              REPO / 'firmware/components/protocol_v2_lora/include/protocol_v2_lora_schema_generated.h']
    paths += [APP / name for name in ('CMakeLists.txt', 'dependencies.lock',
                                      'sdkconfig.defaults', 'sdkconfig', 'partitions.csv')]
    return {str(p.relative_to(REPO)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(set(paths))}


def seal_build(build_dir):
    """Run only immediately after a successful idf.py build."""
    manifest = {'sources': source_manifest(),
                'elf_sha256': hashlib.sha256((build_dir / 'cura_sensor_carrier.elf').read_bytes()).hexdigest(),
                'config_sha256': hashlib.sha256((build_dir / 'config/sdkconfig.json').read_bytes()).hexdigest()}
    (build_dir / 'carrier-build.json').write_text(json.dumps(manifest, indent=2) + '\n')


def verify_build(build_dir, build):
    manifest = json.loads((build_dir / 'carrier-build.json').read_text())
    if (manifest.get('sources') != source_manifest()
            or manifest.get('elf_sha256') != build.elf_sha256
            or manifest.get('config_sha256') != build.config_sha256):
        raise ValueError('source/build manifest mismatch; rebuild and record the build')
    return manifest


class Evidence:
    def __init__(self, path, metadata, build_manifest):
        self.path = Path(path)
        self.data = dict(schema=1, run_id=uuid.uuid4().hex, metadata=metadata,
                         build=build_manifest, status='incomplete', events=[])
        self.path.parent.mkdir(parents=True, exist_ok=True)
        # An old successful run must never be overwritten by a new one.
        with self.path.open('x') as stream:
            json.dump(self.data, stream, indent=2)

    def add(self, kind, **values):
        self.data['events'].append(dict(kind=kind, at=datetime.now(timezone.utc).isoformat(), **values))
        self.save()

    def save(self):
        temporary = self.path.with_suffix('.tmp')
        temporary.write_text(json.dumps(self.data, indent=2) + '\n')
        temporary.replace(self.path)

    def finish(self, status):
        self.data['status'] = status
        self.save()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Record sources immediately after successful idf.py build")
    parser.add_argument("--record-build", type=Path, required=True)
    seal_build(parser.parse_args().record_build.resolve())
