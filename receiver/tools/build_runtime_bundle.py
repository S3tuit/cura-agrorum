#!/usr/bin/env python3
"""Build a source-identified production tree without tests, keys or databases."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


def build(output):
    root = Path(__file__).resolve().parents[2]
    output.mkdir(parents=True, exist_ok=False)
    for relative in ('receiver/cura_receiver', 'receiver/db', 'receiver/deploy',
                     'receiver/native', 'protocol/protocol-v2-lora/python/cura_protocol_v2_lora'):
        shutil.copytree(root / relative, output / relative,
            ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    for relative in ('receiver/requirements-runtime.txt', 'receiver/requirements-radio.txt',
                     'receiver/tools/check_chrony.py', 'receiver/hardware/ds3231/chrony-runtime.conf'):
        destination = output / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(root / relative, destination)
    files = {str(path.relative_to(output)): hashlib.sha256(path.read_bytes()).hexdigest()
             for path in sorted(output.rglob('*')) if path.is_file()}
    head = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
    (output / 'SOURCE_MANIFEST.json').write_text(json.dumps(
        {'baseline_head': head, 'files_sha256': files}, sort_keys=True, indent=2) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    build(parser.parse_args().output)
