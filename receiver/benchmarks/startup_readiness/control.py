#!/usr/bin/python3
"""Root-owned, narrowly scoped Pi benchmark control for the execution agent."""

import hashlib
import json
import os
from pathlib import Path
import pwd
import stat
import subprocess
import sys
import time

ROOT = Path('/var/lib/cura-startup-benchmark')
RUNNER = '/opt/cura-startup-benchmark/source/receiver/benchmarks/startup_readiness/run.py'
UNIT = 'cura-startup-benchmark.service'


def copy_evidence(archive, destination, *, source_uid, target_uid, target_gid):
    # The source directory belongs to the service, and the destination directory
    # to the SSH user. Never follow a substituted link or chown by pathname.
    descriptor = os.open(archive, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(descriptor, 'rb') as source:
        metadata = os.fstat(source.fileno())
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_uid != source_uid:
            raise ValueError('export must be a regular file owned by the service')
        checksum = hashlib.sha256()
        with destination.open('xb') as target:
            os.fchmod(target.fileno(), 0o600)
            os.fchown(target.fileno(), target_uid, target_gid)
            while block := source.read(1024 * 1024):
                checksum.update(block)
                target.write(block)
            target.flush()
            os.fsync(target.fileno())
    return checksum.hexdigest()


def main():
    if len(sys.argv) < 2:
        raise SystemExit('arm/status/analyze/verify/attest-cold/start/service/journal/collect/reboot/poweroff/disable')
    command, *arguments = sys.argv[1:]
    runner = ['/usr/sbin/runuser', '-u', 'cura-receiver', '--', '/usr/bin/python3', RUNNER, '--root', str(ROOT)]
    if command in ('arm', 'status', 'analyze', 'verify', 'attest-cold'):
        return subprocess.call(runner + [command] + arguments)
    if arguments:
        raise SystemExit('this operation takes no arguments')
    if command in ('start', 'disable'):
        return subprocess.call(['/usr/bin/systemctl', command, UNIT])
    if command == 'service':
        return subprocess.call(['/usr/bin/systemctl', 'show', UNIT, '-p', 'ActiveState', '-p', 'Result', '-p', 'ExecMainStatus', '-p', 'ExecMainStartTimestampMonotonic', '-p', 'ExecMainExitTimestampMonotonic'])
    if command == 'journal':
        return subprocess.call(['/usr/bin/journalctl', '-u', UNIT, '--no-pager', '-n', '100'])
    if command in ('reboot', 'poweroff'):
        expected = 'reboot' if command == 'reboot' else 'cold'
        request = json.loads((ROOT / 'armed.json').read_text())
        if request['mode'] != expected or request['armed_boot_id'] != Path('/proc/sys/kernel/random/boot_id').read_text().strip():
            raise SystemExit('no matching attempt armed in this boot')
        return subprocess.call(['/usr/bin/systemctl', command])
    if command == 'collect':
        name = f'cura-startup-benchmark-evidence-{time.time_ns()}.tar.gz'
        archive = ROOT / name
        subprocess.run(runner + ['export', '--output', str(archive)], check=True)
        destination = Path('/home/cura') / name
        user = pwd.getpwnam('cura')
        checksum = copy_evidence(archive, destination,
            source_uid=pwd.getpwnam('cura-receiver').pw_uid,
            target_uid=user.pw_uid, target_gid=user.pw_gid)
        print(json.dumps(dict(download=str(destination), sha256=checksum)))
        return 0
    raise SystemExit('unknown benchmark operation')


if __name__ == '__main__':
    raise SystemExit(main())
