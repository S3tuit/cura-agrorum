"""Bounded child execution shared by the two fixed time adapters."""

from dataclasses import dataclass
import os
import selectors
import subprocess


@dataclass(frozen=True, slots=True)
class _ChildResult:
    returncode: int | None = None
    stdout: bytes = b""
    stderr: bytes = b""
    started: bool = False
    timed_out: bool = False
    overflow: bool = False
    os_errno: int | None = None


def _run_child(argv, clock, deadline, *, reap_timeout_s=1):
    """Internal fixed-adapter primitive; bound output, deadline and child cleanup."""
    if clock.now_monotonic_us() >= deadline:
        return _ChildResult(timed_out=True)
    try:
        child = subprocess.Popen(
            argv,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            close_fds=True,
            env={"LC_ALL": "C", "LANG": "C", "PATH": "/usr/bin:/bin"},
        )
    except OSError as error:
        return _ChildResult(os_errno=error.errno)
    buffers = [bytearray(), bytearray()]
    timeout = overflow = False
    try:
        with selectors.DefaultSelector() as selector:
            for index, stream in enumerate((child.stdout, child.stderr)):
                os.set_blocking(stream.fileno(), False)
                selector.register(stream, selectors.EVENT_READ, index)
            while selector.get_map():
                remaining = deadline - clock.now_monotonic_us()
                if remaining <= 0:
                    timeout = True
                    break
                for key, _ in selector.select(remaining / 1_000_000):
                    part = os.read(key.fd, 4097 - len(buffers[key.data]))
                    if not part:
                        selector.unregister(key.fileobj)
                    else:
                        buffers[key.data].extend(part)
                        if len(buffers[key.data]) > 4096:
                            overflow = True
                            break
                if overflow:
                    break
            if not timeout and not overflow:
                try:
                    child.wait(
                        timeout=max(0, deadline - clock.now_monotonic_us()) / 1_000_000
                    )
                except subprocess.TimeoutExpired:
                    timeout = True
            timeout |= clock.now_monotonic_us() >= deadline
    finally:
        if child.poll() is None:
            child.kill()
        # A killed RTC helper may still be returning from its bounded kernel
        # operation. The adapter supplies that deployment bound for cleanup.
        child.wait(timeout=reap_timeout_s)
        child.stdout.close()
        child.stderr.close()
    return _ChildResult(
        child.returncode,
        bytes(buffers[0][:4096]),
        bytes(buffers[1][:4096]),
        True,
        timeout,
        overflow,
    )
