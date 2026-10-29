"""One bounded best-effort write to systemd's stdout/stderr stream socket."""

import json
import os
import socket
import stat

MAX_SUMMARY_BYTES = 4096


def emit_service_evidence(record, *, fd=1):
    """Never changes shared descriptor flags or leaves buffered exit-time output.

    Arbitrary files/terminals/blocking pipes are deliberately unsupported here:
    only a socket send supports a per-call nonblocking policy without changing
    another owner's descriptor mode. Benchmark JSON files have their own owner.
    """
    try:
        data = (json.dumps(record, separators=(',', ':'), sort_keys=True) + '\n').encode('utf-8')
        if len(data) > MAX_SUMMARY_BYTES:
            return 'TOO_LARGE'
        if not stat.S_ISSOCK(os.fstat(fd).st_mode):
            return 'UNSUPPORTED_SINK'
        duplicate = os.dup(fd)
        try:
            stream = socket.socket(fileno=duplicate)
        except BaseException:
            os.close(duplicate)
            raise
        with stream:
            written = stream.send(data, socket.MSG_DONTWAIT | socket.MSG_NOSIGNAL)
        return 'SENT' if written == len(data) else 'PARTIAL'
    except BlockingIOError:
        return 'BACKPRESSURE'
    except (OSError, TypeError, ValueError, OverflowError):
        return 'UNAVAILABLE'
