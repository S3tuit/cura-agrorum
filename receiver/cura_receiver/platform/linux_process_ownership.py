"""One receiver launch per Linux network namespace, held until process exit."""

import errno
import socket


_ADDRESS = b'\0cura-agrorum.receiver'
_ownership_fd = None


class ReceiverAlreadyRunning(RuntimeError):
    """Another receiver launch already owns the process-lifetime guard."""


def claim_receiver_process():
    """Claim once on the main thread, before constructing any runtime owner.

    Keep the detached, close-on-exec descriptor until OS process termination.
    Neither a returned shutdown deadline nor Python object finalization may
    release it while a daemon persistence worker could still be writing.
    There is deliberately no release API or configurable lock identity.
    """
    global _ownership_fd
    if _ownership_fd is not None:
        raise ReceiverAlreadyRunning('receiver process ownership already claimed')
    guard = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM | socket.SOCK_CLOEXEC)
    try:
        guard.bind(_ADDRESS)
        _ownership_fd = guard.detach()
    except OSError as error:
        if error.errno == errno.EADDRINUSE:
            raise ReceiverAlreadyRunning('receiver already running') from error
        raise
    finally:
        guard.close()
