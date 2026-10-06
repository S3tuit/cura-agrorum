"""Main-thread Linux signal notification and absolute-deadline waiting."""

import os
import select
import signal
from threading import Lock

from ..elapsed_duration import checked_duration_us


class CompletionNotification:
    """Optional worker wake hint; close and notify cannot race descriptor reuse."""

    def __init__(self):
        self._lock = Lock()
        self._read_fd, self._write_fd = os.pipe2(os.O_NONBLOCK | os.O_CLOEXEC)

    def fileno(self):
        if self._read_fd is None:
            raise RuntimeError('completion notification closed')
        return self._read_fd

    def notify(self):
        # Never called by the Python signal handler. Both operations below are
        # nonblocking, and close cannot reuse the fd while a sender holds it.
        with self._lock:
            if self._write_fd is not None:
                try:
                    os.write(self._write_fd, b'\0')
                except OSError:
                    pass  # Full pipe is already readable; outcome stays in RAM.

    def drain(self):
        try:
            os.read(self.fileno(), 4096)
        except BlockingIOError:
            pass

    def close(self):
        with self._lock:
            if self._read_fd is not None:
                read_fd, write_fd = self._read_fd, self._write_fd
                self._read_fd = self._write_fd = None
                try:
                    os.close(read_fd)
                finally:
                    os.close(write_fd)

    def __enter__(self):
        return self

    def __exit__(self, *_exception):
        self.close()


class LinuxSignalWait:
    def __init__(self, clock, stop_intent):
        self._clock = clock
        self._stop = stop_intent
        self._read_fd = self._write_fd = None
        self._previous_wakeup_fd = None
        self._previous_handlers = []

    def _request_stop(self, _number, _frame):
        self._stop.request()

    def __enter__(self):
        if self._read_fd is not None:
            raise RuntimeError('signal wait is already installed')
        self._read_fd, self._write_fd = os.pipe2(os.O_NONBLOCK | os.O_CLOEXEC)
        try:
            # Bytes are wake hints, not a signal queue. A full pipe is already
            # readable and intent remains authoritative even if a byte is lost.
            self._previous_wakeup_fd = signal.set_wakeup_fd(
                self._write_fd, warn_on_full_buffer=False)
            for number in (signal.SIGTERM, signal.SIGINT):
                # Allocate rollback bookkeeping before changing process state.
                self._previous_handlers.append((number, signal.getsignal(number)))
                signal.signal(number, self._request_stop)
        except BaseException:
            self.close()
            raise
        return self

    def close(self):
        if self._read_fd is None:
            return
        for number, previous in reversed(self._previous_handlers):
            signal.signal(number, previous)
        self._previous_handlers.clear()
        if self._previous_wakeup_fd is not None:
            signal.set_wakeup_fd(self._previous_wakeup_fd)
            self._previous_wakeup_fd = None
        # Never close a descriptor while the interpreter still targets it.
        read_fd, write_fd = self._read_fd, self._write_fd
        self._read_fd = self._write_fd = None
        try:
            os.close(read_fd)
        finally:
            os.close(write_fd)

    def __exit__(self, *_exception):
        self.close()

    def wait_until_monotonic_us(self, deadline_monotonic_us, *, completion=None, completed=None):
        checked_duration_us(deadline_monotonic_us)
        if self._read_fd is None:
            raise RuntimeError('signal wait is not installed')
        if (completion is None) != (completed is None):
            raise ValueError('completion descriptor and predicate must be supplied together')
        while not self._stop.is_requested():
            if completed is not None and completed():
                return
            remaining = deadline_monotonic_us - self._clock.now_monotonic_us()
            if remaining <= 0:
                return
            try:
                readers = ((self._read_fd,) if completion is None
                           else (self._read_fd, completion.fileno()))
                ready, _, _ = select.select(readers, (), (), remaining / 1_000_000)
                if self._read_fd in ready:
                    # One bounded read, then recheck intent and the deadline.
                    # Unrelated notifications cannot extend an absolute wait.
                    if not os.read(self._read_fd, 4096):
                        raise RuntimeError('signal notification pipe closed')
                if completion is not None and completion.fileno() in ready:
                    completion.drain()
            except (InterruptedError, BlockingIOError):
                continue
