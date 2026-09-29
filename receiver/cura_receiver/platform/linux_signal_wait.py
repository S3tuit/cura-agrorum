"""Main-thread Linux signal notification and absolute-deadline waiting."""

import os
import select
import signal

from ..elapsed_duration import checked_duration_us


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

    def wait_until_monotonic_us(self, deadline_monotonic_us):
        checked_duration_us(deadline_monotonic_us)
        if self._read_fd is None:
            raise RuntimeError('signal wait is not installed')
        while not self._stop.is_requested():
            remaining = deadline_monotonic_us - self._clock.now_monotonic_us()
            if remaining <= 0:
                return
            try:
                ready, _, _ = select.select((self._read_fd,), (), (), remaining / 1_000_000)
                if ready:
                    # One bounded read, then recheck intent and the deadline.
                    # Unrelated notifications cannot extend an absolute wait.
                    if not os.read(self._read_fd, 4096):
                        raise RuntimeError('signal notification pipe closed')
            except (InterruptedError, BlockingIOError):
                continue
