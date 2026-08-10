"""CSV logging for the balance control loop.

The loop runs at 400 Hz, so it has a 2.5 ms budget per iteration and must
never block on disk I/O. DebugLogger.log() therefore only appends a tuple
to a deque; a daemon thread drains that deque to disk every flush_interval.
deque.append and deque.popleft are each atomic under the GIL, so the hot
path needs no lock.

The queue is bounded. If the writer somehow cannot keep up, the oldest
rows are dropped rather than growing memory without limit, and close()
reports how many were lost so a gap is never silent.
"""

import os
import threading
from collections import deque


def _relax_scheduling():
    """Drop RT priority / core pinning inherited from the control thread.

    The control loop may already be SCHED_FIFO pinned to one core. A thread
    started after that inherits both and would compete with the balance
    loop, so put this one back to normal before it touches the disk.
    """
    try:
        os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
    except (AttributeError, OSError):
        pass
    try:
        os.sched_setaffinity(0, set(range(os.cpu_count() or 1)))
    except (AttributeError, OSError):
        pass


def _fmt(v):
    if isinstance(v, bool):
        return '1' if v else '0'
    if isinstance(v, float):
        return f"{v:.5g}"
    return str(v)


class DebugLogger:
    """Append-only CSV writer safe to call from a real-time control loop."""

    def __init__(self, path, fields, flush_interval=0.25,
                 max_backlog=400 * 300):
        self.path = path
        self.fields = list(fields)
        self._q = deque(maxlen=max_backlog)
        self._flush_interval = flush_interval
        self._n_in = 0
        self._n_out = 0
        self._stop = threading.Event()

        self._fh = open(path, 'w', buffering=1 << 16)
        self._fh.write(','.join(self.fields) + '\n')

        self._thread = threading.Thread(target=self._run, name='debug-log',
                                        daemon=True)
        self._thread.start()

    def log(self, *values):
        """Queue one row. Cheap enough to call every control iteration."""
        self._q.append(values)
        self._n_in += 1

    def _drain(self):
        n = 0
        write = self._fh.write
        while True:
            try:
                row = self._q.popleft()
            except IndexError:
                break
            write(','.join([_fmt(v) for v in row]))
            write('\n')
            n += 1
        self._n_out += n

    def _run(self):
        _relax_scheduling()
        while not self._stop.is_set():
            self._drain()
            self._stop.wait(self._flush_interval)
        self._drain()          # final sweep after stop was set

    def close(self):
        """Stop the writer and flush.

        Returns (rows_written, rows_dropped) so the caller can report it.
        Idempotent: a second call returns the same counts without reopening.
        """
        if self._stop.is_set():
            return self._n_out, self._n_in - self._n_out
        self._stop.set()
        self._thread.join(timeout=5.0)
        try:
            self._drain()      # in case the thread died before draining
            self._fh.flush()
            os.fsync(self._fh.fileno())
        except (OSError, ValueError):
            pass
        finally:
            try:
                self._fh.close()
            except (OSError, ValueError):
                pass

        return self._n_out, self._n_in - self._n_out
