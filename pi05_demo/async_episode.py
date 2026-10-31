"""Bounded recording worker; control never waits for HDF5 or filesystem flushes."""
from copy import deepcopy
from queue import Queue, Empty, Full
from threading import Event, Thread

from .episode import EpisodeWriter


class AsyncEpisodeWriter:
    def __init__(self, *args, capacity=8, writer_factory=EpisodeWriter, **kwargs):
        if capacity < 1:
            raise ValueError('recording queue capacity must be positive')
        self.robolab = kwargs['config'].get('teleop_controller') == 'robolab_torque'
        self.count = 0
        self.high_watermark = 0
        self._queue = Queue(maxsize=capacity)
        self._wake, self._done, self._cancel = Event(), Event(), Event()
        self._finish = self._error = self._path = None
        self._thread = Thread(target=self._run, args=(writer_factory, args, kwargs),
                              name='episode-storage', daemon=True)
        self._thread.start()

    def _run(self, factory, args, kwargs):
        writer = None
        try:
            writer = factory(*args, **kwargs)
            while not self._cancel.is_set():
                try:
                    values = self._queue.get_nowait()
                except Empty:
                    if self._finish is not None:
                        self._path = writer.finish(*self._finish)
                        return
                    self._wake.wait(.01)
                    self._wake.clear()
                else:
                    writer.append(**values)
        except BaseException as exc:
            self._error = exc
        finally:
            try:
                if writer is not None:
                    writer.close_incomplete()
            except BaseException as exc:
                self._error = self._error or exc
            self._done.set()

    def check(self):
        if self._error is not None:
            raise RuntimeError(f'episode storage failed: {self._error}') from self._error
        if self._cancel.is_set():
            raise RuntimeError('episode storage cancelled')

    def append(self, **values):
        self.check()
        if self._finish is not None:
            raise RuntimeError('episode already finishing')
        try:
            # The producer may reuse leader arrays; the worker owns its snapshot.
            self._queue.put_nowait(deepcopy(values))
        except Full as exc:
            raise RuntimeError('episode storage queue full; stopping instead of dropping frames') from exc
        self.count += 1
        self.high_watermark = max(self.high_watermark, self._queue.qsize())
        self._wake.set()

    def begin_finish(self, outcome, reason=''):
        self.check()
        if outcome not in ('success', 'failure', 'aborted'):
            raise ValueError('explicit episode outcome required')
        if self._finish is None:
            self._finish = (outcome, reason)
            self._wake.set()

    def poll_finished(self):
        self.check()
        return self._path if self._done.is_set() else None

    def finish(self, outcome, reason=''):
        # Only shutdown uses the blocking variant, after bridge.close(). During
        # normal saving, CollectionSession polls and continues hold heartbeats.
        self.begin_finish(outcome, reason)
        if not self._done.wait(5.):
            self.close_incomplete()
            raise TimeoutError('episode storage did not finish within 5 seconds')
        self.check()
        return self._path

    def close_incomplete(self):
        if not self._done.is_set():
            self._cancel.set()
            self._wake.set()
