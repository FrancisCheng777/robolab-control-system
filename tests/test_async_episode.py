import tempfile
import threading
import unittest

import h5py
import numpy as np

from pi05_demo.async_episode import AsyncEpisodeWriter
from pi05_demo.episode import EpisodeWriter
import test_low_latency


class AsyncEpisodeTests(unittest.TestCase):
    def test_slow_storage_does_not_block_control_and_keeps_frame_snapshot(self):
        entered, release = threading.Event(), threading.Event()
        saved = []
        class SlowWriter:
            def __init__(self, *a, **k): pass
            def append(self, **values):
                entered.set()
                if not release.wait(2): raise TimeoutError('test storage gate')
                saved.append(values['q'].copy())
            def finish(self, *a): return 'saved.h5'
            def close_incomplete(self): pass
        w = AsyncEpisodeWriter(config={'teleop_controller':'robolab_torque'},
                               writer_factory=SlowWriter, capacity=1)
        try:
            q = np.zeros(7)
            w.append(q=q)
            self.assertTrue(entered.wait(1))
            q[:] = 1
            w.append(q=q)
            q[:] = 2
            with self.assertRaisesRegex(RuntimeError, 'queue full'):
                w.append(q=q)
            w.begin_finish('aborted')
            self.assertIsNone(w.poll_finished())
            release.set()
            self.assertEqual(w.finish('aborted'), 'saved.h5')
            np.testing.assert_array_equal(saved, [np.zeros(7), np.ones(7)])
        finally:
            release.set()
            w.close_incomplete()

    def test_background_failure_is_propagated(self):
        class BrokenWriter:
            def __init__(self, *a, **k): raise OSError('disk full')
        w = AsyncEpisodeWriter(config={}, writer_factory=BrokenWriter)
        self.assertTrue(w._done.wait(1))
        with self.assertRaisesRegex(RuntimeError, 'disk full'): w.check()

    def test_real_hdf5_and_hold_heartbeats_while_finalizing(self):
        session = test_low_latency.LowLatencyTests().session()
        with tempfile.TemporaryDirectory() as root:
            entered, release = threading.Event(), threading.Event()
            class DelayedFinish(EpisodeWriter):
                def finish(self, *args):
                    entered.set()
                    if not release.wait(2): raise TimeoutError('test finish gate')
                    return super().finish(*args)
            w = AsyncEpisodeWriter(root, mock=True, task='test', session='async',
                config=session.config, writer_factory=DelayedFinish)
            session.writer = w
            try:
                session.tick()
                session._finish('success')
                self.assertTrue(entered.wait(1))
                before = session.bridge.holds
                for _ in range(10): session.tick()
                self.assertEqual(session.phase, 'saving')
                self.assertEqual(session.bridge.holds, before+10)
                release.set()
                path = w.finish('success')
                session.tick()
                self.assertEqual(session.phase, 'standby')
                self.assertEqual(session.completed_episodes, 1)
                with h5py.File(path) as f:
                    self.assertTrue(f.attrs['mock'])
                    self.assertTrue(f.attrs['complete'])
                    self.assertEqual(f.attrs['frames'], 1)
                    self.assertEqual(f.attrs['outcome'], 'success')
            finally:
                release.set()
                session.close()


if __name__ == '__main__': unittest.main()
