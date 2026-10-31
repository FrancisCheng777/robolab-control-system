"""Hardware-free regressions for independently scheduled control and recording."""
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from pi05_demo.cameras import SyntheticPair
from pi05_demo.session import CollectionSession


class StubBridge:
    def __init__(self):
        self.seq = 0
        self.commands = []
        self.holds = 0
    def state(self):
        import time
        return dict(session_control_version=1, q=[0, -.6, 0, -2.5, 0, 1.8, 0],
                    dq=[0]*7, gripper_width=.04, hold_ready=True,
                    state_sequence=1, applied_sequence=self.seq,
                    state_bridge_receive_s=time.monotonic(), gripper_bridge_read_s=time.monotonic(),
                    calibration={'teleop_controller': 'robolab_torque'})
    def claim(self): pass
    def hold(self, state): self.seq += 1; self.holds += 1
    def close(self): pass
    def command(self, state, target, width, **kwargs):
        import time
        self.seq += 1
        self.commands.append(target.copy())
        t = time.monotonic()
        return dict(ok=True, command_send_s=t, command_enqueue_s=t, command_ack_s=t)


class Writer:
    robolab = True
    count = 0
    def append(self, **values): self.count += 1
    def finish(self, *args): return 'test.h5'
    def close_incomplete(self): pass


class Leader:
    def __init__(self): self.calls = 0
    def read(self):
        self.calls += 1
        return np.array([0, -.6, 0, -2.5, 0, 1.8, 0]) + .001*self.calls, .04


class LowLatencyTests(unittest.TestCase):
    def session(self, **options):
        bridge = StubBridge()
        config = dict(teleop_controller='robolab_torque', arm_home=bridge.state()['q'],
                      task='test', control_hz=15, teleop_hz=100, async_recording=True, **options)
        session = CollectionSession(bridge, SyntheticPair(), Leader(), config, '.', 'test',
                                    mock=True, announce=lambda *a, **k: None)
        session.phase = 'recording'
        session.writer = Writer()
        return session

    def test_every_tick_updates_all_joints_but_records_at_15hz(self):
        session = self.session()
        with patch('time.monotonic', return_value=100.) as clock:
            for i in range(10):
                clock.return_value = 100. + i*.01
                session.tick()
        self.assertEqual(len(session.bridge.commands), 10)
        self.assertEqual(session.writer.count, 2)
        self.assertEqual(session.leader.calls, 10)
        self.assertTrue(np.all(session.bridge.commands[-1] > session.bridge.commands[0]))

    def test_pause_preempts_next_leader_read(self):
        session = self.session()
        session.tick(['toggle'])
        self.assertEqual(session.phase, 'settling')
        self.assertEqual(session.leader.calls, 0)
        self.assertEqual(session.bridge.holds, 1)

    def test_stale_gello_cache_rejected_without_reading_hardware(self):
        from types import SimpleNamespace
        from pi05_demo.collect import Gello
        g = Gello.__new__(Gello)
        g.max_sample_age_s = .05
        g.agent = SimpleNamespace(_robot=SimpleNamespace(_driver=SimpleNamespace(last_successful_read_s=1.)))
        with patch('time.monotonic', return_value=2.):
            with self.assertRaisesRegex(RuntimeError, 'stale GELLO'):
                g.read()

    def test_no_recording_burst_after_missed_deadline(self):
        session = self.session()
        with patch('time.monotonic', return_value=100.) as clock:
            session.tick()
            clock.return_value = 100.5
            session.tick()
            session.tick()
        self.assertEqual(session.writer.count, 2)
        self.assertEqual(len(session.bridge.commands), 3)

    def test_async_recording_required_for_fast_mode(self):
        bridge = StubBridge()
        with self.assertRaisesRegex(ValueError, 'async_recording'):
            CollectionSession(bridge, SyntheticPair(), Leader(),
                dict(teleop_controller='robolab_torque', arm_home=bridge.state()['q'],
                     task='test', control_hz=15, teleop_hz=100), '.', 'test', mock=True)


if __name__ == '__main__': unittest.main()
