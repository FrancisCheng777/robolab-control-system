"""No-camera trial must not instantiate cameras or create training episodes."""
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from test_low_latency import StubBridge, Leader
from pi05_demo.session import CollectionSession


class TeleopOnlyTests(unittest.TestCase):
    def test_control_pause_and_close_without_camera_or_writer(self):
        events = []
        bridge = StubBridge()
        config = dict(teleop_controller='robolab_torque', arm_home=bridge.state()['q'],
                      task='test', control_hz=15, teleop_hz=100, async_recording=True,
                      recording_enabled=False)
        with tempfile.TemporaryDirectory() as output:
            session = CollectionSession(bridge, None, Leader(), config, output, 'teleop',
                mock=True, announce=lambda event, **data: events.append((event, data)))
            try:
                session.phase = 'standby'
                with patch('pi05_demo.async_episode.AsyncEpisodeWriter', side_effect=AssertionError('must not create a writer')):
                    session.tick(['toggle'])
                    for _ in range(10): session.tick()
                    self.assertEqual(len(bridge.commands), 10)
                    session.tick(['toggle'])
                    session.tick()
                self.assertEqual(session.phase, 'standby')
                self.assertEqual(session.completed_episodes, 1)
                self.assertTrue(any(e == 'teleop_finished' for e, _ in events))
                self.assertFalse(any(e == 'saved' for e, _ in events))
                self.assertEqual(list(Path(output).iterdir()), [])
            finally:
                session.close()

    def test_recording_still_requires_cameras(self):
        bridge = StubBridge()
        config = dict(teleop_controller='robolab_torque', arm_home=bridge.state()['q'],
                      task='test', control_hz=15, recording_enabled=False)
        with self.assertRaisesRegex(ValueError, 'teleop_hz'):
            CollectionSession(bridge, None, Leader(), config, '.', 'test', mock=True)


if __name__ == '__main__': unittest.main()
