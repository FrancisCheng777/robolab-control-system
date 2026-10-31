import json
from pathlib import Path
import time
import unittest
from unittest.mock import Mock

import numpy as np

from test_collection_session import mock_bridge
from test_low_latency import StubBridge, Leader
from pi05_demo.session import CollectionSession

ROOT = Path(__file__).resolve().parents[1]


class FollowupProfileTests(unittest.TestCase):
    def test_fast_wrist_recording_rejected_before_claim(self):
        bridge = StubBridge()
        state = bridge.state()
        state['calibration'].update(max_velocity=[.3]*4+[.5]*3)
        bridge.state = Mock(return_value=state)
        bridge.claim = Mock()
        config = dict(teleop_controller='robolab_torque', arm_home=state['q'],
                      task='test', control_hz=15, teleop_hz=100, async_recording=True)
        with self.assertRaisesRegex(ValueError, 'recording.*action contract'):
            CollectionSession(bridge, None, Leader(), config, '.', 'test', mock=True)
        bridge.claim.assert_not_called()

    def test_new_profiles_preserve_speed_and_collision_limits(self):
        source = json.loads((ROOT/'configs/latency/feedforward/bridge.trial.json').read_text())
        for name, lead in [('feedforward075', 0.), ('feedforward_guarded', .08)]:
            cfg = json.loads((ROOT/f'configs/latency/{name}/bridge.trial.json').read_text())
            for key in ('max_velocity', 'collision_torque', 'collision_force', 'home_max_velocity'):
                self.assertEqual(cfg[key], source[key])
            self.assertEqual(cfg['teleop_velocity_feedforward'], .75)
            self.assertEqual(cfg['teleop_reference_lead_limit_rad'], lead)

    def test_wrong_feedforward_profile_rejected_before_claim(self):
        bridge = StubBridge()
        bridge.claim = Mock()
        config = dict(teleop_controller='robolab_torque', arm_home=bridge.state()['q'],
                      task='test', control_hz=15, teleop_hz=100, async_recording=True,
                      recording_enabled=False, expected_teleop_velocity_feedforward=.75)
        with self.assertRaisesRegex(ValueError, 'profile mismatch'):
            CollectionSession(bridge, None, Leader(), config, '.', 'test', mock=True)
        bridge.claim.assert_not_called()

    def test_wrong_responsive_speed_or_stiffness_rejected_before_claim(self):
        for expected in ('expected_max_velocity', 'expected_teleop_joint_kp'):
            bridge = StubBridge()
            bridge.claim = Mock()
            config = dict(teleop_controller='robolab_torque', arm_home=bridge.state()['q'],
                          task='test', control_hz=15, teleop_hz=100, async_recording=True,
                          recording_enabled=False, **{expected: [1]*7})
            with self.assertRaisesRegex(ValueError, 'profile mismatch'):
                CollectionSession(bridge, None, Leader(), config, '.', 'test', mock=True)
            bridge.claim.assert_not_called()

    def test_mock_follower_moves_past_initial_lead_band_and_holds(self):
        with mock_bridge(robolab=True, config_overrides={
                'teleop_reference_lead_limit_rad': .08,
                'teleop_velocity_feedforward': .75}) as bridge:
            bridge.claim()
            state = bridge.state()
            initial = np.array(state['q'])
            target = initial + .25
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline:
                state = bridge.state()
                bridge.command(state, target, state['gripper_width'])
                if np.min(np.array(state['q'])-initial) > .09:
                    break
                time.sleep(.01)
            self.assertGreater(float(np.min(np.array(state['q'])-initial)), .09)
            bridge.hold(bridge.state())
            deadline = time.monotonic() + 1
            while not bridge.state()['hold_ready']:
                if time.monotonic() > deadline: self.fail('guarded mock did not hold')
                time.sleep(.005)


if __name__ == '__main__': unittest.main()
