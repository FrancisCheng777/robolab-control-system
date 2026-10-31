"""Inspect real launchers without executing device setup or motion."""
import json
from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]


class ReleaseEntrypointTests(unittest.TestCase):
    def inspect(self, script, *args):
        result = subprocess.run(['bash', str(ROOT/'scripts'/script), *args],
                                cwd=ROOT, capture_output=True, text=True, check=True)
        decoder = json.JSONDecoder()
        config, _ = decoder.raw_decode(result.stdout[result.stdout.index('{'):])
        return config

    def test_continuous_entry_uses_compatible_recording_profile(self):
        c = self.inspect('collect_session.sh', '--inspect')
        self.assertEqual(c['latency_profile'], 'responsive_recording')
        self.assertEqual(c['teleop_hz'], 100)
        self.assertEqual(c['control_hz'], 15)
        self.assertLess(max(c['expected_max_velocity']) / c['control_hz'], .03)
        self.assertEqual(c['expected_collision_torque'][1:3], [30,30])

    def test_recording_request_cannot_use_full_wrist_speed(self):
        c = self.inspect('try_latency.sh', 'responsive', '--inspect')
        self.assertEqual(c['latency_profile'], 'responsive_recording')
        self.assertLessEqual(max(c['expected_max_velocity']), .4)

    def test_teleop_only_preserves_accepted_tracking_parameters(self):
        c = self.inspect('try_latency.sh', 'responsive', '--inspect', '--teleop-only')
        self.assertEqual(c['latency_profile'], 'responsive')
        self.assertEqual(c['expected_max_velocity'], [.3]*4+[.5]*3)
        self.assertEqual(c['expected_teleop_velocity_feedforward'], 1.)
        self.assertEqual(c['expected_teleop_joint_kp'][6], 90)
