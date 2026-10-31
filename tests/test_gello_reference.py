"""Exercise the real GELLO read path without opening a serial port."""
import copy
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

import numpy as np

from pi05_demo.collect import Gello
from pi05_demo.contract import LOWER, UPPER, MAX_WIDTH, decode, encode
from pi05_demo.teleop_reference import j7_offset_from_config

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "vendor"))
LEADER_HOME = np.array([0., -np.pi/5, 0., -4*np.pi/5, 0., 3*np.pi/5, np.pi/4])


class GelloReferenceTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((ROOT / "configs/collection.json").read_text())
        self.shift = j7_offset_from_config(self.config)
        self.robot_home = LEADER_HOME.copy()
        self.robot_home[6] -= np.pi / 4

    def reader(self, *samples):
        agent = Mock()
        agent.act.side_effect = samples
        with patch("gello.agents.gello_agent.GelloAgent", return_value=agent):
            return Gello(self.config["gello_port"], j7_offset_rad=self.shift)

    def test_config_pairs_droid_home_with_fixed_leader_mapping(self):
        np.testing.assert_array_equal(self.config["gello_reference_joints_rad"], LEADER_HOME)
        np.testing.assert_array_equal(self.config["arm_home"], self.robot_home)
        self.assertEqual(self.shift, -np.pi / 4)

    def test_default_pose_points_the_standard_robotiq_tool_down(self):
        # Panda modified-DH rotation chain. The original Franka Hand has a
        # -pi/4 flange yaw; the standard Robotiq tool frame has identity rotation.
        # Compare full rotations, not just the numeric J7 shift.
        def rx(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])

        def rz(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])

        def tool_rotation(joints, flange_yaw):
            rotation = np.eye(3)
            twists = [0., -np.pi/2, np.pi/2, np.pi/2, -np.pi/2, np.pi/2, np.pi/2]
            for joint, twist in zip(joints, twists):
                rotation = rotation @ rx(twist) @ rz(joint)
            return rotation @ rz(flange_yaw)

        old_tool = tool_rotation(LEADER_HOME, -np.pi / 4)
        new_tool = tool_rotation(self.config["arm_home"], 0.)
        np.testing.assert_allclose(new_tool, old_tool, atol=1e-12)
        np.testing.assert_allclose(new_tool, np.diag([1., -1., -1.]), atol=1e-12)

    def test_original_leader_pose_maps_to_new_robot_home_without_mutating_input(self):
        raw = np.r_[LEADER_HOME, .3]
        saved = raw.copy()
        target, width = self.reader(raw).read()
        np.testing.assert_array_equal(target, self.robot_home)
        np.testing.assert_array_equal(raw, saved)
        self.assertAlmostEqual(width, .7 * MAX_WIDTH)

    def test_joint_seven_retains_direction_and_one_to_one_motion(self):
        motions = [0., .02, .01, -.02]
        samples = [np.r_[LEADER_HOME[:6], LEADER_HOME[6] + delta, .5]
                   for delta in motions]
        reader = self.reader(*samples)
        for delta in motions:
            target, _ = reader.read()
            np.testing.assert_array_equal(target[:6], LEADER_HOME[:6])
            self.assertAlmostEqual(target[6], self.robot_home[6] + delta, places=12)

    def test_other_joint_motion_and_gripper_endpoints_are_unchanged(self):
        for closure in (0., .5, 1.):
            raw = np.r_[LEADER_HOME + [.01, -.01, .02, .01, -.02, .02, 0.], closure]
            target, width = self.reader(raw).read()
            np.testing.assert_array_equal(target[:6], raw[:6])
            self.assertAlmostEqual(width, (1 - closure) * MAX_WIDTH)

    def test_shift_is_applied_before_robot_limit_check(self):
        # This old GELLO value exceeds the robot limit but its mapped target is legal.
        raw = np.r_[LEADER_HOME[:6], UPPER[6] - .01 - self.shift, .5]
        target, _ = self.reader(raw).read()
        self.assertAlmostEqual(target[6], UPPER[6] - .01)
        # Conversely, a previously legal GELLO value maps below the robot limit.
        raw[6] = LOWER[6] - .01 - self.shift
        with self.assertRaisesRegex(ValueError, "outside joint limits"):
            self.reader(raw).read()

    def test_alignment_can_show_out_of_range_angles_but_control_still_rejects_them(self):
        raw = np.r_[LEADER_HOME, .5]
        raw[0] = LOWER[0] - .1
        target, width = self.reader(raw).read_for_alignment()
        self.assertAlmostEqual(target[0], raw[0])
        self.assertAlmostEqual(target[6], 0.)
        self.assertAlmostEqual(width, .5 * MAX_WIDTH)
        with self.assertRaisesRegex(ValueError, "outside joint limits"):
            self.reader(raw).read()

    def test_robolab_limits_commands_but_preserves_unclipped_leader_reading(self):
        raw=np.r_[LEADER_HOME,.5];raw[0]=LOWER[0]-.1
        reader=self.reader(raw);reader.robolab=True
        command,_=reader.read()
        self.assertAlmostEqual(command[0],LOWER[0])
        self.assertAlmostEqual(reader.last_unclipped_target[0],raw[0])

    def test_encoder_turns_are_unwrapped_after_reference_shift(self):
        first = np.r_[LEADER_HOME[:6], LEADER_HOME[6] + 2 * np.pi, .5]
        second = np.r_[LEADER_HOME[:6], LEADER_HOME[6] + .01, .5]
        reader = self.reader(first, second)
        self.assertAlmostEqual(reader.read()[0][6], self.robot_home[6], places=12)
        self.assertAlmostEqual(reader.read()[0][6], self.robot_home[6] + .01, places=12)

    def test_action_uses_physical_robot_state_without_a_second_shift(self):
        raw = np.r_[LEADER_HOME[:6], LEADER_HOME[6] + .015, .4]
        target, width = self.reader(raw).read()
        measured_q = self.robot_home.copy()
        action = encode(measured_q, target, width)
        self.assertAlmostEqual(float(action[6]), .5)
        decoded_q, decoded_width = decode(measured_q, action)
        np.testing.assert_allclose(decoded_q, target, atol=1e-8)
        self.assertAlmostEqual(decoded_width, width, places=7)
        np.testing.assert_array_equal(measured_q, self.robot_home)
        # Starting the Panda at the old J7 must still fail alignment; never mask it.
        with self.assertRaisesRegex(ValueError, "align GELLO"):
            encode(LEADER_HOME, target, width)

    def test_invalid_references_and_unrequested_six_axis_changes_are_rejected(self):
        for key, index, value in (("arm_home", 0, .1), ("arm_home", 6, 4.),
                                  ("gello_reference_joints_rad", 6, float("nan"))):
            config = copy.deepcopy(self.config)
            config[key][index] = value
            with self.assertRaises(ValueError):
                j7_offset_from_config(config)
        config = copy.deepcopy(self.config)
        del config["gello_reference_joints_rad"]
        with self.assertRaises(ValueError):
            j7_offset_from_config(config)


if __name__ == "__main__":
    unittest.main()
