import unittest
from pi05_demo.operator_guide import format_alignment


class OperatorGuideTests(unittest.TestCase):
    def sample(self, ready=False):
        return dict(ready=ready, detail="", joint_error_degrees=[3, 0, 0, 0, 0, 0, 0],
                    leader_joint_degrees=[3, -36, 0, -144, 0, 108, 0],
                    robot_joint_degrees=[0, -36, 0, -144, 0, 108, 0],
                    gripper_error_mm=10., leader_width_mm=30., robot_width_mm=20.)

    def test_adjustment_is_opposite_to_error_and_not_ready(self):
        text = format_alignment(self.sample())
        self.assertIn("尚未对齐", text)
        self.assertIn("-3.00", text)
        self.assertIn("-10.0", text)
        self.assertNotIn("可以开始", text)

    def test_joint_alignment_alone_does_not_grant_takeover(self):
        data = self.sample()
        data["joint_error_degrees"] = [0.] * 7
        data["gripper_error_mm"] = 0.
        data["detail"] = "wait for hold"
        self.assertNotIn("可以开始", format_alignment(data))

    def test_ready_message_comes_only_from_session_gate(self):
        data = self.sample(True)
        data["joint_error_degrees"] = [0.] * 7
        data["gripper_error_mm"] = 0.
        self.assertIn("可以开始", format_alignment(data))
        self.assertIn("Enter", format_alignment(data))

    def test_missing_feedback_is_never_displayed_as_aligned(self):
        text = format_alignment(dict(ready=False, joint_error_degrees=None,
                                     gripper_error_mm=None, detail="unavailable"))
        self.assertIn("等待有效", text)
        self.assertNotIn("可以开始", text)

    def test_reference_alignment_never_claims_real_takeover(self):
        data = self.sample(True)
        data["reference_only"] = True
        text = format_alignment(data)
        self.assertIn("预摆位完成", text)
        self.assertIn("Franka参考", text)
        self.assertNotIn("可以开始", text)
