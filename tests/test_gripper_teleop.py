import unittest
from pi05_demo.gripper_teleop import Follower, closure_from_degrees


class Gripper:
    def __init__(self):
        self.commands = []
        self.position = 3
        self.fault = 0

    def request(self, op, **kwargs):
        self.commands.append((op, kwargs))
        return dict(activated=True, status=3, fault=self.fault,
                    position=self.position, goto_requested=op == "move")


class FollowerTests(unittest.TestCase):
    def setUp(self):
        self.gripper = Gripper()
        self.follower = Follower(self.gripper, 3, 227)

    def test_stale_and_invalid_inputs_never_move(self):
        for closure, timestamp, now in [(0.5, 0, 1), (float("nan"), 1, 1),
                                        (-0.01, 1, 1), (1.01, 1, 1), (0.5, 2, 1)]:
            with self.assertRaises(ValueError):
                self.follower.update(closure, timestamp, now)
        self.assertFalse(any(op == "move" for op, _ in self.gripper.commands))

    def test_trigger_overtravel_saturates_like_the_collection_mapping(self):
        self.assertEqual(closure_from_degrees(151.61, 200, 158), 1)
        self.assertEqual(closure_from_degrees(203.12, 200, 158), 0)
        self.assertEqual(closure_from_degrees(179, 200, 158), 0.5)

    def test_startup_follows_current_trigger_at_configured_motor_speed(self):
        self.follower.update(1, 1, 1)
        self.assertEqual(self.gripper.commands[-1], ("move", {"position":227}))

    def test_endpoints_and_latest_target_are_sent_without_waiting_for_motion(self):
        self.follower.update(0, 1, 1)
        self.follower.update(1, 2, 2)
        self.follower.update(0.5, 3, 3)
        positions = [kw["position"] for op, kw in self.gripper.commands if op == "move"]
        self.assertEqual(positions, [227, 115])

    def test_static_input_still_polls_feedback_without_reissuing_move(self):
        self.follower.update(0, 1, 1)
        self.follower.update(0.001, 2, 2)
        self.assertTrue(all(op == "status" for op, _ in self.gripper.commands))

    def test_fault_is_rejected_before_another_move(self):
        self.follower.update(0, 1, 1)
        self.gripper.fault = 14
        with self.assertRaises(RuntimeError):
            self.follower.update(1, 2, 2)
        self.assertFalse(any(op == "move" for op, _ in self.gripper.commands))

    def test_close_sends_stop(self):
        self.follower.close()
        self.assertEqual(self.gripper.commands[-1][0], "stop")
