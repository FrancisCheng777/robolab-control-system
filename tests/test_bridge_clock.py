"""The bridge's advertised host clock must match the Python collector clock."""
import time
import unittest

from test_collection_session import mock_bridge


class BridgeClockTests(unittest.TestCase):
    def test_cpp_feedback_uses_python_host_monotonic_clock(self):
        with mock_bridge() as bridge:
            state = bridge.state()
            received = time.monotonic()
            self.assertGreaterEqual(received, state['state_bridge_receive_s'])
            self.assertLess(received - state['state_bridge_receive_s'], .05)
            self.assertGreaterEqual(received, state['gripper_bridge_read_s'])
            self.assertLess(received - state['gripper_bridge_read_s'], .5)
