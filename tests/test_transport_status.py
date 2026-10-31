import unittest
from pi05_demo.transport import Bridge

class TransportStatusTests(unittest.TestCase):
    def reader(self,**changes):
        b=Bridge.__new__(Bridge)
        st=dict(ready=True,fault='',stopped=False,stopping=False,state_age_s=.001,gripper_age_s=.02)
        st.update(changes);b.request=lambda _:st
        return b
    def test_control_fault_is_reported_before_stale_feedback(self):
        with self.assertRaisesRegex(RuntimeError,'libfranka failure'):
            self.reader(ready=False,fault='libfranka failure',state_age_s=1.).state()
    def test_robot_and_gripper_staleness_are_distinguishable(self):
        with self.assertRaisesRegex(RuntimeError,'stale robot feedback'):
            self.reader(state_age_s=.06).state()
        with self.assertRaisesRegex(RuntimeError,'stale gripper feedback'):
            self.reader(gripper_age_s=.6).state()
    def test_stopping_controller_is_never_ready(self):
        with self.assertRaisesRegex(RuntimeError,'controller stopping'):
            self.reader(stopping=True).state()
