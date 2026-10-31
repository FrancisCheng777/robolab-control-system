import time, unittest
from unittest.mock import patch
import numpy as np
from pi05_demo.session import CollectionSession, HomePath

class HoldableRobot:
    def __init__(self,q,dq,robolab):
        self.seq=0;self.commands=[]
        self.sample={'q':q,'dq':dq,'gripper_width':0.,'hold_ready':True,
            'session_control_version':1,'calibration':{'teleop_controller':'robolab_torque' if robolab else 'legacy_position'}}
    def state(self):return self.sample
    def claim(self):pass
    def hold(self,state):self.seq+=1
    def command(self,state,target,width,**kw):self.commands.append(target)
    def close(self):pass

class HomeCompletionTests(unittest.TestCase):
    def pending_home(self):
        goal=np.array([0.,-.628,0.,-2.513,0.,1.885,0.])
        bridge=HoldableRobot(goal,np.zeros(7),True)
        bridge.sample['hold_ready']=False
        cfg={'arm_home':goal.tolist(),'teleop_controller':'robolab_torque'}
        session=CollectionSession(bridge,None,None,cfg,'unused','test',mock=True,
                                  auto_home=True,announce=lambda *a,**k:None)
        return session,bridge

    def test_confirmed_home_waits_for_hold_without_needing_another_key(self):
        session,bridge=self.pending_home()
        session.tick()
        self.assertEqual(session.phase,'init')
        self.assertIsNone(session.writer)
        self.assertEqual(bridge.commands,[])
        bridge.sample['hold_ready']=True
        session.tick()
        self.assertEqual(session.phase,'homing')
        self.assertIsNone(session.writer)

    def test_pending_home_can_be_cancelled_before_motion(self):
        session,bridge=self.pending_home()
        self.assertFalse(session.tick(['q']))
        self.assertIsNone(session.home)
        self.assertEqual(bridge.commands,[])

    def test_pending_home_times_out_instead_of_starting_later(self):
        with patch('pi05_demo.session.time.monotonic',return_value=10.):
            session,bridge=self.pending_home()
        with patch('pi05_demo.session.time.monotonic',return_value=21.):
            with self.assertRaisesRegex(RuntimeError,'startup.*hold'):
                session.tick()
        self.assertEqual(bridge.commands,[])

    def run_home(self,error,velocity=0.,robolab=True):
        goal=np.array([0.,-.628,0.,-2.513,0.,1.885,0.]);q=goal.copy();q[3]-=error
        bridge=HoldableRobot(q,np.full(7,velocity),robolab)
        cfg={'arm_home':goal.tolist(),'task':'home only'}
        if robolab:cfg['teleop_controller']='robolab_torque'
        session=CollectionSession(bridge,None,None,cfg,'unused','test',mock=True,announce=lambda *a,**k:None)
        session.home=HomePath(q,goal,time.monotonic()-3)
        session.home.last=time.monotonic()-.01
        session.phase='homing';session.tick()
        return session,bridge
    def test_robolab_home_accepts_observed_stationary_residual(self):
        session,bridge=self.run_home(.0113)
        self.assertEqual(session.phase,'settling')
        self.assertEqual(session.after_hold,'standby')
        self.assertIsNone(session.writer)
        self.assertEqual(bridge.commands,[])
    def test_robolab_home_does_not_accept_large_error(self):
        session,_=self.run_home(.06)
        self.assertEqual(session.phase,'homing')
    def test_robolab_home_waits_for_motion_to_stop(self):
        session,_=self.run_home(.0113,.03)
        self.assertEqual(session.phase,'homing')
    def test_legacy_controller_retains_its_precision(self):
        session,_=self.run_home(.0113,robolab=False)
        self.assertEqual(session.phase,'homing')
