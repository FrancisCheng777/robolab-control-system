import json, tempfile, time, unittest
from pathlib import Path
import h5py
import numpy as np
from test_collection_session import mock_bridge
from pi05_demo.episode import EpisodeWriter, inspect_episode
from pi05_demo.contract import decode

class RobolabFlowTests(unittest.TestCase):
    def test_raw_recording_keeps_distant_leader_target(self):
        with tempfile.TemporaryDirectory() as d:
            w=EpisodeWriter(d,mock=True,task='trial',session='one',config={'teleop_controller':'robolab_torque'})
            q=np.array([0.,-.6,0.,-2.5,0.,1.8,0.]); target=q+.2
            for i in range(3):
                t=time.monotonic()+i/15
                w.append(q=q+i*.005,target=target,width=.04,command_width=.08,
                    frames={k:(np.zeros((224,224,3),np.uint8),t) for k in ('main','wrist')},
                    timestamp=t,state_sequence=i+1,command_sequence=i+1,ack=True)
            p=w.finish('success')
            self.assertEqual(inspect_episode(p,allow_mock=True)['frames'],3)
            with h5py.File(p) as f:
                np.testing.assert_allclose(f['q_target'][0],target)
                self.assertEqual(f.attrs['action_source'],'next_observed_joint_state')
                self.assertNotIn('action', f)

    def test_absolute_takeover_accepts_mismatch_and_limits_motion(self):
        with mock_bridge(robolab=True) as bridge:
            bridge.claim(); st=bridge.state(); q=np.array(st['q'])
            target=q+.2
            bridge.command(st,target,0.)
            time.sleep(.03)
            st=bridge.state(); travelled=np.array(st['q'])-q
            self.assertTrue(np.all(travelled>0))
            self.assertLess(float(np.max(travelled)),.03)
            bridge.hold(st)
            time.sleep(.03)
            self.assertTrue(bridge.state()['hold_ready'])

    def test_export_actions_reconstruct_actual_motion_without_changing_raw_target(self):
        from pi05_demo.export_actions import training_arrays
        with tempfile.TemporaryDirectory() as d:
            w=EpisodeWriter(d,mock=True,task='trial',session='one',config={'teleop_controller':'robolab_torque'})
            q=np.array([0.,-.6,0.,-2.5,0.,1.8,0.])
            for i in range(3):
                t=time.monotonic()+i/15
                w.append(q=q+i*.01,target=q+.2,width=.04,command_width=.08,
                    frames={k:(np.zeros((224,224,3),np.uint8),t) for k in ('main','wrist')},
                    timestamp=t,state_sequence=i+1,command_sequence=i+1,ack=True)
            p=w.finish('success')
            with h5py.File(p) as f:
                states,actions=training_arrays(f)
                self.assertEqual(len(actions),2)
                for i,a in enumerate(actions):
                    target,width=decode(states[i,:7],a)
                    np.testing.assert_allclose(target,f['q'][i+1],atol=1e-7)
                    self.assertAlmostEqual(width,.08,places=6)
                np.testing.assert_allclose(f['q_target'][0],q+.2)
            with h5py.File(p,'r+') as f:f['q'][1]=q+.1
            with h5py.File(p) as f:
                with self.assertRaisesRegex(ValueError,'no clipping'):training_arrays(f)

    def test_session_starts_without_joint_or_aperture_alignment(self):
        from pi05_demo.session import CollectionSession
        from pi05_demo.cameras import SyntheticPair
        class Leader:
            def read(self):return target.copy(),0.
        with mock_bridge(robolab=True) as bridge,tempfile.TemporaryDirectory() as d:
            q=np.array(bridge.state()['q']);target=q+.2
            cfg={'teleop_controller':'robolab_torque','arm_home':q.tolist(),'task':'trial'}
            s=CollectionSession(bridge,SyntheticPair(),Leader(),cfg,d,'one',mock=True,announce=lambda *a,**k:None)
            try:
                s.phase='standby'
                s.tick();time.sleep(.02);s.tick(['toggle'])
                self.assertEqual(s.phase,'recording')
                s.tick();self.assertEqual(s.writer.count,1)
                s.tick(['f'])
                for _ in range(20):
                    time.sleep(.01);s.tick()
                    if s.completed_episodes:break
                self.assertEqual(s.completed_episodes,1)
            finally:s.close()

    def test_offline_grid_handles_jitter_but_rejects_long_gaps(self):
        from pi05_demo.export_actions import aligned_records
        with tempfile.TemporaryDirectory() as d:
            w=EpisodeWriter(d,mock=True,task='trial',session='one',config={'teleop_controller':'robolab_torque'})
            q=np.array([0.,-.6,0.,-2.5,0.,1.8,0.]);t0=100.
            for i,dt in enumerate([0.,.045,.14,.21,.28]):
                t=t0+dt
                w.append(q=q+dt*.15,target=q+.2,width=.04,command_width=.08,
                    frames={k:(np.zeros((224,224,3),np.uint8),t) for k in ('main','wrist')},
                    timestamp=t,state_sequence=i+1,command_sequence=i+1,ack=True)
            p=w.finish('success')
            with h5py.File(p) as f:
                states,actions,rows,cameras,grid=aligned_records(f,15)
                self.assertEqual(len(actions),4)
                np.testing.assert_allclose(actions[:,:7],1/3,atol=1e-5)
                np.testing.assert_allclose(np.diff(grid),1/15,atol=1e-9)
            with h5py.File(p,'r+') as f:f['timestamp'][-1]=101.
            with h5py.File(p) as f:
                with self.assertRaisesRegex(ValueError,'gap'):aligned_records(f,15)
