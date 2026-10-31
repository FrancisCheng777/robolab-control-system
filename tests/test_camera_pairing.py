import threading,unittest
from collections import deque
from unittest.mock import patch
import numpy as np
from pi05_demo.cameras import Pair

class CameraPairingTests(unittest.TestCase):
    def camera(self):
        p=Pair.__new__(Pair);p.lock=threading.Lock()
        image=np.zeros((224,224,3),np.uint8)
        def sample(t,n):return (image,t,{'receive_monotonic_s':t,'ready_monotonic_s':t,'frame_id':n})
        p.history={'main':deque([sample(9.89,1)]),'wrist':deque([sample(9.90,2),sample(9.99,3)])}
        p.frames={k:(v[-1][0],v[-1][1]) for k,v in p.history.items()}
        p.metadata={k:v[-1][2] for k,v in p.history.items()}
        return p
    def test_matches_received_frames_without_rewriting_timestamps(self):
        p=self.camera()
        with patch('pi05_demo.cameras.time.monotonic',return_value=10.):
            frames,meta=p.capture_with_metadata()
        self.assertEqual(frames['wrist'][1],9.90)
        self.assertEqual(meta['wrist']['frame_id'],2)
        self.assertEqual(meta['main']['receive_monotonic_s'],9.89)
    def test_stale_pair_is_still_rejected(self):
        with patch('pi05_demo.cameras.time.monotonic',return_value=11.):
            with self.assertRaises(ValueError):self.camera().capture_with_metadata()
