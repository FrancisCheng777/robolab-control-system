"""Bounded CPU/mock benchmark: real HDF5 writer, no hardware or robot model."""
import argparse
import json
from pathlib import Path
import sys
import tempfile
import time

import h5py
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'tests')]
from test_collection_session import mock_bridge
from pi05_demo.session import CollectionSession
from pi05_demo.cameras import SyntheticPair


class ReplayPair(SyntheticPair):
    def __init__(self, path):
        super().__init__()
        # Only texture is reused. Output stays mock, with current synthetic
        # timestamps. This is a storage workload, not a real robot rollout.
        with h5py.File(path) as f:
            self.images = {k: f['images'][k][:30] for k in ('main', 'wrist')}
    def capture_with_metadata(self):
        frames, metadata = super().capture_with_metadata()
        return {k: (self.images[k][self.sequence % len(self.images[k])].copy(), v[1])
                for k, v in frames.items()}, metadata


def run(fast, seconds, images):
    events = []
    with mock_bridge(robolab=True) as bridge, tempfile.TemporaryDirectory() as output:
        q = np.array(bridge.state()['q'])
        class Leader:
            def read(self): return q.copy(), .04
        cfg = dict(teleop_controller='robolab_torque', control_hz=15, arm_home=q.tolist(), task='mock benchmark')
        if fast: cfg.update(teleop_hz=100, async_recording=True)
        cameras = ReplayPair(images) if images else SyntheticPair()
        session = CollectionSession(bridge, cameras, Leader(), cfg, output, 'benchmark',
                                    mock=True, announce=lambda event, **data: events.append((event, data)))
        commands, ticks = [], []
        command = bridge.command
        def timed(*args, **kwargs):
            commands.append(time.monotonic())
            return command(*args, **kwargs)
        bridge.command = timed
        try:
            session.phase = 'standby'
            # Wait on the mock's actual hold acknowledgment, bounded.
            deadline = time.monotonic() + 1
            while not bridge.state()['hold_ready']:
                session.tick()
                if time.monotonic() >= deadline: raise TimeoutError('mock hold')
                time.sleep(.002)
            session.tick(['toggle'])
            period = 1 / cfg.get('teleop_hz', 15)
            start = deadline = time.monotonic()
            while time.monotonic()-start < seconds:
                t = time.monotonic()
                session.tick()
                ticks.append(time.monotonic()-t)
                deadline += period
                if time.monotonic()-deadline > period: deadline = time.monotonic()
                time.sleep(max(0, deadline-time.monotonic()))
            session.tick(['f'])
            deadline = time.monotonic()+6
            while not session.completed_episodes:
                session.tick()
                if time.monotonic()>deadline: raise TimeoutError('mock finalization')
                time.sleep(.005)
            path = next(Path(output).glob('*.h5'))
            from pi05_demo.episode import inspect_episode
            inspect_episode(path, allow_mock=True)
            with h5py.File(path) as f:
                stamps=f['timestamp'][:]
                from pi05_demo.timing import audit_timing
                audit=audit_timing(f, 15, allow_resampling=True)
                frame_count=len(stamps)
            return dict(mode='fast' if fast else 'baseline', commands=len(commands), frames=frame_count,
                command_interval_ms_p50_p95_p99=np.percentile(np.diff(commands)*1000,[50,95,99]).tolist(),
                control_tick_ms_p50_p95_p99=np.percentile(np.array(ticks)*1000,[50,95,99]).tolist(),
                recording_interval_ms_p50_p95_p99=np.percentile(np.diff(stamps)*1000,[50,95,99]).tolist(),
                timing_audit=audit, events=events, mock=True)
        finally:
            session.close()


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=5)
    parser.add_argument('--image-source',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if not 1 <= args.seconds <= 30: parser.error('seconds must be in [1,30]')
    report={'hardware_used':False, 'results':[run(fast,args.seconds,args.image_source) for fast in (False,True)]}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps([{k:v for k,v in item.items() if k not in ('events','timing_audit')} for item in report['results']],indent=2))
