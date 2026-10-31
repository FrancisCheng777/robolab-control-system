#!/usr/bin/env python3
"""Bounded hold-only FCI check with real cameras/GELLO and diagnostic image IO.

No home, follow, gripper move, automatic recovery, or training episode is issued.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

ROOT = Path('/home/heinrich/pi05_plug_demo')
sys.path.insert(0, str(ROOT))
import h5py
import numpy as np
from pi05_demo.cameras import Pair
from pi05_demo.collect import Gello
from pi05_demo.gripper_teleop import unowned
from pi05_demo.teleop_reference import j7_offset_from_config
from pi05_demo.transport import Bridge
from fix_fci_network_priority import discover as network_scheduling


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=300)
    args = parser.parse_args()
    if not 10 <= args.seconds <= 600:
        raise SystemExit('Duration must be 10..600 seconds')
    if os.getpriority(os.PRIO_PROCESS, 0) != 0:
        raise SystemExit('Repair priority first and use a fresh login (nice=0 required).')
    lock = open('/tmp/pi05-robolab-trial-20261030.lock', 'a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    config_path = ROOT / 'reports/robolab_restore_20261030/bridge.trial.json'
    config = json.loads(config_path.read_text())
    collection = json.loads((config_path.parent / 'collection.trial.json').read_text())
    for port in (config['gripper_device'], collection['gello_port']):
        unowned(port)
    if Path(collection['socket']).exists():
        raise SystemExit('Collection socket exists; inspect original process first')
    out = ROOT / 'reports/realtime_fix_20261031' / ('stationary_' + time.strftime('%Y%m%d_%H%M%S'))
    out.mkdir()
    subprocess.run([sys.executable, str(ROOT / 'scripts/check_desk_tool.py'), '--config', str(config_path),
                    '--output', str(out / 'desk-before.json')], cwd=ROOT, check=True, timeout=15,
                   stdout=subprocess.DEVNULL)
    pre = json.loads((out / 'desk-before.json').read_text())
    if not pre['matches_expected_tool'] or pre['state']['robot_mode'] != 'Idle' or pre['state']['current_errors']:
        raise SystemExit('Tool mismatch or robot not Idle without errors; no automatic recovery')
    os.sched_setaffinity(0, set(range(20)) - {12})
    report = dict(passed=False, network_scheduling_start=network_scheduling(),
                  requested_seconds=args.seconds, home_commanded=False,
                  follow_commanded=False, gripper_move_commanded=False, samples=[])
    cameras = bridge = proc = data = None
    log = (out / 'bridge.log').open('w')
    try:
        cameras = Pair(collection['cameras'])
        leader = Gello(collection['gello_port'], j7_offset_rad=j7_offset_from_config(collection), robolab=True)
        # GelloAgent's real port initialization must succeed; record driver class for audit.
        driver = leader.agent._robot._driver
        report['leader_driver_type'] = type(driver).__name__
        if type(driver).__name__ != 'DynamixelDriver':
            raise RuntimeError('Real GELLO driver required; refusing a fake fallback')
        path = '/tmp/pi05-rt-check-' + uuid.uuid4().hex[:10] + '.sock'
        env = dict(os.environ, PI05_CONTROL_CPU='12')
        env.pop('LD_LIBRARY_PATH', None)
        proc = subprocess.Popen([str(ROOT / 'build/pi05_franka_bridge'), '--config', str(config_path),
                                 '--robot', '172.16.0.40', '--socket', path,
                                 '--fault-log', str(out / 'franka_fault.csv')], cwd=ROOT, env=env,
                                stdout=log, stderr=subprocess.STDOUT)
        report['bridge_pid'] = proc.pid
        deadline = time.monotonic() + 10
        while bridge is None:
            if proc.poll() is not None:
                raise RuntimeError('Bridge exited during startup')
            try:
                bridge = Bridge(path, mock=False)
            except (OSError, RuntimeError):
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.02)
        initial = bridge.state()
        q0 = np.asarray(initial['q'])
        bridge.claim()
        bridge.hold(initial)
        data = h5py.File(out / 'diagnostic_images.h5', 'w')
        data.attrs['purpose'] = 'stationary communication stress check; NOT a training episode'
        images = {key: data.create_dataset(key, shape=(0,224,224,3), maxshape=(None,224,224,3),
                  chunks=(1,224,224,3), compression='lzf', dtype='u1') for key in ('main','wrist')}
        started = time.monotonic()
        index = 0
        while time.monotonic() - started < args.seconds:
            tick = time.monotonic()
            stage_times = {}
            state = bridge.state()
            stage_times['state_ms'] = 1000*(time.monotonic()-tick)
            drift = float(np.max(np.abs(np.asarray(state['q'])-q0)))
            if drift > .03:
                raise RuntimeError(f'Stationary drift exceeded 0.03 rad: {drift}')
            stage_start=time.monotonic()
            bridge.hold(state)
            stage_times['hold_ms']=1000*(time.monotonic()-stage_start)
            stage_start=time.monotonic()
            leader.read_for_alignment()  # Read only: never used as robot targets.
            stage_times['leader_ms']=1000*(time.monotonic()-stage_start)
            stage_start=time.monotonic()
            frames, metadata = cameras.capture_with_metadata()
            stage_times['camera_ms']=1000*(time.monotonic()-stage_start)
            stage_start=time.monotonic()
            for key, dataset in images.items():
                dataset.resize(index+1, axis=0)
                dataset[index] = frames[key][0]
            stage_times['image_write_ms']=1000*(time.monotonic()-stage_start)
            keys = ('state_age_s', 'gripper_age_s', 'callback_count', 'callback_gap_count',
                    'max_callback_gap_s', 'control_command_success_rate')
            report['samples'].append(dict({key: state[key] for key in keys}, elapsed=tick-started,
                                          max_joint_displacement_rad=drift, stage_times=stage_times))
            index += 1
            if index % 150 == 0:
                # Do not scan all /proc processes inside the command heartbeat.
                stage_start=time.monotonic()
                original = report['network_scheduling_start']
                cpu_text=(Path('/proc/irq')/str(original['irq'])/'effective_affinity_list').read_text().strip()
                if (cpu_text != str(original['cpu']) or os.sched_getscheduler(original['pid']) != original['policy']
                        or os.sched_getparam(original['pid']).sched_priority != original['priority']):
                    raise RuntimeError('FCI network IRQ/thread scheduling changed during validation')
                stage_times['network_check_ms']=1000*(time.monotonic()-stage_start)
                print(json.dumps({k:v for k,v in report['samples'][-1].items() if k != 'stage_times'}), flush=True)
            stage_times['total_work_ms']=1000*(time.monotonic()-tick)
            time.sleep(max(0, 1/15 - (time.monotonic()-tick)))
        report.update(elapsed_seconds=time.monotonic()-started, frames=index, passed=True)
    except BaseException as exc:
        report['error'] = repr(exc)
        if bridge is not None:
            report['last_state'] = getattr(bridge, 'last_state', None)
    finally:
        try:
            if bridge:
                bridge.close()
        except Exception as exc:
            report['stop_error'] = repr(exc)
            report['passed'] = False
        if proc:
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.terminate()
                proc.wait(timeout=5)
            report['bridge_exit_code'] = proc.returncode
            if proc.returncode != 0:
                report['passed'] = False
        if cameras:
            cameras.close()
        if data:
            data.close()
        log.close()
        (out / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print('Verification result:', out / 'result.json', 'passed=', report['passed'], flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
