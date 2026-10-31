"""Explicit action construction; preserve raw leader targets and never clip motion."""
import numpy as np
from .contract import encode


def training_arrays(episode):
    if episode.attrs.get('schema') != 'pi05_plug_demo/2':
        return episode['state'][:], episode['action'][:]
    q=episode['q'][:]
    if len(q)<2:raise ValueError('at least two observations required for realized-motion actions')
    actions=[]
    for i in range(len(q)-1):
        try:
            actions.append(encode(q[i],q[i+1],episode['gripper_command_width'][i]))
        except ValueError as exc:
            raise ValueError(f'frame {i}: observed motion exceeds model action range; no clipping or dropped interior frames') from exc
    return episode['state'][:-1],np.asarray(actions,dtype=np.float32)


def aligned_records(f, fps, max_gap=.2):
    """RoboLab-style offline grid: interpolate state, hold commands, nearest images.

    Adapted from roboLab/scripts/lerobot_tools/core.py's bracket/interpolate
    workflow. Unlike its absolute action output, our adapter uses realized
    next-grid joint deltas. Raw input HDF5 is never rewritten.
    """
    times=np.asarray(f['timestamp'],float)
    if f.attrs.get('schema')!='pi05_plug_demo/2':
        states,actions=training_arrays(f)
        ids=np.arange(len(times))
        return states,actions,ids,{k:ids for k in ('main','wrist')},times
    if len(times)<2 or not np.isfinite(times).all() or np.any(np.diff(times)<=0):
        raise ValueError('need at least two ordered observations')
    if np.max(np.diff(times))>max_gap:
        raise ValueError('recording gap exceeds 200 ms; will not interpolate across it')
    grid=times[0]+np.arange(int(np.floor((times[-1]-times[0])*fps+1e-7))+1)/fps
    if len(grid)<2:raise ValueError('recording too short for one action')
    raw_states=np.asarray(f['state'],float)
    states=np.column_stack([np.interp(grid,times,raw_states[:,i]) for i in range(9)])
    left=np.maximum(0,np.searchsorted(times,grid,side='right')-1)
    widths=np.asarray(f['gripper_command_width'])[left]
    actions=[]
    for i in range(len(grid)-1):
        try:actions.append(encode(states[i,:7],states[i+1,:7],widths[i]))
        except ValueError as exc:raise ValueError(f'grid frame {i}: actual motion exceeds action range; no clipping') from exc
    camera_ids={}
    for name in ('main','wrist'):
        stamps=np.asarray(f['camera_time/'+name])
        ids=np.abs(stamps[:,None]-grid[None,:-1]).argmin(axis=0)
        if np.max(np.abs(stamps[ids]-grid[:-1]))>max_gap:
            raise ValueError('camera gap exceeds 200 ms')
        camera_ids[name]=ids
    return states[:-1].astype(np.float32),np.asarray(actions),left[:-1],camera_ids,grid[:-1]
