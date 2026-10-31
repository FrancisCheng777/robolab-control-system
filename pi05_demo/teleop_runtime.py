"""Opt-in Python scheduling settings; no hardware access or real-time priority."""
import math
import os
import sys


def configure_runtime(config):
    # Invoke after GELLO's reader is created: pin only this thread, not its reader.
    interval = config.get('teleop_gil_switch_interval_s')
    cpu = config.get('teleop_main_cpu')
    if interval is not None and (not math.isfinite(interval) or not .001 <= interval <= .005):
        raise ValueError('teleop GIL interval must be in [0.001, 0.005] seconds')
    if cpu is not None:
        if type(cpu) is not int or cpu < 0 or cpu in (11, 12):
            raise ValueError('teleop CPU must avoid the network and Franka control CPUs')
        if cpu not in os.sched_getaffinity(0):
            raise ValueError('teleop main CPU is outside launcher affinity')
        os.sched_setaffinity(0, {cpu})
    if interval is not None:
        sys.setswitchinterval(interval)
    return dict(main_cpu=cpu, gil_switch_interval_s=sys.getswitchinterval())
