"""Opt-in timing contract. Dataset sampling remains 15 Hz."""
import math


def validate_recording_speed(calibration, fps):
    """Reject a known reference limit outside the unchanged per-frame action range.

    Actual motion is still audited by the exporter: a reference cap cannot
    guarantee bounds on measured motion during contact, overshoot or disturbances.
    """
    from .contract import SCALE
    speeds = calibration.get('max_velocity')
    if speeds is None:  # Older test/transport adapters omit motion calibration.
        return
    cap = calibration.get('teleop_max_velocity', .5)
    if len(speeds) != 7 or any(not math.isfinite(v) or v <= 0 for v in speeds):
        raise ValueError('invalid calibrated recording speeds')
    if not math.isfinite(cap) or cap <= 0:
        raise ValueError('invalid calibrated teleop speed')
    if any(min(v, cap) > float(scale)*fps + 1e-9 for v, scale in zip(speeds, SCALE)):
        raise ValueError('recording speed exceeds the action contract at this FPS; use responsive_recording')


def validate_rates(config):
    if config.get('teleop_controller') != 'robolab_torque':
        raise ValueError('fast teleoperation requires robolab_torque')
    if config.get('async_recording') is not True:
        raise ValueError('fast teleoperation requires async_recording=true')
    if config.get('control_hz') != 15:
        raise ValueError('recording control_hz must remain 15')
    hz = config['teleop_hz']
    if not math.isfinite(hz) or not 15 <= hz <= 100:
        raise ValueError('teleop_hz must be in [15, 100]')
    age = config.get('gello_max_sample_age_s', .05)
    if not math.isfinite(age) or not 0 < age <= .05:
        raise ValueError('GELLO sample age must be in (0, 0.05] seconds')
    config['gello_max_sample_age_s'] = age
