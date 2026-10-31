"""Opt-in timing contract. Dataset sampling remains 15 Hz."""
import math


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
