"""Pair the original GELLO pose with the Robotiq robot pose; never move hardware."""
import numpy as np

from .contract import validate_target, vector


def j7_offset_from_config(config):
    """Return a J7-only shift, in the existing GELLO joint-coordinate convention.

    These are robot-coordinate references, not Dynamixel encoder zero offsets.
    Apply the shift once, before unwrapping and checking the robot joint limits.
    """
    try:
        leader = vector(config["gello_reference_joints_rad"], 7, "GELLO reference")
        robot = vector(config["arm_home"], 7, "Panda arm_home")
    except KeyError as exc:
        raise ValueError(f"collection config missing reference: {exc.args[0]}") from exc
    validate_target(leader)
    validate_target(robot)
    if not np.array_equal(leader[:6], robot[:6]):
        raise ValueError("J7-only remapping requires identical J1-J6 references")
    return float(robot[6] - leader[6])
