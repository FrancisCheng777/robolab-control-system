"""Read-only alignment geometry and optional, nonblocking collection telemetry.

Joint origins match the Panda URDF (not FR3). Link 7 -> flange is +0.107 m.
The tool transform comes from the existing no-camera Desk profile, in column
major order. Geometry is a schematic, not a collision or clearance model.
"""
import json
import os
import socket
import time

import numpy as np

from .contract import MAX_WIDTH, SCALE, validate_target, vector

SOCKET_PATH = f"/tmp/pi05-alignment-{os.getuid()}.sock"
STALE_SECONDS = .5
GRIPPER_TOLERANCE = .005
ORIGINS = (
    ((0, 0, .333), 0), ((0, 0, 0), -np.pi / 2),
    ((0, -.316, 0), np.pi / 2), ((.0825, 0, 0), np.pi / 2),
    ((-.0825, .384, 0), -np.pi / 2), ((0, 0, 0), np.pi / 2),
    ((.088, 0, 0), np.pi / 2),
)


def tool_transform(profile):
    transform = np.asarray(profile["transformation"], dtype=float).reshape(4, 4, order="F")
    if (not np.all(np.isfinite(transform))
            or not np.allclose(transform[3], [0, 0, 0, 1])
            or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3))
            or not np.isclose(np.linalg.det(transform[:3, :3]), 1)):
        raise ValueError("invalid Desk tool transformation")
    return transform


def panda_geometry(q, tool, width=None):
    q = vector(q, 7, "Panda joint pose")
    transform = np.eye(4)
    link_poses = [transform.tolist()]
    points = [[0., 0., 0.]]
    joint_axes = []
    for angle, (xyz, roll) in zip(q, ORIGINS):
        c, s = np.cos(roll), np.sin(roll)
        origin = np.eye(4)
        origin[:3, :3] = [[1, 0, 0], [0, c, -s], [0, s, c]]
        origin[:3, 3] = xyz
        transform = transform @ origin
        points.append(transform[:3, 3].tolist())
        joint_axes.append([transform[:3, 3].tolist(),
                           (transform[:3, 3] + .09 * transform[:3, 2]).tolist()])
        c, s = np.cos(angle), np.sin(angle)
        rotate = np.eye(4)
        rotate[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
        transform = transform @ rotate
        link_poses.append(transform.tolist())
    flange = np.eye(4)
    flange[2, 3] = .107
    transform = transform @ flange
    points.append(transform[:3, 3].tolist())
    tcp = transform @ tool
    points.append(tcp[:3, 3].tolist())
    axes = [[tcp[:3, 3].tolist(), (tcp[:3, 3] + .07 * tcp[:3, i]).tolist()]
            for i in range(3)]
    # Schematic fingers show aperture and J7 rotation; these are not CAD meshes.
    aperture = .06 if width is None else float(width)
    fingers = []
    def point(x, z):
        return (tcp @ np.array([x, 0, z, 1]))[:3].tolist()
    for side in (-1, 1):
        fingers.append([point(side * aperture / 2, -.06), point(side * aperture / 2, 0)])
    fingers.append([point(-aperture / 2, -.06), point(aperture / 2, -.06)])
    return {"points": points, "joint_axes": joint_axes, "tool_axes": axes, "link_poses": link_poses,
            "fingers": fingers, "tcp": tcp[:3, 3].tolist(),
            "rotation": tcp[:3, :3].tolist()}


def alignment_view(sample, tool, now=None):
    """Render a sample without granting motion authority or trusting old readiness."""
    now = time.monotonic() if now is None else now
    if not sample:
        return {"source": "waiting", "phase": "waiting", "stale": True,
                "can_takeover": False, "message": "等待采集程序的对齐数据"}
    age = now - float(sample["sample_time"])
    stale = not 0 <= age <= STALE_SECONDS
    out = {**sample, "age_s": max(0., age), "stale": stale, "can_takeover": False}
    reference = sample.get("q")
    target = sample.get("target")
    if reference is None:
        return out
    q = vector(reference, 7, "reference")
    out["reference"] = panda_geometry(q, tool, sample.get("width"))
    if target is None:
        return out
    target = vector(target, 7, "mapped GELLO target")
    error = target - q
    out["joint_error_degrees"] = np.degrees(error).tolist()
    out["correction_degrees"] = np.degrees(-error).tolist()
    out["joint_ok"] = (np.abs(error) <= SCALE + 1e-9).tolist()
    out["joint_tolerance_degrees"] = np.degrees(SCALE).tolist()
    out["largest_joint"] = int(np.argmax(np.abs(error))) + 1
    out["target_geometry"] = panda_geometry(target, tool, sample.get("target_width"))
    geometry = out["target_geometry"]
    reference_geometry = out["reference"]
    out["tcp_error_mm"] = float(np.linalg.norm(
        np.asarray(geometry["tcp"]) - reference_geometry["tcp"]) * 1000)
    rotation = np.asarray(geometry["rotation"]) @ np.asarray(reference_geometry["rotation"]).T
    out["tool_angle_error_degrees"] = float(np.degrees(
        np.arccos(np.clip((np.trace(rotation) - 1) / 2, -1, 1))))
    width, target_width = sample.get("width"), sample.get("target_width")
    width_error = None if width is None or target_width is None else target_width - width
    out["gripper_error_mm"] = None if width_error is None else width_error * 1000
    out["gripper_ok"] = width_error is not None and abs(width_error) <= GRIPPER_TOLERANCE
    valid = True
    try:
        validate_target(target)
    except ValueError:
        valid = False
    out["within_joint_limits"] = valid
    out["arm_aligned"] = bool(valid and all(out["joint_ok"]))
    out["can_takeover"] = bool(
        sample["source"] == "session" and not sample.get("mock", True)
        and sample["phase"] == "standby" and not stale
        and sample.get("reported_ready", False) and sample.get("held", False)
        and out["arm_aligned"] and out["gripper_ok"])
    return out


def validate_sample(sample):
    if sample.get("source") not in ("session", "home_reference", "demo"):
        raise ValueError("unknown alignment source")
    if not np.isfinite(sample["sample_time"]):
        raise ValueError("invalid timestamp")
    for key in ("q", "target"):
        if sample.get(key) is not None:
            vector(sample[key], 7, key)
    for key in ("width", "target_width"):
        if sample.get(key) is not None and not (np.isfinite(sample[key]) and 0 <= sample[key] <= MAX_WIDTH):
            raise ValueError("invalid aperture")
    if not isinstance(sample.get("phase"), str):
        raise ValueError("invalid phase")
    return sample


class AlignmentPublisher:
    """Best-effort, nonblocking local datagrams. Never opens a robot/serial port."""
    def __init__(self, path=SOCKET_PATH, *, mock):
        self.path, self.mock = path, bool(mock)
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.socket.setblocking(False)

    def update(self, state, phase, *, target=None, width=None, ready=False):
        sample = {"source": "session", "mock": self.mock, "phase": phase,
                  "sample_time": time.monotonic(), "q": list(state["q"]),
                  "width": float(state["gripper_width"]),
                  "target": None if target is None else np.asarray(target).tolist(),
                  "target_width": None if width is None else float(width),
                  "reported_ready": bool(ready),
                  "held": bool(state.get("hold_ready", False)
                               and np.max(np.abs(state["dq"])) <= .01)}
        self._send(sample)

    def _send(self, sample):
        try:
            self.socket.sendto(json.dumps(sample, allow_nan=False).encode(), self.path)
        except (OSError, ValueError, TypeError):
            # A closed, slow or absent viewer must not interrupt the control loop.
            pass

    def close(self):
        self._send({"source": "session", "phase": "closed", "mock": self.mock,
                    "sample_time": time.monotonic(), "reported_ready": False})
        self.socket.close()
