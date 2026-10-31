#!/usr/bin/env python3
"""ROS Noetic viewer adapter. Publishes only visualization markers, never joints."""
import argparse
import json
import math
from pathlib import Path
from urllib.request import urlopen

import rospy
import tf2_ros
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray

BLUE = (0.15, 0.60, 1.0, 0.85)
ORANGE = (1.0, 0.48, 0.08, 0.55)
GREEN = (0.30, 0.95, 0.58, 1.0)
RED = (1.0, 0.40, 0.35, 1.0)
WHITE = (0.9, 0.93, 0.98, 1.0)


def quaternion(m):
    trace = sum(m[i][i] for i in range(3))
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        return ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s,
                (m[1][0] - m[0][1]) / s, .25 * s)
    i = max(range(3), key=lambda k: m[k][k])
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + m[i][i] - m[j][j] - m[k][k]) * 2
    q = [0., 0., 0., 0.]
    q[i] = .25 * s
    q[j] = (m[j][i] + m[i][j]) / s
    q[k] = (m[k][i] + m[i][k]) / s
    q[3] = (m[k][j] - m[j][k]) / s
    return q


class Drawing:
    def __init__(self):
        self.markers = []

    def marker(self, namespace, kind, color):
        marker = Marker()
        marker.header.frame_id = "panda_base"
        marker.header.stamp = rospy.Time(0)
        marker.ns, marker.id = namespace, len(self.markers)
        marker.type, marker.action = kind, Marker.ADD
        marker.pose.orientation.w = 1
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.lifetime = rospy.Duration(.7)
        self.markers.append(marker)
        return marker

    def text(self, text, position, color=WHITE, size=.03):
        marker = self.marker("text", Marker.TEXT_VIEW_FACING, color)
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
        marker.scale.z, marker.text = size, text

    def lines(self, lines, color, width=.008):
        marker = self.marker("lines", Marker.LINE_LIST, color)
        marker.scale.x = width
        marker.points = [Point(*point) for line in lines for point in line]

    def robot(self, geometry, color, mesh_dir):
        for i, pose in enumerate(geometry["link_poses"]):
            mesh = mesh_dir / ("link%d.dae" % i)
            if mesh.exists():
                marker = self.marker("arm", Marker.MESH_RESOURCE, color)
                marker.mesh_resource = mesh.as_uri()
                marker.mesh_use_embedded_materials = False
                marker.scale.x = marker.scale.y = marker.scale.z = 1
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = [pose[k][3] for k in range(3)]
                quat = quaternion(pose)
                marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = quat
        points = geometry["points"]
        self.lines(list(zip(points[:-1], points[1:])), color, .009)
        self.lines(geometry["fingers"], color, .012)
        for axis, axis_color in zip(geometry["tool_axes"], [(1., .2, .2, .9), (.2, 1., .2, .9), (.2, .5, 1., .9)]):
            self.lines([axis], axis_color, .004)


def draw(state, mesh_dir):
    drawing = Drawing()
    mode = state.get("mode", "session")
    simulated = mode == "demo" or state.get("mock", False)
    reference_only = mode == "gello-home"
    stale = state.get("stale", True)
    title = "DEMO / NO HARDWARE" if simulated else ("PRESET HOME / NOT MEASURED" if reference_only else "LIVE / READ ONLY")
    drawing.text(title, [.30, -.95, 1.17], ORANGE if simulated or reference_only else BLUE, .052)
    legend = "BLUE: preset home | ORANGE: live GELLO" if reference_only else "BLUE: Franka | ORANGE: GELLO"
    drawing.text(legend, [.30, -.95, 1.08], WHITE, .042)
    phase = state.get("phase", "waiting")
    outside = state.get("outside_joint_limits", [])
    message = "NO FRESH DATA" if stale else "REFERENCE ONLY / NO TAKEOVER GATE"
    drawing.text(message, [.30, -.95, .99], ORANGE, .038)
    for key, color in [("reference", BLUE), ("target_geometry", ORANGE)]:
        if key in state:
            c = (*color[:3], .15) if stale else color
            drawing.robot(state[key], c, mesh_dir)
    errors = state.get("joint_error_degrees")
    if errors is not None:
        for i, (error, ok) in enumerate(zip(errors, state["joint_ok"])):
            status = "OUTSIDE RANGE" if i + 1 in outside else ""
            text = "J%d  ERR %+5.2f   MOVE %+5.2f  %s" % (i + 1, error, -error, status)
            drawing.text(text, [.30, -.95, .88 - i * .075], WHITE if stale else (RED if i in (0,2) else WHITE), .041)
            # A separate axis arrow identifies the largest-error joint on the model.
        for joint in (0,2):
            axis = state["target_geometry"]["joint_axes"][joint]
            drawing.lines([axis], RED, .012)
            pos = list(axis[1]); pos[2] += .04
            drawing.text("J%d" % (joint + 1), pos, RED, .04)
        width = state.get("gripper_error_mm")
        grip_text = "GRIP: no measured reference" if width is None else "GRIP: %+.1f mm (limit 5 mm)" % width
        drawing.text(grip_text, [.30, -.95, .32], GREEN if not stale and state.get("gripper_ok") else ORANGE, .037)
        drawing.text("TCP est: %.1f mm / %.2f deg" % (state["tcp_error_mm"], state["tool_angle_error_degrees"]), [.30, -.95, .24], WHITE, .039)
    shift = state.get("gello_j7_offset_degrees", -45.)
    drawing.text("Rough pose reference only | J7 offset: %+.1f deg" % shift, [.30, -.95, .15], WHITE, .029)
    drawing.text("Schematic tool / no clearance check", [.30, -.95, .09], WHITE, .029)
    return MarkerArray(markers=drawing.markers)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8787)
    parser.add_argument("--mesh-dir", type=Path, default=Path("/home/heinrich/roboLab/ros_ws/src/franka_description/meshes/visual"))
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("pi05_alignment_view", anonymous=True)
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "world"
    transform.child_frame_id = "panda_base"
    transform.transform.rotation.w = 1
    broadcaster.sendTransform(transform)
    publisher = rospy.Publisher("/pi05_alignment/markers", MarkerArray, queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            with urlopen("http://127.0.0.1:%d/api/state" % args.port, timeout=.25) as response:
                state = json.load(response)
        except Exception:
            state = {"stale": True, "phase": "disconnected"}
        publisher.publish(draw(state, args.mesh_dir.resolve()))
        rate.sleep()


if __name__ == "__main__":
    main()
