"""Read-only GELLO prealignment before the FCI controller is started."""
import argparse
import json
from pathlib import Path
import sys
import termios
import time
import tty

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from pi05_demo.collect import Gello
from pi05_demo.contract import validate_target
from pi05_demo.gripper_teleop import NativeGripper, ready
from pi05_demo.operator_guide import format_alignment
from pi05_demo.session import Keys
from pi05_demo.teleop_reference import j7_offset_from_config

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--config", required=True, type=Path)
parser.add_argument("--bridge-config", required=True, type=Path)
parser.add_argument("--report", required=True, type=Path)
args = parser.parse_args()
if not sys.stdin.isatty():
    parser.error("use an interactive terminal")
config = json.loads(args.config.read_text())
gripper_config = json.loads(args.bridge_config.read_text())
gripper = NativeGripper(gripper_config)
try:
    state = gripper.request("status")
    for _ in range(10):
        if state["fault"] != 9:
            break
        time.sleep(.02)
        state = gripper.request("status")
    ready(state)
    width = float(np.clip(gripper_config["gripper_max_width"] * (
        gripper_config["robotiq_closed_position"] - state["position"]) / (
        gripper_config["robotiq_closed_position"] - gripper_config["robotiq_open_position"]),
        0, gripper_config["gripper_max_width"]))
finally:
    gripper.close()
leader = Gello(config["gello_port"], j7_offset_rad=j7_offset_from_config(config))
saved_terminal = termios.tcgetattr(sys.stdin)
accepted = False
try:
    tty.setcbreak(sys.stdin.fileno())
    keys = Keys()
    reference = np.array(config["arm_home"])
    while True:
        target, target_width = leader.read_for_alignment()
        error = target - reference
        width_error = target_width - width
        detail = ""
        try:
            validate_target(target)
            aligned = bool(np.max(np.abs(error)) <= .03 and abs(width_error) <= .005)
        except ValueError as exc:
            aligned, detail = False, str(exc)
        data = dict(reference_only=True, ready=aligned, detail=detail,
            joint_error_degrees=np.degrees(error).tolist(), gripper_error_mm=width_error * 1000,
            leader_joint_degrees=np.degrees(target).tolist(), robot_joint_degrees=np.degrees(reference).tolist(),
            leader_width_mm=target_width * 1000, robot_width_mm=width * 1000)
        color = "\033[32m" if aligned else "\033[33m"
        print("\033[2J\033[H" + color + format_alignment(data) + "\033[0m", flush=True)
        pressed = keys.read(sys.stdin.fileno())
        if "q" in pressed:
            break
        if "toggle" in pressed and aligned:
            args.report.write_text(json.dumps(data, indent=2) + "\n")
            accepted = True
            break
        time.sleep(.2)
except KeyboardInterrupt:
    pass
finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved_terminal)
    leader.agent._robot._driver.close()
if not accepted:
    raise SystemExit(130)
print("预摆位完成。正在启动试采；稍后看到“准备完成”再按 Enter 回位。", flush=True)
