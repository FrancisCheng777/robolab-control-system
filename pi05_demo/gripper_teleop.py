"""GELLO gripper-only diagnostic; no Panda or camera connection."""

import argparse
import fcntl
import json
import math
import select
import signal
import subprocess
import sys
import termios
import time
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def ready(state):
    if not state["activated"] or state["status"] != 3 or state["fault"] != 0:
        raise RuntimeError(f"Gripper is not ready: {state}")


def closure_from_degrees(degrees, open_degrees, closed_degrees):
    """Match GELLO's endpoint saturation, including normal trigger overtravel."""
    if not all(math.isfinite(v) for v in (degrees, open_degrees, closed_degrees)):
        raise ValueError("Non-finite gripper calibration or sample")
    if open_degrees == closed_degrees:
        raise ValueError("GELLO endpoints must differ")
    return max(0.0, min(1.0, (degrees - open_degrees) / (closed_degrees - open_degrees)))


def unowned(port):
    """Check legacy processes that may not implement flock; never kill an owner."""
    result = subprocess.run(["fuser", str(Path(port).resolve())], capture_output=True, text=True)
    if result.returncode == 0:
        raise RuntimeError(f"Serial port is already in use: {port}; PID(s) {result.stdout.strip()}")
    if result.returncode != 1:
        raise RuntimeError(f"Cannot check serial ownership: {result.stderr}")


class NativeGripper:
    def __init__(self, config):
        unowned(config["gripper_device"])
        self.speed = int(config["gripper_speed"])
        self.force = int(config["gripper_force"])
        self.process = subprocess.Popen(
            [str(ROOT / "build/pi05_gripper_tool"), "--device", config["gripper_device"]],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1,
            start_new_session=True)  # Python handles Ctrl-C and requests a confirmed stop.

    def request(self, op, **values):
        if op == "move":
            values.update(speed=self.speed, force=self.force)
        self.process.stdin.write(json.dumps(dict(op=op, **values)) + "\n")
        self.process.stdin.flush()
        if not select.select([self.process.stdout], [], [], 1.0)[0]:
            raise TimeoutError("Native gripper reply timed out")
        line = self.process.stdout.readline()
        if not line:
            raise RuntimeError("Native gripper process disconnected")
        result = json.loads(line)
        if "error" in result:
            raise RuntimeError(result["error"])
        return result

    def close(self):
        try:
            self.process.stdin.close()  # EOF invokes native stop if motion was commanded.
        except BrokenPipeError:
            pass  # The native watchdog may already have stopped and exited.
        try:
            self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            self.process.wait(timeout=2)


class GelloGripper:
    """Read servo 8 synchronously; never return stale cached data or fake hardware."""
    def __init__(self, port):
        from dynamixel_sdk import PortHandler, PacketHandler
        sys.path.insert(0, str(ROOT / "vendor"))
        from gello.agents.gello_agent import PORT_CONFIG_MAP
        self.servo, self.open_deg, self.closed_deg = PORT_CONFIG_MAP[port].gripper_config
        unowned(port)
        self.handler = PortHandler(port)
        self.packet = PacketHandler(2.0)
        if not self.handler.setBaudRate(57600):
            raise RuntimeError("GELLO serial open failed")
        try:
            fcntl.flock(self.handler.ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(self.handler.ser.fileno(), termios.TIOCEXCL)
            torque, comm, error = self.packet.read1ByteTxRx(self.handler, self.servo, 64)
            if comm != 0 or error:
                raise RuntimeError(f"GELLO servo {self.servo} did not reply; check motor power")
            if torque:
                raise RuntimeError("GELLO gripper motor torque is enabled; use passive leader mode first")
        except BaseException:
            self.close()
            raise

    def read(self):
        started = time.monotonic()
        ticks, comm, error = self.packet.read4ByteTxRx(self.handler, self.servo, 132)
        received = time.monotonic()
        if comm != 0 or error:
            raise RuntimeError(f"GELLO read failed: communication={comm}, device={error}")
        if ticks > 0x7fffffff:
            ticks -= 0x100000000
        degrees = ticks * 360 / 4096
        return dict(closure=closure_from_degrees(degrees, self.open_deg, self.closed_deg), degrees=degrees,
                    gello_read_start_s=started, gello_read_done_s=received)

    def close(self):
        if self.handler.ser and self.handler.ser.is_open:
            fcntl.ioctl(self.handler.ser.fileno(), termios.TIOCNXCL)
            self.handler.closePort()


class Follower:
    def __init__(self, gripper, open_position, closed_position, deadband=3):
        self.gripper = gripper
        self.open_position = open_position
        self.closed_position = closed_position
        self.deadband = deadband
        if not 0 <= open_position < closed_position <= 255:
            raise ValueError("Invalid measured gripper endpoints")
        self.last_target = None

    def update(self, closure, sample_time, now):
        if not all(math.isfinite(x) for x in (closure, sample_time, now)):
            raise ValueError("Non-finite GELLO sample")
        if not 0 <= closure <= 1 or not 0 <= now - sample_time <= 0.1:
            raise ValueError("Invalid or stale GELLO sample")
        target = round(self.open_position + closure * (self.closed_position - self.open_position))
        state = self.gripper.request("status")
        ready(state)
        if self.last_target is None:
            self.last_target = state["position"]
        if abs(target - self.last_target) >= self.deadband:
            state = self.gripper.request("move", position=target)
            ready(state)
            self.last_target = target
        return dict(state, target_position=target)

    def close(self):
        self.gripper.request("stop")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="follow the GELLO trigger; otherwise read only")
    parser.add_argument("--seconds", type=float, default=120)
    parser.add_argument("--hz", type=float, choices=(15, 30), default=15)
    parser.add_argument("--no-prompt", action="store_true", help="start the explicitly requested test immediately")
    parser.add_argument("--speed", type=int, help="test-only register override, 1..255")
    parser.add_argument("--force", type=int, help="test-only register override, 1..255")
    args = parser.parse_args()
    if not 0 < args.seconds <= 600:
        parser.error("--seconds must be in (0, 600]")
    collection = json.loads((ROOT / "configs/collection.json").read_text())
    config = json.loads((ROOT / "configs/bridge.real.json").read_text())
    for name in ("speed", "force"):
        value = getattr(args, name)
        if value is not None:
            if not 1 <= value <= 255:
                parser.error(f"--{name} must be 1..255")
            config[f"gripper_{name}"] = value
    if args.execute and not args.no_prompt:
        print("仅控制 2F-85；Panda 和相机不参与。夹爪行程内保持空置。")
        print("启动即跟随 GELLO 当前旋钮开度；Ctrl-C 停止，超时自动停止。")
        input("按 Enter 开始夹爪试用：")
    path = ROOT / "reports" / ("gripper_teleop_" + time.strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:6] + ".jsonl")
    path.parent.mkdir(parents=True, exist_ok=True)
    print(f"诊断日志（不是 SFT episode）：{path}", flush=True)
    gello = gripper = follower = None
    rows = 0
    failed = False
    interrupted = False
    def stop_signal(signum, frame):
        raise KeyboardInterrupt
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, stop_signal)
    with path.open("x") as log:
        def emit(row):
            log.write(json.dumps(row) + "\n")
            log.flush()
        emit(dict(event="header", diagnostic_only=True, execute=args.execute,
                  hz=args.hz, seconds=args.seconds, config=config,
                  gello_port=collection["gello_port"], timestamp_clock="host_monotonic_seconds"))
        try:
            gello = GelloGripper(collection["gello_port"])
            gripper = NativeGripper(config)
            gripper.request("status")
            time.sleep(0.02)  # An idle device's communication-timeout flag clears on polling.
            ready(gripper.request("status"))
            if args.execute:
                follower = Follower(gripper, config["robotiq_open_position"], config["robotiq_closed_position"])
            start = next_tick = time.monotonic()
            next_print = start
            print(f"{'FOLLOWING' if args.execute else 'READ ONLY'} {args.hz:g} Hz / {args.seconds:g} s", flush=True)
            while time.monotonic() - start < args.seconds:
                sample = gello.read()
                if follower:
                    state = follower.update(sample["closure"], sample["gello_read_done_s"], time.monotonic())
                else:
                    state = gripper.request("status")
                    ready(state)
                done = time.monotonic()
                row = dict(event="sample", **sample, **state, loop_done_s=done)
                emit(row)
                rows += 1
                if done >= next_print:
                    print(f"{done-start:5.1f}s  GELLO {100*sample['closure']:5.1f}% closed  "
                          f"gPR={state['requested_position']:3d} gPO={state['position']:3d} "
                          f"fault={state['fault']}  read→feedback="
                          f"{1000*(done-sample['gello_read_start_s']):.1f} ms", flush=True)
                    next_print = done + 1
                next_tick += 1 / args.hz
                if next_tick < done:
                    next_tick = done  # Do not queue or replay stale leader commands.
                time.sleep(max(0, next_tick - time.monotonic()))
        except KeyboardInterrupt:
            interrupted = True
        except Exception as exc:
            failed = True
            emit(dict(event="error", error=str(exc)))
            print(f"Stopped: {exc}", file=sys.stderr, flush=True)
        finally:
            if follower:
                try:
                    follower.close()
                    emit(dict(event="stop_acknowledged", state=gripper.request("status")))
                except Exception as exc:
                    failed = True
                    emit(dict(event="stop_error", error=str(exc)))
                    print(f"Stop acknowledgement unavailable: {exc}", file=sys.stderr)
            try:
                if gripper:
                    gripper.close()
            finally:
                if gello:
                    gello.close()
            emit(dict(event="finished", samples=rows, failed=failed, interrupted=interrupted))
    print(f"Finished: {rows} samples. Log: {path}", flush=True)
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
