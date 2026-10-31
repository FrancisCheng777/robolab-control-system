"""Exercise the real C++ driver and vendor SDK against a pseudo-terminal gripper."""
import json
import os
from pathlib import Path
import pty
import select
import struct
import subprocess
import threading
import unittest

TOOL = Path(__file__).resolve().parents[1] / "build/pi05_gripper_tool"


def crc(data):
    value = 0xffff
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ 0xa001 if value & 1 else value >> 1
    return value


class Device:
    def __init__(self):
        self.master, self.slave = pty.openpty()
        self.path = os.ttyname(self.slave)
        self.stopped = threading.Event()
        self.writes = []
        self.read_counts = []
        self.fault = 0
        self.clear_idle_fault_after_read = False
        self.thread = threading.Thread(target=self.serve, daemon=True)
        self.thread.start()

    def serve(self):
        buffer = bytearray()
        position, request, action = 3, 0, 1
        while not self.stopped.is_set():
            if not select.select([self.master], [], [], .02)[0]:
                continue
            try:
                buffer.extend(os.read(self.master, 4096))
            except OSError:
                return
            while len(buffer) >= 8:
                length = 9 + buffer[6] if buffer[1] == 16 else 8
                if len(buffer) < length:
                    break
                packet = bytes(buffer[:length])
                del buffer[:length]
                if crc(packet[:-2]) != int.from_bytes(packet[-2:], "little"):
                    continue
                if packet[1] == 16:
                    data = packet[7:7 + packet[6]]
                    action, request = data[0], data[3]
                    if action & 8:
                        position = request
                    self.writes.append(dict(action=action, position=request, data=data))
                    reply = packet[:6]
                else:
                    status = 0xf9 if action & 8 else 0x31
                    count = int.from_bytes(packet[4:6], "big")
                    self.read_counts.append(count)
                    reply = bytes([9, packet[1], count * 2, status, 0, self.fault, request, position, 0]) + bytes(count * 2 - 6)
                    if self.fault == 9 and self.clear_idle_fault_after_read:
                        self.fault = 0
                os.write(self.master, reply + struct.pack("<H", crc(reply)))

    def close(self):
        self.stopped.set()
        self.thread.join(timeout=1)
        os.close(self.slave)
        os.close(self.master)


@unittest.skipUnless(TOOL.exists(), "native gripper tool is not built")
class NativeTests(unittest.TestCase):
    def setUp(self):
        self.device = Device()
        self.process = subprocess.Popen([str(TOOL), "--device", self.device.path],
                                        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE, text=True)

    def tearDown(self):
        if not self.process.stdin.closed:
            self.process.stdin.close()
        self.process.wait(timeout=2)
        self.process.stdout.close()
        self.process.stderr.close()
        self.device.close()

    def request(self, op, **values):
        self.process.stdin.write(json.dumps(dict(op=op, **values)) + "\n")
        self.process.stdin.flush()
        self.assertTrue(select.select([self.process.stdout], [], [], 1)[0], "reply timeout")
        return json.loads(self.process.stdout.readline())

    def test_read_only_sends_no_motor_commands(self):
        state = self.request("status")
        self.assertEqual(state["position"], 3)
        self.assertEqual(state["requested_position"], 0)
        self.assertFalse(self.device.writes)

    def test_watchdog_stops_after_controller_stalls(self):
        self.request("status")
        state = self.request("move", position=64, speed=64, force=16)
        self.assertEqual(state["requested_position"], 64)
        self.assertGreaterEqual(state["command_ack_s"], state["command_send_s"])
        self.process.wait(timeout=2)
        self.assertNotEqual(self.process.returncode, 0)
        self.assertEqual(self.device.writes[-1]["action"], 1)

    def test_eof_stops_motion(self):
        self.request("status")
        self.request("move", position=64, speed=64, force=16)
        self.process.stdin.close()
        self.process.wait(timeout=2)
        self.assertEqual(self.process.returncode, 0)
        self.assertEqual(self.device.writes[-1]["action"], 1)

    def test_command_registers_and_reserved_bytes(self):
        self.request("status")
        self.request("move", position=64, speed=64, force=16)
        self.assertTrue(all(count == 3 for count in self.device.read_counts))
        data = self.device.writes[0]["data"]
        self.assertEqual(data, bytes([9, 0, 0, 64, 64, 16]))

    def test_move_without_fresh_status_is_rejected(self):
        self.assertIn("error", self.request("move", position=64, speed=64, force=16))
        self.assertFalse(self.device.writes)

    def test_second_serial_owner_is_rejected(self):
        self.request("status")
        second = subprocess.run([str(TOOL), "--device", self.device.path],
                                input='{"op":"status"}\n', capture_output=True, text=True, timeout=2)
        self.assertNotEqual(second.returncode, 0)
        self.assertIn("connect failed", second.stdout)


@unittest.skipUnless((TOOL.parent / "gripper_activation_probe").exists(), "activation probe not built")
class ActivationTests(unittest.TestCase):
    def check_startup(self, fault=0, clear_idle=False, success=True):
        device = Device()
        try:
            device.fault = fault
            device.clear_idle_fault_after_read = clear_idle
            result = subprocess.run([str(TOOL.parent / "gripper_activation_probe"), device.path],
                                    capture_output=True, text=True, timeout=2)
            self.assertEqual(result.returncode == 0, success, result.stderr)
            self.assertFalse(device.writes, "startup must not reset or move an already active/faulted gripper")
        finally:
            device.close()

    def test_ready_gripper_is_not_reset_or_moved(self):
        self.check_startup()

    def test_idle_communication_flag_clears_without_motor_commands(self):
        self.check_startup(fault=9, clear_idle=True)

    def test_actual_fault_is_not_automatically_reset(self):
        self.check_startup(fault=14, success=False)
