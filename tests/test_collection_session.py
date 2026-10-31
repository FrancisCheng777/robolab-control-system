"""Session behavior against an isolated C++ mock; never opens hardware."""
import contextlib
import json
import os
from pathlib import Path
import pty
import select
import subprocess
import sys
import tempfile
import time
import unittest
import uuid

import numpy as np
import h5py

from pi05_demo.transport import Bridge
from pi05_demo.cameras import SyntheticPair
from pi05_demo.episode import inspect_episode
from pi05_demo.session import CollectionSession, HomePath, Keys

ROOT = Path(__file__).resolve().parents[1]


@contextlib.contextmanager
def mock_bridge(robolab=False, config_overrides=None, current_profile=False):
    socket = f"/tmp/pi05-f2-test-{uuid.uuid4().hex[:12]}.sock"
    with tempfile.TemporaryFile(mode="w+") as log, tempfile.TemporaryDirectory() as config_dir:
        config_path=ROOT / "configs/bridge.mock.json"
        if robolab:
            cfg=json.loads(config_path.read_text());cfg["teleop_controller"]="robolab_torque"
            if current_profile:
                cfg=json.loads((ROOT / "configs/bridge.robolab.mock.json").read_text())
            cfg.update(config_overrides or {})
            config_path=Path(config_dir)/"bridge.json";config_path.write_text(json.dumps(cfg))
        process = subprocess.Popen([
            str(ROOT / "build/pi05_franka_bridge"), "--mock", "--config",
            str(config_path), "--socket", socket,
        ], stdout=log, stderr=log)
        bridge = None
        try:
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    log.seek(0)
                    raise RuntimeError(log.read())
                try:
                    bridge = Bridge(socket, mock=True)
                    break
                except (OSError, RuntimeError):
                    time.sleep(.005)
            if bridge is None:
                raise TimeoutError("mock bridge did not become ready")
            yield bridge
        finally:
            if bridge is not None:
                bridge.close()
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


class CollectionSessionTests(unittest.TestCase):
    def config(self):
        return json.loads((ROOT / "configs/collection.json").read_text())

    def test_current_profile_completes_one_radian_home_without_recording(self):
        with mock_bridge(robolab=True, current_profile=True) as bridge, tempfile.TemporaryDirectory() as output:
            cfg=json.loads((ROOT / "configs/collection.robolab.json").read_text())
            goal=np.array(bridge.state()["q"])
            goal[0]+=1.
            cfg["arm_home"]=goal.tolist()
            session=CollectionSession(bridge, None, None, cfg, output, "fast-home", mock=True,
                                      auto_home=True, announce=lambda *a, **k:None)
            peak_speed=0.
            try:
                deadline=time.monotonic()+6
                while session.phase!="standby" and time.monotonic()<deadline:
                    session.tick()
                    state=bridge.state()
                    peak_speed=max(peak_speed,float(np.max(np.abs(state["dq"]))))
                    time.sleep(.02)
                self.assertEqual(session.phase,"standby")
                self.assertLess(session.home.duration,3.)
                self.assertGreater(peak_speed,.35)
                self.assertLessEqual(peak_speed,.700001)
                np.testing.assert_allclose(state["q"],goal,atol=.005)
                self.assertIsNone(session.writer)
                self.assertFalse(list(Path(output).glob("*.h5")))
            finally:
                session.close()

    def test_startup_opens_initially_closed_gripper_after_home(self):
        # Exercise the real C++ command-mode gate, not a Python bridge stub.
        class UnusedLeader:
            def read(self):
                raise AssertionError("startup opening must not follow GELLO")

        with mock_bridge(robolab=True) as bridge, tempfile.TemporaryDirectory() as output:
            bridge.claim()
            state = bridge.state()
            fixed_q = np.array(state["q"])
            deadline = time.monotonic() + 1
            while True:
                bridge.command(state, fixed_q, .025)
                time.sleep(.01)
                state = bridge.state()
                if abs(state["gripper_width"] - .025) < .001:
                    break
                self.assertLess(time.monotonic(), deadline, "could not seed closed gripper")
            cfg = self.config()
            cfg.update(teleop_controller="robolab_torque", arm_home=fixed_q.tolist(),
                       open_gripper_after_home=True, home_reference_velocity=.2)
            events = []
            session = CollectionSession(bridge, SyntheticPair(), UnusedLeader(), cfg,
                                        output, "startup-open", mock=True,
                                        announce=lambda event, **data: events.append(event))
            try:
                deadline = time.monotonic() + 1
                while not session.held(bridge.state()):
                    session.tick()
                    time.sleep(.01)
                    self.assertLess(time.monotonic(), deadline)
                session.tick(["toggle"])
                deadline = time.monotonic() + 4
                while session.phase != "standby" and time.monotonic() < deadline:
                    session.tick()
                    state = bridge.state()
                    np.testing.assert_allclose(state["q"], fixed_q, atol=1e-8)
                    if session.phase == "homing":
                        self.assertAlmostEqual(state["gripper_width"], .025)
                    time.sleep(.01)
                self.assertEqual(session.phase, "standby",
                                 f"gripper stayed at {state['gripper_width']:.4f} m")
                self.assertIn("opening_gripper", events)
                self.assertAlmostEqual(state["gripper_width"], .085)
                self.assertIsNone(session.writer)
                self.assertFalse(list(Path(output).glob("*.h5")))
            finally:
                session.close("regression check complete")

    def test_default_home_is_the_visually_accepted_droid_pose(self):
        config = json.loads((ROOT / "configs/collection.json").read_text())
        home = np.array([0., -np.pi/5, 0., -4*np.pi/5, 0., 3*np.pi/5, 0.])
        np.testing.assert_allclose(config["arm_home"], home, atol=1e-14)
        leader = home.copy()
        leader[6] = np.pi/4
        np.testing.assert_allclose(config["gello_reference_joints_rad"], leader, atol=1e-14)

    def test_bridge_has_a_resumable_hold_distinct_from_shutdown(self):
        with mock_bridge() as bridge:
            bridge.claim()
            state = bridge.state()
            reply = bridge.request({
                "op": "hold", "session": bridge.session, "controller_id": bridge.owner,
                "sequence": 1, "state_sequence": state["state_sequence"],
            })
            self.assertTrue(reply["ok"])
            bridge.seq = 1
            deadline = time.monotonic() + 1
            while time.monotonic() < deadline:
                state = bridge.state()
                if state.get("hold_ready"):
                    break
                time.sleep(.005)
            self.assertTrue(state["hold_ready"])
            self.assertFalse(state["stopped"])
            bridge.command(state, np.array(state["q"]) + .005, state["gripper_width"])

    def test_terminal_f2_is_decoded_across_reads_and_b_is_not_a_command(self):
        for sequence in Keys.sequences:
            reader = Keys()
            self.assertEqual(reader.feed(sequence[:1]), [])
            self.assertEqual(reader.feed(sequence[1:2]), [])
            self.assertEqual(reader.feed(sequence[2:]), ["toggle"])
        self.assertEqual(Keys().feed(b"bB\x1b[A\rsfaq"), ["toggle", "s", "f", "a", "q"])

    def test_home_reference_is_bounded_and_rejects_timing_gaps(self):
        goal = np.array(self.config()["arm_home"])
        path = HomePath(goal + .1, goal, 0.)
        times = np.arange(0., path.duration, .01)
        positions = np.array([path.sample(float(t)) for t in times])
        velocity = np.diff(positions, axis=0) / .01
        acceleration = np.diff(velocity, axis=0) / .01
        jerk = np.diff(acceleration, axis=0) / .01
        self.assertLessEqual(np.max(np.abs(velocity)), .05001)
        self.assertLessEqual(np.max(np.abs(acceleration)), .10001)
        self.assertLessEqual(np.max(np.abs(jerk)), .5001)
        np.testing.assert_allclose(path.sample(path.duration), goal)
        with self.assertRaisesRegex(RuntimeError, "gap"):
            path.sample(path.duration + .3)
        with self.assertRaisesRegex(ValueError, "clock"):
            path.sample(-1)

    def test_faster_home_keeps_smooth_acceleration_and_jerk(self):
        goal = np.array(self.config()["arm_home"])
        old = HomePath(goal + 1., goal, 0.)
        fast = HomePath(goal + 1., goal, 0., max_velocity=.15)
        self.assertAlmostEqual(old.duration / fast.duration, 3.)
        dt = .002
        positions = np.array([fast.sample(float(t)) for t in np.arange(0., fast.duration, dt)])
        velocity = np.diff(positions, axis=0) / dt
        acceleration = np.diff(velocity, axis=0) / dt
        jerk = np.diff(acceleration, axis=0) / dt
        self.assertLessEqual(np.max(np.abs(velocity)), .15001)
        self.assertLessEqual(np.max(np.abs(acceleration)), .10001)
        self.assertLessEqual(np.max(np.abs(jerk)), .5001)
        for invalid in (0., -.1, float('nan'), .71):
            with self.assertRaises(ValueError):
                HomePath(goal, goal, 0., max_velocity=invalid)

    def test_quicker_home_has_bounded_velocity_acceleration_and_jerk(self):
        goal = np.array(self.config()["arm_home"])
        old = HomePath(goal + 1., goal, 0., max_velocity=.2)
        fast = HomePath(goal + 1., goal, 0., max_velocity=.7, max_acceleration=.8, max_jerk=3.)
        self.assertLess(fast.duration, 3.)
        self.assertLess(fast.duration, old.duration * .3)
        dt = .002
        positions = np.array([fast.sample(float(t)) for t in np.arange(0., fast.duration, dt)])
        velocity = np.diff(positions, axis=0) / dt
        acceleration = np.diff(velocity, axis=0) / dt
        jerk = np.diff(acceleration, axis=0) / dt
        self.assertLessEqual(np.max(np.abs(velocity)), .70001)
        self.assertLessEqual(np.max(np.abs(acceleration)), .80001)
        self.assertLessEqual(np.max(np.abs(jerk)), 3.0001)
        for invalid in (0., -.1, float('nan'), .81):
            with self.assertRaises(ValueError):
                HomePath(goal, goal, 0., max_acceleration=invalid)

        for invalid in (0., -.1, float('nan'), 3.01):
            with self.assertRaises(ValueError):
                HomePath(goal, goal, 0., max_jerk=invalid)
        distant = goal.copy()
        distant[0] += 2.
        long_path = HomePath(distant, goal, 0., max_velocity=.7, max_acceleration=.8, max_jerk=3.)
        self.assertGreater(long_path.duration, 3.)  # Never force a long move into 3 seconds.

    def test_f2_homes_once_rechecks_alignment_and_excludes_idle_frames(self):
        class Leader:
            def __init__(self, state):
                self.q = np.array(state["q"])
                self.width = state["gripper_width"]
                self.reads = 0
            def read(self):
                self.reads += 1
                return self.q.copy(), self.width

        with mock_bridge() as bridge, tempfile.TemporaryDirectory() as output:
            config = self.config()
            original = bridge.state()
            # A short, legal test home makes this independent of hardware pose.
            config["arm_home"] = (np.array(original["q"]) + .012).tolist()
            leader = Leader(original)
            events = []
            session = CollectionSession(bridge, SyntheticPair(), leader, config,
                                        output, "mock_session", mock=True,
                                        announce=lambda event, **data: events.append((event, data)))
            def tick(keys=()):
                session.tick(keys)
                time.sleep(.04)
            def until(predicate, timeout=5):
                deadline = time.monotonic() + timeout
                while not predicate() and time.monotonic() < deadline:
                    tick()
                self.assertTrue(predicate(), (session.phase, events[-4:]))
            try:
                tick()
                until(lambda: bridge.state()["hold_ready"])
                tick(["toggle"])
                self.assertEqual(session.phase, "homing")
                tick(["toggle"])  # repeated F2 never restarts the home path
                until(lambda: session.phase == "standby")
                self.assertEqual(leader.reads, 0)
                self.assertEqual(list(Path(output).glob("*")), [])
                np.testing.assert_allclose(bridge.state()["q"], config["arm_home"], atol=.005)
                self.assertEqual(bridge.state()["gripper_width"], original["gripper_width"])

                leader.q = np.array(bridge.state()["q"]) + .2
                tick(["toggle"])
                self.assertEqual(session.phase, "standby")
                leader.q = np.array(bridge.state()["q"])
                leader.width -= .02
                tick(["toggle"])
                self.assertEqual(session.phase, "standby")
                self.assertEqual(list(Path(output).glob("*")), [])

                leader.width = original["gripper_width"]
                tick(["toggle"])
                self.assertEqual(session.phase, "recording")
                for _ in range(5):
                    leader.q += .002
                    tick()
                count = session.writer.count
                tick(["toggle"])
                until(lambda: session.phase == "label")
                held = bridge.state()
                leader.q += .2
                leader.width -= .02
                for _ in range(3):
                    tick()
                self.assertEqual(session.writer.count, count)
                np.testing.assert_allclose(bridge.state()["q"], held["q"], atol=.0005)
                self.assertEqual(bridge.state()["gripper_width"], held["gripper_width"])
                tick(["s"])
                first = list(Path(output).glob("*.h5"))[0]
                tick(["toggle"])
                self.assertEqual(session.phase, "standby")  # recheck on EVERY takeover
                self.assertEqual(sum(e == "homing" for e, _ in events), 1)
                leader.q = np.array(bridge.state()["q"])
                leader.width = bridge.state()["gripper_width"]
                tick(["toggle"])
                for _ in range(3):
                    tick()
                tick(["f"])
                until(lambda: session.phase == "standby")
                files = list(Path(output).glob("*.h5"))
                self.assertEqual(len(files), 2)
                self.assertEqual(sum(e == "homing" for e, _ in events), 1)
                tick(["toggle"])
                self.assertEqual(session.phase, "recording")
                # Moving the leader after the gate but before the first sample
                # must still fail, without sending that target or saving a valid trial.
                leader.q += .2
                with self.assertRaisesRegex(ValueError, "align GELLO"):
                    session.tick()
                session.close("leader moved after takeover gate")
                files = list(Path(output).glob("*.h5"))
                self.assertEqual(len(files), 3)
                # Audit files after shutdown: blocking inspection in this test's
                # control thread must not starve its intentional 250 ms watchdog.
                audited = [inspect_episode(f, allow_mock=True) for f in files]
                self.assertEqual({x["outcome"] for x in audited}, {"success", "failure", "aborted"})
                self.assertTrue(all(x["session"] == "mock_session" for x in audited))
                with h5py.File(first) as f:
                    self.assertEqual(f.attrs["outcome"], "success")
                    self.assertEqual(len(f["q"]), count)
                aborted = [x for x in audited if x["outcome"] == "aborted"]
                self.assertEqual(aborted[0]["frames"], 0)
            finally:
                session.close()

    def test_actual_terminal_f2_collects_labels_and_exits_cleanly(self):
        self.exercise_terminal()

    def test_one_episode_exits_after_label_without_another_key(self):
        self.exercise_terminal(one_episode=True)

    def test_robolab_terminal_homes_records_and_saves_one_episode(self):
        self.exercise_terminal(one_episode=True, robolab=True)

    def test_robolab_continuous_terminal_saves_two_episodes_then_q_exits(self):
        self.exercise_terminal(robolab=True, episodes=2)

    def test_confirmed_startup_homes_without_another_key_then_waits_to_record(self):
        self.exercise_terminal(robolab=True, episodes=2, auto_home=True)

    def exercise_terminal(self, one_episode=False, robolab=False, episodes=1, auto_home=False):
        with mock_bridge(robolab=robolab) as bridge, tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = self.config()
            if robolab:
                config["teleop_controller"]="robolab_torque"
                config["open_gripper_after_home"]=True
            config["arm_home"] = bridge.state()["q"]
            leader = list(config["arm_home"])
            leader[6] += np.pi/4
            config["gello_reference_joints_rad"] = leader
            path = root / "collection.json"
            path.write_text(json.dumps(config))
            master, slave = pty.openpty()
            process = subprocess.Popen([
                sys.executable, "-m", "pi05_demo.collect", "--mock", "--interactive",
                "--config", str(path), "--socket", bridge.path, "--output", str(root / "data"),
                "--session", "terminal_test",
                *(["--one-episode"] if one_episode else []),
                *(["--home-on-start"] if auto_home else []),
            ], cwd=ROOT, stdin=slave, stdout=slave, stderr=slave)
            os.close(slave)
            transcript = ""
            def wait_for(predicate, timeout=5):
                nonlocal transcript
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    if select.select([master], [], [], .01)[0]:
                        try:
                            transcript += os.read(master, 65536).decode(errors="replace")
                        except OSError:
                            break
                    if predicate():
                        return
                    if process.poll() is not None:
                        break
                self.fail(transcript)
            try:
                if not auto_home:
                    wait_for(lambda: '"event": "init"' in transcript and bridge.state()["hold_ready"])
                    os.write(master, b"\x1b[12~")
                wait_for(lambda: '"event": "standby"' in transcript)
                self.assertNotIn('"event": "recording"', transcript)
                if robolab:
                    self.assertIn('"event": "opening_gripper"', transcript)
                    state = bridge.state()
                    self.assertAlmostEqual(state["gripper_width"], state["calibration"]["gripper_max_width"], delta=.005)
                    np.testing.assert_allclose(state["q"], config["arm_home"], atol=.005)
                    self.assertEqual(list((root / "data").glob("*.h5")), [])
                os.write(master, b"b\r")  # b is ignored; Enter aliases F2
                wait_for(lambda: '"event": "recording"' in transcript)
                sequence = bridge.state()["last_sequence"]
                wait_for(lambda: bridge.state()["last_sequence"] >= sequence + 4)
                os.write(master, b"\x1bOQ")
                wait_for(lambda: '"event": "label"' in transcript)
                os.write(master, b"s")
                wait_for(lambda: '"event": "saved"' in transcript)
                for episode_index in range(1, episodes):
                    self.assertIsNone(process.poll(), transcript)
                    os.write(master, b"\r")
                    wait_for(lambda: transcript.count('"event": "recording"') >= episode_index + 1)
                    sequence = bridge.state()["last_sequence"]
                    wait_for(lambda: bridge.state()["last_sequence"] >= sequence + 4)
                    os.write(master, b"s")
                    wait_for(lambda: transcript.count('"event": "saved"') >= episode_index + 1)
                if not one_episode:
                    self.assertIsNone(process.poll(), transcript)
                    os.write(master, b"q")
                self.assertEqual(process.wait(timeout=5), 0, transcript)
                files = list((root / "data").glob("*.h5"))
                self.assertEqual(len(files), episodes)
                info = inspect_episode(files[0], allow_mock=True)
                self.assertEqual(info["outcome"], "success")
                self.assertGreaterEqual(info["frames"], 3)
                self.assertEqual(info["session"], "terminal_test")
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                os.close(master)

    def test_hold_heartbeat_does_not_disable_watchdog_or_allow_fault_rearm(self):
        with mock_bridge() as bridge:
            bridge.claim()
            bridge.hold(bridge.state())
            deadline = time.monotonic() + 1
            while time.monotonic() < deadline:
                state = bridge.request({"op": "state"})
                if state["fault"]:
                    break
                time.sleep(.01)
            self.assertIn("watchdog", state["fault"])
            with self.assertRaisesRegex(RuntimeError, "stopped/unready"):
                bridge.hold(state)


if __name__ == "__main__":
    unittest.main()
