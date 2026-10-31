"""F2 collection sessions: explicit startup home, held standby, aligned takeover."""
import json
import math
import os
import select
import time

import numpy as np

from .contract import encode, validate_target, validate_frames, vector
from .episode import EpisodeWriter


class TeleopCounter:
    """In-memory count only; no images, synthetic observations, or dataset file."""
    robolab = True
    def __init__(self): self.count = 0
    def close_incomplete(self): pass


class Keys:
    """Decode terminal F2/Enter, including escape sequences split across reads."""
    sequences = (b"\x1bOQ", b"\x1b[12~", b"\x1b[[B")

    def __init__(self):
        self.pending = b""

    def feed(self, data):
        self.pending += data
        keys = []
        while self.pending:
            match = next((s for s in self.sequences if self.pending.startswith(s)), None)
            if match is not None:
                keys.append("toggle")
                self.pending = self.pending[len(match):]
            elif any(s.startswith(self.pending) for s in self.sequences):
                break
            elif self.pending[0] == 27:
                # Drop unknown terminal function keys rather than treating their
                # trailing letters (notably 'B') as a command.
                if len(self.pending) == 1:
                    break
                end = next((i for i in range(2, len(self.pending))
                            if 64 <= self.pending[i] <= 126), None)
                if end is None and len(self.pending) < 32:
                    break
                self.pending = self.pending[(end + 1) if end is not None else 1:]
            else:
                char = chr(self.pending[0]).lower()
                self.pending = self.pending[1:]
                if char in "\r\n":
                    keys.append("toggle")
                elif char in "sfaciq":
                    keys.append(char)
        return keys

    def read(self, fd):
        if not select.select([fd], [], [], 0)[0]:
            return []
        data = os.read(fd, 256)
        if not data:
            raise EOFError("collection terminal disconnected")
        return self.feed(data)


class HomePath:
    """Rest-to-rest quintic reference; the bridge independently limits motion."""
    def __init__(self, start, goal, now, *, max_velocity=.05):
        self.start = vector(start, 7, "home start").copy()
        self.goal = vector(goal, 7, "home goal").copy()
        validate_target(self.start)
        validate_target(self.goal)
        self.delta = self.goal - self.start
        distance = float(np.max(np.abs(self.delta)))
        if not np.isfinite(max_velocity) or not 0 < max_velocity <= .2:
            raise ValueError("home reference velocity must be in (0, 0.2] rad/s")
        # Configurable peak speed; retain 0.10 rad/s² and 0.5 rad/s³ bounds.
        # The RoboLab bridge independently caps each reference update.
        self.duration = max(1., 1.875 * distance / max_velocity,
                            np.sqrt(5.774 * distance / .10),
                            (60 * distance / .5) ** (1/3))
        self.started = self.last = float(now)

    def sample(self, now):
        if not np.isfinite(now) or now < self.last:
            raise ValueError("invalid home clock")
        if now - self.last > .2:
            raise RuntimeError("home update gap exceeded 200 ms")
        self.last = now
        u = min(1., (now - self.started) / self.duration)
        return self.start + (10*u**3 - 15*u**4 + 6*u**5) * self.delta


def record_frame(bridge, cameras, leader, writer, *, critical=False,
                 intervention=False, require_alignment=False, record=True, state=None):
    state = bridge.state() if state is None else state
    received = time.monotonic()
    q = np.asarray(state["q"], dtype=float)
    if record:
        frames, metadata = cameras.capture_with_metadata()
    target, width = leader.read()
    read_time = time.monotonic()
    validate_target(target)
    if not writer.robolab: encode(q, target, width)
    if require_alignment and abs(width - state["gripper_width"]) > .005:
        raise ValueError("align GELLO gripper to within 5 mm before takeover")
    stamp = time.monotonic()
    if record:
        validate_frames(frames, stamp)
    state_age = stamp - state["state_bridge_receive_s"]
    gripper_age = stamp - state["gripper_bridge_read_s"]
    if state_age > .05 or gripper_age > .5:
        raise RuntimeError("feedback expired during input collection")
    reply = bridge.command(state, target, width)
    if not record:
        return
    timing = {"state_bridge_receive_s": state["state_bridge_receive_s"],
              "state_client_receive_s": received,
              "gripper_bridge_read_s": state["gripper_bridge_read_s"],
              "gello_read_s": read_time,
              **{k: reply[k] for k in
                 ("command_send_s", "command_enqueue_s", "command_ack_s")}}
    writer.append(q=q, target=target, width=state["gripper_width"], command_width=width,
                  frames=frames, timestamp=stamp, state_sequence=state["state_sequence"],
                  command_sequence=bridge.seq, ack=reply.get("ok"), critical=critical,
                  intervention=intervention, state_age=state_age, gripper_age=gripper_age,
                  timing=timing, camera_metadata=metadata,
                  gello_unclipped_target=getattr(leader,"last_unclipped_target",None))


class CollectionSession:
    def __init__(self, bridge, cameras, leader, config, output, session, *, mock,
                 announce=None):
        state = bridge.state()
        if state.get("session_control_version") != 1:
            raise ValueError("rebuild the bridge: F2 session control is unavailable")
        validate_target(config["arm_home"])
        self.bridge, self.cameras, self.leader = bridge, cameras, leader
        self.config, self.output, self.session = config, output, session
        self.mock = mock
        self.recording_enabled = config.get('recording_enabled', True)
        self.fast_control = 'teleop_hz' in config
        if not self.recording_enabled and not self.fast_control:
            raise ValueError('teleop-only mode requires explicit teleop_hz')
        if self.fast_control:
            from .control_timing import validate_rates
            validate_rates(config)
        self.record_period = 1 / config.get('control_hz', 15)
        self.next_record_at = None
        self.control_intervals = []
        self.last_control_at = None
        self.robolab = config.get("teleop_controller")=="robolab_torque"
        if self.robolab != (state["calibration"].get("teleop_controller")=="robolab_torque"):
            raise ValueError("collector and bridge controller modes differ")
        for expected, field in [('expected_teleop_velocity_feedforward', 'teleop_velocity_feedforward'),
                                ('expected_reference_lead_limit_rad', 'teleop_reference_lead_limit_rad'),
                                ('expected_max_velocity', 'max_velocity'),
                                ('expected_teleop_joint_kp', 'teleop_joint_kp')]:
            if expected in config and state['calibration'].get(field, 0.) != config[expected]:
                raise ValueError(f'bridge profile mismatch: {field}')
        self.announce = announce or (lambda event, **data:
                                    print(json.dumps({"event": event, **data}), flush=True))
        self.phase = "init"
        self.writer = None
        self.home = None
        self.critical = self.intervention = False
        self.pending_outcome = None
        self.after_hold = "standby"
        self.hold_sequence = 0
        self.settling_started = None
        self.first_frame = False
        self.last_alignment_display = 0.
        self.last_home_report = 0.
        self.closed = False
        self.completed_episodes = 0
        bridge.claim()
        self.announce('control_profile', profile=config.get('latency_profile', 'unspecified'),
                      teleop_hz=config.get('teleop_hz', config.get('control_hz', 15)),
                      feedforward_gain=state['calibration'].get('teleop_velocity_feedforward', 0.),
                      reference_lead_limit_rad=state['calibration'].get('teleop_reference_lead_limit_rad', 0.),
                      max_velocity_rad_s=state['calibration'].get('max_velocity'),
                      teleop_joint_kp=state['calibration'].get('teleop_joint_kp', [120,120,120,120,120,150,60]),
                      recording_enabled=self.recording_enabled)
        self.announce("init", message="F2/Enter: startup home; q/Ctrl-C: exit. Clear the home path first.")

    @staticmethod
    def held(state):
        return state.get("hold_ready", False) and np.max(np.abs(state["dq"])) <= .01

    def _settle(self, state, after, outcome=None):
        self.bridge.hold(state)
        self.hold_sequence = self.bridge.seq
        self.after_hold, self.pending_outcome = after, outcome
        self.settling_started = time.monotonic()
        self.phase = "settling"
        self.announce("settling")

    def _finish(self, outcome, reason="operator label"):
        if not self.recording_enabled:
            updates = self.writer.count
            self.writer = None
            self.completed_episodes += 1
            self.pending_outcome = None
            self.phase = 'standby'
            self.announce('teleop_finished', updates=updates, recorded=False)
            if self.control_intervals:
                self.announce('control_timing', samples=len(self.control_intervals),
                              interval_ms_p50_p95_p99=np.percentile(self.control_intervals, [50,95,99]).tolist())
            return
        if self.writer.count == 0:
            outcome, reason = "aborted", "empty trial"
        if hasattr(self.writer, 'begin_finish') and not self.closed:
            self.writer.begin_finish(outcome, reason)
            self.saving_outcome = outcome
            self.saving_started = time.monotonic()
            self.phase = 'saving'
            self.announce('saving', message='Arm held; waiting for recording storage.')
            return
        # A previously accepted human label remains authoritative if the user
        # exits while it is draining. A still-recording episode is aborted.
        if self.phase == 'saving':
            outcome = self.saving_outcome
        path = self.writer.finish(outcome, reason)
        self._saved(path, outcome)

    def _saved(self, path, outcome):
        count = self.writer.count
        if self.fast_control and self.control_intervals:
            self.announce('control_timing', samples=len(self.control_intervals),
                          interval_ms_p50_p95_p99=np.percentile(self.control_intervals, [50, 95, 99]).tolist(),
                          recording_queue_peak=getattr(self.writer, 'high_watermark', 0))
        self.writer = None
        self.completed_episodes += 1
        self.phase = "standby"
        self.pending_outcome = None
        self.announce("saved", episode=str(path), outcome=outcome, frames=count)
        if not self.closed:
            self.announce("standby", message="Align GELLO arm and gripper; F2/Enter starts the next trial.")

    def tick(self, keys=()):
        if self.closed:
            raise RuntimeError("session is closed")
        if "q" in keys:
            self.close("operator exit")
            return False
        state = self.bridge.state()
        if self.phase == "recording":
            ending = next((k for k in keys if k in ("toggle", "s", "f", "a")), None)
            if ending is not None:
                outcome = {"s": "success", "f": "failure", "a": "aborted"}.get(ending)
                self._settle(state, "label", outcome if self.recording_enabled else "aborted")
                return True
            for key in keys:
                if key == "c":
                    self.critical = not self.critical
                    self.announce("critical", value=self.critical)
                elif key == "i":
                    self.intervention = not self.intervention
                    self.announce("intervention", value=self.intervention)
            now = time.monotonic()
            record = True
            if self.fast_control:
                if hasattr(self.writer, 'check'):
                    self.writer.check()
                if self.last_control_at is not None:
                    self.control_intervals.append(1000*(now-self.last_control_at))
                    # Bounded diagnostic history, about 10 minutes at 100 Hz.
                    if len(self.control_intervals) > 60000:
                        del self.control_intervals[:10000]
                self.last_control_at = now
                if self.next_record_at is None:
                    self.next_record_at = now
                record = now + 1e-9 >= self.next_record_at
                if record:
                    missed = max(0, math.floor((now-self.next_record_at)/self.record_period))
                    self.next_record_at += (missed+1)*self.record_period
            record_frame(self.bridge, self.cameras, self.leader, self.writer,
                         critical=self.critical, intervention=self.intervention,
                         require_alignment=self.first_frame, record=record and self.recording_enabled,
                         state=state if self.fast_control else None)
            if not self.recording_enabled:
                self.writer.count += 1
            self.first_frame = False
            return True
        if self.phase == 'saving':
            self.bridge.hold(state)
            path = self.writer.poll_finished()
            if path is not None:
                self._saved(path, self.saving_outcome)
            elif time.monotonic() - self.saving_started > 5:
                raise TimeoutError('episode storage failed to drain within 5 seconds')
            return True
        if self.phase == "homing":
            if "a" in keys:
                self.close("startup home cancelled")
                return False
            now = time.monotonic()
            target = self.home.sample(now)
            if not self.robolab: encode(state["q"], target, state["gripper_width"])
            # RoboLab torque PD uses a 0.05-rad home acceptance threshold.
            # Preserve the older position controller's 0.005-rad threshold.
            tolerance = .05 if self.robolab else .005
            errors = np.asarray(state["q"]) - self.home.goal
            max_error = float(np.max(np.abs(errors)))
            max_speed = float(np.max(np.abs(state["dq"])))
            if now - self.last_home_report >= 1:
                self.announce("home_progress", joint_error_rad=errors.tolist(),
                              max_error_rad=max_error, tolerance_rad=tolerance,
                              max_speed_rad_s=max_speed, elapsed_s=now-self.home.started)
                self.last_home_report = now
            if now - self.home.started > self.home.duration + 10:
                raise RuntimeError(f"startup home failed to settle: max error {max_error:.5f} rad "
                                   f"(limit {tolerance:.5f}), max speed {max_speed:.5f} rad/s (limit 0.01)")
            if (now - self.home.started >= self.home.duration
                    and max_error <= tolerance and max_speed <= .01):
                self._settle(state, "opening_gripper" if self.config.get("open_gripper_after_home", False) else "standby")
            else:
                self.bridge.command(state, target, state["gripper_width"], motion_profile="home")
            return True

        if self.phase == "opening_gripper":
            if "a" in keys:
                self.close("startup gripper opening cancelled")
                return False
            if time.monotonic() - self.gripper_open_started > 10:
                raise RuntimeError("startup gripper failed to open within 10 s")
            width = state["calibration"]["gripper_max_width"]
            if abs(state["gripper_width"] - width) <= .005:
                self._settle(state, "standby")
            else:
                # Home mode deliberately freezes the gripper in the bridge.
                # Use Track with the latched stationary arm pose to open it;
                # this phase never reads a GELLO target or records an episode.
                self.bridge.command(state, self.gripper_open_q, width, motion_profile="teleop")
            return True

        # Keep watchdog and ownership active while held. No GELLO targets are
        # sent and no camera frames are recorded in these phases.
        self.bridge.hold(state)
        if self.phase == "settling":
            if self.after_hold == "label":
                label = next((k for k in keys if k in ("s", "f", "a")), None)
                if label is not None:
                    self.pending_outcome = {"s": "success", "f": "failure", "a": "aborted"}[label]
            if time.monotonic() - self.settling_started > 10:
                raise RuntimeError("arm/gripper did not enter hold")
            if self.held(state) and state["applied_sequence"] >= self.hold_sequence:
                self.phase = self.after_hold
                if self.phase == "opening_gripper":
                    self.gripper_open_q = np.asarray(state["q"], dtype=float).copy()
                    self.gripper_open_started = time.monotonic()
                if self.phase == "label" and self.pending_outcome is not None:
                    self._finish(self.pending_outcome)
                else:
                    self.announce(self.phase, message=("s=success f=failure a=discard; arm is held."
                                  if self.phase == "label" else "Align GELLO; F2/Enter starts recording."))
        elif self.phase == "init":
            if "toggle" in keys:
                if not self.held(state):
                    self.announce("waiting_for_hold", message="Wait for hold, then press F2/Enter again.")
                else:
                    self.home = HomePath(state["q"], self.config["arm_home"], time.monotonic(),
                                         max_velocity=self.config.get("home_reference_velocity", .05) if self.robolab else .05)
                    self.phase = "homing"
                    self.announce("homing", duration_s=self.home.duration, open_gripper=bool(self.config.get("open_gripper_after_home", False)))
        elif self.phase == "label":
            outcome = next((k for k in keys if k in ("s", "f", "a")), None)
            if outcome:
                self._finish({"s": "success", "f": "failure", "a": "aborted"}[outcome])
        elif self.phase == "standby":
            error, width_error = None, None
            target, width = None, None
            try:
                target, width = getattr(self.leader, "read_for_alignment", self.leader.read)()
                error = np.asarray(target) - np.asarray(state["q"])
                width_error = width - state["gripper_width"]
                validate_target(target)
                if not self.robolab: encode(state["q"], target, width)
                aligned = bool(self.held(state) and (self.robolab or abs(width_error) <= .005))
                detail = "" if aligned else "align gripper to within 5 mm / wait for hold"
            except ValueError as exc:
                aligned, detail = False, str(exc)
            if "toggle" in keys or time.monotonic() - self.last_alignment_display >= 1:
                self.announce("alignment", ready=aligned, detail=detail, robolab=self.robolab,
                              joint_error_degrees=None if error is None else np.degrees(error).tolist(),
                              gripper_error_mm=None if width_error is None else width_error * 1000,
                              leader_joint_degrees=None if target is None else np.degrees(target).tolist(),
                              robot_joint_degrees=np.degrees(state["q"]).tolist(),
                              leader_width_mm=None if width is None else width * 1000,
                              robot_width_mm=state["gripper_width"] * 1000)
                self.last_alignment_display = time.monotonic()
            if "toggle" in keys and aligned:
                writer_type = EpisodeWriter
                if self.fast_control:
                    from .async_episode import AsyncEpisodeWriter
                    writer_type = AsyncEpisodeWriter
                self.next_record_at = self.last_control_at = None
                self.control_intervals = []
                if self.recording_enabled:
                    self.writer = writer_type(self.output, mock=self.mock, task=self.config["task"],
                        session=self.session, config={**self.config, "bridge_calibration": state["calibration"]})
                else:
                    self.writer = TeleopCounter()
                self.critical = self.intervention = False
                self.first_frame = not self.robolab
                self.phase = "recording"
                self.announce("recording", teleop_only=not self.recording_enabled, message="F2/Enter: stop following." if not self.recording_enabled else "F2/Enter: stop then label; s/f/a: stop with label.")
        return True

    def close(self, reason="session closed"):
        if self.closed:
            return
        self.closed = True
        try:
            self.bridge.close()  # stop before any potentially blocking file finalization
        finally:
            if self.writer is not None:
                try:
                    self._finish("aborted", reason)
                finally:
                    if self.writer is not None:
                        self.writer.close_incomplete()
            self.phase = "closed"
