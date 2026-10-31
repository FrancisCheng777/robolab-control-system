"""F2/Enter collection sessions with aligned GELLO takeover; no automatic recovery."""
import argparse,importlib.util,json,sys,termios,time,tty
from pathlib import Path
import numpy as np
from .contract import MAX_WIDTH,encode,validate_target,validate_frames
from .cameras import Pair,SyntheticPair
from .episode import EpisodeWriter
from .transport import Bridge
from .teleop_reference import j7_offset_from_config
from .session import CollectionSession, Keys, record_frame

ROOT=Path(__file__).resolve().parents[1]
class Gello:
    def __init__(self,port,*,j7_offset_rad,robolab=False,max_sample_age_s=None):
        self.max_sample_age_s=max_sample_age_s
        self.robolab=robolab;self.last_unclipped_target=None
        if not np.isfinite(j7_offset_rad):raise ValueError("GELLO J7 offset must be finite")
        self.j7_offset_rad=float(j7_offset_rad)
        sys.path.insert(0,str(ROOT/"vendor"))
        from gello.agents.gello_agent import GelloAgent
        spec=importlib.util.spec_from_file_location("demo_gello_normalizer",ROOT/"vendor/gello_publish_zenoh.py")
        mod=importlib.util.module_from_spec(spec);sys.modules[spec.name]=mod;spec.loader.exec_module(mod)
        self.agent=GelloAgent(port=port);self.normalizer=mod.PandaJointPositionNormalizer()
    def _read(self, *, for_alignment=False):
        if self.max_sample_age_s is not None:
            # Check before returning a cached vendor reading. Never refresh an
            # old leader sample merely because the fast loop read it again.
            stamp = self.agent._robot._driver.last_successful_read_s
            age = time.monotonic() - stamp
            if stamp <= 0 or not 0 <= age <= self.max_sample_age_s:
                raise RuntimeError(f'stale GELLO sample: {age:.3f}s')
        raw=np.asarray(self.agent.act({}),dtype=float).copy()
        if raw.shape!=(8,) or not 0<=raw[7]<=1:raise ValueError("GELLO requires 7 joints + closure in [0,1]")
        # Re-reference only the leader target; robot feedback stays in physical coordinates.
        raw[6]+=self.j7_offset_rad
        action,clipped=self.normalizer.normalize(raw)
        self.last_unclipped_target=self.normalizer._continuous.copy()
        if clipped and not self.robolab:
            if not for_alignment:
                raise ValueError(f"GELLO outside joint limits: {clipped}")
            # Display only: preserve the actual unwrapped angle so the operator
            # can adjust an out-of-range joint. The session still rejects it.
            return self.normalizer._continuous.copy(),float((1-action[7])*MAX_WIDTH)
        return action[:7],float((1-action[7])*MAX_WIDTH)
    def read(self):
        return self._read()
    def read_for_alignment(self):
        return self._read(for_alignment=True)

class MockLeader:
    def __init__(self, bridge):
        self.bridge = bridge

    def read(self):
        state = self.bridge.state()
        return np.asarray(state["q"]) + .0002, float(state["gripper_width"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(ROOT / "configs/collection.json"))
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--mock", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument("--interactive", action="store_true", help="exercise F2 sessions with --mock")
    parser.add_argument("--socket")
    parser.add_argument("--output")
    parser.add_argument("--guide", action="store_true", help="show per-joint operator guidance")
    parser.add_argument("--event-log", help="JSONL events for the operator guide")
    parser.add_argument("--teleop-only", action="store_true", help="interactive control only; no cameras or datasets")
    parser.add_argument("--one-episode", action="store_true", help="exit after one labeled episode")
    parser.add_argument("--home-on-start", action="store_true",
                        help="home after device checks; launcher must obtain startup confirmation first")
    parser.add_argument("--frames", type=int, default=0, help="noninteractive mock length")
    parser.add_argument("--session", required=True, help="physical collection session; train/val split unit")
    parser.add_argument("--mock-outcome", choices=["success", "failure"], default="success")
    args = parser.parse_args()
    config = json.loads(Path(args.config).read_text())
    try:
        shift = j7_offset_from_config(config)
    except ValueError as exc:
        parser.error(str(exc))
    if 'teleop_hz' in config:
        from .control_timing import validate_rates
        validate_rates(config)
    interactive = args.execute or args.interactive
    if args.teleop_only:
        if not interactive or 'teleop_hz' not in config:
            parser.error('--teleop-only requires interactive mode and explicit teleop_hz')
        config['recording_enabled'] = False
    if args.home_on_start and not interactive:
        parser.error("--home-on-start requires an interactive session")
    if args.execute and (args.frames or not config["hardware_verified"]):
        parser.error("real collection needs hardware_verified=true and interactive outcome labels")
    if interactive and not sys.stdin.isatty():
        parser.error("F2 sessions require an interactive terminal")
    if args.interactive and args.frames:
        parser.error("--interactive cannot be combined with --frames")
    if args.mock and not interactive and args.frames < 1:
        parser.error("--mock requires --frames > 0 or --interactive")
    output = args.output or str(ROOT / "data" / ("mock_raw" if args.mock else "raw"))
    bridge = cameras = session = writer = terminal = guide = None
    error = None
    outcome, reason = "aborted", ""
    try:
        cameras = None if args.teleop_only else (SyntheticPair() if args.mock else Pair(config["cameras"]))
        leader = None if args.mock else Gello(config["gello_port"], j7_offset_rad=shift,robolab=config.get("teleop_controller")=="robolab_torque",
            max_sample_age_s=config.get("gello_max_sample_age_s") if "teleop_hz" in config else None)
        # The first vendor read waits ~100 ms for the Dynamixel cache. Warm it
        # before claiming control, so initialization cannot expire robot feedback.
        if leader is not None:
            # Wait only before claiming the bridge. Production reads remain
            # bounded and reject stale cached USB input.
            if leader.max_sample_age_s is not None:
                warm_deadline = time.monotonic() + 3
                while leader.agent._robot._driver.last_successful_read_s <= 0:
                    if time.monotonic() >= warm_deadline:
                        raise RuntimeError('GELLO did not produce a fresh sample')
                    time.sleep(.01)
            leader.read_for_alignment()
        bridge = Bridge(args.socket or config["socket"], mock=args.mock)
        leader = MockLeader(bridge) if args.mock else leader
        from .teleop_runtime import configure_runtime
        runtime_settings = configure_runtime(config)
        period = 1 / config.get("teleop_hz", config["control_hz"])
        if interactive:
            print(f"Panda home (rad): {config['arm_home']}", flush=True)
            print(f"GELLO home reference (rad): {config['gello_reference_joints_rad']}; J7 shift={shift:+.6f}", flush=True)
            if args.guide:
                from .operator_guide import OperatorGuide
                guide = OperatorGuide(args.event_log)
                guide('teleop_runtime', **runtime_settings)
            session = CollectionSession(bridge, cameras, leader, config, output,
                                        args.session, mock=args.mock, announce=guide,
                                        auto_home=args.home_on_start)
            terminal = termios.tcgetattr(sys.stdin)
            if args.home_on_start:
                # Discard keys typed during device startup, never queue recording.
                termios.tcflush(sys.stdin, termios.TCIFLUSH)
            tty.setcbreak(sys.stdin.fileno())
            keys = Keys()
            deadline = time.monotonic()
            while session.tick(keys.read(sys.stdin.fileno())):
                if args.one_episode and session.completed_episodes:
                    print("试动结束，未录制数据。" if args.teleop_only else "本条已保存，单条试采结束。", flush=True)
                    break
                deadline += period
                lag = time.monotonic() - deadline
                if lag > period:
                    if config.get("teleop_controller")=="robolab_torque":
                        deadline=time.monotonic()  # preserve real timestamps; do not emit catch-up bursts
                    else:
                        raise RuntimeError(f"missed control deadline by {lag:.3f}s")
                time.sleep(max(0, deadline - time.monotonic()))
        else:
            period = 1 / config["control_hz"]
            # Preserve the automated mock collector contract used by smoke/export.
            state = bridge.state()
            bridge.claim()
            writer = EpisodeWriter(output, mock=True, task=config["task"], session=args.session,
                                   config={**config, "bridge_calibration": state["calibration"]})
            deadline = time.monotonic()
            while writer.count < args.frames:
                record_frame(bridge, cameras, leader, writer)
                deadline += period
                lag = time.monotonic() - deadline
                if lag > period:
                    if config.get("teleop_controller")=="robolab_torque":
                        deadline=time.monotonic()  # preserve real timestamps; do not emit catch-up bursts
                    else:
                        raise RuntimeError(f"missed control deadline by {lag:.3f}s")
                time.sleep(max(0, deadline - time.monotonic()))
            outcome = args.mock_outcome
    except (Exception, KeyboardInterrupt) as exc:
        error = exc
        reason = f"{type(exc).__name__}: {exc}"
    finally:
        try:
            if session is not None:
                session.close(reason or "operator exit")
            elif bridge is not None:
                bridge.close()
        except Exception as exc:
            outcome = "aborted"
            reason += f" stop/finalization failed: {exc}"
            error = error or exc
        finally:
            if terminal is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal)
        if writer is not None:
            if error is not None:
                outcome = "aborted"
            try:
                path = writer.finish(outcome, reason)
                print(json.dumps({"episode": str(path), "outcome": outcome, "frames": writer.count}))
            finally:
                writer.close_incomplete()
        if cameras is not None:
            cameras.close()
        if guide is not None:
            guide.close()
    if error is not None:
        raise SystemExit(reason)


if __name__ == "__main__":
    main()

