"""Read live FCI tool parameters with libfranka's echo example; never move/reset."""
import argparse
import datetime
import json
import math
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", default="172.16.0.40")
    parser.add_argument("--echo", default="/home/heinrich/libfranka-0.9.0/build/examples/echo_robot_state")
    parser.add_argument("--config", type=Path, default=ROOT / "configs/bridge.real.json",
                        help="Bridge configuration whose Desk tool will be checked")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    expected = config["expected_desk_tool"]
    result = subprocess.run([args.echo, args.ip], capture_output=True, text=True, timeout=10)
    if result.returncode:
        raise RuntimeError(f"FCI read failed: {result.stderr[-1500:]}")
    samples = []
    for line in result.stdout.splitlines():
        if line.startswith("{"):
            samples.append(json.loads(line))
    if not samples:
        raise RuntimeError("FCI example returned no JSON state")
    state = samples[-1]
    errors = []
    for name, state_name, tolerance in [
        ("mass", "m_ee", 1e-4), ("centerOfMass", "F_x_Cee", 1e-5),
        ("inertia", "I_ee", 1e-6), ("transformation", "F_T_NE", 1e-5),
        ("transformation", "F_T_EE", 1e-5),
    ]:
        want, got = expected[name], state[state_name]
        a = want if isinstance(want, list) else [want]
        b = got if isinstance(got, list) else [got]
        if len(a) != len(b) or any(not math.isfinite(y) or abs(x-y) > tolerance for x, y in zip(a, b)):
            errors.append(state_name)
    report = {
        "utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "read_only": True, "motion_executed": False,
        "matches_expected_tool": not errors, "mismatched_fields": errors,
        "expected_profile_id": expected["id"],
        "expected_config": str(args.config.resolve()),
        "state": {k: state.get(k) for k in (
            "robot_mode", "current_errors", "last_motion_errors", "q", "q_d",
            "m_ee", "F_x_Cee", "I_ee", "F_T_NE", "F_T_EE", "NE_T_EE",
            "m_load", "F_x_Cload", "I_load")},
    }
    path = args.output or ROOT / "reports/desk_tool_latest.json"
    path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0 if not errors else 2


if __name__ == "__main__":
    raise SystemExit(main())
