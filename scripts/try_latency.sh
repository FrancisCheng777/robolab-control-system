#!/usr/bin/env bash
set -euo pipefail
LATENCY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LATENCY_PROFILE="${1:-fast}"
LATENCY_MODE="${2:---inspect}"
case "$LATENCY_PROFILE" in baseline|fast|feedforward|feedforward075|feedforward_guarded) ;; *) echo 'profile: baseline | fast | feedforward | feedforward075 | feedforward_guarded'; exit 2;; esac
case "$LATENCY_MODE" in --inspect|--execute) ;; *) echo 'mode: --inspect | --execute'; exit 2;; esac
[[ $# -le 3 ]] || exit 2
case "${3:-}" in
  --teleop-only)
    [[ "$LATENCY_PROFILE" != baseline ]] || { echo 'teleop-only requires a fast/feedforward profile'; exit 2; }
    export PI05_TELEOP_ONLY=1 ;;
  "") export PI05_TELEOP_ONLY=0 ;;
  *) echo 'optional third argument: --teleop-only'; exit 2 ;;
esac
export PI05_TRIAL_CONFIG_DIR="$LATENCY_ROOT/configs/latency/$LATENCY_PROFILE"
export PI05_TRIAL_RUN_ROOT="$LATENCY_ROOT/reports/latency_trials/$LATENCY_PROFILE"
export PI05_TRIAL_DATA_ROOT="$LATENCY_ROOT/data/latency_trials/$LATENCY_PROFILE"
export PI05_TRIAL_SOCKET=/tmp/robolab-latency-trial.sock
export PI05_LATENCY_TRIAL=1
export PYTHONDONTWRITEBYTECODE=1
printf 'Profile: %s\nConfig: %s\nMode: %s\n' "$LATENCY_PROFILE" "$PI05_TRIAL_CONFIG_DIR" "$LATENCY_MODE"
if [[ "$LATENCY_MODE" == --inspect ]]; then
  cat "$PI05_TRIAL_CONFIG_DIR/collection.trial.json"
  echo 'Inspection only: no devices opened, no processes started.'
  exit 0
fi
# Refuse to share a physical robot even when the original bridge uses another
# socket. Do not stop it, remove its socket, or perform automatic recovery.
"$LATENCY_ROOT/.venv/bin/python" - <<'PY'
from pathlib import Path
for process in Path('/proc').iterdir():
    if not process.name.isdigit(): continue
    try:
        args=(process/'cmdline').read_bytes().split(b'\0')
    except (OSError, PermissionError): continue
    if args and Path(args[0].decode(errors='replace')).name == 'pi05_franka_bridge' and b'--mock' not in args:
        raise SystemExit(f'Existing physical bridge PID {process.name}; exit its owning session before testing. No process was stopped.')
PY
exec "$LATENCY_ROOT/scripts/try_one.sh"
