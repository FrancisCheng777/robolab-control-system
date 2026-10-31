#!/usr/bin/env bash
# Run this in a terminal on the 4090 desktop. Starts visualization only.
set -eo pipefail
ALIGNMENT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ALIGNMENT_ROS_PORT="${ALIGNMENT_ROS_PORT:-11321}"
ALIGNMENT_HTTP_PORT="${ALIGNMENT_HTTP_PORT:-8787}"
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI="http://127.0.0.1:${ALIGNMENT_ROS_PORT}"
export ROS_HOSTNAME=localhost
ALIGNMENT_PIDS=()
cleanup() {
  for pid in "${ALIGNMENT_PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM
cd "$ALIGNMENT_ROOT"
.venv/bin/python - "$ALIGNMENT_HTTP_PORT" "$ALIGNMENT_ROS_PORT" <<'PY'
import socket, sys
for port in map(int, sys.argv[1:]):
    with socket.socket() as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe.bind(('127.0.0.1', port))
        except OSError:
            raise SystemExit('Alignment port %d is already occupied; close the previous viewer first.' % port)
PY
.venv/bin/python -m pi05_demo.alignment_viewer --port "$ALIGNMENT_HTTP_PORT" "$@" &
ALIGNMENT_PIDS+=("$!")
roscore -p "$ALIGNMENT_ROS_PORT" &
ALIGNMENT_PIDS+=("$!")
for attempt in {1..50}; do
  if rosparam list >/dev/null 2>&1; then break; fi
  sleep .1
done
kill -0 "${ALIGNMENT_PIDS[0]}" || exit 1
kill -0 "${ALIGNMENT_PIDS[1]}" || exit 1
/usr/bin/python3 scripts/alignment_rviz.py --port "$ALIGNMENT_HTTP_PORT" &
ALIGNMENT_PIDS+=("$!")
rviz -d configs/alignment.rviz &
ALIGNMENT_PIDS+=("$!")
wait "${ALIGNMENT_PIDS[3]}"
