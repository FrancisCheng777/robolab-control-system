#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
export PYTHONPATH="$ROOT:$ROOT/runtime"
# Reuse the existing CPU LeRobot interpreter read-only; datasets remain in this project.
exec /home/heinrich/roboLab/.venv-lerobot/bin/python -m pi05_demo.export "$@"
