#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
case "${1:-}" in
  --mock) exec "$ROOT/build/pi05_franka_bridge" --mock --config "$ROOT/configs/bridge.mock.json" --socket /tmp/pi05-plug-demo.sock ;;
  --execute)
    "$ROOT/.venv/bin/python" -m pi05_demo.preflight --hardware-config
    exec "$ROOT/build/pi05_franka_bridge" --config "$ROOT/configs/bridge.real.json" --robot 172.16.0.40 --socket /tmp/pi05-plug-demo.sock ;;
  *) echo "Usage: $0 --mock | --execute"; exit 2 ;;
esac
