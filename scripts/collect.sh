#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
export PYTHONPATH="$ROOT:$ROOT/runtime:$ROOT/vendor${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="/home/heinrich/freenect2_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$ROOT/.venv/bin/python" -m pi05_demo.collect "$@"
