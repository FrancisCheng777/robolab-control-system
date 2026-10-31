#!/usr/bin/env bash
set -euo pipefail
COLLECTION_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[[ $# -le 1 ]] || { echo 'usage: collect_session.sh [--execute|--inspect]'; exit 2; }
COLLECTION_MODE="${1:---execute}"
case "$COLLECTION_MODE" in --execute|--inspect) ;; *) echo 'mode: --execute | --inspect'; exit 2;; esac
exec "$COLLECTION_ROOT/scripts/try_latency.sh" responsive_recording "$COLLECTION_MODE" --continuous
