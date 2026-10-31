#!/usr/bin/env bash
set -euo pipefail
COLLECTION_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$COLLECTION_ROOT/scripts/try_one.sh" --continuous "$@"
