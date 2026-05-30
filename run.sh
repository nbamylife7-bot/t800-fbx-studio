#!/usr/bin/env bash
# T800 Web Studio — viser-only (no desktop MuJoCo viewer).
#
#   cd web-version && ./run.sh
#   → http://localhost:8080
set -euo pipefail

WEB_ROOT="$(cd "$(dirname "$0")" && pwd)"
CONDA="${CONDA_EXE:-/opt/miniconda3/bin/conda}"
ENV_NAME="${T800_WEB_ENV:-t800-studio}"

export GMR_ROOT="${GMR_ROOT:-$WEB_ROOT/gmr}"
export T800_WEB_HOST="${T800_WEB_HOST:-0.0.0.0}"
export T800_WEB_PORT="${T800_WEB_PORT:-8080}"
export T800_DEMO_DIR="${T800_DEMO_DIR:-$WEB_ROOT/data/out}"

echo "T800 FBX Studio (web)"
echo "  GMR_ROOT=$GMR_ROOT"
echo "  Open http://localhost:${T800_WEB_PORT}"
echo ""

if [[ ! -d "$GMR_ROOT/general_motion_retargeting" ]]; then
  echo "ERROR: bundled backend missing. Run: ./scripts/bundle_gmr.sh" >&2
  exit 1
fi

exec env -u VIRTUAL_ENV \
  PYTHONPATH="$GMR_ROOT:$GMR_ROOT/third_party:${PYTHONPATH:-}" \
  "$CONDA" run --no-capture-output -n "$ENV_NAME" \
  python -m app.studio
