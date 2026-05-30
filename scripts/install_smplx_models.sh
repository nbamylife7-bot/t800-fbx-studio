#!/usr/bin/env bash
# Copy or link SMPL-X body model(s) into the bundled GMR assets path.
#
# Usage:
#   ./scripts/install_smplx_models.sh ~/Downloads/SMPLX_NEUTRAL_2020.npz
#   ./scripts/install_smplx_models.sh ~/Downloads/SMPLX_NEUTRAL_2020.npz ~/Downloads/SMPLX_MALE_2020.npz
set -euo pipefail

WEB_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
GMR_ROOT="${GMR_ROOT:-$WEB_ROOT/gmr}"
DEST="$GMR_ROOT/assets/body_models/smplx"

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 /path/to/SMPLX_NEUTRAL*.npz [SMPLX_MALE*.npz ...]" >&2
  echo "" >&2
  echo "Target directory: $DEST" >&2
  echo "See gmr/assets/body_models/README.md" >&2
  exit 1
fi

mkdir -p "$DEST"

for src in "$@"; do
  if [[ ! -f "$src" ]]; then
    echo "ERROR: not a file: $src" >&2
    exit 1
  fi
  base="$(basename "$src")"
  case "$base" in
    SMPLX_*.npz|SMPLX_*.pkl) ;;
    *)
      echo "WARN: unexpected filename $base (expected SMPLX_NEUTRAL*.npz etc.)" >&2
      ;;
  esac
  cp -f "$src" "$DEST/$base"
  echo "Installed $base -> $DEST/"
done

echo ""
echo "Verify:"
"$WEB_ROOT/scripts/verify_setup.py" 2>/dev/null | grep -E 'SMPL-X|Setup looks' || true
echo ""
echo "Studio: Input type = npz, upload Kimodo AMASS SMPL-X export."
