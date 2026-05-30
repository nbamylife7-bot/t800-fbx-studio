#!/usr/bin/env bash
# Refresh bundled GMR backend from the parent cyanpuppets checkout (maintainers only).
set -euo pipefail

WEB_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="${GMR_SOURCE:-$WEB_ROOT/../Новая папка 8/GMR}"
DST="$WEB_ROOT/gmr"

if [[ ! -d "$SRC/general_motion_retargeting" ]]; then
  echo "Source GMR not found: $SRC" >&2
  echo "Set GMR_SOURCE=/path/to/GMR" >&2
  exit 1
fi

echo "Bundling $SRC -> $DST"
rm -rf "$DST"
mkdir -p "$DST/third_party" "$DST/scripts" "$DST/assets"

rsync -a "$SRC/general_motion_retargeting/" "$DST/general_motion_retargeting/"
rsync -a "$SRC/assets/t800/" "$DST/assets/t800/"
rsync -a "$SRC/third_party/FbxCommon.py" "$SRC/third_party/poselib/" "$DST/third_party/"
rsync -a \
  "$SRC/scripts/fbx_to_robot.py" \
  "$SRC/scripts/bvh_to_robot.py" \
  "$SRC/scripts/smplx_npz_to_robot.py" \
  "$SRC/scripts/t800_viser_robot.py" \
  "$SRC/scripts/t800_foot_postprocess.py" \
  "$DST/scripts/"

mkdir -p "$DST/assets/body_models/smplx"
if [[ -d "$WEB_ROOT/gmr/assets/body_models" ]]; then
  rsync -a "$WEB_ROOT/gmr/assets/body_models/" "$DST/assets/body_models/"
elif [[ -d "$SRC/assets/body_models" ]]; then
  rsync -a "$SRC/assets/body_models/" "$DST/assets/body_models/"
else
  touch "$DST/assets/body_models/smplx/.gitkeep"
fi
cp "$SRC/setup.py" "$DST/setup.py"
touch "$DST/scripts/__init__.py"

du -sh "$DST"
echo "Done."
