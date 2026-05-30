#!/usr/bin/env bash
# Maintainer: repackage the working fbx module from conda env into vendor/fbx_wheels/.
set -euo pipefail

WEB_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CONDA="${CONDA_EXE:-/opt/miniconda3/bin/conda}"
ENV_NAME="${T800_WEB_ENV:-gmr}"
OUT_DIR="$WEB_ROOT/vendor/fbx_wheels"

SITE="$("$CONDA" run --no-capture-output -n "$ENV_NAME" python -c "import site; print(site.getsitepackages()[0])")"
SO="$SITE/fbx.cpython-"$("$CONDA" run --no-capture-output -n "$ENV_NAME" python -c 'import sys; print(f"{sys.version_info.major}{sys.version_info.minor}")')"-darwin.so"
META="$SITE/fbx-2020.3.7.dist-info"

if [[ ! -f "$SO" ]]; then
  echo "fbx module not found in $ENV_NAME — build/install fbx first." >&2
  exit 1
fi

TAG="$("$CONDA" run --no-capture-output -n "$ENV_NAME" python -c 'import sys,platform; print(f"cp{sys.version_info.major}{sys.version_info.minor}-cp{sys.version_info.major}{sys.version_info.minor}-macosx_10_15_{platform.machine()}")')"
WHEEL="$OUT_DIR/fbx-2020.3.7-${TAG}.whl"

STAGING="$(mktemp -d)"
cp "$SO" "$STAGING/"
cp -R "$META" "$STAGING/"
( cd "$STAGING" && zip -qr "$WHEEL" . )
rm -rf "$STAGING"

echo "Wrote $WHEEL ($(du -h "$WHEEL" | awk '{print $1}'))"
