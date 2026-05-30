#!/usr/bin/env bash
# Install Autodesk FBX Python module (`import fbx`) into conda env t800-studio.
#
# Order:
#   1. Prebuilt wheel in vendor/fbx_wheels/ (macOS arm64 + py3.10 — no compile)
#   2. Build from FBX SDK sources (download_fbx_sdk.sh)
set -euo pipefail

WEB_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CONDA="${CONDA_EXE:-/opt/miniconda3/bin/conda}"
ENV_NAME="${T800_WEB_ENV:-t800-studio}"
CACHE_ROOT="${FBX_CACHE_ROOT:-$WEB_ROOT/.fbx_sdk_cache}"
WHEELS_DIR="$WEB_ROOT/vendor/fbx_wheels"

run_py() {
  "$CONDA" run --no-capture-output -n "$ENV_NAME" python "$@"
}

if run_py -c "import fbx" 2>/dev/null; then
  echo "FBX SDK Python module already importable (import fbx OK)."
  exit 0
fi

# --- 1) Prebuilt wheel (bundled in repo) ---
PY_TAG="$(run_py -c 'import sys; print(f"cp{sys.version_info.major}{sys.version_info.minor}")')"
OS="$(uname -s)"
ARCH="$(uname -m)"

pick_prebuilt_wheel() {
  local wheel=""
  if [[ "$OS" == "Darwin" && "$ARCH" == "arm64" && "$PY_TAG" == "cp310" ]]; then
    wheel="$WHEELS_DIR/fbx-2020.3.7-cp310-cp310-macosx_10_15_arm64.whl"
  fi
  if [[ -n "$wheel" && -f "$wheel" ]]; then
    echo "$wheel"
  fi
}

PREBUILT="$(pick_prebuilt_wheel || true)"
if [[ -n "$PREBUILT" ]]; then
  echo "Installing prebuilt fbx wheel (no compile): $(basename "$PREBUILT")"
  "$CONDA" run --no-capture-output -n "$ENV_NAME" python -m pip install -q "$PREBUILT"
  run_py -c "import fbx; print('import fbx OK (prebuilt wheel)')"
  exit 0
fi

# --- 2) Build from Autodesk FBX SDK sources ---
if [[ "${T800_AUTO_DOWNLOAD_FBX:-0}" == "1" ]]; then
  bash "$WEB_ROOT/scripts/download_fbx_sdk.sh"
  # shellcheck disable=SC1090
  [[ -f "$CACHE_ROOT/paths.env" ]] && source "$CACHE_ROOT/paths.env"
fi

FBXSDK_ROOT="${FBXSDK_ROOT:-${FBX_SDK_ROOT:-}}"
FBX_BINDINGS_DIR="${FBX_BINDINGS_DIR:-}"

if [[ -z "$FBXSDK_ROOT" && -f "$CACHE_ROOT/paths.env" ]]; then
  # shellcheck disable=SC1090
  source "$CACHE_ROOT/paths.env"
fi

if [[ -z "$FBXSDK_ROOT" ]]; then
  for candidate in \
    "$CACHE_ROOT/sdk_expanded/Root.pkg/Payload/Applications/Autodesk/FBX SDK/2020.3.7" \
    "/Applications/Autodesk/FBX SDK/2020.3.7"; do
    if [[ -d "$candidate/include" ]]; then
      FBXSDK_ROOT="$candidate"
      break
    fi
  done
fi

if [[ -z "$FBX_BINDINGS_DIR" ]]; then
  for candidate in \
    "$CACHE_ROOT/bindings_expanded/Root.pkg/Payload/Applications/Autodesk/FBXPythonBindings" \
    "$(find "$CACHE_ROOT" -type f -name pyproject.toml -path '*/FBXPythonBindings/*' 2>/dev/null | head -n 1 | xargs dirname 2>/dev/null || true)"; do
    if [[ -n "$candidate" && -f "$candidate/pyproject.toml" ]]; then
      FBX_BINDINGS_DIR="$candidate"
      break
    fi
  done
fi

if [[ -z "$FBXSDK_ROOT" || ! -d "$FBXSDK_ROOT/include" ]]; then
  cat <<EOF

FBX SDK C++ headers/libs NOT found.

macOS Apple Silicon + Python 3.10: prebuilt wheel missing or wrong platform.
  Expected: vendor/fbx_wheels/fbx-2020.3.7-cp310-cp310-macosx_10_15_arm64.whl

Otherwise download + build:

  ./scripts/download_fbx_sdk.sh
  source .fbx_sdk_cache/paths.env
  ./scripts/install_fbx_sdk.sh

APS download page:
  https://aps.autodesk.com/developer/overview/fbx-sdk

Without \`import fbx\`, BVH/PKL still work — only FBX upload is disabled.
EOF
  exit 1
fi

if [[ -z "$FBX_BINDINGS_DIR" || ! -f "$FBX_BINDINGS_DIR/pyproject.toml" ]]; then
  echo "FBXPythonBindings not found. Run ./scripts/download_fbx_sdk.sh first." >&2
  exit 1
fi

RB_TREE="$FBXSDK_ROOT/include/fbxsdk/core/base/fbxredblacktree.h"
if [[ -f "$RB_TREE" ]] && grep -q 'mLefttChild' "$RB_TREE"; then
  echo "Patching FBX SDK header typo in fbxredblacktree.h"
  sed -i.bak 's/mLefttChild/mLeftChild/g' "$RB_TREE"
fi

echo "Installing sip (FBX bindings require sip 6.6.x, not 6.7+)"
"$CONDA" run --no-capture-output -n "$ENV_NAME" python -m pip install -q 'sip>=6.6.2,<6.7'

echo "Building fbx Python module from sources"
echo "  FBXSDK_ROOT=$FBXSDK_ROOT"
echo "  FBX_BINDINGS_DIR=$FBX_BINDINGS_DIR"

env -u VIRTUAL_ENV FBXSDK_ROOT="$FBXSDK_ROOT" \
  "$CONDA" run --no-capture-output -n "$ENV_NAME" \
  python -m pip install "$FBX_BINDINGS_DIR"

run_py -c "import fbx; print('import fbx OK (built from sources)')"
