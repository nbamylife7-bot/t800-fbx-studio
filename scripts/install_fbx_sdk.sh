#!/usr/bin/env bash
# Build and install Autodesk FBX Python module (`import fbx`) into conda env t800-studio.
#
# Requires FBX SDK 2020.3.7 C++ headers/libs + FBXPythonBindings sources.
# Easiest path on a fresh machine:
#   ./scripts/download_fbx_sdk.sh
#   source .fbx_sdk_cache/paths.env
#   ./scripts/install_fbx_sdk.sh
set -euo pipefail

WEB_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CONDA="${CONDA_EXE:-/opt/miniconda3/bin/conda}"
ENV_NAME="${T800_WEB_ENV:-t800-studio}"
CACHE_ROOT="${FBX_CACHE_ROOT:-$WEB_ROOT/.fbx_sdk_cache}"

if "$CONDA" run --no-capture-output -n "$ENV_NAME" python -c "import fbx" 2>/dev/null; then
  echo "FBX SDK Python module already importable (import fbx OK)."
  exit 0
fi

# Optional: auto-download when user runs install without manual download step.
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
    "/Applications/Autodesk/FBX SDK/2020.3.7" \
    "$HOME/Downloads/cyanpuppets_1.6.8/out/fbx_sdk/sdk_pkg_expanded/Root.pkg/Payload/Applications/Autodesk/FBX SDK/2020.3.7"; do
    if [[ -d "$candidate/include" ]]; then
      FBXSDK_ROOT="$candidate"
      break
    fi
  done
fi

if [[ -z "$FBX_BINDINGS_DIR" ]]; then
  for candidate in \
    "$CACHE_ROOT/bindings_expanded/Root.pkg/Payload/Applications/Autodesk/FBXPythonBindings" \
    "$CACHE_ROOT/bindings_linux" \
    "$(find "$CACHE_ROOT" -type f -name pyproject.toml -path '*/FBXPythonBindings/*' 2>/dev/null | head -n 1 | xargs dirname 2>/dev/null || true)"; do
    if [[ -n "$candidate" && -f "$candidate/pyproject.toml" ]]; then
      FBX_BINDINGS_DIR="$candidate"
      break
    fi
  done
fi

if [[ -z "$FBXSDK_ROOT" || ! -d "$FBXSDK_ROOT/include" ]]; then
  cat <<'EOF'

FBX SDK C++ headers/libs NOT found.

Run the verified download + extract step first:

  ./scripts/download_fbx_sdk.sh
  source .fbx_sdk_cache/paths.env
  ./scripts/install_fbx_sdk.sh

Manual downloads (Autodesk APS page → Past FBX SDK downloads → 2020.3.7):
  https://aps.autodesk.com/developer/overview/fbx-sdk

macOS direct URLs (curl-friendly):
  https://damassets.autodesk.net/content/dam/autodesk/www/files/fbx202037_fbxsdk_clang_mac.pkg.tgz
  https://damassets.autodesk.net/content/dam/autodesk/www/files/fbx202037_fbxpythonbindings_mac.pkg.tgz

Without `import fbx`, BVH/PKL still work — FBX upload will fail.
EOF
  exit 1
fi

if [[ -z "$FBX_BINDINGS_DIR" || ! -f "$FBX_BINDINGS_DIR/pyproject.toml" ]]; then
  echo "FBXPythonBindings not found. Run ./scripts/download_fbx_sdk.sh first." >&2
  exit 1
fi

# Fix known typo in some 2020.3.7 headers (breaks clang build on recent macOS).
RB_TREE="$FBXSDK_ROOT/include/fbxsdk/core/base/fbxredblacktree.h"
if [[ -f "$RB_TREE" ]] && grep -q 'mLefttChild' "$RB_TREE"; then
  echo "Patching FBX SDK header typo in fbxredblacktree.h"
  sed -i.bak 's/mLefttChild/mLeftChild/g' "$RB_TREE"
fi

echo "Installing sip build tool (FBX bindings require sip 6.6.x, not 6.7+)"
"$CONDA" run --no-capture-output -n "$ENV_NAME" python -m pip install -q 'sip>=6.6.2,<6.7'

echo "Building fbx Python module"
echo "  FBXSDK_ROOT=$FBXSDK_ROOT"
echo "  FBX_BINDINGS_DIR=$FBX_BINDINGS_DIR"

env -u VIRTUAL_ENV FBXSDK_ROOT="$FBXSDK_ROOT" \
  "$CONDA" run --no-capture-output -n "$ENV_NAME" \
  python -m pip install "$FBX_BINDINGS_DIR"

"$CONDA" run --no-capture-output -n "$ENV_NAME" python -c "import fbx; print('import fbx OK')"
