#!/usr/bin/env bash
# Install Autodesk FBX SDK Python bindings into the active conda env.
#
# You must download FBX SDK 2020.3.x from Autodesk first (free):
# https://www.autodesk.com/developer-network/platform-technologies/fbx-sdk-2020-3-7
set -euo pipefail

WEB_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CONDA="${CONDA_EXE:-/opt/miniconda3/bin/conda}"
ENV_NAME="${T800_WEB_ENV:-t800-studio}"

if "$CONDA" run --no-capture-output -n "$ENV_NAME" python -c "import fbx" 2>/dev/null; then
  echo "FBX SDK Python module already importable (import fbx OK)."
  exit 0
fi

FBX_SDK_ROOT="${FBX_SDK_ROOT:-}"

if [[ -z "$FBX_SDK_ROOT" ]]; then
  for candidate in \
    "$HOME/Downloads/FBX_SDK" \
    "/Applications/Autodesk/FBX SDK/2020.3.7" \
    "/usr/local/fbx-sdk" \
    "/opt/fbx-sdk"; do
    if [[ -d "$candidate" ]]; then
      FBX_SDK_ROOT="$candidate"
      break
    fi
  done
fi

BINDINGS=""
if [[ -n "$FBX_SDK_ROOT" ]]; then
  BINDINGS="$(find "$FBX_SDK_ROOT" -type d -name "FBXPythonBindings" 2>/dev/null | head -n 1 || true)"
fi

if [[ -z "$BINDINGS" ]]; then
  cat <<'EOF'

FBX SDK Python bindings NOT installed.

Steps:
  1. Download and install Autodesk FBX SDK 2020.3.7 for your OS.
  2. Find the folder named FBXPythonBindings inside the SDK.
  3. Run (replace path):

     export FBX_SDK_ROOT="/path/to/FBX SDK/2020.3.7"
     ./scripts/install_fbx_sdk.sh

  Reference:
  https://github.com/nv-tlabs/ASE/tree/main/ase/poselib#importing-from-fbx

Without this, BVH/PKL still work — FBX upload will fail.
EOF
  exit 1
fi

echo "Installing FBX bindings from: $BINDINGS"
"$CONDA" run --no-capture-output -n "$ENV_NAME" \
  python "$BINDINGS/setup.py" install

"$CONDA" run --no-capture-output -n "$ENV_NAME" python -c "import fbx; print('import fbx OK')"
