#!/usr/bin/env bash
# One-time setup for T800 FBX Studio (local PC / GitHub clone).
set -euo pipefail

WEB_ROOT="$(cd "$(dirname "$0")" && pwd)"
CONDA="${CONDA_EXE:-/opt/miniconda3/bin/conda}"
ENV_NAME="${T800_WEB_ENV:-t800-studio}"

echo "==> Creating conda env: $ENV_NAME (python 3.10)"
if "$CONDA" env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
  echo "    env exists, skipping create"
else
  "$CONDA" create -y -n "$ENV_NAME" python=3.10
fi

echo "==> Installing Python packages"
"$CONDA" run --no-capture-output -n "$ENV_NAME" pip install -U pip
"$CONDA" run --no-capture-output -n "$ENV_NAME" pip install -r "$WEB_ROOT/requirements-web.txt"
"$CONDA" run --no-capture-output -n "$ENV_NAME" pip install -e "$WEB_ROOT/gmr"

echo "==> FBX SDK (Autodesk Python bindings — required for .fbx upload)"
if ! bash "$WEB_ROOT/scripts/install_fbx_sdk.sh"; then
  echo ""
  echo "WARN: FBX module not installed. Studio works for BVH/PKL only."
  echo "      To enable FBX→PKL:"
  echo "        ./scripts/download_fbx_sdk.sh"
  echo "        source .fbx_sdk_cache/paths.env"
  echo "        ./scripts/install_fbx_sdk.sh"
  echo ""
fi

echo "==> Verifying installation"
cd "$WEB_ROOT"
"$CONDA" run --no-capture-output -n "$ENV_NAME" \
  env PYTHONPATH="$WEB_ROOT:$WEB_ROOT/gmr:$WEB_ROOT/gmr/third_party" \
  python "$WEB_ROOT/scripts/verify_setup.py"

echo ""
echo "Done. Start studio:"
echo "  cd $WEB_ROOT && ./run.sh"
