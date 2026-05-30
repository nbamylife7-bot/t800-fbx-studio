#!/usr/bin/env bash
# Download and extract Autodesk FBX SDK 2020.3.7 + Python bindings (macOS / Linux).
#
# Official download page (scroll to "Past FBX SDK downloads" → 2020.3.7):
#   https://aps.autodesk.com/developer/overview/fbx-sdk
#
# We use the same direct Autodesk CDN URLs that work with curl (verified May 2026).
set -euo pipefail

WEB_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CACHE_ROOT="${FBX_CACHE_ROOT:-$WEB_ROOT/.fbx_sdk_cache}"
OS="$(uname -s)"

mkdir -p "$CACHE_ROOT"

case "$OS" in
  Darwin)
    SDK_URL="https://damassets.autodesk.net/content/dam/autodesk/www/files/fbx202037_fbxsdk_clang_mac.pkg.tgz"
    BINDINGS_URL="https://damassets.autodesk.net/content/dam/autodesk/www/files/fbx202037_fbxpythonbindings_mac.pkg.tgz"
    SDK_TGZ="$CACHE_ROOT/fbxsdk_202037_mac.tgz"
    BINDINGS_TGZ="$CACHE_ROOT/fbxpythonbindings_202037_mac.tgz"
    SDK_PKG="$CACHE_ROOT/fbx202037_fbxsdk_clang_macos.pkg"
    BINDINGS_PKG="$CACHE_ROOT/fbx202037_fbxpythonbindings_macos.pkg"
    SDK_EXPANDED="$CACHE_ROOT/sdk_expanded"
    BINDINGS_EXPANDED="$CACHE_ROOT/bindings_expanded"
    ;;
  Linux)
    SDK_URL="https://damassets.autodesk.net/content/dam/autodesk/www/files/fbx202037_fbxsdk_gcc_linux.tar.gz"
    BINDINGS_URL="https://damassets.autodesk.net/content/dam/autodesk/www/files/fbx202037_fbxpythonbindings_linux.tar.gz"
    SDK_TGZ="$CACHE_ROOT/fbxsdk_202037_linux.tar.gz"
    BINDINGS_TGZ="$CACHE_ROOT/fbxpythonbindings_202037_linux.tar.gz"
    SDK_EXPANDED="$CACHE_ROOT/sdk_linux"
    BINDINGS_EXPANDED="$CACHE_ROOT/bindings_linux"
    ;;
  *)
    echo "Unsupported OS: $OS (macOS and Linux only)" >&2
    exit 1
    ;;
esac

download() {
  local url="$1"
  local dest="$2"
  if [[ -f "$dest" ]]; then
    echo "  cached: $dest"
    return 0
  fi
  echo "  downloading: $url"
  curl -L --fail --retry 3 --retry-delay 2 "$url" -o "$dest"
}

echo "==> FBX SDK cache: $CACHE_ROOT"

if [[ "$OS" == "Darwin" ]]; then
  download "$SDK_URL" "$SDK_TGZ"
  download "$BINDINGS_URL" "$BINDINGS_TGZ"

  tar -xzf "$SDK_TGZ" -C "$CACHE_ROOT"
  tar -xzf "$BINDINGS_TGZ" -C "$CACHE_ROOT"

  rm -rf "$SDK_EXPANDED" "$BINDINGS_EXPANDED"
  pkgutil --expand-full "$SDK_PKG" "$SDK_EXPANDED"
  pkgutil --expand-full "$BINDINGS_PKG" "$BINDINGS_EXPANDED"

  export FBXSDK_ROOT="$SDK_EXPANDED/Root.pkg/Payload/Applications/Autodesk/FBX SDK/2020.3.7"
  export FBX_BINDINGS_DIR="$BINDINGS_EXPANDED/Root.pkg/Payload/Applications/Autodesk/FBXPythonBindings"
else
  download "$SDK_URL" "$SDK_TGZ"
  download "$BINDINGS_URL" "$BINDINGS_TGZ"

  rm -rf "$SDK_EXPANDED" "$BINDINGS_EXPANDED"
  mkdir -p "$SDK_EXPANDED" "$BINDINGS_EXPANDED"
  tar -xzf "$SDK_TGZ" -C "$SDK_EXPANDED"
  tar -xzf "$BINDINGS_TGZ" -C "$BINDINGS_EXPANDED"

  # Linux archives unpack to a versioned directory; locate include/lib.
  FBXSDK_ROOT="$(find "$SDK_EXPANDED" -type f -path '*/include/fbxsdk/fbxsdk_version.h' 2>/dev/null | head -n 1 | xargs dirname | xargs dirname | xargs dirname)"
  FBX_BINDINGS_DIR="$(find "$BINDINGS_EXPANDED" -type f -name pyproject.toml -path '*/FBXPythonBindings/*' 2>/dev/null | head -n 1 | xargs dirname)"

  if [[ -z "$FBXSDK_ROOT" || ! -d "$FBXSDK_ROOT/include" ]]; then
    echo "Could not locate Linux FBX SDK root under $SDK_EXPANDED" >&2
    exit 1
  fi
  if [[ -z "$FBX_BINDINGS_DIR" || ! -f "$FBX_BINDINGS_DIR/pyproject.toml" ]]; then
    echo "Could not locate Linux FBXPythonBindings under $BINDINGS_EXPANDED" >&2
    exit 1
  fi
  export FBXSDK_ROOT FBX_BINDINGS_DIR
fi

cat >"$CACHE_ROOT/paths.env" <<EOF
# Source before install:  source $CACHE_ROOT/paths.env
export FBXSDK_ROOT="$FBXSDK_ROOT"
export FBX_BINDINGS_DIR="$FBX_BINDINGS_DIR"
EOF

echo ""
echo "Extracted:"
echo "  FBXSDK_ROOT=$FBXSDK_ROOT"
echo "  FBX_BINDINGS_DIR=$FBX_BINDINGS_DIR"
echo ""
echo "Next:"
echo "  source $CACHE_ROOT/paths.env"
echo "  ./scripts/install_fbx_sdk.sh"
