# Prebuilt Autodesk FBX Python module (`import fbx`)

This folder contains a **pre-compiled** `fbx` wheel so macOS users on **Python 3.10 arm64** do not need to build FBXPythonBindings from source.

| File | Platform |
|------|----------|
| `fbx-2020.3.7-cp310-cp310-macosx_10_15_arm64.whl` | macOS Apple Silicon, CPython 3.10 |

Built from Autodesk FBX SDK **2020.3.7** with the `fbxredblacktree.h` typo patch (`mLefttChild` → `mLeftChild`).

## Install

```bash
conda activate t800-studio
pip install vendor/fbx_wheels/fbx-2020.3.7-cp310-cp310-macosx_10_15_arm64.whl
python -c "import fbx; print('ok')"
```

Or run `./scripts/install_fbx_sdk.sh` — it picks the matching wheel automatically.

## Linux / Intel Mac

No prebuilt wheel yet — use `./scripts/download_fbx_sdk.sh` + build (see main README).

## License

Autodesk FBX SDK is proprietary. By using this wheel you agree to [Autodesk FBX SDK terms](https://aps.autodesk.com/developer/overview/fbx-sdk). We redistribute this binary **only** as a convenience for T800 Studio users; it is not a substitute for accepting Autodesk's license.
