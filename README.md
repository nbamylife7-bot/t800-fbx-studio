# T800 FBX Studio

**FBX / BVH → EngineAI T800 `.pkl`** with in-browser 3D preview (viser).

First open-source pipeline for **Mixamo/FBX motion → T800 robot** retargeting with flat feet and web preview. No desktop MuJoCo window required.

## Quick start

```bash
git clone https://github.com/nbamylife7-bot/t800-fbx-studio.git
cd t800-fbx-studio

chmod +x install.sh run.sh scripts/*.sh
./install.sh          # conda env + Python deps
./run.sh              # http://localhost:8080
```

1. Open **http://localhost:8080**
2. Block **1. Source** → set **Input type** (`fbx` / `bvh` / `pkl`) or pick a **Demo clip**
3. Upload or set path — auto convert/load
4. Play in browser; `.pkl` saved under `data/out/`

## Supported inputs

All three formats work in the web UI:

| Format | Supported | FBX SDK required? | Notes |
|--------|-----------|-------------------|-------|
| `.fbx` | **Yes** | **Yes** | Mixamo / OptiTrack; root `Hips` |
| `.bvh` | **Yes** | No | LaFAN1 / `human_robot_hit` — works after `./install.sh` |
| `.pkl` | **Yes** | No | T800 motion; demo clips in `examples/demos/` |

> **FBX SDK required?** = whether Autodesk SDK is needed. **No** means the format works without `import fbx`.

**Try without FBX:** Demo clip → `Boxing` / `Martelo_2` / `Flair`, or upload `examples/sample_human_robot_hit.bvh` with Input type = `bvh`.

## FBX SDK (only for `.fbx`)

### Fast path — prebuilt wheel (macOS Apple Silicon, Python 3.10)

Bundled in repo — **no compile**:

```bash
./install.sh
./scripts/install_fbx_sdk.sh    # installs vendor/fbx_wheels/*.whl
python -c "import fbx; print('ok')"
```

See `vendor/fbx_wheels/README.md` (Autodesk license applies).

### Build from sources (Linux, Intel Mac, other Python)

Official page (scroll to **Past FBX SDK downloads → 2020.3.7**):

https://aps.autodesk.com/developer/overview/fbx-sdk

**Automated (recommended):**

```bash
./scripts/download_fbx_sdk.sh
source .fbx_sdk_cache/paths.env
```

This uses the same Autodesk CDN files we tested with `curl`:

| Platform | C++ SDK | Python bindings |
|----------|---------|-----------------|
| **macOS** | [fbx202037_fbxsdk_clang_mac.pkg.tgz](https://damassets.autodesk.net/content/dam/autodesk/www/files/fbx202037_fbxsdk_clang_mac.pkg.tgz) | [fbx202037_fbxpythonbindings_mac.pkg.tgz](https://damassets.autodesk.net/content/dam/autodesk/www/files/fbx202037_fbxpythonbindings_mac.pkg.tgz) |
| **Linux** | [fbx202037_fbxsdk_gcc_linux.tar.gz](https://damassets.autodesk.net/content/dam/autodesk/www/files/fbx202037_fbxsdk_gcc_linux.tar.gz) | [fbx202037_fbxpythonbindings_linux.tar.gz](https://damassets.autodesk.net/content/dam/autodesk/www/files/fbx202037_fbxpythonbindings_linux.tar.gz) |

> **Note:** The old link `autodesk.com/developer-network/platform-technologies/fbx-sdk-2020-3-7` returns **404**. Use APS page or direct URLs above.

**Manual macOS (same as our working setup):**

```bash
mkdir -p .fbx_sdk_cache && cd .fbx_sdk_cache
curl -L -O "https://damassets.autodesk.net/content/dam/autodesk/www/files/fbx202037_fbxsdk_clang_mac.pkg.tgz"
curl -L -O "https://damassets.autodesk.net/content/dam/autodesk/www/files/fbx202037_fbxpythonbindings_mac.pkg.tgz"
tar -xzf fbx202037_fbxsdk_clang_mac.pkg.tgz
tar -xzf fbx202037_fbxpythonbindings_mac.pkg.tgz
pkgutil --expand-full fbx202037_fbxsdk_clang_macos.pkg sdk_expanded
pkgutil --expand-full fbx202037_fbxpythonbindings_macos.pkg bindings_expanded
export FBXSDK_ROOT="$PWD/sdk_expanded/Root.pkg/Payload/Applications/Autodesk/FBX SDK/2020.3.7"
export FBX_BINDINGS_DIR="$PWD/bindings_expanded/Root.pkg/Payload/Applications/Autodesk/FBXPythonBindings"
cd ..
```

### Step 2 — Build Python `fbx` module

Requires **Python 3.10** (conda env `t800-studio` from `./install.sh`):

```bash
./scripts/install_fbx_sdk.sh
```

This runs `pip install` on FBXPythonBindings with `FBXSDK_ROOT` set (same method as GMR/ASE poselib). It also patches a known typo in `fbxredblacktree.h` on some 2020.3.7 builds.

Verify:

```bash
conda activate t800-studio
python -c "import fbx; print('ok')"
```

Full FBX→PKL smoke test:

```bash
export T800_TEST_FBX="/path/to/your/motion.fbx"
conda activate t800-studio
PYTHONPATH=".:gmr:gmr/third_party" python scripts/verify_setup.py
```

### One-shot install (download + build)

```bash
T800_AUTO_DOWNLOAD_FBX=1 ./install.sh
```

## Repo layout

```
t800-fbx-studio/
  app/              # viser web UI
  gmr/              # bundled retarget backend + T800 assets (~100 MB)
  vendor/fbx_wheels/  # prebuilt `fbx` wheel (macOS arm64 py3.10)
  examples/         # demo .pkl + sample .bvh
  scripts/
    download_fbx_sdk.sh
    install_fbx_sdk.sh
    verify_setup.py
  data/out/         # converted PKL (gitignored)
  install.sh
  run.sh
```

## Requirements

- macOS or Linux (Windows untested)
- Miniconda, **Python 3.10**
- ~4 GB RAM for retargeting
- FBX: prebuilt wheel (macOS arm64) or build from Autodesk 2020.3.7
- See `requirements-web.txt`

## Credits

Built on [GMR](https://github.com/YanjieZe/GMR) retargeting patterns, extended for **EngineAI T800** with custom FBX IK (`fbx_to_t800.json`), foot flattening, and viser preview.

FBX install flow follows [ASE poselib](https://github.com/nv-tlabs/ASE/tree/main/ase/poselib#importing-from-fbx) (`FBXSDK_ROOT` + `pip install FBXPythonBindings`).

## License

MIT — see [LICENSE](LICENSE). T800 mesh/texture assets: use under your EngineAI / project terms.

---

## Русский — форматы и FBX

| Формат | Работает? | FBX SDK |
|--------|-----------|---------|
| `.fbx` | да | нужен (или prebuilt wheel) |
| `.bvh` | да | **не нужен** |
| `.pkl` | да | **не нужен** |

1. `./install.sh`  
2. `./scripts/install_fbx_sdk.sh` — на Mac M1/M2/M3 ставит **готовый wheel** из `vendor/fbx_wheels/`  
3. `./run.sh` → upload `.fbx` / `.bvh` / `.pkl` или Demo clip  

Примеры: `examples/demos/`, `examples/sample_human_robot_hit.bvh`
