# T800 FBX Studio

**FBX / BVH → EngineAI T800 `.pkl`** with in-browser 3D preview (viser).

First open-source pipeline for **Mixamo/FBX motion → T800 robot** retargeting with flat feet and web preview. No desktop MuJoCo window required.

## Quick start (GitHub clone)

```bash
git clone https://github.com/YOUR_USER/t800-fbx-studio.git
cd t800-fbx-studio

chmod +x install.sh run.sh scripts/*.sh
./install.sh          # conda env + deps + FBX SDK check
./run.sh              # http://localhost:8080
```

1. Open **http://localhost:8080**
2. Block **1. Source** → upload `.fbx` (Mixamo, root joint `Hips`)
3. Status shows: `Loading…` → `Retargeting…` → `Ready — N frames`
4. Play back in the browser; download `.pkl` from `data/out/`

## FBX SDK (required for `.fbx`)

Autodesk FBX SDK **cannot** be redistributed in this repo. Each user installs it once:

1. Download [FBX SDK 2020.3.7](https://www.autodesk.com/developer-network/platform-technologies/fbx-sdk-2020-3-7)
2. Install for your OS
3. Point to bindings and run:

```bash
export FBX_SDK_ROOT="/path/to/FBX SDK/2020.3.7"
./scripts/install_fbx_sdk.sh
```

Verify:

```bash
conda activate t800-studio
python -c "import fbx; print('ok')"
```

Full FBX→PKL smoke test:

```bash
export T800_TEST_FBX="/path/to/your/motion.fbx"
conda activate t800-studio
PYTHONPATH="gmr:gmr/third_party:." python scripts/verify_setup.py
```

## Supported inputs

| Type | Needs FBX SDK | Notes |
|------|---------------|-------|
| `.fbx` | **Yes** | Mixamo / OptiTrack; root `Hips` |
| `.bvh` | No | LaFAN1 / human_robot_hit profiles |
| `.pkl` | No | Previously converted T800 motion |

## Repo layout

```
t800-fbx-studio/
  app/              # viser web UI
  gmr/              # bundled retarget backend + T800 assets (~100 MB)
  scripts/          # install / verify / bundle helpers
  data/out/         # converted PKL (gitignored)
  data/uploads/     # uploads (gitignored)
  install.sh
  run.sh
```

## Publish to GitHub

```bash
cd t800-fbx-studio
git init
git add .
git commit -m "Initial T800 FBX Studio release"
gh repo create t800-fbx-studio --public --source=. --push
```

## Requirements

- macOS or Linux (Windows untested)
- Miniconda, Python 3.10
- ~4 GB RAM for retargeting
- Autodesk FBX SDK for `.fbx` files
- See `requirements-web.txt`

## What is NOT included

- Autodesk FBX SDK binaries (user installs)
- Multi-user server / cookies (local single-user tool)
- Motion blend (removed)

## Credits

Built on [GMR](https://github.com/YanjieZe/GMR) retargeting patterns, extended for **EngineAI T800** with custom FBX IK (`fbx_to_t800.json`), foot flattening, and viser preview.

## License

MIT — see [LICENSE](LICENSE). T800 mesh/texture assets: use under your EngineAI / project terms.
