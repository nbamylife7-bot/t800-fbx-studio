# FBX tips for T800 Studio

Use **Mixamo** or similar humanoid FBX:

- Root joint name: **Hips** (default in UI)
- FPS: match source (often 30)
- Retarget mode: **position** (recommended for martial arts / foot contact)
- Enable **Flat feet on ground**

After upload, check **Status**:

- `Parsed N frames` — FBX SDK read OK
- `Retargeting …` — IK running
- `Ready — N frames` — PKL saved under `data/out/`

## If FBX fails

```bash
./scripts/download_fbx_sdk.sh
source .fbx_sdk_cache/paths.env
./scripts/install_fbx_sdk.sh
conda activate t800-studio
python -c "import fbx; print('ok')"
```

Download page (not the old 404 URL): https://aps.autodesk.com/developer/overview/fbx-sdk
