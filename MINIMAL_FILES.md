# Minimal file list for T800 Web Studio deployment

## What this folder is

`web-version/` is the **web entry point only** (~6 Python files + run script).
3D preview runs in the browser via **viser**.

There is **no desktop MuJoCo viewer**. The `mujoco` **Python library** is still
required on the server for IK retargeting and loading T800 meshes/textures.

---

## A. Always ship: `web-version/`

```
web-version/
  run.sh
  requirements-web.txt
  .env.example
  app/
    paths.py          # paths + PYTHONPATH bootstrap
    motion_io.py      # PKL load/save
    studio.py         # viser UI + convert pipeline
    __main__.py
  data/
    out/              # converted .pkl (created at runtime)
    uploads/          # user uploads (created at runtime)
  deploy/
    nginx.conf.example
    t800-web.service.example
```

---

## B. Backend bundle: minimal GMR subset

Place next to `web-version/` or set `GMR_ROOT` in `.env`.

### Required directories

| Path | Purpose |
|------|---------|
| `general_motion_retargeting/` | IK retargeting core |
| `assets/t800/mujoco/` | `t800_full_gmr.xml` |
| `assets/t800/meshes/` | robot OBJ/STL meshes |
| `assets/t800/texture/` | PNG textures (full skin) |
| `scripts/fbx_to_robot.py` | FBX → T800 |
| `scripts/bvh_to_robot.py` | BVH → T800 |
| `scripts/smplx_npz_to_robot.py` | Kimodo AMASS NPZ → T800 |
| `scripts/t800_viser_robot.py` | viser 3D robot |
| `scripts/t800_foot_postprocess.py` | flat feet |
| `assets/body_models/smplx/` | SMPL-X model install dir (gitignored `.npz`) |

### Required IK configs (`general_motion_retargeting/ik_configs/`)

- `fbx_to_t800.json` — FBX upload
- `smplx_to_t800.json` — Kimodo AMASS NPZ
- `bvh_lafan1_to_t800_origin_manual.json` — BVH lafan1
- `bvh_human_robot_hit_to_t800--mild_two_stage.json` — human_robot_hit profile

### FBX only (if users upload `.fbx`)

| Path | Purpose |
|------|---------|
| `third_party/FbxCommon.py` | FBX SDK Python wrapper |
| `third_party/poselib/` | FBX skeleton reader |

FBX SDK native libs must be installed on the server (same as local GMR setup).

### BVH-only deployment

If you **only** accept `.bvh` / `.pkl`, you can omit `third_party/poselib` and FBX SDK.

---

## C. Quick local run

```bash
cd web-version
chmod +x run.sh
./run.sh
# → http://localhost:8080
```

---

## D. Server deploy (summary)

1. Linux VPS, 4 GB+ RAM, Python 3.10+
2. Create conda env, `pip install -r requirements-web.txt`, `pip install -e GMR_ROOT`
3. Copy `web-version/` + GMR subset (section B)
4. systemd service → `deploy/t800-web.service.example`
5. nginx reverse proxy with WebSocket → `deploy/nginx.conf.example`
6. HTTPS via certbot

See `deploy/` for config templates.

---

## E. rsync example (minimal backend)

From repo root on your machine:

```bash
DEST=user@server:/opt/cyanpuppets/

rsync -av web-version/ "$DEST/web-version/"

rsync -av \
  "Новая папка 8/GMR/general_motion_retargeting/" \
  "$DEST/Новая папка 8/GMR/general_motion_retargeting/"

rsync -av \
  "Новая папка 8/GMR/assets/t800/" \
  "$DEST/Новая папка 8/GMR/assets/t800/"

rsync -av \
  "Новая папка 8/GMR/scripts/fbx_to_robot.py" \
  "Новая папка 8/GMR/scripts/bvh_to_robot.py" \
  "Новая папка 8/GMR/scripts/t800_viser_robot.py" \
  "Новая папка 8/GMR/scripts/t800_foot_postprocess.py" \
  "$DEST/Новая папка 8/GMR/scripts/"

rsync -av \
  "Новая папка 8/GMR/third_party/FbxCommon.py" \
  "Новая папка 8/GMR/third_party/poselib/" \
  "$DEST/Новая папка 8/GMR/third_party/"
```

Optional demo clips: `rsync -av out/*_t800.pkl "$DEST/out/"`
