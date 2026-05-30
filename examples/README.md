# Example files (included in repo)

## Demo PKL (T800 retargeted motion)

Load from UI: **Demo clip** → Boxing / Martelo_2 / Flair

| File | Description |
|------|-------------|
| `demos/Boxing_t800.pkl` | Boxing combo |
| `demos/Martelo_2_t800.pkl` | Martelo kick |
| `demos/Flair_t800.pkl` | Flair motion |

Set **Input type** = `pkl`, pick file path, or use demo dropdown. **No FBX SDK needed.**

## Sample BVH

| File | Profile |
|------|---------|
| `sample_human_robot_hit.bvh` | human_robot_hit (boxing mocap style) |

Set **Input type** = `bvh`, **Source profile** = `human_robot_hit` (or `auto`). **No FBX SDK needed.**

## Kimodo AMASS SMPL-X NPZ

| Requirement | Location |
|-------------|----------|
| Motion file | Your Kimodo export (`pose_body`, `root_orient`, `trans`) |
| SMPL-X body model | `gmr/assets/body_models/smplx/` (not in git) |

```bash
./scripts/install_smplx_models.sh ~/Downloads/SMPLX_NEUTRAL_2020.npz
```

Set **Input type** = `npz`, upload `.npz`. **No FBX SDK needed.** See `gmr/assets/body_models/README.md`.

## FBX

Use your own Mixamo `.fbx` (root joint `Hips`). Requires `import fbx` — see main README / prebuilt wheel in `vendor/fbx_wheels/`.
