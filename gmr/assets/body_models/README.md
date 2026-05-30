# SMPL-X body models (not included in git)

Kimodo **AMASS SMPL-X** `.npz` motion files need the official SMPL-X body model for forward kinematics before T800 retargeting.

## Install (one time)

1. Register at [SMPL-X](https://smpl-x.is.tue.mpg.de/) and download the model(s) you need.
2. From the repo root:

```bash
./scripts/install_smplx_models.sh ~/Downloads/SMPLX_NEUTRAL_2020.npz
```

Or copy manually into this folder:

```
gmr/assets/body_models/smplx/
  SMPLX_NEUTRAL.npz          # or SMPLX_NEUTRAL_2020.npz
  SMPLX_MALE.npz               # optional
  SMPLX_FEMALE.npz             # optional
```

Supported filenames: `SMPLX_{GENDER}.npz`, `SMPLX_{GENDER}_2020.npz`, or the same with `.pkl`.

## Alternative path

If models live elsewhere:

```bash
export SMPLX_BODY_MODELS=/path/to/body_models
# directory must contain smplx/SMPLX_NEUTRAL*.npz
```

Add to `.env` (see `.env.example`).

## License

SMPL-X assets are **not redistributable**. Each user must download their own copy. Model files in `smplx/` are gitignored.
