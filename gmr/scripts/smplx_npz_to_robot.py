"""AMASS / Kimodo SMPL-X NPZ → T800 robot motion PKL.

Supports Kimodo exports in AMASS SMPL-X layout (pose_body, root_orient, trans,
pose_hand, mocap_frame_rate, …). Kimodo native NPZ (posed_joints) is detected
and rejected with a clear message — use AMASS export from Kimodo instead.
"""

from __future__ import annotations

import os
import pathlib
import pickle
from typing import Callable, Literal

import numpy as np

import scripts.bvh_to_robot as bvr
import scripts.fbx_to_robot as fr
import scripts.t800_foot_postprocess as foot_pp
from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting.utils.smpl import (
    _as_scalar_fps,
    get_smplx_data_offline_fast,
    load_smplx_file,
)

AMASS_SMPLX_KEYS = ("pose_body", "root_orient", "trans")
KIMODO_NATIVE_KEYS = ("posed_joints", "joint_names")

NpzKind = Literal["amass_smplx", "kimodo_native", "unknown"]


def detect_npz_kind(npz_path: str) -> NpzKind:
    data = np.load(npz_path, allow_pickle=True)
    keys = set(data.files)
    if all(k in keys for k in AMASS_SMPLX_KEYS):
        return "amass_smplx"
    if any(k in keys for k in KIMODO_NATIVE_KEYS):
        return "kimodo_native"
    return "unknown"


def describe_npz_kind(kind: NpzKind) -> str:
    if kind == "amass_smplx":
        return "AMASS SMPL-X (Kimodo export compatible)"
    if kind == "kimodo_native":
        return "Kimodo native joints NPZ (not supported — re-export as AMASS SMPL-X)"
    return "unknown NPZ layout"


def resolve_smplx_body_models_path() -> pathlib.Path:
    env = os.environ.get("SMPLX_BODY_MODELS", "").strip()
    if env:
        path = pathlib.Path(env).expanduser()
        if path.is_dir():
            return path
        raise FileNotFoundError(
            f"SMPLX_BODY_MODELS={env} is not a directory. "
            "Point it at the folder that contains smplx/SMPLX_NEUTRAL.pkl (or .npz)."
        )

    default = pathlib.Path(__file__).resolve().parent.parent / "assets" / "body_models"
    smplx_dir = default / "smplx"
    if smplx_dir.is_dir():
        return default
    raise FileNotFoundError(
        "SMPL-X body models not found. Download from https://smpl-x.is.tue.mpg.de/ "
        f"and install under {default}/smplx/ "
        "(SMPLX_NEUTRAL.pkl or .npz, plus MALE/FEMALE if needed), "
        "or set SMPLX_BODY_MODELS to your models directory."
    )


def convert_smplx_amass_npz(
    npz_path: str,
    *,
    tgt_fps: int = 30,
    human_height: float = 0.0,
    auto_ground: bool = True,
    flatten_feet: bool = False,
    robot: str = "t800",
    output_path: pathlib.Path,
    status: Callable[[str], None] = lambda _msg: None,
) -> tuple[list[np.ndarray], int]:
    kind = detect_npz_kind(npz_path)
    if kind == "kimodo_native":
        raise RuntimeError(
            "This NPZ is Kimodo native format (posed_joints). "
            "In Kimodo, export as AMASS SMPL-X NPZ instead (pose_body / root_orient / trans)."
        )
    if kind != "amass_smplx":
        raise RuntimeError(
            "Unrecognized NPZ layout. Expected AMASS SMPL-X keys: "
            + ", ".join(AMASS_SMPLX_KEYS)
        )

    body_models = resolve_smplx_body_models_path()
    status(f"Loading AMASS SMPL-X {pathlib.Path(npz_path).name} …")
    smplx_data, body_model, smplx_output, detected_height = load_smplx_file(
        npz_path, body_models
    )
    src_fps = _as_scalar_fps(
        smplx_data["mocap_frame_rate"] if "mocap_frame_rate" in smplx_data.files else None
    )
    num_frames = int(smplx_data["pose_body"].shape[0])
    status(
        f"Parsed {num_frames} frames @ {src_fps:.0f} fps → retarget @ {int(tgt_fps)} fps …"
    )

    human_frames, aligned_fps = get_smplx_data_offline_fast(
        smplx_data, body_model, smplx_output, tgt_fps=int(tgt_fps)
    )
    if not human_frames:
        raise RuntimeError("No frames after SMPL-X forward kinematics / FPS alignment.")

    height = float(human_height) if human_height > 0 else float(detected_height)
    retargeter = GMR(
        actual_human_height=height,
        src_human="smplx",
        tgt_robot=robot,
        ik_safety_break=False,
        verbose=False,
    )

    if auto_ground:
        ground = bvr.estimate_ground_offset(retargeter, human_frames)
        retargeter.set_ground_offset(ground)

    qpos_frames: list[np.ndarray] = []
    for i, frame in enumerate(human_frames):
        qpos = retargeter.retarget(frame, frame_index=i)
        if flatten_feet:
            qpos = foot_pp.postprocess_robot_qpos_feet(retargeter.model, qpos, flatten=True)
        qpos_frames.append(qpos.copy())

    motion_fps = int(round(aligned_fps)) if aligned_fps else int(tgt_fps)
    motion = fr._build_motion_data_from_qpos_list(qpos_frames, motion_fps)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "wb") as fh:
        pickle.dump(motion, fh)
    status(f"Saved {output_path.name} ({len(qpos_frames)} frames @ {motion_fps} fps).")
    return qpos_frames, motion_fps
