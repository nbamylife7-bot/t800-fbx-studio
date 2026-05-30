from __future__ import annotations

import copy

import numpy as np
from scipy.spatial.transform import Rotation as R


def resolve_ik_safety_break(disable_ik_safety_break: bool) -> bool:
    return not disable_ik_safety_break


def resolve_max_iter(max_iter: int | None) -> int:
    if max_iter is None:
        return 10
    if max_iter <= 0:
        raise ValueError("--max_iter must be greater than 0.")
    return int(max_iter)


def resolve_actual_human_height(loader_human_height: float | None, source_profile: str) -> float | None:
    """Return the height passed into GMR for source-profile scaling.

    `human_robot_hit` BVH files already require unit normalization in the loader.
    Feeding their motion-derived height back into GMR shrinks the whole target
    again by roughly `height / human_height_assumption`, which makes T800 crouch.
    """

    if source_profile == "human_robot_hit":
        return None
    return loader_human_height


def _root_rotation_from_hips(frame: dict) -> R | None:
    if "LeftUpLeg" not in frame or "RightUpLeg" not in frame:
        return None

    left_hip = np.asarray(frame["LeftUpLeg"][0], dtype=np.float64)
    right_hip = np.asarray(frame["RightUpLeg"][0], dtype=np.float64)
    left_axis = left_hip - right_hip
    left_axis[2] = 0.0
    left_axis_norm = np.linalg.norm(left_axis)
    if left_axis_norm < 1e-9:
        return None
    left_axis = left_axis / left_axis_norm

    up_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    forward_axis = np.cross(left_axis, up_axis)
    forward_axis_norm = np.linalg.norm(forward_axis)
    if forward_axis_norm < 1e-9:
        return None
    forward_axis = forward_axis / forward_axis_norm
    left_axis = np.cross(up_axis, forward_axis)

    return R.from_matrix(np.column_stack([forward_axis, left_axis, up_axis]))


def calibrate_human_robot_hit_frame(frame: dict) -> dict:
    """Calibrate official BVH root orientation to a T800/MuJoCo-friendly frame.

    The official boxing BVH root quaternion is not a reliable T800 body-frame
    guide. For retargeting we construct a stable pelvis frame: local +Y points
    from right hip to left hip, and local +Z is world-up.
    """

    calibrated_frame = copy.deepcopy(frame)
    root_rotation = _root_rotation_from_hips(calibrated_frame)
    if root_rotation is None or "Hips" not in calibrated_frame:
        return calibrated_frame

    calibrated_frame["Hips"][1] = root_rotation.as_quat(scalar_first=True)
    return calibrated_frame


def calibrate_human_robot_hit_frames(frames: list[dict]) -> list[dict]:
    return [calibrate_human_robot_hit_frame(frame) for frame in frames]
