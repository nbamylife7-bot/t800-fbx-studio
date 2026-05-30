"""Post-IK foot flattening and ground snap for T800 offline retarget (FBX/BVH)."""

from __future__ import annotations

from typing import Iterable, List, Mapping, Optional, Tuple

import mujoco as mj
import numpy as np

from general_motion_retargeting.motion_grounding import (
    align_motion_root_to_ground,
    find_support_geom_ids,
    geom_lowest_z,
)

Vec3 = np.ndarray
HumanFrame = dict[str, Tuple[Vec3, np.ndarray]]

_ANKLE_PITCH = ("J04_ANKLE_PITCH_L", "J10_ANKLE_PITCH_R")
_ANKLE_ROLL = ("J05_ANKLE_ROLL_L", "J11_ANKLE_ROLL_R")
_FOOT_SUPPORT_BODIES = ("LINK_ANKLE_ROLL_L", "LINK_ANKLE_ROLL_R")
_NEUTRAL_ANKLE_PITCH = (-0.01, -0.01)


def _joint_qpos_addr(model: mj.MjModel, joint_name: str) -> Optional[int]:
    joint_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        return None
    return int(model.jnt_qposadr[joint_id])


def _support_geom_for_side(model: mj.MjModel, side: int, support_geom_ids: list[int]) -> Optional[int]:
    body_name = _FOOT_SUPPORT_BODIES[side]
    body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        return None
    for geom_id in support_geom_ids:
        if int(model.geom_bodyid[geom_id]) == body_id:
            return int(geom_id)
    return None


def snap_robot_feet_to_ground(
    model: mj.MjModel,
    qpos: np.ndarray,
    clearance: float = 0.002,
    support_geom_ids: list[int] | None = None,
) -> None:
    """Shift root Z so T800 training foot collision boxes touch the ground."""
    support_ids = support_geom_ids or find_support_geom_ids(model)
    data = mj.MjData(model)
    n = min(len(qpos), model.nq)
    data.qpos[:n] = qpos[:n]
    mj.mj_forward(model, data)

    min_bottom_z: Optional[float] = None
    for geom_id in support_ids:
        bottom_z = geom_lowest_z(model, data, geom_id)
        if min_bottom_z is None or bottom_z < min_bottom_z:
            min_bottom_z = bottom_z

    if min_bottom_z is None:
        return
    qpos[2] += float(clearance - min_bottom_z)


def _maximize_foot_flatness(
    model: mj.MjModel,
    qpos: np.ndarray,
    side: int,
    support_geom_id: int,
    *,
    pitch_steps: int = 41,
    roll_steps: int = 25,
    clearance: float = 0.002,
) -> None:
    """Pick ankle pitch/roll that flattens the training foot collision box."""
    pitch_name = _ANKLE_PITCH[side]
    roll_name = _ANKLE_ROLL[side]
    pitch_addr = _joint_qpos_addr(model, pitch_name)
    roll_addr = _joint_qpos_addr(model, roll_name)
    if pitch_addr is None or roll_addr is None:
        return

    pitch_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, pitch_name)
    roll_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, roll_name)
    pitch_lo, pitch_hi = model.jnt_range[pitch_id]
    roll_lo, roll_hi = model.jnt_range[roll_id]

    data = mj.MjData(model)
    n = min(len(qpos), model.nq)
    neutral_pitch = _NEUTRAL_ANKLE_PITCH[side]

    best_score = -1e9
    best_pitch = float(qpos[pitch_addr])
    best_roll = float(qpos[roll_addr])

    pitch_vals = np.linspace(pitch_lo, pitch_hi, pitch_steps)
    roll_vals = np.linspace(roll_lo, roll_hi, roll_steps)
    for pitch in pitch_vals:
        for roll in roll_vals:
            data.qpos[:n] = qpos[:n]
            data.qpos[pitch_addr] = float(pitch)
            data.qpos[roll_addr] = float(roll)
            mj.mj_forward(model, data)
            mat = data.geom_xmat[support_geom_id].reshape(3, 3)
            flatness = float(mat[2, 2])
            bottom_z = geom_lowest_z(model, data, support_geom_id)
            ground_penalty = max(0.0, bottom_z - clearance) * 8.0
            neutral_penalty = abs(float(pitch) - neutral_pitch) * 0.15
            score = flatness - ground_penalty - neutral_penalty
            if score > best_score:
                best_score = score
                best_pitch = float(pitch)
                best_roll = float(roll)

    qpos[pitch_addr] = best_pitch
    qpos[roll_addr] = best_roll


def flatten_robot_feet_ankles(
    model: mj.MjModel,
    qpos: np.ndarray,
    *,
    plant_height: float = 0.10,
    min_flat_dot: float = 0.96,
    clearance: float = 0.002,
    support_geom_ids: list[int] | None = None,
) -> None:
    """Adjust ankle pitch/roll so planted feet use flat sole collision boxes."""
    support_ids = support_geom_ids or find_support_geom_ids(model)
    data = mj.MjData(model)
    n = min(len(qpos), model.nq)
    data.qpos[:n] = qpos[:n]
    mj.mj_forward(model, data)

    for side in range(2):
        geom_id = _support_geom_for_side(model, side, support_ids)
        if geom_id is None:
            continue
        bottom_z = geom_lowest_z(model, data, geom_id)
        if bottom_z > plant_height:
            continue
        mat = data.geom_xmat[geom_id].reshape(3, 3)
        if float(mat[2, 2]) >= min_flat_dot and bottom_z <= clearance + 0.01:
            continue
        _maximize_foot_flatness(model, qpos, side, geom_id, clearance=clearance)

    snap_robot_feet_to_ground(model, qpos, clearance=clearance, support_geom_ids=support_ids)


def postprocess_robot_qpos_feet(
    model: mj.MjModel,
    qpos: np.ndarray,
    *,
    flatten: bool = True,
    clearance: float = 0.002,
) -> np.ndarray:
    """Return a copy of qpos with planted feet flattened and snapped to ground."""
    out = np.asarray(qpos, dtype=np.float64).copy()
    if flatten:
        flatten_robot_feet_ankles(model, out, clearance=clearance)
    else:
        snap_robot_feet_to_ground(model, out, clearance=clearance)
    return out


def postprocess_robot_qpos_list(
    model: mj.MjModel,
    qpos_list: Iterable[np.ndarray],
    *,
    flatten: bool = True,
    clearance: float = 0.002,
) -> List[np.ndarray]:
    support_ids = find_support_geom_ids(model)
    out = [
        postprocess_robot_qpos_feet(model, q, flatten=flatten, clearance=clearance)
        for q in qpos_list
    ]
    motion = {
        "fps": 30,
        "root_pos": np.array([q[:3] for q in out], dtype=np.float64),
        "root_rot": np.array([q[3:7][[1, 2, 3, 0]] for q in out], dtype=np.float64),
        "dof_pos": np.array([q[7:] for q in out], dtype=np.float64),
        "local_body_pos": None,
        "link_body_list": None,
    }
    grounded, _stats = align_motion_root_to_ground(
        motion,
        model,
        clearance=clearance,
        mode="per_frame",
        inplace=False,
        support_geom_ids=support_ids,
    )
    reground: list[np.ndarray] = []
    for i in range(len(out)):
        q = out[i].copy()
        q[2] = float(grounded["root_pos"][i, 2])
        reground.append(q)
    return reground


def adjust_human_foot_sole_targets(
    frames: List[HumanFrame],
    *,
    plant_height: float = 0.20,
) -> None:
    """Move planted foot IK targets from ankle height down to sole (mid-foot on ground).

    Mixamo ankle joints often sit above the toe while the foot is visually flat.
    Matching ankle height makes T800 pitch onto its toes even when orientation is flat.
    """
    for foot_key, toe_key, mod_key in (
        ("LeftFoot", "LeftToeBase", "LeftFootMod"),
        ("RightFoot", "RightToeBase", "RightFootMod"),
    ):
        zs = [
            min(float(f[foot_key][0][2]), float(f[toe_key][0][2]))
            for f in frames
            if foot_key in f and toe_key in f
        ]
        ground = min(zs) if zs else 0.0

        for f in frames:
            if foot_key not in f or toe_key not in f:
                continue
            ankle = np.asarray(f[foot_key][0], dtype=np.float64)
            toe = np.asarray(f[toe_key][0], dtype=np.float64)
            if (float(min(ankle[2], toe[2])) - ground) >= plant_height:
                continue
            sole = (ankle + toe) * 0.5
            sole[2] = ground
            quat = f.get(mod_key, f[foot_key])[1]
            f[mod_key] = (sole, quat.copy())
            f[foot_key] = (sole.copy(), f[foot_key][1].copy())
