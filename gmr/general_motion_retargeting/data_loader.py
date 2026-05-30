
import pickle
from pathlib import Path

import numpy as np


def _load_qpos_npy(motion_file, motion_fps):
    if motion_fps is None:
        raise ValueError("Loading .npy robot motion requires explicit motion_fps.")
    qpos = np.load(motion_file)
    if qpos.ndim != 2 or qpos.shape[1] < 7:
        raise ValueError(f"Expected .npy qpos with shape (frames, >=7), got {qpos.shape}.")
    return {
        "fps": motion_fps,
        "root_pos": qpos[:, :3],
        "root_rot": qpos[:, 3:7],
        "dof_pos": qpos[:, 7:],
        "local_body_pos": None,
        "link_body_list": None,
    }


def _load_rl_trajectory_npz(motion_file):
    npz = np.load(motion_file)
    required_keys = {"joint_pos", "body_pos_w", "body_quat_w", "fps"}
    missing_keys = sorted(required_keys.difference(npz.files))
    if missing_keys:
        raise ValueError(
            f"Expected RL trajectory .npz with keys {sorted(required_keys)}, "
            f"missing {missing_keys}. Found keys: {npz.files}."
        )

    dof_pos = np.asarray(npz["joint_pos"])
    body_pos_w = np.asarray(npz["body_pos_w"])
    body_quat_w = np.asarray(npz["body_quat_w"])
    fps = float(np.asarray(npz["fps"]).reshape(-1)[0])

    if dof_pos.ndim != 2:
        raise ValueError(f"Expected joint_pos with shape (frames, dofs), got {dof_pos.shape}.")
    if body_pos_w.ndim != 3 or body_pos_w.shape[0] != dof_pos.shape[0] or body_pos_w.shape[2] != 3:
        raise ValueError(
            "Expected body_pos_w with shape (frames, bodies, 3) and the same frame count "
            f"as joint_pos, got {body_pos_w.shape} and {dof_pos.shape}."
        )
    if body_quat_w.ndim != 3 or body_quat_w.shape[0] != dof_pos.shape[0] or body_quat_w.shape[2] != 4:
        raise ValueError(
            "Expected body_quat_w with shape (frames, bodies, 4) and the same frame count "
            f"as joint_pos, got {body_quat_w.shape} and {dof_pos.shape}."
        )

    return {
        "fps": fps,
        "root_pos": body_pos_w[:, 0],
        "root_rot": body_quat_w[:, 0],
        "dof_pos": dof_pos,
        "local_body_pos": None,
        "link_body_list": None,
    }


def load_robot_motion(motion_file, motion_fps=None):
    """
    Load robot motion data from a pickle file, MuJoCo qpos .npy file, or RL trajectory .npz file.
    """
    motion_file = Path(motion_file)
    if motion_file.suffix.lower() == ".npy":
        motion_data = _load_qpos_npy(motion_file, motion_fps)
        motion_fps = motion_data["fps"]
        motion_root_pos = motion_data["root_pos"]
        motion_root_rot = motion_data["root_rot"]
        motion_dof_pos = motion_data["dof_pos"]
        motion_local_body_pos = None
        motion_link_body_list = None
        return motion_data, motion_fps, motion_root_pos, motion_root_rot, motion_dof_pos, motion_local_body_pos, motion_link_body_list

    if motion_file.suffix.lower() == ".npz":
        motion_data = _load_rl_trajectory_npz(motion_file)
        motion_fps = motion_data["fps"]
        motion_root_pos = motion_data["root_pos"]
        motion_root_rot = motion_data["root_rot"]
        motion_dof_pos = motion_data["dof_pos"]
        motion_local_body_pos = None
        motion_link_body_list = None
        return motion_data, motion_fps, motion_root_pos, motion_root_rot, motion_dof_pos, motion_local_body_pos, motion_link_body_list

    with open(motion_file, "rb") as f:
        motion_data = pickle.load(f)
    motion_fps = motion_data["fps"]
    motion_root_pos = motion_data["root_pos"]
    motion_root_rot = motion_data["root_rot"][:, [3, 0, 1, 2]] # from xyzw to wxyz
    motion_dof_pos = motion_data["dof_pos"]
    motion_local_body_pos = motion_data["local_body_pos"]
    motion_link_body_list = motion_data["link_body_list"]
    return motion_data, motion_fps, motion_root_pos, motion_root_rot, motion_dof_pos, motion_local_body_pos, motion_link_body_list
