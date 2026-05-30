"""PKL motion load/save helpers (no motion-blend)."""

from __future__ import annotations

import pickle
from typing import Sequence

import numpy as np


def load_motion_pkl(path: str) -> dict:
    with open(path, "rb") as fh:
        return pickle.load(fh)


def motion_to_qpos_list(motion: dict) -> tuple[list[np.ndarray], int]:
    root_pos = np.asarray(motion["root_pos"], dtype=np.float64)
    root_rot_xyzw = np.asarray(motion["root_rot"], dtype=np.float64)
    dof = np.asarray(motion["dof_pos"], dtype=np.float64)
    fps = int(motion.get("fps", 30))
    frames: list[np.ndarray] = []
    for i in range(len(root_pos)):
        q = np.concatenate(
            [
                root_pos[i],
                root_rot_xyzw[i][[3, 0, 1, 2]],  # xyzw -> wxyz
                dof[i],
            ]
        )
        frames.append(q)
    return frames, fps


def qpos_list_to_motion(qpos_list: Sequence[np.ndarray], fps: int) -> dict:
    root_pos = np.array([q[:3] for q in qpos_list], dtype=np.float64)
    root_rot = np.array([q[3:7][[1, 2, 3, 0]] for q in qpos_list], dtype=np.float64)
    dof_pos = np.array([q[7:] for q in qpos_list], dtype=np.float64)
    return {
        "fps": int(fps),
        "root_pos": root_pos,
        "root_rot": root_rot,
        "dof_pos": dof_pos,
        "local_body_pos": None,
        "link_body_list": None,
    }


def save_motion_pkl(path: str, motion: dict) -> None:
    with open(path, "wb") as fh:
        pickle.dump(motion, fh)
