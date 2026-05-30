import argparse
import pathlib
import pickle
import sys
import time
from typing import Dict, List, Mapping, Tuple

import numpy as np
from rich import print
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm

SCRIPT_DIR = pathlib.Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
THIRD_PARTY_ROOT = REPO_ROOT / "third_party"
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if str(THIRD_PARTY_ROOT) not in sys.path:
    sys.path.insert(0, str(THIRD_PARTY_ROOT))

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting import RobotMotionViewer
from general_motion_retargeting.motion_retarget_options import (
    resolve_ik_safety_break,
    resolve_max_iter,
)
import scripts.t800_foot_postprocess as foot_pp
Vec3 = np.ndarray
QuatWXYZ = np.ndarray
HumanFrame = Dict[str, Tuple[Vec3, QuatWXYZ]]


def _load_fbx_as_human_frames(
    fbx_file: str,
    *,
    fps: int,
    root_joint: str,
) -> List[HumanFrame]:
    """Use PoseLib FBX parser and convert to GMR-compatible frame list."""
    try:
        from poselib.skeleton.skeleton3d import SkeletonMotion
    except ModuleNotFoundError as exc:
        if "torch" in str(exc):
            raise ModuleNotFoundError(
                "Missing dependency 'torch' for FBX import. Install torch in the same Python "
                "environment used to run this script."
            ) from exc
        raise

    motion = SkeletonMotion.from_fbx(
        fbx_file_path=fbx_file,
        root_joint=root_joint,
        fps=fps,
    )
    # PoseLib's helper assumes specific naming with '_' separator.
    # Build frame dict directly from tensors to support Mixamo-style names.
    from poselib.core.rotation3d import quat_mul_norm, quat_rotate
    import torch

    # Match PoseLib `to_retarget_motion_file()` world convention.
    rot_fix = torch.tensor([0.70711, 0.0, 0.0, 0.70711], dtype=motion.global_rotation.dtype)
    global_positions = quat_rotate(rot_fix, motion.global_translation).detach().cpu().numpy() / 100.0
    global_quaternions = quat_mul_norm(rot_fix, motion.global_rotation).detach().cpu().numpy()

    joint_names = list(motion.skeleton_tree.node_names)
    raw_frames: List[dict] = []
    for frame_idx in range(global_positions.shape[0]):
        motion_frame: dict = {}
        for joint_idx, joint_name in enumerate(joint_names):
            canonical = _canonical_joint_name(joint_name)
            quat_xyzw = global_quaternions[frame_idx, joint_idx]
            quat_wxyz = [float(quat_xyzw[3]), float(quat_xyzw[0]), float(quat_xyzw[1]), float(quat_xyzw[2])]
            motion_frame[canonical] = [
                global_positions[frame_idx, joint_idx].tolist(),
                quat_wxyz,
            ]
        raw_frames.append(motion_frame)

    return [_normalize_human_frame(frame) for frame in raw_frames]


def _arr3(values: Mapping) -> Vec3:
    return np.asarray(values, dtype=np.float64)


def _arr4(values: Mapping) -> QuatWXYZ:
    quat = np.asarray(values, dtype=np.float64)
    norm = float(np.linalg.norm(quat))
    if norm > 1e-8:
        quat = quat / norm
    else:
        quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return quat


def _canonical_joint_name(name: str) -> str:
    # Common namespace separators in FBX rigs: "mixamorig:Hips", "Armature|Hips", etc.
    base = str(name).split("|")[-1].split(":")[-1]
    return base


def _copy_alias(target: HumanFrame, dst: str, candidates: List[str]) -> None:
    if dst in target:
        return
    for name in candidates:
        if name in target:
            pos, quat = target[name]
            target[dst] = (pos.copy(), quat.copy())
            return


def _normalize_human_frame(frame: Mapping[str, List[List[float]]]) -> HumanFrame:
    """Harmonize PoseLib FBX joint names with T800 BVH route expectations."""
    normalized: HumanFrame = {}
    for name, value in frame.items():
        if not isinstance(value, (list, tuple)) or len(value) != 2:
            continue
        pos, quat = value
        normalized[name] = (_arr3(pos), _arr4(quat))

    # Common skeleton aliases.
    _copy_alias(normalized, "Hips", ["Pelvis"])
    _copy_alias(normalized, "Spine", ["Spine0", "Spine1", "Spine01"])
    _copy_alias(normalized, "Spine1", ["Spine1", "Spine2", "Spine02", "Chest"])
    _copy_alias(normalized, "Spine2", ["Spine2", "Spine3", "UpperChest", "Chest"])
    _copy_alias(normalized, "Neck", ["Neck", "Neck1"])
    _copy_alias(normalized, "Head", ["Head", "HeadEnd"])

    _copy_alias(normalized, "LeftUpLeg", ["LeftUpLeg", "LeftThigh"])
    _copy_alias(normalized, "RightUpLeg", ["RightUpLeg", "RightThigh"])
    _copy_alias(normalized, "LeftLeg", ["LeftLeg", "LeftCalf", "LeftShin"])
    _copy_alias(normalized, "RightLeg", ["RightLeg", "RightCalf", "RightShin"])
    _copy_alias(normalized, "LeftFoot", ["LeftFoot", "LeftAnkle"])
    _copy_alias(normalized, "RightFoot", ["RightFoot", "RightAnkle"])
    _copy_alias(normalized, "LeftToeBase", ["LeftToeBase", "LeftToe"])
    _copy_alias(normalized, "RightToeBase", ["RightToeBase", "RightToe"])

    # Mixamo and LaFAN1 share arm-chain semantics:
    # LeftArm = shoulder/upper-arm joint, LeftForeArm = elbow, LeftHand = wrist.
    # Keep the real joints; do NOT synthesize the elbow.
    _copy_alias(normalized, "LeftShoulder", ["LeftShoulder", "LeftCollar"])
    _copy_alias(normalized, "RightShoulder", ["RightShoulder", "RightCollar"])
    _copy_alias(normalized, "LeftArm", ["LeftArm", "LeftUpperArm"])
    _copy_alias(normalized, "RightArm", ["RightArm", "RightUpperArm"])
    _copy_alias(normalized, "LeftForeArm", ["LeftForeArm", "LeftLowerArm"])
    _copy_alias(normalized, "RightForeArm", ["RightForeArm", "RightLowerArm"])
    _copy_alias(normalized, "LeftHand", ["LeftHand", "LeftWrist"])
    _copy_alias(normalized, "RightHand", ["RightHand", "RightWrist"])

    # T800 configs rely on LeftFootMod/RightFootMod.
    if "LeftFoot" in normalized:
        toe_quat = normalized.get("LeftToeBase", normalized["LeftFoot"])[1]
        normalized["LeftFootMod"] = (normalized["LeftFoot"][0].copy(), toe_quat.copy())
    if "RightFoot" in normalized:
        toe_quat = normalized.get("RightToeBase", normalized["RightFoot"])[1]
        normalized["RightFootMod"] = (normalized["RightFoot"][0].copy(), toe_quat.copy())

    return normalized


def _detect_src_human(frames: List[HumanFrame]) -> str:
    """Heuristic source-profile detection for FBX rigs."""
    if not frames:
        return "bvh_lafan1"
    keys = set(frames[0].keys())
    # Mixamo-like rigs and most DCC exports map better to the LaFAN1 config.
    mixamo_markers = {"HeadTop_End", "LeftHandIndex1", "RightHandIndex1"}
    if any(marker in keys for marker in mixamo_markers):
        return "bvh_lafan1"
    # Official competition profile tends to include richer spine/neck/toe schema.
    hit_markers = {"Spine3", "Neck1", "LeftToeBase", "RightToeBase"}
    if hit_markers.issubset(keys):
        return "bvh_human_robot_hit"
    return "bvh_lafan1"


# Full-body position-task weights for T800.
# FBX/Mixamo bind-frame quaternions do not match the orientation-driven GMR
# config, so we drive the retarget by POSITIONS only (orientation cost = 0).
T800_FBX_POSITION_WEIGHTS: dict[str, float] = {
    "Hips": 60.0,
    "Spine2": 12.0,
    "Head": 4.0,
    "LeftUpLeg": 12.0,
    "RightUpLeg": 12.0,
    "LeftLeg": 12.0,
    "RightLeg": 12.0,
    "LeftFootMod": 90.0,
    "RightFootMod": 90.0,
    "LeftArm": 6.0,
    "RightArm": 6.0,
    "LeftForeArm": 22.0,
    "RightForeArm": 22.0,
    "LeftHand": 45.0,
    "RightHand": 45.0,
}


def _apply_position_task_mode(retargeter: GMR, weights: dict[str, float]) -> None:
    """Drive IK by positions only (zero orientation), skeleton-agnostic.

    Mirrors the validated live T800 path: foreign bind-frame quaternions are
    ignored so a Mixamo/other FBX rig no longer tilts the whole robot.
    """
    for task_map_name, config_name, pos_offsets_name in [
        ("human_body_to_task1", "task_config1", "pos_offsets1"),
        ("human_body_to_task2", "task_config2", "pos_offsets2"),
    ]:
        task_map = getattr(retargeter, task_map_name, {})
        task_config = getattr(retargeter, config_name, {})
        pos_offsets = getattr(retargeter, pos_offsets_name, {})
        for body_name, task in task_map.items():
            position_cost = float(weights.get(body_name, 0.0))
            if hasattr(task, "set_position_cost"):
                task.set_position_cost(position_cost)
            if hasattr(task, "set_orientation_cost"):
                task.set_orientation_cost(0.0)
            if body_name in task_config:
                task_config[body_name]["position_cost"] = position_cost
                task_config[body_name]["orientation_cost"] = 0.0
                task_config[body_name]["effective_pos_offset"] = [0.0, 0.0, 0.0]
                task_config[body_name]["rot_offset_wxyz"] = [1.0, 0.0, 0.0, 0.0]
            if body_name in pos_offsets:
                pos_offsets[body_name] = np.zeros(3, dtype=np.float64)


def _unit(vec: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(vec))
    return vec / n if n > 1e-9 else vec


def _matrix_to_wxyz(matrix: np.ndarray) -> np.ndarray:
    from scipy.spatial.transform import Rotation as R

    xyzw = R.from_matrix(matrix).as_quat()
    return np.array([xyzw[3], xyzw[0], xyzw[1], xyzw[2]], dtype=np.float64)


def _clean_pelvis_quat(f: HumanFrame) -> np.ndarray | None:
    if not all(k in f for k in ("LeftUpLeg", "RightUpLeg", "Hips", "Spine2")):
        return None
    left = _unit(f["LeftUpLeg"][0] - f["RightUpLeg"][0])
    up = _unit(f["Spine2"][0] - f["Hips"][0])
    fwd = _unit(np.cross(left, up))
    if np.linalg.norm(fwd) < 1e-6:
        return None
    left = _unit(np.cross(up, fwd))
    up = _unit(np.cross(fwd, left))
    return _matrix_to_wxyz(np.column_stack([fwd, left, up]))


def _foot_target_quats(
    frames: List[HumanFrame],
    ankle_key: str,
    toe_key: str,
    *,
    plant_height: float = 0.20,
) -> list[np.ndarray | None]:
    """Per-frame foot orientation: SOLE-FLAT when planted, follow foot when airborne.

    A flat human foot bone already points ~30 deg down (ankle->toe), so copying it
    verbatim makes the robot stand on its toes. When the foot is near the ground and
    the body is upright, we force the sole flat (up=+Z); otherwise (kicks, jumps,
    inverted flair) we let the foot follow its real direction.
    """
    zs = [
        min(float(f[ankle_key][0][2]), float(f[toe_key][0][2]))
        for f in frames
        if ankle_key in f and toe_key in f
    ]
    ground = min(zs) if zs else 0.0

    out: list[np.ndarray | None] = []
    for f in frames:
        if ankle_key not in f or toe_key not in f:
            out.append(None)
            continue
        ankle = np.asarray(f[ankle_key][0], dtype=np.float64)
        toe = np.asarray(f[toe_key][0], dtype=np.float64)

        upright = True
        pelv = _clean_pelvis_quat(f)
        if pelv is not None:
            Rp = R.from_quat([pelv[1], pelv[2], pelv[3], pelv[0]])
            upright = float(Rp.apply([0.0, 0.0, 1.0])[2]) > 0.5

        planted = (float(ankle[2]) - ground) < plant_height and upright
        if planted:
            fwd = np.array([toe[0] - ankle[0], toe[1] - ankle[1], 0.0], dtype=np.float64)
            if np.linalg.norm(fwd) < 1e-6:
                fwd = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        else:
            fwd = toe - ankle

        fwd = _unit(fwd)
        up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        left = _unit(np.cross(up, fwd))
        if np.linalg.norm(left) < 1e-6:
            out.append(None)
            continue
        up = _unit(np.cross(fwd, left))
        out.append(_matrix_to_wxyz(np.column_stack([fwd, left, up])))
    return out


# (human body, ankle key, toe key) for clean foot orientation.
_FOOT_ORIENTATION_BODIES = [
    ("LeftFootMod", "LeftFoot", "LeftToeBase"),
    ("RightFootMod", "RightFoot", "RightToeBase"),
]


def _apply_clean_orientation_targets(
    retargeter: GMR,
    frames: List[HumanFrame],
    *,
    pelvis_cost: float = 40.0,
    foot_cost: float = 50.0,
    include_pelvis: bool = True,
    include_feet: bool = True,
) -> None:
    """Add position-derived orientation for pelvis + feet (fixes torso plane / foot twist).

    Mixamo bind quaternions are unreliable, so we synthesize clean world-frame
    orientations from joint positions and auto-calibrate the bind offset at frame 0
    (clean_quat0 -> robot link rest orientation), keeping the rig skeleton-agnostic.
    """
    import mujoco as mj
    from scipy.spatial.transform import Rotation as R

    model = retargeter.model
    data = retargeter.configuration.data
    mj.mj_forward(model, data)

    targets: dict[str, list[np.ndarray]] = {}
    body_cost: dict[str, float] = {}

    def _collect(body_name: str, quats: list[np.ndarray | None], cost: float) -> None:
        if any(q is None for q in quats):
            return
        targets[body_name] = [q for q in quats if q is not None]
        body_cost[body_name] = cost

    if include_pelvis:
        _collect("LINK_BASE_pelvis", [_clean_pelvis_quat(f) for f in frames], pelvis_cost)
        # Pelvis body in the task map is the human "Hips".
        if "LINK_BASE_pelvis" in targets:
            targets["Hips"] = targets.pop("LINK_BASE_pelvis")
            body_cost["Hips"] = body_cost.pop("LINK_BASE_pelvis")

    if include_feet:
        for body_name, ankle_key, toe_key in _FOOT_ORIENTATION_BODIES:
            _collect(body_name, _foot_target_quats(frames, ankle_key, toe_key), foot_cost)

    for task_map_name, config_name, rot_offsets_name in [
        ("human_body_to_task1", "task_config1", "rot_offsets1"),
        ("human_body_to_task2", "task_config2", "rot_offsets2"),
    ]:
        task_map = getattr(retargeter, task_map_name, {})
        task_config = getattr(retargeter, config_name, {})
        rot_offsets = getattr(retargeter, rot_offsets_name, {})
        body_to_frame = getattr(retargeter, f"task{task_map_name[-1]}_body_to_frame", {})
        for body_name, cost in body_cost.items():
            if body_name not in task_map:
                continue
            link_name = body_to_frame.get(body_name)
            if link_name is None:
                continue
            body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, link_name)
            if body_id < 0:
                continue
            # Clean orientation is built in world axes (X=forward, Y=left, Z=up).
            # The robot link rest frame uses the same convention, so the target
            # world orientation equals the clean human orientation directly:
            #   target = R_human_clean * rot_offset, with rot_offset = R_link_rest.
            # (Do NOT calibrate against animation frame 0 — that desyncs facing
            #  from the world-space position targets and throws limbs behind/crossed.)
            link_rest_wxyz = np.array(data.xquat[body_id], dtype=np.float64)
            r_link = R.from_quat([link_rest_wxyz[1], link_rest_wxyz[2], link_rest_wxyz[3], link_rest_wxyz[0]])
            rot_offsets[body_name] = r_link
            task = task_map[body_name]
            if hasattr(task, "set_orientation_cost"):
                task.set_orientation_cost(float(cost))
            if body_name in task_config:
                task_config[body_name]["orientation_cost"] = float(cost)

    # Overwrite frame quaternions for these bodies with the clean orientation.
    for body_name, quats in targets.items():
        for i, f in enumerate(frames):
            if body_name in f:
                f[body_name] = (f[body_name][0], quats[i])


def _estimate_ground_offset(retargeter: GMR, frames: List[HumanFrame]) -> float:
    offset = np.inf
    for human_data in frames:
        human_data = retargeter.to_numpy(human_data)
        human_data = retargeter.scale_human_data(
            human_data, retargeter.human_root_name, retargeter.human_scale_table
        )
        human_data = retargeter.offset_human_data(
            human_data, retargeter.pos_offsets1, retargeter.rot_offsets1
        )
        for pos, _quat in human_data.values():
            if pos[2] < offset:
                offset = pos[2]
    return float(offset)


def _build_motion_data_from_qpos_list(qpos_list: List[np.ndarray], motion_fps: int) -> dict:
    root_pos = np.array([qpos[:3] for qpos in qpos_list])
    root_rot = np.array([qpos[3:7][[1, 2, 3, 0]] for qpos in qpos_list])  # wxyz -> xyzw
    dof_pos = np.array([qpos[7:] for qpos in qpos_list])
    return {
        "fps": motion_fps,
        "root_pos": root_pos,
        "root_rot": root_rot,
        "dof_pos": dof_pos,
        "local_body_pos": None,
        "link_body_list": None,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Retarget FBX motion to robot directly (no Blender BVH conversion)."
    )
    parser.add_argument("--fbx_file", required=True, help="Input FBX motion file.")
    parser.add_argument(
        "--robot",
        choices=[
            "unitree_g1",
            "unitree_g1_with_hands",
            "booster_t1",
            "stanford_toddy",
            "fourier_n1",
            "engineai_pm01",
            "pal_talos",
            "t800",
            "t800_transparent",
        ],
        default="t800",
    )
    parser.add_argument(
        "--src-human",
        choices=["auto", "bvh_human_robot_hit", "bvh_lafan1"],
        default="auto",
        help="IK profile route used by GMR after FBX joint aliasing.",
    )
    parser.add_argument(
        "--retarget-mode",
        choices=["position", "orientation"],
        default="position",
        help=(
            "position: drive IK by joint positions only (recommended for FBX/Mixamo; "
            "ignores foreign bind-frame quaternions). orientation: original GMR bvh route."
        ),
    )
    parser.add_argument(
        "--no-foot-flatten",
        action="store_true",
        help="Disable keeping planted soles flat (feet may stand on toes).",
    )
    parser.add_argument(
        "--no-pelvis-orientation",
        action="store_true",
        help="Disable root/pelvis orientation stabilization (pure position IK; root may flip).",
    )
    parser.add_argument("--root-joint", default="Hips", help="FBX root joint name for PoseLib.")
    parser.add_argument("--fps", type=int, default=30, help="Output motion FPS.")
    parser.add_argument("--actual_human_height", type=float, default=1.75)
    parser.add_argument("--headless", action="store_true", help="Run without MuJoCo viewer.")
    parser.add_argument("--rate_limit", action="store_true", default=False)
    parser.add_argument("--save_path", default=None, help="Path to save retargeted robot motion PKL.")
    parser.add_argument("--record_video", action="store_true", default=False)
    parser.add_argument("--video_path", type=str, default="videos/fbx_to_robot.mp4")
    parser.add_argument("--disable_ik_safety_break", action="store_true", default=False)
    parser.add_argument("--max_iter", type=int, default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.save_path is not None:
        save_dir = pathlib.Path(args.save_path).expanduser().resolve().parent
        save_dir.mkdir(parents=True, exist_ok=True)

    print(f"[FBX] Loading: {args.fbx_file}")
    frames = _load_fbx_as_human_frames(
        args.fbx_file,
        fps=int(args.fps),
        root_joint=str(args.root_joint),
    )
    if len(frames) == 0:
        raise RuntimeError("No frames parsed from FBX.")
    print(f"[FBX] Parsed frames: {len(frames)}")

    src_human = args.src_human
    if src_human == "auto":
        src_human = _detect_src_human(frames)
        print(f"[FBX] Auto-detected src-human profile: {src_human}")

    retargeter = GMR(
        src_human=src_human,
        tgt_robot=args.robot,
        actual_human_height=float(args.actual_human_height),
        ik_safety_break=resolve_ik_safety_break(args.disable_ik_safety_break),
        max_iter=resolve_max_iter(args.max_iter),
    )

    if args.retarget_mode == "position" and args.robot in ("t800", "t800_transparent"):
        _apply_position_task_mode(retargeter, T800_FBX_POSITION_WEIGHTS)
        print("[FBX] Using POSITION-driven IK (limb orientation costs zeroed).")
        if not args.no_pelvis_orientation:
            foot_pp.adjust_human_foot_sole_targets(frames)
            _apply_clean_orientation_targets(
                retargeter,
                frames,
                include_pelvis=True,
                include_feet=not args.no_foot_flatten,
            )
            extra = "" if args.no_foot_flatten else " + flat feet"
            print(f"[FBX] Stabilizing orientation from positions (pelvis{extra}).")

    ground_offset = _estimate_ground_offset(retargeter, frames)
    retargeter.set_ground_offset(ground_offset)
    print(f"[FBX] Estimated source ground offset: {ground_offset:.6f}")

    viewer = None
    if not args.headless:
        viewer = RobotMotionViewer(
            robot_type=args.robot,
            motion_fps=int(args.fps),
            transparent_robot=1 if "transparent" in args.robot else 0,
            record_video=args.record_video,
            video_path=args.video_path,
        )

    qpos_list: List[np.ndarray] = []
    pbar = tqdm(total=len(frames), desc="Retargeting FBX")
    fps_counter = 0
    fps_start_time = time.time()

    try:
        for i, frame in enumerate(frames):
            pbar.update(1)
            fps_counter += 1

            now = time.time()
            if now - fps_start_time >= 2.0:
                actual_fps = fps_counter / (now - fps_start_time)
                print(f"[FBX] Actual rendering FPS: {actual_fps:.2f}")
                fps_counter = 0
                fps_start_time = now

            qpos = retargeter.retarget(frame, frame_index=i)
            if not args.no_foot_flatten:
                qpos = foot_pp.postprocess_robot_qpos_feet(retargeter.model, qpos, flatten=True)
            if args.save_path is not None:
                qpos_list.append(qpos.copy())

            if viewer is not None:
                viewer.step(
                    root_pos=qpos[:3],
                    root_rot=qpos[3:7],
                    dof_pos=qpos[7:],
                    human_motion_data=retargeter.scaled_human_data,
                    rate_limit=args.rate_limit,
                    follow_camera=True,
                )
    finally:
        pbar.close()
        if viewer is not None:
            viewer.close()

    if args.save_path is not None and len(qpos_list) > 0:
        motion_data = _build_motion_data_from_qpos_list(qpos_list, int(args.fps))
        with open(args.save_path, "wb") as f:
            pickle.dump(motion_data, f)
        print(f"[FBX] Saved retarget motion: {args.save_path}")


if __name__ == "__main__":
    main()
