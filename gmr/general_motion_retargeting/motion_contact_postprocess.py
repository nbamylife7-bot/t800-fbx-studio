from __future__ import annotations

"""Optional contact-aware postprocess for exported robot motion PKL files.

这个模块的目标很克制：
- 不改 GMR 的主求解流程；
- 不把项目重写成动力学优化器；
- 只在导出 PKL 之后，做一层显式可选的接触一致性修复。

当前策略分三步：
1. 用真实 support geoms 估计左右脚的 stance phase；
2. 在 stance 段里通过平移 root 的 x/y 来减少支撑脚滑移；
3. 用真实 support geoms 做 grounding，并对 root_z 修正量做轻量平滑。

注意：
- 这里的“锁地”是最小侵入版本：主要锁住 stance 脚的平面位置，z 方向交给 grounding。
- 这不是完整的 inverse-dynamics / kinodynamic retargeting，只是训练前的数据清洗工具。
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import mujoco as mj
import numpy as np

from .motion_grounding import _as_model, _xyzw_to_wxyz, align_motion_root_to_ground, geom_lowest_z, find_support_geom_ids


LEFT_SIDE_MARKERS: tuple[str, ...] = ("left", "_l", "-l")
RIGHT_SIDE_MARKERS: tuple[str, ...] = ("right", "_r", "-r")


@dataclass
class ContactAwarePostprocessConfig:
    """Tunable knobs for the contact-aware export cleanup.

    这些参数都是“导出后处理”的风格参数，不会反向影响 GMR 的 IK 求解。
    换句话说，它们控制的是：
    - stance 判定有多严格
    - grounding 有多保守/激进
    - root_z 修正要不要更平滑
    """
    stance_height_threshold: float = 0.03
    stance_speed_threshold: float = 0.15
    stance_min_frames: int = 3
    ground_clearance: float = 0.002
    ground_mode: str = "per_frame"
    root_z_smoothing_window: int = 5


CONTACT_AWARE_PROFILE_PRESETS: dict[str, ContactAwarePostprocessConfig] = {
    # 更保守：尽量少改轨迹，适合先做轻量清洗。
    "conservative": ContactAwarePostprocessConfig(
        stance_height_threshold=0.02,
        stance_speed_threshold=0.10,
        stance_min_frames=4,
        ground_clearance=0.002,
        ground_mode="global",
        root_z_smoothing_window=7,
    ),
    # 当前默认行为，保持与现有实现的默认参数一致。
    "balanced": ContactAwarePostprocessConfig(
        stance_height_threshold=0.03,
        stance_speed_threshold=0.15,
        stance_min_frames=3,
        ground_clearance=0.002,
        ground_mode="per_frame",
        root_z_smoothing_window=5,
    ),
    # 更激进：更容易判成支撑、修正更快，适合坏数据更明显的序列。
    "aggressive": ContactAwarePostprocessConfig(
        stance_height_threshold=0.05,
        stance_speed_threshold=0.20,
        stance_min_frames=2,
        ground_clearance=0.003,
        ground_mode="per_frame",
        root_z_smoothing_window=3,
    ),
}


def build_contact_aware_config(
    profile: str = "balanced",
    *,
    stance_height_threshold: float | None = None,
    stance_speed_threshold: float | None = None,
    stance_min_frames: int | None = None,
    ground_clearance: float | None = None,
    ground_mode: str | None = None,
    root_z_smoothing_window: int | None = None,
) -> ContactAwarePostprocessConfig:
    """Build a contact-aware config from a profile plus optional expert overrides.

    设计意图是把“日常使用”和“专家调参”分开：
    - 普通使用只选 `conservative / balanced / aggressive`
    - 需要精调时，再按需覆写单个阈值

    这样既能把命令行收口，又不会失去细调能力。
    """
    if profile not in CONTACT_AWARE_PROFILE_PRESETS:
        raise ValueError(
            f"Unsupported contact-aware profile: {profile}. "
            f"Expected one of {sorted(CONTACT_AWARE_PROFILE_PRESETS.keys())}."
        )

    preset = CONTACT_AWARE_PROFILE_PRESETS[profile]
    return ContactAwarePostprocessConfig(
        stance_height_threshold=(
            preset.stance_height_threshold if stance_height_threshold is None else float(stance_height_threshold)
        ),
        stance_speed_threshold=(
            preset.stance_speed_threshold if stance_speed_threshold is None else float(stance_speed_threshold)
        ),
        stance_min_frames=preset.stance_min_frames if stance_min_frames is None else int(stance_min_frames),
        ground_clearance=preset.ground_clearance if ground_clearance is None else float(ground_clearance),
        ground_mode=preset.ground_mode if ground_mode is None else str(ground_mode),
        root_z_smoothing_window=(
            preset.root_z_smoothing_window
            if root_z_smoothing_window is None
            else int(root_z_smoothing_window)
        ),
    )


def _clone_motion_data(motion_data: dict) -> dict:
    # 后处理默认返回一个拷贝，避免用户误以为只是“读取统计”却把原始数组改掉。
    return {
        key: value.copy() if isinstance(value, np.ndarray) else value
        for key, value in motion_data.items()
    }


def _detect_side(text: str) -> str | None:
    lowered = text.lower()
    if any(marker in lowered for marker in LEFT_SIDE_MARKERS):
        return "left"
    if any(marker in lowered for marker in RIGHT_SIDE_MARKERS):
        return "right"
    return None


def _group_support_geom_ids_by_default_pose_y(
    model: mj.MjModel,
    geom_ids: list[int],
) -> dict[str, list[int]]:
    """Fallback side grouping when names are not informative enough.

    很多模型的 geom/body 命名不一定带 `left/right`，但默认站立姿态下左右脚的世界 y
    往往天然分离。这里利用默认姿态的 geom 世界坐标作为兜底。
    """
    data = mj.MjData(model)
    mj.mj_forward(model, data)

    grouped: dict[str, list[int]] = {"left": [], "right": []}
    for geom_id in geom_ids:
        side = "left" if float(data.geom_xpos[geom_id][1]) >= 0.0 else "right"
        grouped[side].append(geom_id)
    return grouped


def group_support_geom_ids_by_side(
    model_or_path: mj.MjModel | str | Path,
    support_geom_ids: Iterable[int] | None = None,
) -> dict[str, list[int]]:
    """Split support geoms into left/right buckets.

    正常路径优先用 geom/body 名称中的 left/right 语义；
    如果模型命名不规范，再退化到默认站立姿态的世界 y 坐标。

    这里之所以一定要分左右，是因为后续的 stance 检测和锁脚都需要“每只脚各自的时序状态”，
    不能只看整个 support set 的全局最低点。
    """
    model = _as_model(model_or_path)
    geom_ids = list(support_geom_ids) if support_geom_ids is not None else find_support_geom_ids(model)

    grouped: dict[str, list[int]] = {"left": [], "right": []}
    for geom_id in geom_ids:
        geom_name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, geom_id) or ""
        body_name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, model.geom_bodyid[geom_id]) or ""
        side = _detect_side(f"{geom_name} {body_name}")
        if side is not None:
            grouped[side].append(geom_id)

    missing_sides = [side for side, ids in grouped.items() if not ids]
    if missing_sides:
        # 对第三方模型来说，命名未必总是标准；这里不要因为标签缺失就整条链路报废。
        grouped = _group_support_geom_ids_by_default_pose_y(model, geom_ids)
        missing_sides = [side for side, ids in grouped.items() if not ids]
        if missing_sides:
            raise ValueError(
                "Failed to split support geoms into left/right groups. "
                f"Missing sides={missing_sides}, support_geom_count={len(geom_ids)}."
            )

    return grouped


def compute_side_support_state(
    model_or_path: mj.MjModel | str | Path,
    root_pos: np.ndarray,
    root_rot: np.ndarray,
    dof_pos: np.ndarray,
    side_geom_ids: dict[str, list[int]] | None = None,
) -> dict[str, dict[str, np.ndarray]]:
    """Replay motion and measure per-side support features used by stance detection.

    这里每帧收集两类量：
    - `centroid`: 这一侧支撑几何的平均世界位置，主要用于估计平面速度
    - `min_z`:    这一侧支撑几何的最低点，主要用于判断是否接近地面

    它本质上是把“导出的 qpos 序列”重新解释回 MuJoCo 世界里的足部接触代理量。
    """
    model = _as_model(model_or_path)
    data = mj.MjData(model)

    root_pos = np.asarray(root_pos, dtype=np.float64)
    root_rot = np.asarray(root_rot, dtype=np.float64)
    dof_pos = np.asarray(dof_pos, dtype=np.float64)

    grouped = side_geom_ids or group_support_geom_ids_by_side(model)
    frame_count = root_pos.shape[0]
    state = {
        side: {
            "centroid": np.zeros((frame_count, 3), dtype=np.float64),
            "min_z": np.zeros(frame_count, dtype=np.float64),
        }
        for side in grouped
    }

    for frame_idx in range(frame_count):
        # 注意：这里是逐帧重放导出的 floating-base 状态，不做动力学积分，只做几何测量。
        data.qpos[:3] = root_pos[frame_idx, :3]
        data.qpos[3:7] = _xyzw_to_wxyz(root_rot[frame_idx])
        data.qpos[7:] = dof_pos[frame_idx]
        mj.mj_forward(model, data)

        for side, geom_ids in grouped.items():
            xpos = np.asarray([data.geom_xpos[geom_id] for geom_id in geom_ids], dtype=np.float64)
            state[side]["centroid"][frame_idx] = np.mean(xpos, axis=0)
            state[side]["min_z"][frame_idx] = min(
                geom_lowest_z(model, data, geom_id) for geom_id in geom_ids
            )

    return state


def _compute_planar_speed(positions: np.ndarray, fps: float) -> np.ndarray:
    """Compute XY speed magnitude from a world-space trajectory."""
    velocity = np.zeros_like(positions, dtype=np.float64)
    if positions.shape[0] >= 2:
        velocity[1:] = (positions[1:] - positions[:-1]) * float(fps)
        velocity[0] = velocity[1]
    return np.linalg.norm(velocity[:, :2], axis=1)


def _remove_short_true_segments(mask: np.ndarray, min_frames: int) -> np.ndarray:
    """Drop tiny stance spikes caused by noisy threshold crossings."""
    if min_frames <= 1:
        return mask.copy()

    cleaned = mask.copy()
    start = None
    for idx, active in enumerate(mask):
        if active and start is None:
            start = idx
        if not active and start is not None:
            if idx - start < min_frames:
                cleaned[start:idx] = False
            start = None
    if start is not None and len(mask) - start < min_frames:
        cleaned[start:] = False
    return cleaned


def detect_stance_masks(
    side_state: dict[str, dict[str, np.ndarray]],
    fps: float,
    height_threshold: float,
    speed_threshold: float,
    min_frames: int,
) -> dict[str, np.ndarray]:
    """Detect stance frames from support height and planar foot speed.

    当前采用的是最小侵入启发式：
    - 够低：脚/踝支撑几何已经贴近地面
    - 够慢：这一侧脚的平面速度已经足够小

    两者同时满足时才当作 stance，目的是避免把摆动脚误当成支撑脚。
    """
    masks: dict[str, np.ndarray] = {}
    for side, state in side_state.items():
        speed = _compute_planar_speed(state["centroid"], fps)
        stance = (state["min_z"] <= float(height_threshold)) & (speed <= float(speed_threshold))
        masks[side] = _remove_short_true_segments(stance, int(min_frames))
    return masks


def compute_stance_confidence(
    side_state: dict[str, dict[str, np.ndarray]],
    stance_masks: dict[str, np.ndarray],
    fps: float,
    height_threshold: float,
    speed_threshold: float,
) -> dict[str, np.ndarray]:
    """Estimate per-frame stance confidence used to resolve double-stance conflicts.

    同时满足“更贴近地面”和“更接近静止”的脚，应该在双支撑时拥有更高权重，
    这样比左右脚简单平均更不容易互相拉扯。
    """
    height_denom = max(float(height_threshold), 1e-6)
    speed_denom = max(float(speed_threshold), 1e-6)
    confidence: dict[str, np.ndarray] = {}

    for side, state in side_state.items():
        speed = _compute_planar_speed(state["centroid"], fps)
        height_score = np.clip((float(height_threshold) - state["min_z"]) / height_denom, 0.0, 1.0)
        speed_score = np.clip((float(speed_threshold) - speed) / speed_denom, 0.0, 1.0)
        confidence[side] = height_score * speed_score * stance_masks[side].astype(np.float64)

    return confidence


def _iter_true_segments(mask: np.ndarray) -> Iterable[tuple[int, int]]:
    """Yield contiguous `[start, end)` segments of a boolean mask."""
    start = None
    for idx, active in enumerate(mask):
        if active and start is None:
            start = idx
        if not active and start is not None:
            yield start, idx
            start = None
    if start is not None:
        yield start, len(mask)


def apply_stance_xy_lock(
    motion_data: dict,
    model_or_path: mj.MjModel | str | Path,
    stance_height_threshold: float,
    stance_speed_threshold: float,
    stance_min_frames: int,
    side_geom_ids: dict[str, list[int]] | None = None,
    inplace: bool = False,
) -> tuple[dict, dict[str, object]]:
    """Reduce support-foot sliding by adjusting root x/y during stance segments.

    这里不直接改足部关节角，而是反过来平移 root 的平面位置：
    - 对每个 stance 段，取段起点的足部 XY 作为 anchor
    - 后续帧如果脚在地面上“滑”了，就用 root 平移把它拉回 anchor

    这是一个很典型的运动学后处理技巧：
    - 优点：改动小、实现稳定、不碰 IK 主链
    - 缺点：只是一种几何补偿，不保证动力学严格一致
    """
    adjusted_motion = motion_data if inplace else _clone_motion_data(motion_data)
    root_pos = adjusted_motion["root_pos"]
    fps = float(adjusted_motion.get("fps", 30.0))

    side_state = compute_side_support_state(
        model_or_path=model_or_path,
        root_pos=adjusted_motion["root_pos"],
        root_rot=adjusted_motion["root_rot"],
        dof_pos=adjusted_motion["dof_pos"],
        side_geom_ids=side_geom_ids,
    )
    stance_masks = detect_stance_masks(
        side_state=side_state,
        fps=fps,
        height_threshold=stance_height_threshold,
        speed_threshold=stance_speed_threshold,
        min_frames=stance_min_frames,
    )
    stance_confidence = compute_stance_confidence(
        side_state=side_state,
        stance_masks=stance_masks,
        fps=fps,
        height_threshold=stance_height_threshold,
        speed_threshold=stance_speed_threshold,
    )

    per_side_deltas: dict[str, np.ndarray] = {
        side: np.zeros((root_pos.shape[0], 2), dtype=np.float64)
        for side in stance_masks
    }

    for side, mask in stance_masks.items():
        positions_xy = side_state[side]["centroid"][:, :2]
        for start, end in _iter_true_segments(mask):
            # stance 段起点视作“脚已经站稳”的参考位置，整段都尽量往这个 anchor 靠。
            anchor_xy = positions_xy[start].copy()
            per_side_deltas[side][start:end] = anchor_xy - positions_xy[start:end]

    combined_delta = np.zeros((root_pos.shape[0], 2), dtype=np.float64)
    total_weight = np.zeros(root_pos.shape[0], dtype=np.float64)
    for side, confidence in stance_confidence.items():
        # 单支撑时基本等价于直接使用该脚的修正量；
        # 双支撑时则按置信度加权，避免简单平均导致两只脚互相妥协。
        combined_delta += per_side_deltas[side] * confidence[:, None]
        total_weight += confidence

    valid = total_weight > 1e-9
    combined_delta[valid] /= total_weight[valid, None]
    root_pos[:, :2] += combined_delta

    stats = {
        "stance_frames_left": int(np.count_nonzero(stance_masks["left"])),
        "stance_frames_right": int(np.count_nonzero(stance_masks["right"])),
        "double_stance_frames": int(np.count_nonzero(stance_masks["left"] & stance_masks["right"])),
        "max_xy_lock_shift": float(np.max(np.linalg.norm(combined_delta, axis=1))),
        "median_xy_lock_shift": float(np.median(np.linalg.norm(combined_delta, axis=1))),
    }
    return adjusted_motion, stats


def smooth_root_z_signal(root_z: np.ndarray, window: int) -> np.ndarray:
    """Simple edge-padded moving average used for light root_z regularization.

    这里平滑的是“root_z 修正量”，不是原始 root_z 本身。
    这样可以尽量保留动作原始的竖直起伏，只削弱 grounding 带来的高频锯齿。
    """
    root_z = np.asarray(root_z, dtype=np.float64)
    window = int(window)
    if window <= 1 or root_z.shape[0] <= 2:
        return root_z.copy()

    pad = window // 2
    kernel = np.ones(window, dtype=np.float64) / float(window)
    padded = np.pad(root_z, (pad, pad), mode="edge")
    smoothed = np.convolve(padded, kernel, mode="valid")
    return smoothed[: root_z.shape[0]]


def apply_contact_aware_postprocess(
    motion_data: dict,
    model_or_path: mj.MjModel | str | Path,
    config: ContactAwarePostprocessConfig | None = None,
    inplace: bool = False,
) -> tuple[dict, dict[str, object]]:
    """Run the optional contact-aware cleanup pipeline on exported motion data.

    处理顺序很重要：
    1. 先做 stance XY 锁地，减少平面滑脚
    2. 再 grounding，解决穿地/悬空
    3. 然后只平滑 root_z 修正量
    4. 最后再 grounding 一次，确保平滑没有把脚重新压回地板下面

    这条顺序的核心思想是：
    - 先修“脚在地上横着滑”
    - 再修“脚在 z 方向穿地或悬空”
    """
    cfg = config or ContactAwarePostprocessConfig()
    adjusted_motion = motion_data if inplace else _clone_motion_data(motion_data)
    model = _as_model(model_or_path)
    support_ids = find_support_geom_ids(model)
    side_geom_ids = group_support_geom_ids_by_side(model, support_ids)

    adjusted_motion, stance_stats = apply_stance_xy_lock(
        motion_data=adjusted_motion,
        model_or_path=model,
        stance_height_threshold=cfg.stance_height_threshold,
        stance_speed_threshold=cfg.stance_speed_threshold,
        stance_min_frames=cfg.stance_min_frames,
        side_geom_ids=side_geom_ids,
        inplace=True,
    )

    z_before_grounding = adjusted_motion["root_pos"][:, 2].copy()
    adjusted_motion, grounding_stats = align_motion_root_to_ground(
        motion_data=adjusted_motion,
        model_or_path=model,
        clearance=cfg.ground_clearance,
        mode=cfg.ground_mode,
        inplace=False,
        support_geom_ids=support_ids,
    )

    raw_z_after_grounding = adjusted_motion["root_pos"][:, 2].copy()
    raw_z_delta = raw_z_after_grounding - z_before_grounding
    if cfg.root_z_smoothing_window > 1:
        # 只平滑“修正量”，避免把原动作中真实存在的跳跃/下蹲等竖直动态一并抹平。
        smoothed_z_delta = smooth_root_z_signal(raw_z_delta, cfg.root_z_smoothing_window)
        adjusted_motion["root_pos"][:, 2] = z_before_grounding + smoothed_z_delta
        adjusted_motion, final_grounding_stats = align_motion_root_to_ground(
            motion_data=adjusted_motion,
            model_or_path=model,
            clearance=cfg.ground_clearance,
            mode=cfg.ground_mode,
            inplace=False,
            support_geom_ids=support_ids,
        )
    else:
        smoothed_z_delta = raw_z_delta
        final_grounding_stats = grounding_stats

    stats = {
        "stance": stance_stats,
        "grounding_before_smoothing": grounding_stats,
        "grounding_after_smoothing": final_grounding_stats,
        "root_z_smoothing_window": int(cfg.root_z_smoothing_window),
        "raw_root_z_shift_max": float(np.max(raw_z_delta)),
        "raw_root_z_shift_median": float(np.median(raw_z_delta)),
        "smoothed_root_z_shift_max": float(np.max(smoothed_z_delta)),
        "smoothed_root_z_shift_median": float(np.median(smoothed_z_delta)),
    }
    return adjusted_motion, stats
