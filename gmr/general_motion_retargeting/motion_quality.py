from __future__ import annotations

"""Source-side quality audit and conservative repair helpers for robot motion PKLs.

The functions in this module intentionally operate on the GMR PKL format before
IsaacLab conversion.  Accepted fixes should be made here, then converted again
so the final training NPZ keeps joint, body, and velocity references consistent.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence
import json
import os
import pickle
import tempfile

import numpy as np


@dataclass
class MotionQualityConfig:
    robot: str = "t800"
    joint_names: Sequence[str] | None = None
    joint_lower_limits: np.ndarray | None = None
    joint_upper_limits: np.ndarray | None = None
    jump_threshold_rad: float = 0.7
    velocity_threshold_rad_s: float = 12.0
    limit_margin_rad: float = 0.03
    floor_clearance: float = 0.0
    review_padding_frames: int = 12
    max_review_windows_per_kind: int = 8
    collision_margin: float = 0.01
    collision_stride: int = 5
    max_collision_pairs: int = 50
    model_path: str | Path | None = None


@dataclass
class RepairSpec:
    frame_start: int
    frame_end: int
    joint_names: Sequence[str]
    method: str
    rationale: str
    max_correction_rad: float = 5.0


def load_motion_pkl(path: str | Path) -> dict[str, Any]:
    with Path(path).open("rb") as f:
        data = pickle.load(f)
    return data


def save_motion_pkl(motion_data: dict[str, Any], path: str | Path, *, overwrite: bool = False) -> None:
    target = Path(path)
    if target.exists() and not overwrite:
        raise FileExistsError(f"Output file already exists: {target}")
    _atomic_write_bytes(target, pickle.dumps(motion_data))


def _atomic_write_bytes(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    tmp_path = Path(tmp_name)
    try:
        with os.fdopen(fd, "wb") as f:
            f.write(payload)
        os.replace(tmp_path, path)
    except Exception:
        try:
            tmp_path.unlink()
        except FileNotFoundError:
            pass
        raise


def build_motion_data(
    *,
    fps: int | float,
    root_pos: np.ndarray,
    root_rot: np.ndarray,
    dof_pos: np.ndarray,
) -> dict[str, Any]:
    root_pos = np.asarray(root_pos, dtype=np.float64)
    root_rot = np.asarray(root_rot, dtype=np.float64)
    dof_pos = np.asarray(dof_pos, dtype=np.float64)
    if root_pos.ndim != 2 or root_pos.shape[1] != 3:
        raise ValueError(f"Expected root_pos with shape [T, 3], got {root_pos.shape}.")
    if root_rot.ndim != 2 or root_rot.shape[1] != 4:
        raise ValueError(f"Expected root_rot with shape [T, 4], got {root_rot.shape}.")
    if dof_pos.ndim != 2:
        raise ValueError(f"Expected dof_pos with shape [T, D], got {dof_pos.shape}.")
    if not (root_pos.shape[0] == root_rot.shape[0] == dof_pos.shape[0]):
        raise ValueError("root_pos, root_rot, and dof_pos must have the same frame count.")

    frame_count = dof_pos.shape[0]
    fps_value = float(fps)
    if frame_count >= 2 and fps_value > 0:
        dt = 1.0 / fps_value
        root_lin_vel = np.gradient(root_pos, dt, axis=0)
        dof_vel = np.gradient(dof_pos, dt, axis=0)
    else:
        root_lin_vel = np.zeros_like(root_pos)
        dof_vel = np.zeros_like(dof_pos)

    return {
        "fps": int(fps) if float(fps).is_integer() else fps_value,
        "root_pos": root_pos,
        "root_rot": root_rot,
        "dof_pos": dof_pos,
        "dof_vel": dof_vel,
        "root_lin_vel": root_lin_vel,
        "root_ang_vel": np.zeros((frame_count, 3), dtype=np.float64),
        "local_body_pos": None,
        "link_body_list": None,
    }


def _as_array(motion_data: dict[str, Any], key: str, ndim: int) -> np.ndarray:
    if key not in motion_data:
        raise ValueError(f"Motion PKL missing required field: {key}")
    value = np.asarray(motion_data[key], dtype=np.float64)
    if value.ndim != ndim:
        raise ValueError(f"Expected {key} to have {ndim} dims, got shape {value.shape}.")
    return value


def validate_motion_schema(motion_data: dict[str, Any]) -> tuple[float, np.ndarray, np.ndarray, np.ndarray]:
    if "fps" not in motion_data:
        raise ValueError("Motion PKL missing required field: fps")
    fps = float(motion_data["fps"])
    if not np.isfinite(fps) or fps <= 0:
        raise ValueError(f"Expected positive finite fps, got {motion_data['fps']!r}.")

    root_pos = _as_array(motion_data, "root_pos", 2)
    root_rot = _as_array(motion_data, "root_rot", 2)
    dof_pos = _as_array(motion_data, "dof_pos", 2)
    if root_pos.shape[1] != 3:
        raise ValueError(f"Expected root_pos shape [T, 3], got {root_pos.shape}.")
    if root_rot.shape[1] != 4:
        raise ValueError(f"Expected root_rot shape [T, 4], got {root_rot.shape}.")
    if not (root_pos.shape[0] == root_rot.shape[0] == dof_pos.shape[0]):
        raise ValueError("root_pos, root_rot, and dof_pos must have the same frame count.")
    if root_pos.shape[0] < 2:
        raise ValueError("Motion PKL must contain at least 2 frames for quality audit or repair.")
    for key, value in {"root_pos": root_pos, "root_rot": root_rot, "dof_pos": dof_pos}.items():
        if not np.all(np.isfinite(value)):
            raise ValueError(f"Motion field {key} contains NaN or Inf values.")
    return fps, root_pos, root_rot, dof_pos


def _default_joint_names(dof_count: int) -> list[str]:
    return [f"J{i:02d}" for i in range(dof_count)]


def _joint_names(config: MotionQualityConfig, dof_count: int) -> list[str]:
    names = list(config.joint_names) if config.joint_names is not None else _default_joint_names(dof_count)
    if len(names) != dof_count:
        raise ValueError(f"Expected {dof_count} joint names, got {len(names)}.")
    return [str(name) for name in names]


def _review_window(frame: int, frame_count: int, fps: float, padding_frames: int) -> dict[str, int | float]:
    start = max(0, int(frame) - int(padding_frames))
    end = min(frame_count, int(frame) + int(padding_frames) + 1)
    return {
        "start_frame": start,
        "end_frame": end,
        "start_sec": round(start / fps, 6),
        "end_sec": round(end / fps, 6),
    }


def _issue(
    *,
    kind: str,
    frame: int,
    fps: float,
    frame_count: int,
    padding_frames: int,
    **fields: Any,
) -> dict[str, Any]:
    return {
        "kind": kind,
        "frame": int(frame),
        "time_sec": round(int(frame) / fps, 6),
        "review_window": _review_window(frame, frame_count, fps, padding_frames),
        **fields,
    }


def _issue_severity(item: dict[str, Any]) -> float:
    for key in (
        "abs_delta_rad",
        "abs_velocity_rad_s",
        "distance_to_limit_rad",
        "support_min_z",
        "clearance_m",
    ):
        if key not in item:
            continue
        value = float(item[key])
        if key in {"distance_to_limit_rad", "support_min_z", "clearance_m"}:
            return -value
        return value
    return 0.0


def _merge_review_windows(
    issues_by_kind: dict[str, list[dict[str, Any]]],
    fps: float,
    max_per_kind: int,
) -> list[dict[str, Any]]:
    raw_windows: list[dict[str, Any]] = []
    for kind, issues in issues_by_kind.items():
        selected = sorted(issues, key=_issue_severity, reverse=True)[: max(0, int(max_per_kind))]
        for item in selected:
            window = dict(item["review_window"])
            window["reasons"] = [kind]
            raw_windows.append(window)
    raw_windows.sort(key=lambda item: (int(item["start_frame"]), int(item["end_frame"])))

    merged: list[dict[str, Any]] = []
    for window in raw_windows:
        if not merged or int(window["start_frame"]) > int(merged[-1]["end_frame"]):
            merged.append(window)
            continue
        merged[-1]["end_frame"] = max(int(merged[-1]["end_frame"]), int(window["end_frame"]))
        merged[-1]["start_sec"] = round(int(merged[-1]["start_frame"]) / fps, 6)
        merged[-1]["end_sec"] = round(int(merged[-1]["end_frame"]) / fps, 6)
        merged[-1]["reasons"] = sorted(set(merged[-1]["reasons"]) | set(window["reasons"]))
    return merged


def _top_abs_deltas(
    dof_pos: np.ndarray,
    fps: float,
    joint_names: Sequence[str],
    threshold: float,
    padding_frames: int,
) -> list[dict[str, Any]]:
    if dof_pos.shape[0] < 2:
        return []
    diff = np.diff(dof_pos, axis=0)
    frames, joints = np.where(np.abs(diff) > float(threshold))
    issues = []
    for diff_frame, joint_idx in zip(frames, joints):
        frame = int(diff_frame) + 1
        issues.append(
            _issue(
                kind="qpos_jump",
                frame=frame,
                fps=fps,
                frame_count=dof_pos.shape[0],
                padding_frames=padding_frames,
                joint_index=int(joint_idx),
                joint_name=str(joint_names[joint_idx]),
                delta_rad=float(diff[diff_frame, joint_idx]),
                abs_delta_rad=float(abs(diff[diff_frame, joint_idx])),
            )
        )
    return sorted(issues, key=lambda item: item["abs_delta_rad"], reverse=True)


def _velocity_spikes(
    motion_data: dict[str, Any],
    dof_pos: np.ndarray,
    fps: float,
    joint_names: Sequence[str],
    threshold: float,
    padding_frames: int,
) -> list[dict[str, Any]]:
    stored_vel = None
    if "dof_vel" in motion_data:
        stored_vel = np.asarray(motion_data["dof_vel"], dtype=np.float64)
        if stored_vel.shape != dof_pos.shape:
            stored_vel = None
    gradient_vel = np.gradient(dof_pos, 1.0 / fps, axis=0)
    step_vel = np.zeros_like(dof_pos, dtype=np.float64)
    if dof_pos.shape[0] >= 2:
        step_vel[1:] = np.diff(dof_pos, axis=0) * fps
        step_vel[0] = step_vel[1]
    candidates = [gradient_vel, step_vel]
    if stored_vel is not None:
        candidates.append(stored_vel)
    abs_candidates = np.stack([np.abs(candidate) for candidate in candidates], axis=0)
    best_source = np.argmax(abs_candidates, axis=0)
    dof_vel = np.take_along_axis(np.stack(candidates, axis=0), best_source[None, ...], axis=0)[0]
    frames, joints = np.where(np.abs(dof_vel) >= float(threshold))
    issues = []
    for frame, joint_idx in zip(frames, joints):
        issues.append(
            _issue(
                kind="velocity_spike",
                frame=int(frame),
                fps=fps,
                frame_count=dof_pos.shape[0],
                padding_frames=padding_frames,
                joint_index=int(joint_idx),
                joint_name=str(joint_names[joint_idx]),
                velocity_rad_s=float(dof_vel[frame, joint_idx]),
                abs_velocity_rad_s=float(abs(dof_vel[frame, joint_idx])),
            )
        )
    return sorted(issues, key=lambda item: item["abs_velocity_rad_s"], reverse=True)


def _limit_pressure(
    dof_pos: np.ndarray,
    fps: float,
    joint_names: Sequence[str],
    lower: np.ndarray | None,
    upper: np.ndarray | None,
    margin: float,
    padding_frames: int,
) -> list[dict[str, Any]]:
    if lower is None or upper is None:
        return []
    lower = np.asarray(lower, dtype=np.float64)
    upper = np.asarray(upper, dtype=np.float64)
    if lower.shape != (dof_pos.shape[1],) or upper.shape != (dof_pos.shape[1],):
        raise ValueError(
            f"Joint limits must have shape ({dof_pos.shape[1]},), got {lower.shape} and {upper.shape}."
        )
    near_lower = dof_pos <= lower + float(margin)
    near_upper = dof_pos >= upper - float(margin)
    frames, joints = np.where(near_lower | near_upper)
    issues = []
    for frame, joint_idx in zip(frames, joints):
        side = "lower" if near_lower[frame, joint_idx] else "upper"
        distance = (
            dof_pos[frame, joint_idx] - lower[joint_idx]
            if side == "lower"
            else upper[joint_idx] - dof_pos[frame, joint_idx]
        )
        issues.append(
            _issue(
                kind="limit_pressure",
                frame=int(frame),
                fps=fps,
                frame_count=dof_pos.shape[0],
                padding_frames=padding_frames,
                joint_index=int(joint_idx),
                joint_name=str(joint_names[joint_idx]),
                side=side,
                value_rad=float(dof_pos[frame, joint_idx]),
                limit_rad=float(lower[joint_idx] if side == "lower" else upper[joint_idx]),
                distance_to_limit_rad=float(distance),
            )
        )
    return issues


def _support_floor_issues(
    root_pos: np.ndarray,
    root_rot: np.ndarray,
    dof_pos: np.ndarray,
    config: MotionQualityConfig,
    fps: float,
) -> tuple[list[dict[str, Any]], dict[str, Any] | None]:
    if config.model_path is None:
        return [], None
    try:
        from .motion_grounding import compute_support_min_z, summarize_support_min_z

        support_min_z = compute_support_min_z(config.model_path, root_pos, root_rot, dof_pos)
    except Exception as exc:  # pragma: no cover - environment-dependent diagnostic path
        return [], {"status": "skipped", "reason": str(exc)}

    below = np.where(support_min_z < float(config.floor_clearance))[0]
    issues = [
        _issue(
            kind="floor_anomaly",
            frame=int(frame),
            fps=fps,
            frame_count=root_pos.shape[0],
            padding_frames=config.review_padding_frames,
            support_min_z=float(support_min_z[frame]),
            clearance=float(config.floor_clearance),
        )
        for frame in below
    ]
    return issues, summarize_support_min_z(support_min_z, clearance=config.floor_clearance)


def _geom_radius(model: Any, geom_id: int) -> float:
    rbound = float(model.geom_rbound[geom_id]) if hasattr(model, "geom_rbound") else 0.0
    if np.isfinite(rbound) and rbound > 0.0:
        return rbound
    size = np.asarray(model.geom_size[geom_id], dtype=np.float64)
    return float(np.linalg.norm(size))


def _candidate_collision_issues(
    root_pos: np.ndarray,
    root_rot: np.ndarray,
    dof_pos: np.ndarray,
    config: MotionQualityConfig,
    fps: float,
) -> tuple[list[dict[str, Any]], dict[str, Any] | None]:
    if config.model_path is None or config.collision_stride <= 0 or config.max_collision_pairs == 0:
        return [], None
    try:
        import mujoco as mj

        from .motion_grounding import _xyzw_to_wxyz

        model = mj.MjModel.from_xml_path(str(config.model_path))
        data = mj.MjData(model)
    except Exception as exc:  # pragma: no cover - environment-dependent diagnostic path
        return [], {"status": "skipped", "reason": str(exc)}

    geom_ids = [
        geom_id
        for geom_id in range(model.ngeom)
        if int(model.geom_type[geom_id]) != int(mj.mjtGeom.mjGEOM_PLANE)
        and (int(model.geom_contype[geom_id]) != 0 or int(model.geom_conaffinity[geom_id]) != 0)
    ]
    pair_hits: dict[tuple[int, int], dict[str, Any]] = {}
    step = max(1, int(config.collision_stride))
    for frame in range(0, root_pos.shape[0], step):
        data.qpos[:3] = root_pos[frame, :3]
        data.qpos[3:7] = _xyzw_to_wxyz(root_rot[frame])
        data.qpos[7:] = dof_pos[frame]
        mj.mj_forward(model, data)
        for index_a, geom_a in enumerate(geom_ids):
            body_a = int(model.geom_bodyid[geom_a])
            radius_a = _geom_radius(model, geom_a)
            for geom_b in geom_ids[index_a + 1 :]:
                body_b = int(model.geom_bodyid[geom_b])
                if body_a == body_b:
                    continue
                if int(model.body_parentid[body_a]) == body_b or int(model.body_parentid[body_b]) == body_a:
                    continue
                radius_b = _geom_radius(model, geom_b)
                distance = float(np.linalg.norm(data.geom_xpos[geom_a] - data.geom_xpos[geom_b]))
                clearance = distance - radius_a - radius_b
                if clearance >= float(config.collision_margin):
                    continue
                key = (geom_a, geom_b)
                best = pair_hits.get(key)
                if best is None or clearance < best["clearance_m"]:
                    pair_hits[key] = {
                        "frame": int(frame),
                        "clearance_m": float(clearance),
                    }
        if len(pair_hits) >= int(config.max_collision_pairs) * 4:
            break

    issues = []
    for (geom_a, geom_b), hit in sorted(pair_hits.items(), key=lambda item: item[1]["clearance_m"])[
        : int(config.max_collision_pairs)
    ]:
        geom_a_name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, geom_a) or f"geom_{geom_a}"
        geom_b_name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, geom_b) or f"geom_{geom_b}"
        body_a = int(model.geom_bodyid[geom_a])
        body_b = int(model.geom_bodyid[geom_b])
        body_a_name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, body_a) or f"body_{body_a}"
        body_b_name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, body_b) or f"body_{body_b}"
        issues.append(
            _issue(
                kind="candidate_collision",
                frame=int(hit["frame"]),
                fps=fps,
                frame_count=root_pos.shape[0],
                padding_frames=config.review_padding_frames,
                geom_a=geom_a_name,
                geom_b=geom_b_name,
                body_a=body_a_name,
                body_b=body_b_name,
                clearance_m=float(hit["clearance_m"]),
                diagnostic="sphere-bound approximation; confirm visually or with short-window PhysX if needed",
            )
        )
    return issues, {"status": "ok", "sample_stride": step, "checked_geom_count": len(geom_ids)}


def audit_motion_quality(
    motion_data: dict[str, Any],
    *,
    config: MotionQualityConfig | None = None,
    motion_path: str | Path | None = None,
) -> dict[str, Any]:
    config = config or MotionQualityConfig()
    fps, root_pos, root_rot, dof_pos = validate_motion_schema(motion_data)
    joint_names = _joint_names(config, dof_pos.shape[1])

    qpos_jumps = _top_abs_deltas(
        dof_pos,
        fps,
        joint_names,
        threshold=config.jump_threshold_rad,
        padding_frames=config.review_padding_frames,
    )
    velocity_spikes = _velocity_spikes(
        motion_data,
        dof_pos,
        fps,
        joint_names,
        threshold=config.velocity_threshold_rad_s,
        padding_frames=config.review_padding_frames,
    )
    limit_pressure = _limit_pressure(
        dof_pos,
        fps,
        joint_names,
        lower=config.joint_lower_limits,
        upper=config.joint_upper_limits,
        margin=config.limit_margin_rad,
        padding_frames=config.review_padding_frames,
    )
    floor_anomalies, floor_summary = _support_floor_issues(root_pos, root_rot, dof_pos, config, fps)
    candidate_collisions, collision_summary = _candidate_collision_issues(root_pos, root_rot, dof_pos, config, fps)

    issues = {
        "qpos_jumps": qpos_jumps,
        "velocity_spikes": velocity_spikes,
        "limit_pressure": limit_pressure,
        "floor_anomalies": floor_anomalies,
        "candidate_collisions": candidate_collisions,
    }
    return {
        "motion_path": str(motion_path) if motion_path is not None else None,
        "robot": config.robot,
        "schema": {
            "fps": fps,
            "frame_count": int(root_pos.shape[0]),
            "dof_count": int(dof_pos.shape[1]),
            "fields": sorted(str(key) for key in motion_data.keys()),
        },
        "summary": {
            "qpos_jump_count": len(qpos_jumps),
            "velocity_spike_count": len(velocity_spikes),
            "limit_pressure_count": len(limit_pressure),
            "floor_anomaly_count": len(floor_anomalies),
            "candidate_collision_count": len(candidate_collisions),
            "root_z_min": float(np.min(root_pos[:, 2])),
            "root_z_max": float(np.max(root_pos[:, 2])),
            "finite": True,
        },
        "issues": issues,
        "floor_summary": floor_summary,
        "collision_summary": collision_summary,
        "review_windows": _merge_review_windows(issues, fps, config.max_review_windows_per_kind),
        "review_commands": [],
    }


def build_review_commands(report: dict[str, Any]) -> list[str]:
    motion_path = report.get("motion_path")
    robot = report.get("robot", "t800")
    if not motion_path:
        return []
    commands = []
    for window in report.get("review_windows", []):
        commands.append(
            "python scripts/vis_robot_motion.py "
            f"--robot {robot} --robot_motion_path {_quote_cli_arg(str(motion_path))} "
            f"--frame_start {int(window['start_frame'])} --frame_end {int(window['end_frame'])}"
        )
    return commands


def _quote_cli_arg(value: str) -> str:
    if not value or any(char.isspace() for char in value):
        return '"' + value.replace('"', '\\"') + '"'
    return value


def load_repair_specs(path: str | Path) -> list[RepairSpec]:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    raw_repairs = payload.get("repairs")
    if not isinstance(raw_repairs, list):
        raise ValueError("Repair spec JSON must contain a list field named 'repairs'.")
    if not raw_repairs:
        raise ValueError("Repair spec JSON must contain at least one repair item.")
    specs = []
    for item in raw_repairs:
        joint_names = [str(name) for name in item["joint_names"]]
        if not joint_names:
            raise ValueError("Each repair item must include at least one joint name.")
        specs.append(
            RepairSpec(
                frame_start=int(item["frame_start"]),
                frame_end=int(item["frame_end"]),
                joint_names=joint_names,
                method=str(item["method"]),
                rationale=str(item.get("rationale", "")),
                max_correction_rad=float(item.get("max_correction_rad", 5.0)),
            )
        )
    return specs


def _clone_motion(motion_data: dict[str, Any]) -> dict[str, Any]:
    return {key: value.copy() if isinstance(value, np.ndarray) else value for key, value in motion_data.items()}


def _joint_index_map(joint_names: Sequence[str]) -> dict[str, int]:
    result = {}
    for index, name in enumerate(joint_names):
        if name in result:
            raise ValueError(f"Duplicate joint name in joint_names: {name}")
        result[str(name)] = index
    return result


def _max_adjacent_abs_delta(values: np.ndarray) -> float:
    if values.shape[0] < 2:
        return 0.0
    return float(np.max(np.abs(np.diff(values, axis=0))))


def _slice_with_context(values: np.ndarray, frame_start: int, frame_end: int) -> np.ndarray:
    context_start = max(0, frame_start - 1)
    context_end = min(values.shape[0], frame_end + 1)
    return values[context_start:context_end]


def apply_repair_specs(
    motion_data: dict[str, Any],
    repair_specs: Iterable[RepairSpec],
    *,
    joint_names: Sequence[str],
    inplace: bool = False,
) -> tuple[dict[str, Any], dict[str, Any]]:
    fps, root_pos, root_rot, dof_pos = validate_motion_schema(motion_data)
    del root_pos, root_rot
    if len(joint_names) != dof_pos.shape[1]:
        raise ValueError(f"Expected {dof_pos.shape[1]} joint names, got {len(joint_names)}.")
    index_by_name = _joint_index_map(joint_names)
    repaired = motion_data if inplace else _clone_motion(motion_data)
    repaired_dof = np.asarray(repaired["dof_pos"], dtype=np.float64).copy()

    repair_reports: list[dict[str, Any]] = []
    for spec in repair_specs:
        frame_start = int(spec.frame_start)
        frame_end = int(spec.frame_end)
        if spec.method != "linear_interpolate":
            raise ValueError(f"Unsupported repair method: {spec.method}")
        if frame_start < 0 or frame_end > repaired_dof.shape[0] or frame_start >= frame_end:
            raise ValueError(
                f"Invalid repair frame range [{frame_start}, {frame_end}) for {repaired_dof.shape[0]} frames."
            )
        if frame_end - frame_start < 2:
            raise ValueError("Repair frame range must contain at least two frames for interpolation anchors.")
        indices = []
        for joint_name in spec.joint_names:
            if joint_name not in index_by_name:
                raise ValueError(f"Unknown repair joint name: {joint_name}")
            indices.append(index_by_name[joint_name])
        if not indices:
            raise ValueError("Repair spec must include at least one joint name.")

        before_context = _slice_with_context(repaired_dof[:, indices], frame_start, frame_end).copy()
        before = repaired_dof[frame_start:frame_end, indices].copy()
        after = before.copy()
        steps = frame_end - frame_start
        for local_joint_index in range(after.shape[1]):
            after[:, local_joint_index] = np.linspace(before[0, local_joint_index], before[-1, local_joint_index], steps)
        correction = np.abs(after - before)
        max_correction = float(np.max(correction)) if correction.size else 0.0
        if max_correction > float(spec.max_correction_rad):
            raise ValueError(
                f"Repair [{frame_start}, {frame_end}) for {list(spec.joint_names)} "
                f"exceeds max_correction_rad: {max_correction:.6f} > {spec.max_correction_rad:.6f}"
            )
        candidate_dof = repaired_dof.copy()
        candidate_dof[frame_start:frame_end, indices] = after
        after_context = _slice_with_context(candidate_dof[:, indices], frame_start, frame_end)
        before_context_delta = _max_adjacent_abs_delta(before_context)
        after_context_delta = _max_adjacent_abs_delta(after_context)
        before_window_delta = _max_adjacent_abs_delta(before)
        if after_context_delta > before_context_delta + 1e-9:
            raise ValueError(
                f"Repair [{frame_start}, {frame_end}) for {list(spec.joint_names)} "
                "degrades continuity: "
                f"{after_context_delta:.6f} > {before_context_delta:.6f}"
            )
        if before_context_delta > before_window_delta + 1e-9 and after_context_delta >= before_context_delta - 1e-9:
            raise ValueError(
                f"Repair [{frame_start}, {frame_end}) for {list(spec.joint_names)} "
                "degrades continuity: selected window leaves the largest adjacent jump at a boundary."
            )
        repaired_dof = candidate_dof
        repair_reports.append(
            {
                "frame_start": frame_start,
                "frame_end": frame_end,
                "joint_names": list(spec.joint_names),
                "method": spec.method,
                "rationale": spec.rationale,
                "max_correction_rad": max_correction,
                "before_max_abs_delta_rad": _max_adjacent_abs_delta(before),
                "after_max_abs_delta_rad": _max_adjacent_abs_delta(after),
                "before_context_max_abs_delta_rad": before_context_delta,
                "after_context_max_abs_delta_rad": after_context_delta,
            }
        )

    repaired["dof_pos"] = repaired_dof
    repaired["derived_fields_invalidated"] = True
    repaired["local_body_pos"] = None
    repaired["link_body_list"] = None
    if repaired_dof.shape[0] >= 2 and fps > 0:
        repaired["dof_vel"] = np.gradient(repaired_dof, 1.0 / fps, axis=0)
    else:
        repaired["dof_vel"] = np.zeros_like(repaired_dof)

    report = {
        "diagnostic_only_final_npz_smoothing": False,
        "source_side_repair_required_before_npz_acceptance": True,
        "fps": fps,
        "frame_count": int(repaired_dof.shape[0]),
        "repairs": repair_reports,
    }
    return repaired, report


def write_json_report(report: dict[str, Any], path: str | Path, *, overwrite: bool = False) -> None:
    target = Path(path)
    if target.exists() and not overwrite:
        raise FileExistsError(f"Output file already exists: {target}")
    payload = (json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n").encode("utf-8")
    _atomic_write_bytes(target, payload)
