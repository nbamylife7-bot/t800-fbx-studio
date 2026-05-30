from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Mapping

import numpy as np
from scipy.spatial.transform import Rotation as R


LIVE_BODY_NAMES = (
    "Hips",
    "Spine2",
    "Head",
    "LeftUpLeg",
    "LeftLeg",
    "LeftFootMod",
    "RightUpLeg",
    "RightLeg",
    "RightFootMod",
    "LeftArm",
    "LeftForeArm",
    "LeftHand",
    "RightArm",
    "RightForeArm",
    "RightHand",
)

VNECT_NAME_MAP = {
    "hip": "Hips",
    "spine": "Spine2",
    "head": "Head",
    "lThighBend": "LeftUpLeg",
    "rThighBend": "RightUpLeg",
    "lShin": "LeftLeg",
    "rShin": "RightLeg",
    "lFoot": "LeftFootMod",
    "rFoot": "RightFootMod",
    "lShldrBend": "LeftArm",
    "rShldrBend": "RightArm",
    "lForearmBend": "LeftForeArm",
    "rForearmBend": "RightForeArm",
    "lHand": "LeftHand",
    "rHand": "RightHand",
}

SEGMENTS = (
    ("Hips", "Spine2"),
    ("Spine2", "Head"),
    ("Hips", "LeftUpLeg"),
    ("LeftUpLeg", "LeftLeg"),
    ("LeftLeg", "LeftFootMod"),
    ("Hips", "RightUpLeg"),
    ("RightUpLeg", "RightLeg"),
    ("RightLeg", "RightFootMod"),
    ("Spine2", "LeftArm"),
    ("LeftArm", "LeftForeArm"),
    ("LeftForeArm", "LeftHand"),
    ("Spine2", "RightArm"),
    ("RightArm", "RightForeArm"),
    ("RightForeArm", "RightHand"),
)

CORE_HOLD_SEGMENTS = {
    "Hips->Spine2",
    "Spine2->Head",
    "Hips->LeftUpLeg",
    "Hips->RightUpLeg",
    "LeftUpLeg->LeftLeg",
    "RightUpLeg->RightLeg",
    "LeftLeg->LeftFootMod",
    "RightLeg->RightFootMod",
}

SEGMENT_MAX_LENGTHS_M = {
    "Hips->Spine2": 0.55,
    "Spine2->Head": 0.70,
    "Hips->LeftUpLeg": 0.55,
    "LeftUpLeg->LeftLeg": 0.75,
    "LeftLeg->LeftFootMod": 0.80,
    "Hips->RightUpLeg": 0.55,
    "RightUpLeg->RightLeg": 0.75,
    "RightLeg->RightFootMod": 0.80,
    "Spine2->LeftArm": 0.75,
    "LeftArm->LeftForeArm": 0.75,
    "LeftForeArm->LeftHand": 0.75,
    "Spine2->RightArm": 0.75,
    "RightArm->RightForeArm": 0.75,
    "RightForeArm->RightHand": 0.75,
}

T800_QPOS_ADDR = {
    "J00_HIP_PITCH_L": 7,
    "J01_HIP_ROLL_L": 8,
    "J02_HIP_YAW_L": 9,
    "J03_KNEE_PITCH_L": 10,
    "J04_ANKLE_PITCH_L": 11,
    "J05_ANKLE_ROLL_L": 12,
    "J06_HIP_PITCH_R": 13,
    "J07_HIP_ROLL_R": 14,
    "J08_HIP_YAW_R": 15,
    "J09_KNEE_PITCH_R": 16,
    "J10_ANKLE_PITCH_R": 17,
    "J11_ANKLE_ROLL_R": 18,
    "J12_TORSO_YAW": 19,
    "J13_SHOULDER_PITCH_L": 20,
    "J14_SHOULDER_ROLL_L": 21,
    "J15_SHOULDER_YAW_L": 22,
    "J16_ELBOW_PITCH_L": 23,
    "J17_ELBOW_YAW_L": 24,
    "J20_SHOULDER_PITCH_R": 25,
    "J21_SHOULDER_ROLL_R": 26,
    "J22_SHOULDER_YAW_R": 27,
    "J23_ELBOW_PITCH_R": 28,
    "J24_ELBOW_YAW_R": 29,
    "J27_HEAD_PITCH": 30,
    "J28_HEAD_YAW": 31,
}

T800_JOINT_LIMITS = {
    "J00_HIP_PITCH_L": (-3.316, 2.269),
    "J01_HIP_ROLL_L": (-1.082, 2.059),
    "J02_HIP_YAW_L": (-1.42244667, 3.6022778),
    "J03_KNEE_PITCH_L": (0.0, 2.355),
    "J04_ANKLE_PITCH_L": (-0.68068, 0.68068),
    "J05_ANKLE_ROLL_L": (-0.3491, 0.1745),
    "J06_HIP_PITCH_R": (-3.316, 2.269),
    "J07_HIP_ROLL_R": (-2.059, 1.082),
    "J08_HIP_YAW_R": (-3.6022778, 1.42244667),
    "J09_KNEE_PITCH_R": (0.0, 2.355),
    "J10_ANKLE_PITCH_R": (-0.68068, 0.68068),
    "J11_ANKLE_ROLL_R": (-0.1745, 0.3491),
    "J12_TORSO_YAW": (-4.381, 1.2392),
    "J13_SHOULDER_PITCH_L": (-2.967, 2.793),
    "J14_SHOULDER_ROLL_L": (-0.384, 2.443),
    "J15_SHOULDER_YAW_L": (-2.618, 2.618),
    "J16_ELBOW_PITCH_L": (-2.286, 0.262),
    "J17_ELBOW_YAW_L": (-2.618, 2.618),
    "J20_SHOULDER_PITCH_R": (-2.967, 2.793),
    "J21_SHOULDER_ROLL_R": (-2.443, 0.384),
    "J22_SHOULDER_YAW_R": (-2.618, 2.618),
    "J23_ELBOW_PITCH_R": (-2.286, 0.262),
    "J24_ELBOW_YAW_R": (-2.618, 2.618),
    "J27_HEAD_PITCH": (-0.523, 0.523),
    "J28_HEAD_YAW": (-1.222, 1.222),
}

LIVE_SAFE_JOINT_LIMITS = {
    "J00_HIP_PITCH_L": (-0.9, 0.9),
    "J01_HIP_ROLL_L": (-0.35, 0.35),
    "J02_HIP_YAW_L": (-0.6, 0.6),
    "J03_KNEE_PITCH_L": (0.0, 1.2),
    "J04_ANKLE_PITCH_L": (-0.35, 0.35),
    "J05_ANKLE_ROLL_L": (-0.2, 0.2),
    "J06_HIP_PITCH_R": (-0.9, 0.9),
    "J07_HIP_ROLL_R": (-0.35, 0.35),
    "J08_HIP_YAW_R": (-0.6, 0.6),
    "J09_KNEE_PITCH_R": (0.0, 1.2),
    "J10_ANKLE_PITCH_R": (-0.35, 0.35),
    "J11_ANKLE_ROLL_R": (-0.2, 0.2),
    "J12_TORSO_YAW": (-0.5, 0.5),
    "J13_SHOULDER_PITCH_L": (-1.2, 1.2),
    "J14_SHOULDER_ROLL_L": (-0.9, 0.9),
    "J15_SHOULDER_YAW_L": (-0.8, 0.8),
    "J16_ELBOW_PITCH_L": (-1.4, -0.03),
    "J17_ELBOW_YAW_L": (-0.8, 0.8),
    "J20_SHOULDER_PITCH_R": (-1.2, 1.2),
    "J21_SHOULDER_ROLL_R": (-0.9, 0.35),
    "J22_SHOULDER_YAW_R": (-0.8, 0.8),
    "J23_ELBOW_PITCH_R": (-1.4, -0.03),
    "J24_ELBOW_YAW_R": (-0.8, 0.8),
    "J27_HEAD_PITCH": (-0.35, 0.35),
    "J28_HEAD_YAW": (-0.7, 0.7),
}


@dataclass(frozen=True)
class PoseValidationReport:
    is_valid: bool
    reasons: tuple[str, ...]
    segment_lengths_m: dict[str, float]


def make_valid_pose_packet(
    points: Mapping[str, object],
    *,
    root_pos: np.ndarray | None = None,
    root_yaw: float | None = None,
    bad_segments: list[str] | tuple[str, ...] = (),
) -> dict[str, object]:
    packet: dict[str, object] = {
        "_valid": True,
        "_points": {
            name: _as_vec3(value).tolist()
            for name, value in points.items()
            if _as_vec3(value) is not None
        },
        "_bad_segments": list(bad_segments),
    }
    if root_pos is not None:
        packet["_root_pos"] = np.asarray(root_pos, dtype=np.float64).tolist()
    if root_yaw is not None:
        packet["_root_yaw"] = float(root_yaw)
    return packet


def make_invalid_pose_packet(reasons: list[str] | tuple[str, ...]) -> dict[str, object]:
    return {"_valid": False, "_reasons": list(reasons)}


def unwrap_live_pose_packet(packet: Mapping[str, object]) -> tuple[PoseValidationReport, dict[str, object]]:
    if packet.get("_valid") is False:
        reasons = packet.get("_reasons", ["invalid"])
        if not isinstance(reasons, list):
            reasons = ["invalid"]
        return PoseValidationReport(False, tuple(str(reason) for reason in reasons), {}), {}

    points = packet.get("_points")
    if isinstance(points, Mapping):
        envelope_reasons = packet.get("_bad_segments", [])
        if not isinstance(envelope_reasons, list):
            envelope_reasons = []
        validation = LivePoseGate().validate(points)
        merged_reasons = tuple(dict.fromkeys(
            [str(reason) for reason in envelope_reasons] + list(validation.reasons)
        ))
        is_valid = not any(_is_fatal_pose_reason(reason) for reason in merged_reasons)
        return PoseValidationReport(is_valid, merged_reasons, validation.segment_lengths_m), dict(points)

    return LivePoseGate().validate(packet), dict(packet)


def _is_fatal_pose_reason(reason: str) -> bool:
    return reason.startswith("missing:") or reason.startswith("too_few_keypoints:")


def should_hold_live_frame(
    report: PoseValidationReport,
    *,
    max_soft_bad_segments: int = 2,
) -> bool:
    if not report.is_valid:
        return True
    segment_reasons = [reason for reason in report.reasons if "->" in reason]
    if len(segment_reasons) > int(max_soft_bad_segments):
        return True
    return any(reason in CORE_HOLD_SEGMENTS for reason in segment_reasons)


def extract_root_pose(packet: Mapping[str, object]) -> tuple[np.ndarray | None, float | None]:
    root_pos = _as_vec3(packet.get("_root_pos"))
    root_yaw_value = packet.get("_root_yaw")
    if isinstance(root_yaw_value, (int, float)) and np.isfinite(root_yaw_value):
        root_yaw = float(root_yaw_value)
    else:
        root_yaw = None
    return root_pos, root_yaw


def _as_vec3(value: object) -> np.ndarray | None:
    try:
        array = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        return None
    return array


def _clamp(value: float, lower: float, upper: float) -> float:
    return float(min(max(value, lower), upper))


def _unit(vector: np.ndarray) -> np.ndarray | None:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-8:
        return None
    return vector / norm


def normalize_live_pose(
    packet: Mapping[str, object],
    *,
    source_scale: float = 1.0,
    pelvis_anchor: np.ndarray | None = None,
) -> dict[str, np.ndarray]:
    """Map VNect camera coordinates to T800-friendly robot coordinates.

    VNect gives x=screen right, y=up, z=depth. The T800 MuJoCo frame uses
    x=forward, y=left, z=up, so the axis map is [z, -x, y].
    """

    hip = _as_vec3(packet.get("Hips"))
    if hip is None:
        raise ValueError("Live pose is missing a valid Hips keypoint.")

    anchor = np.array([0.0, 0.0, 1.02], dtype=np.float64)
    if pelvis_anchor is not None:
        anchor = np.asarray(pelvis_anchor, dtype=np.float64)

    normalized: dict[str, np.ndarray] = {}
    for name, value in packet.items():
        point = _as_vec3(value)
        if point is None:
            continue
        rel = (point - hip) * float(source_scale)
        normalized[name] = np.array([rel[2], -rel[0], rel[1]], dtype=np.float64) + anchor
    return normalized


def _camera_to_robot_vector(vector: np.ndarray, source_scale: float) -> np.ndarray:
    rel = np.asarray(vector, dtype=np.float64) * float(source_scale)
    return np.array([rel[2], -rel[0], rel[1]], dtype=np.float64)


def map_vnect_skeleton(
    skeleton: Mapping[str, object],
    *,
    source_scale: float = 0.005,
    pelvis_anchor: np.ndarray | None = None,
) -> dict[str, list[float]]:
    mapped: dict[str, np.ndarray] = {}
    for source_name, value in skeleton.items():
        target_name = VNECT_NAME_MAP.get(source_name)
        if target_name is None:
            continue
        point = _as_vec3(value)
        if point is not None:
            mapped[target_name] = point

    normalized = normalize_live_pose(
        mapped,
        source_scale=source_scale,
        pelvis_anchor=pelvis_anchor,
    )
    return {name: point.tolist() for name, point in normalized.items() if name in LIVE_BODY_NAMES}


class LivePoseNormalizer:
    def __init__(
        self,
        *,
        source_scale: float = 0.005,
        pelvis_anchor: np.ndarray | None = None,
        root_height_m: float = 1.05,
        root_alpha: float = 1.0,
        yaw_alpha: float = 1.0,
        max_root_step_m: float = 0.08,
        max_yaw_step_rad: float = 0.12,
    ) -> None:
        self.source_scale = float(source_scale)
        self.pelvis_anchor = np.array([0.0, 0.0, 1.02], dtype=np.float64)
        if pelvis_anchor is not None:
            self.pelvis_anchor = np.asarray(pelvis_anchor, dtype=np.float64)
        self.root_height_m = float(root_height_m)
        self.root_alpha = float(root_alpha)
        self.yaw_alpha = float(yaw_alpha)
        self.max_root_step_m = float(max_root_step_m)
        self.max_yaw_step_rad = float(max_yaw_step_rad)
        self.origin_hip: np.ndarray | None = None
        self.root_pos: np.ndarray | None = None
        self.root_yaw: float | None = None

    def normalize(self, skeleton: Mapping[str, object]) -> tuple[dict[str, list[float]], np.ndarray, float]:
        mapped: dict[str, np.ndarray] = {}
        for source_name, value in skeleton.items():
            target_name = VNECT_NAME_MAP.get(source_name)
            if target_name is None:
                continue
            point = _as_vec3(value)
            if point is not None:
                mapped[target_name] = point

        hip = _as_vec3(mapped.get("Hips"))
        if hip is None:
            raise ValueError("Live pose is missing a valid Hips keypoint.")

        if self.origin_hip is None:
            self.origin_hip = hip.copy()

        points = normalize_live_pose(
            mapped,
            source_scale=self.source_scale,
            pelvis_anchor=self.pelvis_anchor,
        )
        desired_root = self.pelvis_anchor + _camera_to_robot_vector(hip - self.origin_hip, self.source_scale)
        desired_root[2] = self.root_height_m
        self.root_pos = self._smooth_root(desired_root)
        desired_yaw = _estimate_root_yaw(points)
        self.root_yaw = self._smooth_yaw(0.0 if desired_yaw is None else desired_yaw)
        return (
            {name: point.tolist() for name, point in points.items() if name in LIVE_BODY_NAMES},
            self.root_pos.copy(),
            float(self.root_yaw),
        )

    def _smooth_root(self, desired: np.ndarray) -> np.ndarray:
        if self.root_pos is None:
            return desired.copy()
        blended = self.root_pos + self.root_alpha * (desired - self.root_pos)
        delta = blended - self.root_pos
        delta_norm = float(np.linalg.norm(delta[:2]))
        if delta_norm > self.max_root_step_m:
            delta[:2] *= self.max_root_step_m / delta_norm
        delta[2] = desired[2] - self.root_pos[2]
        return self.root_pos + delta

    def _smooth_yaw(self, desired: float) -> float:
        if self.root_yaw is None:
            return float(desired)
        delta = _wrap_angle(desired - self.root_yaw)
        delta = _clamp(delta * self.yaw_alpha, -self.max_yaw_step_rad, self.max_yaw_step_rad)
        return _wrap_angle(self.root_yaw + delta)


def build_human_frame(
    packet: Mapping[str, object],
    *,
    root_pos: np.ndarray | None = None,
    root_yaw: float | None = None,
) -> dict[str, list[np.ndarray]]:
    """Build a BVH-like GMR frame from live VNect keypoints.

    The BVH loader feeds GMR global positions plus global quaternions. VNect gives
    only points, so we reconstruct stable segment orientations where local +X
    points from each joint toward its child. That mirrors LAFAN1's convention.
    """

    points = {
        name: point
        for name in LIVE_BODY_NAMES
        if (point := _as_vec3(packet.get(name))) is not None
    }

    if root_pos is not None and "Hips" in points:
        offset = np.asarray(root_pos, dtype=np.float64) - points["Hips"]
        points = {name: point + offset for name, point in points.items()}

    root_forward, root_left, root_up = _root_axes_from_points(points, root_yaw=root_yaw)
    flat_up = np.array([0.0, 0.0, 1.0], dtype=np.float64)

    child_by_body = {
        "Hips": "Spine2",
        "Spine2": "Head",
        "Head": "Spine2",
        "LeftUpLeg": "LeftLeg",
        "LeftLeg": "LeftFootMod",
        "RightUpLeg": "RightLeg",
        "RightLeg": "RightFootMod",
        "LeftArm": "LeftForeArm",
        "LeftForeArm": "LeftHand",
        "LeftHand": "LeftForeArm",
        "RightArm": "RightForeArm",
        "RightForeArm": "RightHand",
        "RightHand": "RightForeArm",
    }

    human_frame: dict[str, list[np.ndarray]] = {}
    for name in LIVE_BODY_NAMES:
        point = points.get(name)
        if point is None:
            continue

        if name in ("LeftFootMod", "RightFootMod"):
            quat = _rotation_from_x_axis(root_forward, y_hint=root_left, z_hint=flat_up)
        else:
            child_name = child_by_body.get(name)
            child_point = points.get(child_name) if child_name is not None else None
            if child_point is None:
                quat = _rotation_from_x_axis(root_forward, y_hint=root_left, z_hint=root_up)
            else:
                direction = child_point - point
                if name in ("Head", "LeftHand", "RightHand"):
                    direction = -direction
                quat = _rotation_from_x_axis(direction, y_hint=root_left, z_hint=root_up)

        human_frame[name] = [point, quat]
    return human_frame


def _root_axes_from_points(
    points: Mapping[str, np.ndarray],
    *,
    root_yaw: float | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if root_yaw is not None and np.isfinite(root_yaw):
        forward = np.array([math.cos(root_yaw), math.sin(root_yaw), 0.0], dtype=np.float64)
        left = np.array([-math.sin(root_yaw), math.cos(root_yaw), 0.0], dtype=np.float64)
        return forward, left, np.array([0.0, 0.0, 1.0], dtype=np.float64)

    left_vectors = []
    if "LeftArm" in points and "RightArm" in points:
        left_vectors.append(points["LeftArm"] - points["RightArm"])
    if "LeftUpLeg" in points and "RightUpLeg" in points:
        left_vectors.append(points["LeftUpLeg"] - points["RightUpLeg"])
    left = _unit(np.mean(left_vectors, axis=0)) if left_vectors else None
    if left is None:
        left = np.array([0.0, 1.0, 0.0], dtype=np.float64)

    up_vectors = []
    if "Hips" in points and "Spine2" in points:
        up_vectors.append(points["Spine2"] - points["Hips"])
    if "Spine2" in points and "Head" in points:
        up_vectors.append(points["Head"] - points["Spine2"])
    up = _unit(np.mean(up_vectors, axis=0)) if up_vectors else None
    if up is None:
        up = np.array([0.0, 0.0, 1.0], dtype=np.float64)

    forward = _unit(np.cross(left, up))
    if forward is None:
        forward = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    left = _unit(np.cross(up, forward))
    if left is None:
        left = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    up = _unit(np.cross(forward, left))
    if up is None:
        up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    return forward, left, up


def _rotation_from_x_axis(
    x_axis: np.ndarray,
    *,
    y_hint: np.ndarray | None = None,
    z_hint: np.ndarray | None = None,
) -> np.ndarray:
    x = _unit(np.asarray(x_axis, dtype=np.float64))
    if x is None:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    y = None
    if y_hint is not None:
        candidate = np.asarray(y_hint, dtype=np.float64)
        candidate = candidate - x * float(np.dot(candidate, x))
        y = _unit(candidate)

    if y is None and z_hint is not None:
        y = _unit(np.cross(np.asarray(z_hint, dtype=np.float64), x))

    if y is None:
        fallback = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        if abs(float(np.dot(fallback, x))) > 0.92:
            fallback = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        fallback = fallback - x * float(np.dot(fallback, x))
        y = _unit(fallback)

    if y is None:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    z = _unit(np.cross(x, y))
    if z is None:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    y = _unit(np.cross(z, x))
    if y is None:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    matrix = np.column_stack([x, y, z])
    if np.linalg.det(matrix) < 0:
        y = -y
        z = _unit(np.cross(x, y))
        matrix = np.column_stack([x, y, z])
    return R.from_matrix(matrix).as_quat(scalar_first=True)


class LivePoseGate:
    def __init__(
        self,
        *,
        max_segment_length_m: float = 1.2,
        min_required_keypoints: int = 12,
    ) -> None:
        self.max_segment_length_m = float(max_segment_length_m)
        self.min_required_keypoints = int(min_required_keypoints)

    def validate(self, packet: Mapping[str, object]) -> PoseValidationReport:
        points = {
            name: point
            for name in LIVE_BODY_NAMES
            if (point := _as_vec3(packet.get(name))) is not None
        }
        reasons: list[str] = []
        if "Hips" not in points:
            reasons.append("missing:Hips")
        if len(points) < self.min_required_keypoints:
            reasons.append(f"too_few_keypoints:{len(points)}")

        lengths: dict[str, float] = {}
        for parent, child in SEGMENTS:
            if parent not in points or child not in points:
                continue
            label = f"{parent}->{child}"
            length = float(np.linalg.norm(points[child] - points[parent]))
            lengths[label] = length
            max_length = min(self.max_segment_length_m, SEGMENT_MAX_LENGTHS_M.get(label, self.max_segment_length_m))
            if length > max_length:
                reasons.append(label)

        return PoseValidationReport(
            is_valid=len(reasons) == 0,
            reasons=tuple(reasons),
            segment_lengths_m=lengths,
        )


class LivePosePointFilter:
    def __init__(
        self,
        *,
        max_point_step_m: float = 0.07,
        alpha: float = 0.55,
    ) -> None:
        self.max_point_step_m = float(max_point_step_m)
        self.alpha = float(alpha)
        self.previous_points: dict[str, np.ndarray] | None = None

    def reset(self) -> None:
        self.previous_points = None

    def apply(
        self,
        packet: Mapping[str, object],
        *,
        bad_segments: set[str] | tuple[str, ...] | list[str] = (),
    ) -> dict[str, np.ndarray]:
        points = {
            name: point
            for name in LIVE_BODY_NAMES
            if (point := _as_vec3(packet.get(name))) is not None
        }
        if self.previous_points is None:
            self.previous_points = {name: point.copy() for name, point in points.items()}
            return {name: point.copy() for name, point in points.items()}

        frozen_children = _bad_segment_children(bad_segments)
        filtered: dict[str, np.ndarray] = {}
        for name, point in points.items():
            previous = self.previous_points.get(name)
            if previous is None:
                filtered[name] = point.copy()
                continue
            if name in frozen_children:
                filtered[name] = previous.copy()
                continue

            desired = previous + self.alpha * (point - previous)
            delta = desired - previous
            delta_norm = float(np.linalg.norm(delta))
            if delta_norm > self.max_point_step_m:
                delta *= self.max_point_step_m / delta_norm
            filtered[name] = previous + delta

        for name, previous in self.previous_points.items():
            filtered.setdefault(name, previous.copy())

        self.previous_points = {name: point.copy() for name, point in filtered.items()}
        return filtered


def _bad_segment_children(bad_segments: set[str] | tuple[str, ...] | list[str]) -> set[str]:
    children: set[str] = set()
    for segment in bad_segments:
        if not isinstance(segment, str) or "->" not in segment:
            continue
        _parent, child = segment.split("->", 1)
        if child in LIVE_BODY_NAMES:
            children.add(child)
    return children


class T800AnalyticLimbEstimator:
    def estimate(
        self,
        packet: Mapping[str, object],
        *,
        bad_segments: set[str] | tuple[str, ...] | list[str] = (),
        include_legs: bool = True,
        include_arms: bool = True,
    ) -> dict[str, float]:
        points = {
            name: point
            for name in LIVE_BODY_NAMES
            if (point := _as_vec3(packet.get(name))) is not None
        }
        bad_segment_set = set(bad_segments)
        joints: dict[str, float] = {}
        if include_legs:
            self._estimate_leg(points, joints, "L", bad_segment_set)
            self._estimate_leg(points, joints, "R", bad_segment_set)
        if include_arms:
            self._estimate_arm(points, joints, "L", bad_segment_set)
            self._estimate_arm(points, joints, "R", bad_segment_set)
        return {name: self._clamp_joint(name, value) for name, value in joints.items()}

    def _estimate_leg(
        self,
        points: Mapping[str, np.ndarray],
        joints: dict[str, float],
        side: str,
        bad_segments: set[str],
    ) -> None:
        if side == "L":
            upper, knee, foot = "LeftUpLeg", "LeftLeg", "LeftFootMod"
            hip_pitch, hip_roll, knee_pitch, ankle_pitch, ankle_roll = (
                "J00_HIP_PITCH_L",
                "J01_HIP_ROLL_L",
                "J03_KNEE_PITCH_L",
                "J04_ANKLE_PITCH_L",
                "J05_ANKLE_ROLL_L",
            )
        else:
            upper, knee, foot = "RightUpLeg", "RightLeg", "RightFootMod"
            hip_pitch, hip_roll, knee_pitch, ankle_pitch, ankle_roll = (
                "J06_HIP_PITCH_R",
                "J07_HIP_ROLL_R",
                "J09_KNEE_PITCH_R",
                "J10_ANKLE_PITCH_R",
                "J11_ANKLE_ROLL_R",
            )

        if upper not in points or knee not in points or foot not in points:
            return
        if f"{upper}->{knee}" in bad_segments or f"{knee}->{foot}" in bad_segments:
            return

        thigh_vector = points[knee] - points[upper]
        pitch, roll = _direction_pitch_roll(thigh_vector)
        knee_angle = _two_bone_bend(points[upper], points[knee], points[foot])

        joints[hip_pitch] = pitch
        joints[hip_roll] = roll
        joints[knee_pitch] = knee_angle
        joints[ankle_pitch] = -0.45 * pitch - 0.25 * knee_angle
        joints[ankle_roll] = -0.35 * roll

    def _estimate_arm(
        self,
        points: Mapping[str, np.ndarray],
        joints: dict[str, float],
        side: str,
        bad_segments: set[str],
    ) -> None:
        if side == "L":
            upper, elbow, hand = "LeftArm", "LeftForeArm", "LeftHand"
            shoulder_pitch, shoulder_roll, elbow_pitch = (
                "J13_SHOULDER_PITCH_L",
                "J14_SHOULDER_ROLL_L",
                "J16_ELBOW_PITCH_L",
            )
        else:
            upper, elbow, hand = "RightArm", "RightForeArm", "RightHand"
            shoulder_pitch, shoulder_roll, elbow_pitch = (
                "J20_SHOULDER_PITCH_R",
                "J21_SHOULDER_ROLL_R",
                "J23_ELBOW_PITCH_R",
            )

        if upper not in points or elbow not in points or hand not in points:
            return
        if f"{upper}->{elbow}" in bad_segments or f"{elbow}->{hand}" in bad_segments:
            return

        upper_arm_vector = points[elbow] - points[upper]
        pitch, roll = _direction_pitch_roll(upper_arm_vector)
        elbow_angle = _two_bone_bend(points[upper], points[elbow], points[hand])

        joints[shoulder_pitch] = pitch
        joints[shoulder_roll] = roll
        joints[elbow_pitch] = -elbow_angle

    def _clamp_joint(self, joint_name: str, value: float) -> float:
        lower, upper = T800_JOINT_LIMITS[joint_name]
        return _clamp(float(value), lower, upper)


def _direction_pitch_roll(vector: np.ndarray) -> tuple[float, float]:
    direction = _unit(vector)
    if direction is None:
        return 0.0, 0.0
    pitch = math.atan2(float(direction[0]), float(-direction[2]))
    roll = math.atan2(float(direction[1]), float(-direction[2]))
    return pitch, roll


def _two_bone_bend(proximal: np.ndarray, middle: np.ndarray, distal: np.ndarray) -> float:
    upper = float(np.linalg.norm(middle - proximal))
    lower = float(np.linalg.norm(distal - middle))
    reach = float(np.linalg.norm(distal - proximal))
    if upper < 1e-6 or lower < 1e-6:
        return 0.0
    cosine = _clamp((upper * upper + lower * lower - reach * reach) / (2.0 * upper * lower), -1.0, 1.0)
    internal_angle = math.acos(cosine)
    return _clamp(math.pi - internal_angle, 0.0, math.pi)


def _estimate_root_yaw(points: Mapping[str, np.ndarray]) -> float | None:
    lateral_vectors = []
    if "LeftArm" in points and "RightArm" in points:
        lateral_vectors.append(points["LeftArm"] - points["RightArm"])
    if "LeftUpLeg" in points and "RightUpLeg" in points:
        lateral_vectors.append(points["LeftUpLeg"] - points["RightUpLeg"])
    if not lateral_vectors:
        return None
    lateral = np.mean(lateral_vectors, axis=0)
    xy_norm = float(np.linalg.norm(lateral[:2]))
    if xy_norm < 1e-6:
        return None
    left_x, left_y = lateral[0] / xy_norm, lateral[1] / xy_norm
    return math.atan2(float(-left_x), float(left_y))


def _wrap_angle(angle: float) -> float:
    return float((angle + math.pi) % (2.0 * math.pi) - math.pi)


def _yaw_quat_wxyz(yaw: float) -> np.ndarray:
    half = 0.5 * float(yaw)
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=np.float64)


def _yaw_from_quat_wxyz(quat: np.ndarray) -> float:
    quat = np.asarray(quat, dtype=np.float64)
    if quat.shape != (4,) or not np.all(np.isfinite(quat)):
        return 0.0
    norm = float(np.linalg.norm(quat))
    if norm < 1e-8:
        return 0.0
    w, x, y, z = quat / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class LiveGMRQposFilter:
    def __init__(
        self,
        *,
        max_joint_step_rad: float = 0.16,
        max_root_step_m: float = 0.08,
        max_root_z_step_m: float = 0.06,
        max_yaw_step_rad: float = 0.12,
        blend_alpha: float = 0.65,
        neutral_root_pos: np.ndarray | None = None,
    ) -> None:
        self.max_joint_step_rad = float(max_joint_step_rad)
        self.max_root_step_m = float(max_root_step_m)
        self.max_root_z_step_m = float(max_root_z_step_m)
        self.max_yaw_step_rad = float(max_yaw_step_rad)
        self.blend_alpha = float(blend_alpha)
        self.neutral_root_pos = np.array([0.0, 0.0, 1.05], dtype=np.float64)
        if neutral_root_pos is not None:
            self.neutral_root_pos = np.asarray(neutral_root_pos, dtype=np.float64)
        self.previous_qpos: np.ndarray | None = None

    def reset(self) -> None:
        self.previous_qpos = None

    def neutral_qpos(self, size: int = 32) -> np.ndarray:
        qpos = np.zeros(size, dtype=np.float64)
        qpos[:3] = self.neutral_root_pos
        qpos[3] = 1.0
        return qpos

    def apply(
        self,
        gmr_qpos: np.ndarray,
        *,
        root_pos: np.ndarray | None = None,
        root_yaw: float | None = None,
    ) -> np.ndarray:
        desired = np.asarray(gmr_qpos, dtype=np.float64).copy()
        if desired.shape[0] < 8:
            raise ValueError("Expected a floating-base qpos vector.")

        if root_pos is not None:
            live_root = np.asarray(root_pos, dtype=np.float64)
            desired[:2] = live_root[:2]
        if root_yaw is not None:
            desired[3:7] = _yaw_quat_wxyz(root_yaw)

        if self.previous_qpos is None:
            self.previous_qpos = desired.copy()
            return desired

        previous = self.previous_qpos
        filtered = desired.copy()

        root_delta = desired[:2] - previous[:2]
        root_delta_norm = float(np.linalg.norm(root_delta))
        if root_delta_norm > self.max_root_step_m:
            root_delta *= self.max_root_step_m / root_delta_norm
        filtered[:2] = previous[:2] + root_delta
        filtered[2] = previous[2] + _clamp(
            float(desired[2] - previous[2]),
            -self.max_root_z_step_m,
            self.max_root_z_step_m,
        )

        desired_yaw = _yaw_from_quat_wxyz(desired[3:7])
        previous_yaw = _yaw_from_quat_wxyz(previous[3:7])
        yaw_delta = _clamp(
            _wrap_angle(desired_yaw - previous_yaw),
            -self.max_yaw_step_rad,
            self.max_yaw_step_rad,
        )
        filtered[3:7] = _yaw_quat_wxyz(_wrap_angle(previous_yaw + yaw_delta))

        for qpos_addr in range(7, desired.shape[0]):
            blended = previous[qpos_addr] + self.blend_alpha * (desired[qpos_addr] - previous[qpos_addr])
            delta = _clamp(
                float(blended - previous[qpos_addr]),
                -self.max_joint_step_rad,
                self.max_joint_step_rad,
            )
            filtered[qpos_addr] = previous[qpos_addr] + delta

        self.previous_qpos = filtered.copy()
        return filtered

    def hold_or_relax(self, size: int = 32) -> np.ndarray:
        if self.previous_qpos is not None:
            return self.previous_qpos.copy()
        return self.neutral_qpos(size)


def ground_qpos_to_support_geoms(
    model: object,
    data: object,
    qpos: np.ndarray,
    *,
    support_geom_ids: list[int] | tuple[int, ...] | None = None,
    clearance: float = 0.002,
    max_correction_m: float = 0.20,
) -> np.ndarray:
    """Shift floating-base z so the lowest foot support geom touches the floor."""

    import mujoco as mj

    from general_motion_retargeting.motion_grounding import (
        find_support_geom_ids,
        geom_lowest_z,
    )

    grounded = np.asarray(qpos, dtype=np.float64).copy()
    if grounded.shape[0] < 7:
        raise ValueError("Expected a floating-base qpos vector.")

    data.qpos[:] = grounded
    mj.mj_forward(model, data)
    support_ids = list(support_geom_ids) if support_geom_ids is not None else find_support_geom_ids(model)
    min_support_z = min(geom_lowest_z(model, data, geom_id) for geom_id in support_ids)
    correction = _clamp(
        float(clearance) - float(min_support_z),
        -float(max_correction_m),
        float(max_correction_m),
    )
    grounded[2] += correction
    return grounded


class HybridT800QposFilter:
    def __init__(
        self,
        *,
        max_joint_step_rad: float = 0.18,
        relax_step_rad: float = 0.08,
        blend_alpha: float = 0.75,
        use_base_qpos: bool = True,
        use_live_safe_limits: bool = True,
        fixed_root_pos: np.ndarray | None = None,
        fixed_root_quat: np.ndarray | None = None,
    ) -> None:
        self.max_joint_step_rad = float(max_joint_step_rad)
        self.relax_step_rad = float(relax_step_rad)
        self.blend_alpha = float(blend_alpha)
        self.use_base_qpos = bool(use_base_qpos)
        self.use_live_safe_limits = bool(use_live_safe_limits)
        self.fixed_root_pos = np.array([0.0, 0.0, 1.05], dtype=np.float64)
        if fixed_root_pos is not None:
            self.fixed_root_pos = np.asarray(fixed_root_pos, dtype=np.float64)
        self.fixed_root_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        if fixed_root_quat is not None:
            self.fixed_root_quat = np.asarray(fixed_root_quat, dtype=np.float64)
        self.previous_qpos: np.ndarray | None = None

    def reset(self) -> None:
        self.previous_qpos = None

    def neutral_qpos(self, size: int = 32) -> np.ndarray:
        qpos = np.zeros(size, dtype=np.float64)
        qpos[:3] = self.fixed_root_pos
        qpos[3:7] = self.fixed_root_quat
        return qpos

    def apply(
        self,
        base_qpos: np.ndarray,
        analytic_joints: Mapping[str, float],
        *,
        root_pos: np.ndarray | None = None,
        root_yaw: float | None = None,
    ) -> np.ndarray:
        base = np.asarray(base_qpos, dtype=np.float64)
        if base.shape[0] < 32:
            raise ValueError("Expected a T800 qpos vector with at least 32 values.")
        if self.use_base_qpos:
            qpos = base.copy()
        elif self.previous_qpos is not None:
            qpos = self.previous_qpos.copy()
        else:
            qpos = self.neutral_qpos(base.shape[0])

        qpos[:3] = self.fixed_root_pos if root_pos is None else np.asarray(root_pos, dtype=np.float64)
        qpos[3:7] = self.fixed_root_quat if root_yaw is None else _yaw_quat_wxyz(root_yaw)
        previous = qpos if self.previous_qpos is None else self.previous_qpos

        for joint_name, analytic_value in analytic_joints.items():
            qpos_addr = T800_QPOS_ADDR.get(joint_name)
            limits = self._limits_for_joint(joint_name)
            if qpos_addr is None or limits is None:
                continue
            desired = (1.0 - self.blend_alpha) * qpos[qpos_addr] + self.blend_alpha * float(analytic_value)
            lower, upper = limits
            desired = _clamp(desired, lower, upper)
            delta = _clamp(desired - float(previous[qpos_addr]), -self.max_joint_step_rad, self.max_joint_step_rad)
            qpos[qpos_addr] = _clamp(float(previous[qpos_addr]) + delta, lower, upper)

        self.previous_qpos = qpos.copy()
        return qpos

    def relax_to_neutral(
        self,
        size: int = 32,
        *,
        root_pos: np.ndarray | None = None,
        root_yaw: float | None = None,
    ) -> np.ndarray:
        previous = self.neutral_qpos(size) if self.previous_qpos is None else self.previous_qpos
        qpos = previous.copy()
        neutral = self.neutral_qpos(previous.shape[0])
        for joint_name, qpos_addr in T800_QPOS_ADDR.items():
            limits = self._limits_for_joint(joint_name)
            if limits is None:
                continue
            delta = _clamp(
                float(neutral[qpos_addr] - previous[qpos_addr]),
                -self.relax_step_rad,
                self.relax_step_rad,
            )
            lower, upper = limits
            qpos[qpos_addr] = _clamp(float(previous[qpos_addr]) + delta, lower, upper)
        if root_pos is not None:
            qpos[:3] = np.asarray(root_pos, dtype=np.float64)
        elif self.previous_qpos is None:
            qpos[:3] = self.fixed_root_pos
        if root_yaw is not None:
            qpos[3:7] = _yaw_quat_wxyz(root_yaw)
        elif self.previous_qpos is None:
            qpos[3:7] = self.fixed_root_quat
        self.previous_qpos = qpos.copy()
        return qpos

    def _limits_for_joint(self, joint_name: str) -> tuple[float, float] | None:
        if self.use_live_safe_limits and joint_name in LIVE_SAFE_JOINT_LIMITS:
            return LIVE_SAFE_JOINT_LIMITS[joint_name]
        return T800_JOINT_LIMITS.get(joint_name)


def write_pose_json_atomic(path: str | Path, packet: Mapping[str, object]) -> None:
    output_path = Path(path)
    tmp_path = output_path.with_suffix(output_path.suffix + ".tmp")
    import json

    with tmp_path.open("w", encoding="utf-8") as pose_file:
        json.dump(packet, pose_file)
    tmp_path.replace(output_path)
