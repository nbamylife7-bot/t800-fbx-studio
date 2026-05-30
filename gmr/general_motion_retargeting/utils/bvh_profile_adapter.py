from __future__ import annotations

import json
import re
from pathlib import Path

import numpy as np

import general_motion_retargeting.utils.lafan_vendor.utils as utils
from general_motion_retargeting.utils.lafan_vendor.extract import Anim


# 这里保留 BVH 旋转通道到轴名的映射，后续会把每个关节自己的通道顺序转成 Mink/LAFAN 工具函数认识的 xyz 字符串。
_ROTATION_CHANNEL_TO_AXIS = {
    "Xrotation": "x",
    "Yrotation": "y",
    "Zrotation": "z",
}


# 这里保留 BVH 平移通道到位置向量下标的映射，用于按关节逐项回填局部平移。
_POSITION_CHANNEL_TO_AXIS = {
    "Xposition": 0,
    "Yposition": 1,
    "Zposition": 2,
}


# 这个集合对应官方 LAFAN1 在 GMR 里真正依赖的主体骨架，不包含当前项目 `hit_data` 里额外出现的手指和扩展脊柱节点。
_OFFICIAL_LAFAN1_CORE_JOINTS = {
    "Hips",
    "LeftUpLeg",
    "LeftLeg",
    "LeftFoot",
    "LeftToe",
    "RightUpLeg",
    "RightLeg",
    "RightFoot",
    "RightToe",
    "Spine",
    "Spine1",
    "Spine2",
    "Neck",
    "Head",
    "LeftShoulder",
    "LeftArm",
    "LeftForeArm",
    "LeftHand",
    "RightShoulder",
    "RightArm",
    "RightForeArm",
    "RightHand",
}


def _parse_bvh_layout(bvh_file: str | Path) -> dict:
    """解析 BVH 的层级定义与 motion 头信息。

    这里故意不直接复用 vendor 版 `read_bvh`，因为当前项目 `hit_data` 的关键问题之一，
    正是“不同关节使用了不同的 CHANNELS 旋转顺序”。
    vendor 版本只会从第一个关节推断一次顺序，然后默认所有关节都一样；
    这个假设对官方 LAFAN1 成立，但对当前项目的 BVH 不成立。
    """

    # 使用 utf-8 + ignore 是为了兼容不同来源 BVH 里偶发的非标准字符，不让编码问题干扰结构解析。
    text_lines = Path(bvh_file).read_text(encoding="utf-8", errors="ignore").splitlines()

    # 下面这些数组会按“真实关节出现顺序”记录骨架拓扑，后续 FK / IK 也依赖这个顺序。
    joint_names: list[str] = []
    joint_offsets: list[list[float]] = []
    joint_parents: list[int] = []
    joint_channels: list[list[str]] = []

    # `active_joint_index` 指向当前正在解析的关节；`inside_end_site` 用于跳过 End Site 伪节点。
    active_joint_index = -1
    inside_end_site = False
    motion_header_index = None

    # 第一阶段只解析 HIERARCHY 区域，遇到 `MOTION` 就停止。
    for line_index, raw_line in enumerate(text_lines):
        stripped_line = raw_line.strip()

        if stripped_line == "HIERARCHY":
            continue

        if stripped_line == "MOTION":
            motion_header_index = line_index
            break

        root_match = re.match(r"ROOT\s+(\w+)", stripped_line)
        if root_match:
            joint_names.append(root_match.group(1))
            joint_offsets.append([0.0, 0.0, 0.0])
            joint_parents.append(active_joint_index)
            joint_channels.append([])
            active_joint_index = len(joint_names) - 1
            continue

        joint_match = re.match(r"JOINT\s+(\w+)", stripped_line)
        if joint_match:
            joint_names.append(joint_match.group(1))
            joint_offsets.append([0.0, 0.0, 0.0])
            joint_parents.append(active_joint_index)
            joint_channels.append([])
            active_joint_index = len(joint_names) - 1
            continue

        if stripped_line == "End Site":
            inside_end_site = True
            continue

        if stripped_line == "{":
            continue

        if stripped_line == "}":
            if inside_end_site:
                inside_end_site = False
            else:
                active_joint_index = joint_parents[active_joint_index]
            continue

        offset_match = re.match(r"OFFSET\s+([-\d\.eE]+)\s+([-\d\.eE]+)\s+([-\d\.eE]+)", stripped_line)
        if offset_match and not inside_end_site:
            joint_offsets[active_joint_index] = [float(value) for value in offset_match.groups()]
            continue

        channel_match = re.match(r"CHANNELS\s+(\d+)\s+(.+)", stripped_line)
        if channel_match and not inside_end_site:
            declared_channel_count = int(channel_match.group(1))
            declared_channel_names = channel_match.group(2).split()
            joint_channels[active_joint_index] = declared_channel_names[:declared_channel_count]
            continue

    if motion_header_index is None:
        raise ValueError(f"Invalid BVH file without MOTION section: {bvh_file}")

    # 第二阶段解析 motion 头信息：总帧数、帧间隔，以及真正的帧数据起始位置。
    frame_count = None
    frame_time = None
    frame_data_start_index = None
    for line_index in range(motion_header_index + 1, len(text_lines)):
        stripped_line = text_lines[line_index].strip()
        frame_match = re.match(r"Frames:\s+(\d+)", stripped_line)
        if frame_match:
            frame_count = int(frame_match.group(1))
            continue

        frame_time_match = re.match(r"Frame Time:\s+([-\d\.eE]+)", stripped_line)
        if frame_time_match:
            frame_time = float(frame_time_match.group(1))
            frame_data_start_index = line_index + 1
            break

    if frame_count is None or frame_time is None or frame_data_start_index is None:
        raise ValueError(f"Invalid BVH motion header in file: {bvh_file}")

    return {
        "bvh_file": str(bvh_file),
        "lines": text_lines,
        "joint_names": joint_names,
        "joint_offsets": np.asarray(joint_offsets, dtype=np.float64),
        "joint_parents": np.asarray(joint_parents, dtype=np.int32),
        "joint_channels": joint_channels,
        "frame_count": frame_count,
        "frame_time": frame_time,
        "frame_data_start_index": frame_data_start_index,
    }


def inspect_bvh_profile(bvh_file: str | Path) -> dict:
    """提取一个 BVH 的结构画像，方便和官方标准数据做比对。"""

    layout = _parse_bvh_layout(bvh_file)

    # 这里记录每个关节自身的旋转顺序；当前项目的 `hit_data` 恰恰就是在这一项上和官方 LAFAN1 不一致。
    rotation_order_by_joint: dict[str, str] = {}
    rotation_order_set: set[str] = set()
    for joint_name, joint_channel_names in zip(layout["joint_names"], layout["joint_channels"]):
        rotation_order = "".join(
            _ROTATION_CHANNEL_TO_AXIS[channel_name]
            for channel_name in joint_channel_names
            if channel_name in _ROTATION_CHANNEL_TO_AXIS
        )
        if rotation_order:
            rotation_order_by_joint[joint_name] = rotation_order
            rotation_order_set.add(rotation_order)

    joint_name_set = set(layout["joint_names"])

    # 这些布尔特征是当前项目 BVH 与官方 LAFAN1 最稳定、最容易自动识别的差异。
    has_toe_base = {"LeftToeBase", "RightToeBase"}.issubset(joint_name_set)
    has_standard_toe = {"LeftToe", "RightToe"}.issubset(joint_name_set)
    has_spine3 = "Spine3" in joint_name_set
    has_neck1 = "Neck1" in joint_name_set
    has_fingers = any(
        finger_keyword in joint_name
        for joint_name in layout["joint_names"]
        for finger_keyword in ["Thumb", "Index", "Middle", "Ring", "Pinky"]
    )

    # 这里给出一个“项目级 profile”判定，用来驱动后续适配层。
    #
    # 注意这里不是在判断“拳击动作”或“某个文件名”，而是在判断骨架长相：
    # 官方这批 BVH 稳定地带 ToeBase、Spine3/Neck1 和手指；LAFAN1 则是更干净的 22 个主体关节。
    # 用骨架特征判定，比靠路径名或文件名前缀更稳，后面如果换一批同结构 BVH 也还能复用。
    if has_toe_base and has_spine3 and has_neck1 and has_fingers:
        detected_profile = "human_robot_hit"
    elif has_standard_toe and not has_toe_base and not has_spine3 and not has_neck1:
        detected_profile = "lafan1_official"
    else:
        detected_profile = "unknown"

    return {
        "bvh_file": str(bvh_file),
        "joint_count": len(layout["joint_names"]),
        "frame_count": layout["frame_count"],
        "frame_time": layout["frame_time"],
        "fps": 1.0 / layout["frame_time"] if layout["frame_time"] > 0 else None,
        "joint_names": layout["joint_names"],
        "rotation_order_by_joint": rotation_order_by_joint,
        "rotation_orders": sorted(rotation_order_set),
        "has_mixed_rotation_orders": len(rotation_order_set) > 1,
        "has_toe_base": has_toe_base,
        "has_standard_toe": has_standard_toe,
        "has_spine3": has_spine3,
        "has_neck1": has_neck1,
        "has_fingers": has_fingers,
        "missing_official_core_joints": sorted(_OFFICIAL_LAFAN1_CORE_JOINTS - joint_name_set),
        "extra_project_joints": sorted(joint_name_set - _OFFICIAL_LAFAN1_CORE_JOINTS),
        "detected_profile": detected_profile,
    }


def build_bvh_comparison_report(reference_bvh_file: str | Path, target_bvh_file: str | Path) -> dict:
    """生成“官方标准 BVH vs 当前项目 BVH”的结构差异报告。"""

    reference_profile = inspect_bvh_profile(reference_bvh_file)
    target_profile = inspect_bvh_profile(target_bvh_file)

    shared_joint_names = sorted(set(reference_profile["joint_names"]) & set(target_profile["joint_names"]))
    rotation_order_differences = []
    for joint_name in shared_joint_names:
        reference_order = reference_profile["rotation_order_by_joint"].get(joint_name)
        target_order = target_profile["rotation_order_by_joint"].get(joint_name)
        if reference_order != target_order:
            rotation_order_differences.append(
                {
                    "joint_name": joint_name,
                    "reference_order": reference_order,
                    "target_order": target_order,
                }
            )

    # 这里把我们这次已经确认的几个核心兼容建议直接写进报告，方便以后复盘或脚本化输出。
    gmr_adapter_hints = []
    if target_profile["has_toe_base"] and not target_profile["has_standard_toe"]:
        gmr_adapter_hints.append("Use LeftToeBase/RightToeBase as LeftToe/RightToe aliases for GMR foot targets.")
    if target_profile["has_spine3"]:
        gmr_adapter_hints.append("Use Spine3 as the adapted upper-torso proxy because official LAFAN1 shoulders branch from Spine2.")
    if target_profile["has_mixed_rotation_orders"]:
        gmr_adapter_hints.append("Parse BVH rotations with per-joint channel orders instead of one global order.")

    return {
        "reference": reference_profile,
        "target": target_profile,
        "shared_joint_names": shared_joint_names,
        "rotation_order_differences": rotation_order_differences,
        "gmr_adapter_hints": gmr_adapter_hints,
    }


def read_bvh_with_joint_orders(bvh_file: str | Path, start: int | None = None, end: int | None = None) -> Anim:
    """按“每个关节自己的 CHANNELS 顺序”解析 BVH。

    这是当前项目适配层里最关键的一步：
    官方 LAFAN1 可以近似看成“所有主体关节都遵循同一旋转顺序”，
    但当前项目 `hit_data` 的肩、手、手指混用了不同顺序。
    如果还按单一顺序解析，得到的局部四元数就会系统性错误，后面的 FK / IK 全都会跑偏。
    """

    layout = _parse_bvh_layout(bvh_file)

    # 把 motion 行整理出来；这里会过滤掉空行，避免行尾换行或多余空白干扰解析。
    frame_lines = [line.strip() for line in layout["lines"][layout["frame_data_start_index"] :] if line.strip()]

    # 这里沿用 vendor 版 `read_bvh` 的接口语义：`start/end` 是原始帧索引范围。
    raw_start_frame = 0 if start is None else start
    raw_end_frame = layout["frame_count"] if end is None else end

    if raw_start_frame < 0 or raw_end_frame > layout["frame_count"] or raw_start_frame >= raw_end_frame:
        raise ValueError(
            f"Invalid frame range [{raw_start_frame}, {raw_end_frame}) for BVH with {layout['frame_count']} frames."
        )

    frame_count_to_load = raw_end_frame - raw_start_frame
    joint_count = len(layout["joint_names"])

    # `positions` 先用静态 offset 作为默认局部平移；没有显式平移通道的关节会保留这个默认值。
    positions = np.repeat(layout["joint_offsets"][np.newaxis, :, :], frame_count_to_load, axis=0)

    # `rotations` 默认全是单位四元数；只有真实含有旋转通道的关节才会被覆盖。
    rotations = np.zeros((frame_count_to_load, joint_count, 4), dtype=np.float64)
    rotations[..., 0] = 1.0

    for loaded_frame_index, raw_frame_index in enumerate(range(raw_start_frame, raw_end_frame)):
        frame_values = np.fromstring(frame_lines[raw_frame_index], sep=" ", dtype=np.float64)
        cursor = 0

        for joint_index, joint_channel_names in enumerate(layout["joint_channels"]):
            joint_channel_count = len(joint_channel_names)
            joint_channel_values = frame_values[cursor : cursor + joint_channel_count]
            cursor += joint_channel_count

            # 先单独收集该关节的欧拉角与真实旋转顺序，再统一转成四元数。
            joint_rotation_values = []
            joint_rotation_order = []

            for channel_name, channel_value in zip(joint_channel_names, joint_channel_values):
                if channel_name in _POSITION_CHANNEL_TO_AXIS:
                    axis_index = _POSITION_CHANNEL_TO_AXIS[channel_name]
                    positions[loaded_frame_index, joint_index, axis_index] = channel_value
                    continue

                if channel_name in _ROTATION_CHANNEL_TO_AXIS:
                    joint_rotation_values.append(channel_value)
                    joint_rotation_order.append(_ROTATION_CHANNEL_TO_AXIS[channel_name])
                    continue

            if joint_rotation_order:
                joint_rotation_euler = np.radians(np.asarray(joint_rotation_values, dtype=np.float64))[np.newaxis, :]
                rotations[loaded_frame_index, joint_index] = utils.euler_to_quat(
                    joint_rotation_euler,
                    order="".join(joint_rotation_order),
                )[0]

    # 保持和 vendor 版一致：沿时间维去除四元数符号跳变。
    # 四元数 q 和 -q 表示同一个旋转，但直接逐帧看会像突然翻面；
    # 不先消掉这个符号跳变，后面的速度、插值、debug 曲线都会很难判断。
    rotations = utils.remove_quat_discontinuities(rotations)

    return Anim(
        quats=rotations,
        pos=positions,
        offsets=layout["joint_offsets"],
        parents=layout["joint_parents"],
        bones=layout["joint_names"],
    )


def estimate_height_from_raw_global_positions(positions_by_body: dict[str, np.ndarray]) -> float | None:
    """Estimate source skeleton height in raw BVH units before meter conversion."""

    if "Head" not in positions_by_body:
        return None

    foot_series = [
        positions_by_body[foot_name][:, 2]
        for foot_name in ["LeftFoot", "RightFoot", "LeftToe", "RightToe", "LeftToeBase", "RightToeBase"]
        if foot_name in positions_by_body
    ]
    if not foot_series:
        return None

    head_z = np.asarray(positions_by_body["Head"][:, 2], dtype=np.float64)
    foot_z = np.min(np.vstack(foot_series), axis=0)
    return float(np.percentile(head_z, 95) - np.percentile(foot_z, 5))


def detect_bvh_unit_divisor(raw_height: float | None, detected_profile: str) -> float:
    """选择 raw BVH 坐标转米时使用的除数。

    这不是说 BVH 文件头里明确写了“单位是英寸”。
    这里做的是工程判定：官方 `human_robot_hit` 的 raw 身高大约 57。
    如果按厘米除以 100，人体只有 0.57m；如果按 inch-style 除以 39.37，
    身高会落在正常人体范围，也更接近官方 40 列 NPY 参考动作的尺度。
    """

    if detected_profile == "human_robot_hit" and raw_height is not None:
        inch_height_m = raw_height / 39.37
        cm_height_m = raw_height / 100.0
        # 只有在“按英寸像正常人、按厘米像小人”这两个条件同时成立时才切到 39.37。
        # 这样可以避免误伤其它本来就是厘米制、但恰好被识别成 unknown/LAFAN1 的 BVH。
        if 1.2 <= inch_height_m <= 2.2 and cm_height_m < 1.0:
            return 39.37

    return 100.0


def detect_bvh_unit_divisor_from_anim(data: Anim, detected_profile: str, rotation_matrix: np.ndarray) -> tuple[float, float | None]:
    """Estimate BVH raw-unit scale from a short FK pass."""

    # 不用整段动作估身高，前 200 帧已经足够判断单位量级；
    # 这样长动作启动时不会因为一个单位检查多耗太多时间。
    sample_end = min(200, data.pos.shape[0])
    global_data = utils.quat_fk(data.quats[:sample_end], data.pos[:sample_end], data.parents)

    positions_by_body = {}
    for joint_index, bone in enumerate(data.bones):
        positions_by_body[bone] = global_data[1][:sample_end, joint_index] @ rotation_matrix.T

    raw_height = estimate_height_from_raw_global_positions(positions_by_body)
    return detect_bvh_unit_divisor(raw_height, detected_profile), raw_height


def adapt_frame_for_gmr(frame_data: dict, detected_profile: str) -> dict:
    """把项目 BVH 的语义对齐到 GMR 默认使用的 LAFAN1 主体骨架。"""

    # 这里复制一份，避免调用方原始字典被原地覆盖，后续如果需要调试原始关节语义会更方便。
    adapted_frame = dict(frame_data)

    if detected_profile == "human_robot_hit":
        # 当前项目骨架使用 `ToeBase`，而 GMR 的 LAFAN1 配置默认读取 `Toe`。
        # 这里不是删掉 ToeBase，也不是重建脚模型，只是多放一个别名，方便旧的 IK 目标名继续工作。
        if "LeftToe" not in adapted_frame and "LeftToeBase" in adapted_frame:
            adapted_frame["LeftToe"] = adapted_frame["LeftToeBase"]
        if "RightToe" not in adapted_frame and "RightToeBase" in adapted_frame:
            adapted_frame["RightToe"] = adapted_frame["RightToeBase"]

        # 官方 LAFAN1 的肩膀从 `Spine2` 分叉；当前项目骨架则是 `Spine3`。
        # GMR 的 BVH IK 配置把“上躯干目标”写成了 `Spine2`，所以这里把项目里的 `Spine3`
        # 映射成 GMR 语义下的 `Spine2`，让 torso / shoulder 参考点更接近官方样例。
        # 这属于语义桥接，不改变原始 `Spine3` 本身；后面如果要做更细的胸/颈跟踪，仍然可以回看原关节。
        if "Spine3" in adapted_frame:
            adapted_frame["Spine2"] = adapted_frame["Spine3"]

    return adapted_frame


def estimate_human_height_from_frames(frames: list[dict]) -> float | None:
    """从一段 BVH 动作里粗略估计人体身高。

    当前项目数据不是标准 LAFAN1，继续硬编码 1.75 容易把比例误差带进 IK。
    这里采用一个稳健一些的估计：
    - 头部高度取 95 分位，尽量接近“站直时”的头顶高度；
    - 足底高度取双脚最低值的 5 分位，尽量接近地面；
    - 两者相减得到近似身高。
    """

    if not frames:
        return None

    if "Head" not in frames[0] or "LeftFootMod" not in frames[0] or "RightFootMod" not in frames[0]:
        return None

    head_height_series = np.asarray([frame["Head"][0][2] for frame in frames], dtype=np.float64)
    foot_height_series = np.asarray(
        [
            min(frame["LeftFootMod"][0][2], frame["RightFootMod"][0][2])
            for frame in frames
        ],
        dtype=np.float64,
    )

    estimated_height = float(np.percentile(head_height_series, 95) - np.percentile(foot_height_series, 5))
    # 这里故意把可接受区间收紧到常见成人身高附近。
    # 当前项目这份 BVH 在直接按轨迹估高时只会得到约 0.55m，
    # 这通常说明它的骨架尺度/单位约定与官方 LAFAN1 不同，
    # 此时继续把这个估计值喂给 GMR 只会把缩放误差进一步放大。
    if estimated_height <= 1.0 or estimated_height >= 2.3:
        return None

    return estimated_height


def save_bvh_comparison_report(reference_bvh_file: str | Path, target_bvh_file: str | Path, output_json_file: str | Path) -> dict:
    """生成并保存 BVH 对比报告，供脚本和人工排查共用。"""

    report = build_bvh_comparison_report(reference_bvh_file, target_bvh_file)
    Path(output_json_file).write_text(
        json.dumps(report, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return report
