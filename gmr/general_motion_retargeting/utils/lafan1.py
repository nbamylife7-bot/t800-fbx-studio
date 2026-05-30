import numpy as np
from scipy.spatial.transform import Rotation as R

import general_motion_retargeting.utils.lafan_vendor.utils as utils
from general_motion_retargeting.utils.lafan_vendor.extract import read_bvh
from general_motion_retargeting.utils.bvh_profile_adapter import (
    adapt_frame_for_gmr,
    detect_bvh_unit_divisor_from_anim,
    estimate_human_height_from_frames,
    inspect_bvh_profile,
    read_bvh_with_joint_orders,
)


def load_bvh_file(bvh_file, format="lafan1"):
    """
    Must return a dictionary with the following structure:
    {
        "Hips": (position, orientation),
        "Spine": (position, orientation),
        ...
    }
    """
    # 先检查 BVH 结构画像。
    # 这样做有两个目的：
    # 1. 识别当前输入是否是官方标准 LAFAN1，还是当前项目 `hit_data` 这种扩展骨架；
    # 2. 判断是否需要启用“按关节真实 CHANNELS 顺序”解析的适配层。
    bvh_profile = inspect_bvh_profile(bvh_file)

    # 官方 LAFAN1 大多可以继续沿用 vendor 版读取器；
    # 但当前项目 `hit_data` 存在混合旋转顺序，如果仍然使用单一全局顺序解析，局部四元数会系统性错误。
    if bvh_profile["has_mixed_rotation_orders"] or bvh_profile["detected_profile"] == "human_robot_hit":
        data = read_bvh_with_joint_orders(bvh_file)
    else:
        data = read_bvh(bvh_file)
    global_data = utils.quat_fk(data.quats, data.pos, data.parents)

    # GMR 里这一路 BVH loader 最终希望把源动作放到项目约定的世界坐标里。
    # 这里的矩阵是沿用原 GMR/LAFAN1 入口的坐标换基；后面 official BVH 的修复主要在
    # “真实 CHANNELS 顺序、单位缩放、骨架语义别名、IK config”上做，而不是改这个全局换基。
    rotation_matrix = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    rotation_quat = R.from_matrix(rotation_matrix).as_quat(scalar_first=True)
    # 先用一小段 FK 估 raw 身高，再决定 raw 坐标除以多少变成米。
    # LAFAN1 大多按厘米理解；官方 human_robot_hit 这批 raw 身高约 57，
    # 按 /100 会变成 0.57m，明显不合理，所以会被判到 /39.37 的 inch-style 路线。
    unit_divisor, raw_height = detect_bvh_unit_divisor_from_anim(
        data,
        bvh_profile["detected_profile"],
        rotation_matrix,
    )
    if bvh_profile["detected_profile"] == "human_robot_hit":
        print(
            "[BVH adapter] "
            f"profile=human_robot_hit raw_height={raw_height:.3f} "
            f"unit_divisor={unit_divisor:.2f}"
        )

    frames = []
    for frame in range(data.pos.shape[0]):
        result = {}
        for i, bone in enumerate(data.bones):
            orientation = utils.quat_mul(rotation_quat, global_data[0][frame, i])
            # 到这里才真正把 BVH raw 坐标变成米。这个除数如果错了，
            # 后面 IK 看起来就会像“人体比例不对”，再怎么调权重也只是补偿尺度错误。
            position = global_data[1][frame, i] @ rotation_matrix.T / unit_divisor
            result[bone] = [position, orientation]

        # 这里把“项目 BVH 的骨架语义”适配成 GMR 更熟悉的 LAFAN1 主体骨架语义。
        # 当前主要做两类最小改动：
        # 1. `LeftToeBase/RightToeBase` -> `LeftToe/RightToe`；
        # 2. 用 `Spine3` 覆盖 `Spine2`，让 torso 目标更接近官方 LAFAN1 的上躯干含义。
        result = adapt_frame_for_gmr(result, bvh_profile["detected_profile"])

        if format == "lafan1":
            # `format=lafan1` 在当前脚本里表示“输出给 GMR 的 LAFAN1 风格主体字段”，
            # 不是说输入文件一定是标准 LAFAN1。official BVH 仍然要配合 source_profile=human_robot_hit。
            left_toe_name = "LeftToe" if "LeftToe" in result else "LeftToeBase"
            right_toe_name = "RightToe" if "RightToe" in result else "RightToeBase"
            if left_toe_name not in result or right_toe_name not in result:
                missing = [name for name in [left_toe_name, right_toe_name] if name not in result]
                raise KeyError(
                    f"Missing toe joints for BVH foot alignment: {missing}. "
                    "Expected LeftToe/RightToe or LeftToeBase/RightToeBase."
                )
            result["LeftFootMod"] = [result["LeftFoot"][0], result[left_toe_name][1]]
            result["RightFootMod"] = [result["RightFoot"][0], result[right_toe_name][1]]
        elif format == "nokov":
            result["LeftFootMod"] = [result["LeftFoot"][0], result["LeftToeBase"][1]]
            result["RightFootMod"] = [result["RightFoot"][0], result["RightToeBase"][1]]
        else:
            raise ValueError(f"Invalid format: {format}")
            
        frames.append(result)

    # 对官方标准 LAFAN1 继续保持原来的保守默认值，避免无关动作的缩放行为发生明显漂移；
    # 对当前项目 `hit_data` 这类扩展骨架，则优先尝试从动作本身估计身高，减少比例误差。
    human_height = 1.75  # cm to m
    if bvh_profile["detected_profile"] == "human_robot_hit":
        estimated_human_height = estimate_human_height_from_frames(frames)
        if estimated_human_height is not None:
            human_height = estimated_human_height

    return frames, human_height


