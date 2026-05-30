# T800 Assets For GMR

这里存放 `GMR` 专用的 `T800` 资产。

## 说明

- `mujoco/t800_full_gmr.xml`
  是当前默认给 `GMR` retargeting / auto-ik 流程使用的 `T800` floating-base 模型。
- `mujoco/t800_from_urdf.xml`
  是从训练侧 URDF 工具转换后补齐 GMR 基本结构的基线版本。
- `mujoco/t800_gmr.xml`
  是更早期的手工整理轻量版模型，仍可用于对照和回退。

## 当前默认模型

- `general_motion_retargeting/params.py` 中，`t800` 目前默认注册到 `assets/t800/mujoco/t800_full_gmr.xml`。
- 这份默认模型的目标是：
  - 满足 `GMR` 对 `qpos = root(7) + joints` 的结构假设。
  - 保持与训练侧 `T800` 一致的主链 body / joint 语义。
  - 尽量保留训练侧 URDF 的质量、惯量、limit 与执行器力限制信息。
  - 使用参考 `unitree_rl_lab/deploy/robots/t800_sim2sim/assets/resource/robot/t800` 中的 `OBJ + PNG` 外观资源，并保留适合 GMR 的 collision proxy / actuator 结构。

## URDF / Mesh 对应关系

- 运动学主链来自 `whole_body_tracking_engineai/source/whole_body_tracking/whole_body_tracking/assets/t800/urdf/serial_t800.urdf`。
- 当前仓库保留了从训练侧 `.dae` 批量导出的 `assets/t800/meshes/*.stl` 作为几何对照，同时新增参考外观用的 `assets/t800/meshes/*.obj` 与 `assets/t800/texture/*.png`。
- `t800_full_gmr.xml` 和 `t800_full_gmr_transparent.xml` 当前默认使用参考侧 `OBJ + PNG` 贴图外观；`scripts/apply_t800_reference_visual_mjcf.py` 只替换 visual mesh / material / texture，不改 joint、inertial、collision 或 actuator。
- `assets/t800/meshes/colored/*.stl` 与 `scripts/build_t800_colored_mjcf.py` 仍保留为旧的 DAE material 拆色路线。训练侧 `.dae` 中部分左右镜像 link 使用 determinant 为负的 scene transform；如重新生成 colored STL，脚本仍会检测该镜像 transform 并翻转三角面顺序，避免局部法线异常。
- `t800_from_urdf.xml`：以工具从 URDF 导出的基线 MJCF 为底稿，再补上 `LINK_BASE/freejoint`、visual mesh、末端 body 和 actuator。
- `t800_full_gmr.xml`：在 `t800_from_urdf.xml` 上进一步精确对齐了 URDF 的质量/惯量，并给 actuator 补上了基于 URDF effort 的 `ctrlrange/forcerange`；碰撞体已明确拆成两层：`collision_urdf`（group 3，对应训练侧 URDF 原生碰撞）和 `collision_fallback`（group 4，对应无原始 collision link 的补体）。
- `t800_gmr.xml`：保留了相同 body / joint 命名，但更偏轻量化和历史兼容用途。

## 边界

- 这里的模型不是训练时的动力学资产。
- 训练仍然应使用 `whole_body_tracking_engineai` 仓库中的原始 `T800` 资产。
- `GMR` 这些模型主要用于：
  - BVH 到 T800 的重定向求解
  - auto-ik 参数生成与 FK 对齐
  - 可视化调试
  - 导出中间动作结果
- 当前仍然没有从 URDF 直接继承的真实传动/摩擦参数；`joint damping/armature` 仍是 GMR 侧的经验值。

