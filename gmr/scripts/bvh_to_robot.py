import argparse
import pathlib
import time
import sys

SCRIPT_DIR = pathlib.Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting import RobotMotionViewer
from general_motion_retargeting.motion_contact_postprocess import apply_contact_aware_postprocess, build_contact_aware_config
from general_motion_retargeting.motion_grounding import (
    GROUNDING_MODES,
    align_motion_root_to_ground,
    save_grounding_diagnostics_plot,
)
from general_motion_retargeting.motion_retarget_options import (
    calibrate_human_robot_hit_frames,
    resolve_actual_human_height,
    resolve_ik_safety_break,
    resolve_max_iter,
)
from general_motion_retargeting.utils.lafan1 import load_bvh_file
from rich import print
from tqdm import tqdm
import os
import numpy as np


def slice_motion_frames(motion_frames, frame_start=0, frame_end=None, frame_step=1):
    frame_count = len(motion_frames)
    frame_start = int(frame_start)
    frame_step = int(frame_step)
    frame_end = frame_count if frame_end is None else int(frame_end)

    if frame_step <= 0:
        raise ValueError("--frame_step must be greater than 0.")
    if frame_start < 0:
        raise ValueError("--frame_start must be greater than or equal to 0.")
    if frame_end > frame_count:
        raise ValueError(
            f"--frame_end ({frame_end}) exceeds loaded frame count ({frame_count})."
        )
    if frame_start >= frame_end:
        raise ValueError(
            f"--frame_start ({frame_start}) must be smaller than --frame_end ({frame_end})."
        )

    return motion_frames[frame_start:frame_end:frame_step]


def maybe_step_viewer(viewer, qpos, human_motion_data, rate_limit):
    if viewer is None:
        return
    viewer.step(
        root_pos=qpos[:3],
        root_rot=qpos[3:7],
        dof_pos=qpos[7:],
        human_motion_data=human_motion_data,
        rate_limit=rate_limit,
        follow_camera=True,
    )


def build_motion_data_from_qpos_list(qpos_list, motion_fps):
    root_pos = np.array([qpos[:3] for qpos in qpos_list])
    # save from wxyz to xyzw
    root_rot = np.array([qpos[3:7][[1, 2, 3, 0]] for qpos in qpos_list])
    dof_pos = np.array([qpos[7:] for qpos in qpos_list])
    if len(qpos_list) >= 2 and motion_fps > 0:
        dt = 1.0 / float(motion_fps)
        root_lin_vel = np.gradient(root_pos, dt, axis=0)
        dof_vel = np.gradient(dof_pos, dt, axis=0)
    else:
        root_lin_vel = np.zeros_like(root_pos)
        dof_vel = np.zeros_like(dof_pos)
    root_ang_vel = np.zeros_like(root_pos)
    return {
        "fps": motion_fps,
        "root_pos": root_pos,
        "root_rot": root_rot,
        "dof_pos": dof_pos,
        "dof_vel": dof_vel,
        "root_lin_vel": root_lin_vel,
        "root_ang_vel": root_ang_vel,
        "local_body_pos": None,
        "link_body_list": None,
    }


def estimate_ground_offset(retargeter: GMR, motion_frames):
    """Estimate a source-side global z offset from the human motion itself.

    这是“重定向前”的 auto ground：
    - 观测的是源人体关键点最低点；
    - 调整的是输入 human motion 的整体高度。

    它和后面的 `foot_ground_align` 不是一回事：
    - auto_ground: 先把源动作大致摆正；
    - foot_ground_align: 重定向结束后，再按机器人真实支撑碰撞体做一次校准。
    """
    lowest_z = np.inf
    for human_data in motion_frames:
        human_data = retargeter.to_numpy(human_data)
        human_data = retargeter.scale_human_data(
            human_data,
            retargeter.human_root_name,
            retargeter.human_scale_table,
        )
        human_data = retargeter.offset_human_data(
            human_data,
            retargeter.pos_offsets1,
            retargeter.rot_offsets1,
        )
        for pos, _ in human_data.values():
            if pos[2] < lowest_z:
                lowest_z = pos[2]

    return float(lowest_z)


def build_retargeter(
    *,
    source_profile: str,
    robot: str,
    actual_human_height: float | None,
    debug_log_path: str | None,
    debug_log_every_n: int,
    disable_ik_safety_break: bool,
    max_iter: int | None,
):
    return GMR(
        src_human=f"bvh_{source_profile}",
        tgt_robot=robot,
        actual_human_height=resolve_actual_human_height(actual_human_height, source_profile),
        debug_log_path=debug_log_path,
        debug_log_every_n=debug_log_every_n,
        ik_safety_break=resolve_ik_safety_break(disable_ik_safety_break),
        max_iter=resolve_max_iter(max_iter),
    )

if __name__ == "__main__":
    
    HERE = pathlib.Path(__file__).parent

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--bvh_file",
        help="BVH motion file to load.",
        required=True,
        type=str,
    )
    
    parser.add_argument(
        "--format",
        choices=["lafan1", "nokov"],
        default="lafan1",
    )

    parser.add_argument(
        "--source_profile",
        choices=["auto", "lafan1", "human_robot_hit", "nokov"],
        default="auto",
        help=(
            "Explicit GMR BVH source profile. Use human_robot_hit for official competition BVH files; "
            "--format lafan1 only means the loader emits LAFAN1-style body keys."
        ),
    )
    
    parser.add_argument(
        "--loop",
        default=False,
        action="store_true",
        help="Loop the motion.",
    )
    
    parser.add_argument(
        "--robot",
        # 这里把 T800 也接入到 BVH retarget 入口。
        # 这样在命令行里就可以直接使用 `--robot t800` 或 `--robot t800_transparent`
        # 走完整的 GMR BVH 重定向流程。
        choices=["unitree_g1", "unitree_g1_with_hands", "booster_t1", "stanford_toddy", "fourier_n1", "engineai_pm01", "pal_talos", "t800", "t800_transparent", "t800_transparent_manual", "t800_transparent_upperbody_core_candidate"],
        default="unitree_g1",
    )
    
    
    parser.add_argument(
        "--record_video",
        action="store_true",
        default=False,
    )

    parser.add_argument(
        "--headless",
        action="store_true",
        default=False,
        help="Run retargeting without launching the interactive MuJoCo viewer.",
    )

    parser.add_argument(
        "--disable_ik_safety_break",
        action="store_true",
        default=False,
        help=(
            "Let Mink's configuration-limit constraint handle temporary limit pressure "
            "instead of raising before solve_ik. Useful for complete high-energy official BVH exports."
        ),
    )

    parser.add_argument(
        "--video_path",
        type=str,
        default="videos/example.mp4",
    )

    parser.add_argument(
        "--rate_limit",
        action="store_true",
        default=False,
    )

    parser.add_argument(
        "--save_path",
        default=None,
        help="Path to save the robot motion.",
    )
    
    parser.add_argument(
        "--motion_fps",
        default=30,
        type=int,
    )

    parser.add_argument(
        "--auto_ground",
        action="store_true",
        default=False,
        help="Automatically offset source motion to ground before retargeting.",
    )

    parser.add_argument(
        "--auto_ground_margin",
        type=float,
        default=0.0,
        help="Target clearance above ground (meters) when --auto_ground is enabled.",
    )

    parser.add_argument(
        "--debug_log_path",
        type=str,
        default=None,
        help="可选：把每帧 IK 数值误差与 target/current frame 信息写成 jsonl 调试日志。",
    )

    parser.add_argument(
        "--debug_log_every_n",
        type=int,
        default=1,
        help="调试日志采样间隔。默认每帧都记录；设为 10 表示每 10 帧记录一次。",
    )

    parser.add_argument(
        "--max_iter",
        type=int,
        default=None,
        help=(
            "Optional IK iteration cap per stage. Defaults to the current GMR solver limit of 10; "
            "higher values are for A/B diagnosis and are not automatically better."
        ),
    )

    parser.add_argument(
        "--frame_start",
        type=int,
        default=0,
        help="Start frame index for diagnostic retargeting windows.",
    )

    parser.add_argument(
        "--frame_end",
        type=int,
        default=None,
        help="Exclusive end frame index for diagnostic retargeting windows.",
    )

    parser.add_argument(
        "--frame_step",
        type=int,
        default=1,
        help="Frame stride for diagnostic retargeting windows or explicit downsampling.",
    )

    parser.add_argument(
        "--foot-ground-align",
        dest="foot_ground_align",
        action="store_true",
        help="Ground the saved PKL using actual robot support geoms.",
    )
    parser.add_argument(
        "--foot-ground-mode",
        choices=list(GROUNDING_MODES),
        default="per_frame",
        help="Vertical grounding strategy applied to saved motion.",
    )
    parser.add_argument(
        "--foot-ground-clearance",
        type=float,
        default=0.002,
        help="Target minimum support clearance above ground in meters for saved motion.",
    )
    parser.add_argument(
        "--foot-ground-smooth-window",
        type=int,
        default=9,
        help="Moving-average window used by --foot-ground-mode smooth_per_frame.",
    )
    parser.add_argument(
        "--foot-ground-smooth-contact-threshold",
        type=float,
        default=0.04,
        help="Only smooth support-height candidates below this threshold in smooth_per_frame mode.",
    )
    parser.add_argument(
        "--foot-ground-max-shift-step",
        type=float,
        default=None,
        help="Maximum adjacent-frame root-z correction step in meters for --foot-ground-mode contact_lowfreq.",
    )
    parser.add_argument(
        "--foot-ground-plot-path",
        default=None,
        help="Optional PNG path for before/after root_z, support_min_z, and applied_shift curves.",
    )
    parser.add_argument(
        "--contact-aware-postprocess",
        action="store_true",
        default=False,
        help="Apply stance detection, stance-foot XY lock, support-geom grounding, and root_z smoothing to the saved PKL.",
    )
    parser.add_argument(
        "--contact-profile",
        choices=["conservative", "balanced", "aggressive"],
        default="balanced",
        # 这里把日常使用入口收敛成一个 profile。
        # 绝大多数情况下先选档位即可，不需要直接面对一串阈值。
        help="Preset strength for contact-aware postprocess. Expert flags below can still override individual thresholds.",
    )
    parser.add_argument(
        "--contact-stance-height-threshold",
        type=float,
        default=None,
        help="Expert override: frames with support min_z below this threshold can be treated as stance candidates.",
    )
    parser.add_argument(
        "--contact-stance-speed-threshold",
        type=float,
        default=None,
        help="Expert override: maximum planar foot speed (m/s) for stance detection.",
    )
    parser.add_argument(
        "--contact-stance-min-frames",
        type=int,
        default=None,
        help="Expert override: minimum segment length to keep a stance phase.",
    )
    parser.add_argument(
        "--contact-ground-mode",
        choices=list(GROUNDING_MODES),
        default=None,
        help="Expert override: grounding mode used inside the contact-aware postprocess.",
    )
    parser.add_argument(
        "--contact-ground-clearance",
        type=float,
        default=None,
        help="Expert override: target minimum support clearance above ground in meters inside the contact-aware postprocess.",
    )
    parser.add_argument(
        "--contact-root-z-smoothing-window",
        type=int,
        default=None,
        help="Expert override: moving-average window used to smooth root_z correction inside the contact-aware postprocess.",
    )
    
    args = parser.parse_args()
    
    if args.save_path is not None:
        save_dir = os.path.dirname(args.save_path)
        if save_dir:  # Only create directory if it's not empty
            os.makedirs(save_dir, exist_ok=True)
        qpos_list = []

    
    # Load SMPLX trajectory
    lafan1_data_frames, actual_human_height = load_bvh_file(args.bvh_file, format=args.format)
    original_frame_count = len(lafan1_data_frames)
    lafan1_data_frames = slice_motion_frames(
        lafan1_data_frames,
        frame_start=args.frame_start,
        frame_end=args.frame_end,
        frame_step=args.frame_step,
    )
    print(
        "[frame_window] "
        f"loaded={original_frame_count}, "
        f"selected={len(lafan1_data_frames)}, "
        f"start={args.frame_start}, "
        f"end={original_frame_count if args.frame_end is None else args.frame_end}, "
        f"step={args.frame_step}"
    )
    
    
    # Initialize the retargeting system
    if args.source_profile == "auto":
        source_profile = args.format
    else:
        source_profile = args.source_profile
    if source_profile == "human_robot_hit":
        lafan1_data_frames = calibrate_human_robot_hit_frames(lafan1_data_frames)

    retargeter = build_retargeter(
        source_profile=source_profile,
        robot=args.robot,
        actual_human_height=actual_human_height,
        debug_log_path=args.debug_log_path,
        debug_log_every_n=args.debug_log_every_n,
        disable_ik_safety_break=args.disable_ik_safety_break,
        max_iter=args.max_iter,
    )

    if args.auto_ground:
        estimated_lowest_z = estimate_ground_offset(retargeter, lafan1_data_frames)
        if not np.isfinite(estimated_lowest_z):
            raise RuntimeError("Failed to estimate ground offset from BVH frames.")
        applied_ground_offset = estimated_lowest_z - args.auto_ground_margin
        retargeter.set_ground_offset(applied_ground_offset)
        print(
            "[auto_ground] "
            f"min_z={estimated_lowest_z:.6f}, "
            f"margin={args.auto_ground_margin:.6f}, "
            f"applied_ground_offset={applied_ground_offset:.6f}"
        )

    motion_fps = args.motion_fps
    
    robot_motion_viewer = None
    if not args.headless:
        robot_motion_viewer = RobotMotionViewer(robot_type=args.robot,
                                                motion_fps=motion_fps,
                                                transparent_robot=0,
                                                record_video=args.record_video,
                                                video_path=args.video_path,
                                                # video_width=2080,
                                                # video_height=1170
                                                )
    
    # FPS measurement variables
    fps_counter = 0
    fps_start_time = time.time()
    fps_display_interval = 2.0  # Display FPS every 2 seconds
    
    print(f"mocap_frame_rate: {motion_fps}")
    
    # Create tqdm progress bar for the total number of frames
    pbar = tqdm(total=len(lafan1_data_frames), desc="Retargeting")
    
    # Start the viewer
    i = 0
    


    while True:
        
        # FPS measurement
        fps_counter += 1
        current_time = time.time()
        if current_time - fps_start_time >= fps_display_interval:
            actual_fps = fps_counter / (current_time - fps_start_time)
            print(f"Actual rendering FPS: {actual_fps:.2f}")
            fps_counter = 0
            fps_start_time = current_time
            
        # Update progress bar
        pbar.update(1)

        # Update task targets.
        smplx_data = lafan1_data_frames[i]

        # retarget
        qpos = retargeter.retarget(smplx_data, frame_index=i)
        if args.save_path is not None:
            qpos_list.append(qpos)
        

        # visualize unless running headless diagnostics/export.
        maybe_step_viewer(
            viewer=robot_motion_viewer,
            qpos=qpos,
            human_motion_data=retargeter.scaled_human_data,
            rate_limit=args.rate_limit,
        )

        if args.loop:
            i = (i + 1) % len(lafan1_data_frames)
        else:
            i += 1
            if i >= len(lafan1_data_frames):
                break
   
        
    if args.save_path is not None:
        import pickle
        motion_data = build_motion_data_from_qpos_list(qpos_list, motion_fps)
        if args.contact_aware_postprocess:
            # 新的推荐路径：
            # 先通过 profile 生成一组稳定的默认参数，再允许用户用 expert flags 覆盖单项阈值。
            motion_data, contact_stats = apply_contact_aware_postprocess(
                motion_data=motion_data,
                model_or_path=retargeter.xml_file,
                config=build_contact_aware_config(
                    profile=args.contact_profile,
                    stance_height_threshold=args.contact_stance_height_threshold,
                    stance_speed_threshold=args.contact_stance_speed_threshold,
                    stance_min_frames=args.contact_stance_min_frames,
                    ground_clearance=args.contact_ground_clearance,
                    ground_mode=args.contact_ground_mode,
                    root_z_smoothing_window=args.contact_root_z_smoothing_window,
                ),
                inplace=False,
            )
            print("[contact_aware_postprocess]", contact_stats)
        elif args.foot_ground_align:
            # 兼容旧路径：只做 grounding，不做 stance 检测 / 锁脚 / root_z 平滑。
            # 注意：这里只在“保存 pkl”时做后处理，不会影响前面的 viewer 回放。
            # 这样方便你先看原始 retarget 效果，再决定是否对训练数据做 grounding 修复。
            motion_data, grounding_stats = align_motion_root_to_ground(
                motion_data=motion_data,
                model_or_path=retargeter.xml_file,
                clearance=args.foot_ground_clearance,
                mode=args.foot_ground_mode,
                inplace=False,
                smooth_window=args.foot_ground_smooth_window,
                smooth_contact_threshold=args.foot_ground_smooth_contact_threshold,
                max_shift_step=args.foot_ground_max_shift_step,
                return_diagnostics=args.foot_ground_plot_path is not None,
            )
            diagnostics = grounding_stats.pop("diagnostics", None)
            if args.foot_ground_plot_path is not None:
                if diagnostics is None:
                    raise RuntimeError("Grounding diagnostics were not returned; cannot write --foot-ground-plot-path.")
                save_grounding_diagnostics_plot(
                    plot_path=args.foot_ground_plot_path,
                    diagnostics=diagnostics,
                    title=f"{pathlib.Path(args.save_path).name} -> {args.foot_ground_mode}",
                )
                print(f"[foot_ground_align] saved plot to {args.foot_ground_plot_path}")
            print("[foot_ground_align]", grounding_stats)
        with open(args.save_path, "wb") as f:
            pickle.dump(motion_data, f)
        print(f"Saved to {args.save_path}")

    # Close progress bar
    pbar.close()
    
    if robot_motion_viewer is not None:
        robot_motion_viewer.close()
       
