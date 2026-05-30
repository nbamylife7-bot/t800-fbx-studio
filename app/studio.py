"""T800 web studio — viser 3D in the browser (no desktop MuJoCo viewer).

Run:  web-version/run.sh
      → http://localhost:8080

Requires GMR backend on PYTHONPATH (see README.md).
"""

from __future__ import annotations

import os
import pathlib
import pickle
import sys
import threading
import time
from typing import List, Literal, Optional

import numpy as np

from app import motion_io as mio
from app.paths import DEMO_DIR, OUT_DIR, UPLOAD_DIR, bootstrap

WEB_ROOT, REPO_ROOT, GMR_ROOT = bootstrap()

import mujoco as mj  # noqa: E402
import viser  # noqa: E402

import scripts.bvh_to_robot as bvr  # noqa: E402
import scripts.fbx_to_robot as fr  # noqa: E402
import scripts.t800_foot_postprocess as foot_pp  # noqa: E402
import scripts.t800_viser_robot as tvr  # noqa: E402
from general_motion_retargeting import GeneralMotionRetargeting as GMR  # noqa: E402
from general_motion_retargeting.motion_retarget_options import calibrate_human_robot_hit_frames  # noqa: E402
from general_motion_retargeting.params import ROBOT_XML_DICT  # noqa: E402
from general_motion_retargeting.utils.lafan1 import load_bvh_file  # noqa: E402

ROBOT = "t800"
SkinMode = tvr.SkinMode
DEMO_CLIPS = ("Boxing", "Martelo_2", "Flair")


def _safe_stem(path: str) -> str:
    return pathlib.Path(path).stem.replace(" ", "_")


def _output_path(source_path: str, custom_name: str) -> pathlib.Path:
    name = (custom_name or "").strip()
    if name:
        if not name.endswith(".pkl"):
            name = f"{name}.pkl"
        return OUT_DIR / name
    return OUT_DIR / f"{_safe_stem(source_path)}_t800.pkl"


def _resolve_fbx_src_human(profile: str, frames: list) -> str:
    if profile == "auto":
        return fr._detect_src_human(frames)
    if profile == "human_robot_hit":
        return "bvh_human_robot_hit"
    return "bvh_lafan1"


def _resolve_bvh_source_profile(profile: str, bvh_format: str) -> str:
    if profile == "auto":
        return bvh_format
    return profile


def convert_fbx(
    fbx_path: str,
    *,
    fps: int,
    src_human: str,
    human_height: float,
    root_joint: str,
    retarget_mode: str,
    stabilize_pelvis: bool,
    flatten_feet: bool,
    output_name: str,
    status=lambda s: None,
) -> tuple[list[np.ndarray], str]:
    status(f"Loading FBX {os.path.basename(fbx_path)} …")
    frames = fr._load_fbx_as_human_frames(fbx_path, fps=int(fps), root_joint=root_joint)
    if not frames:
        raise RuntimeError("No frames parsed from FBX.")

    src_profile = _resolve_fbx_src_human(src_human, frames)
    status(f"Parsed {len(frames)} frames ({src_profile}). Retargeting …")

    height = float(human_height) if human_height > 0 else 1.75
    retargeter = GMR(
        src_human=src_profile,
        tgt_robot=ROBOT,
        actual_human_height=height,
        ik_safety_break=False,
        verbose=False,
    )

    use_position = retarget_mode == "position"
    if use_position:
        fr._apply_position_task_mode(retargeter, fr.T800_FBX_POSITION_WEIGHTS)
        if stabilize_pelvis:
            if flatten_feet:
                foot_pp.adjust_human_foot_sole_targets(frames)
            fr._apply_clean_orientation_targets(
                retargeter,
                frames,
                include_pelvis=True,
                include_feet=flatten_feet,
            )

    ground = fr._estimate_ground_offset(retargeter, frames)
    retargeter.set_ground_offset(ground)

    qpos_frames: list[np.ndarray] = []
    for i, f in enumerate(frames):
        qpos = retargeter.retarget(f, frame_index=i)
        if flatten_feet and use_position:
            qpos = foot_pp.postprocess_robot_qpos_feet(retargeter.model, qpos, flatten=True)
        qpos_frames.append(qpos.copy())

    save_path = _output_path(fbx_path, output_name)
    motion = fr._build_motion_data_from_qpos_list(qpos_frames, int(fps))
    with open(save_path, "wb") as fh:
        pickle.dump(motion, fh)
    status(f"Saved {save_path.name} ({len(qpos_frames)} frames).")
    return qpos_frames, str(save_path)


def convert_bvh(
    bvh_path: str,
    *,
    fps: int,
    bvh_format: str,
    source_profile: str,
    human_height: float,
    auto_ground: bool,
    flatten_feet: bool,
    frame_start: int,
    frame_end: int,
    frame_step: int,
    output_name: str,
    status=lambda s: None,
) -> tuple[list[np.ndarray], str]:
    status(f"Loading BVH {os.path.basename(bvh_path)} …")
    frames, detected_height = load_bvh_file(bvh_path, format=bvh_format)
    end = None if frame_end <= 0 else int(frame_end)
    frames = bvr.slice_motion_frames(
        frames,
        frame_start=int(frame_start),
        frame_end=end,
        frame_step=max(1, int(frame_step)),
    )
    if not frames:
        raise RuntimeError("No frames selected from BVH.")

    profile = _resolve_bvh_source_profile(source_profile, bvh_format)
    if profile == "human_robot_hit":
        frames = calibrate_human_robot_hit_frames(frames)

    height = float(human_height) if human_height > 0 else float(detected_height)
    status(f"Selected {len(frames)} frames ({profile}). Retargeting …")

    retargeter = bvr.build_retargeter(
        source_profile=profile,
        robot=ROBOT,
        actual_human_height=height,
        debug_log_path=None,
        debug_log_every_n=1,
        disable_ik_safety_break=True,
        max_iter=None,
    )

    if auto_ground:
        estimated = bvr.estimate_ground_offset(retargeter, frames)
        retargeter.set_ground_offset(estimated)

    qpos_frames: list[np.ndarray] = []
    for i, frame in enumerate(frames):
        qpos = retargeter.retarget(frame, frame_index=i)
        if flatten_feet:
            qpos = foot_pp.postprocess_robot_qpos_feet(retargeter.model, qpos, flatten=True)
        qpos_frames.append(qpos.copy())

    save_path = _output_path(bvh_path, output_name)
    motion = bvr.build_motion_data_from_qpos_list(qpos_frames, int(fps))
    with open(save_path, "wb") as fh:
        pickle.dump(motion, fh)
    status(f"Saved {save_path.name} ({len(qpos_frames)} frames).")
    return qpos_frames, str(save_path)


class WebStudio:
    def __init__(self) -> None:
        host = os.environ.get("T800_WEB_HOST", os.environ.get("FBX_STUDIO_HOST", "0.0.0.0"))
        port = int(os.environ.get("T800_WEB_PORT", os.environ.get("FBX_STUDIO_PORT", "8080")))
        self.server = viser.ViserServer(host=host, port=port)
        self.server.scene.set_up_direction("+z")
        self._grid = self.server.scene.add_grid("/ground", width=4.0, height=4.0, plane="xy")
        self._foot_model = mj.MjModel.from_xml_path(str(ROBOT_XML_DICT[ROBOT]))
        self.robot = tvr.RobotScene(
            self.server,
            self._foot_model,
            skin="white",
            initial_qpos=tvr.t800_standing_qpos(self._foot_model),
        )

        self.qpos_frames: List[np.ndarray] = []
        self.fps: int = 30
        self.frame_idx: int = 0
        self.playing: bool = False
        self.loop_playback: bool = True
        self.last_pkl: Optional[str] = None
        self._lock = threading.Lock()
        self._process_lock = threading.Lock()
        self._last_processed_path: Optional[str] = None

        self._build_gui()

    def _build_gui(self) -> None:
        s = self.server.gui
        s.add_markdown("## T800 Web Studio")
        s.add_markdown("Browser 3D via **viser** — no desktop MuJoCo window.")
        self.status = s.add_text("Status", initial_value="Ready.", disabled=True)

        with s.add_folder("1. Source"):
            s.add_markdown(
                "Upload FBX/BVH/PKL or pick a demo — convert/load runs automatically. "
                "Progress in **Status**."
            )
            self.source_kind = s.add_dropdown(
                "Input type",
                options=["fbx", "bvh", "pkl"],
                initial_value="fbx",
            )
            self.path_box = s.add_text("File path", initial_value="")
            self.upload_btn = s.add_upload_button("Upload file", mime_type=".fbx,.bvh,.pkl")

            @self.upload_btn.on_upload
            def _(_) -> None:
                f = self.upload_btn.value
                if f is None:
                    return
                dest = UPLOAD_DIR / f.name
                with open(dest, "wb") as fh:
                    fh.write(f.content)
                ext = dest.suffix.lower().lstrip(".")
                if ext in ("fbx", "bvh", "pkl"):
                    self.source_kind.value = ext
                self.path_box.value = str(dest)
                self._set_status(f"Uploaded {f.name} — processing…")
                self._schedule_auto_process(force=True)

            @self.path_box.on_update
            def _(_) -> None:
                if self._resolve_path(self.path_box.value or ""):
                    self._schedule_auto_process()

            @self.source_kind.on_update
            def _(_) -> None:
                if self._resolve_path(self.path_box.value or ""):
                    self._schedule_auto_process(force=True)

            self.feet_on_load_box = s.add_checkbox(
                "Apply flat-feet fix when loading PKL",
                initial_value=True,
            )

            self.demo_box = s.add_dropdown(
                "Demo clip",
                options=["(none)", *DEMO_CLIPS],
                initial_value="(none)",
            )
            demo_btn = s.add_button("Load demo clip")

            @demo_btn.on_click
            def _(_) -> None:
                name = self.demo_box.value
                if not name or name == "(none)":
                    self._set_status("Pick a demo clip first.")
                    return
                pkl = DEMO_DIR / f"{name}_t800.pkl"
                if not pkl.is_file():
                    pkl = OUT_DIR / f"{name}_t800.pkl"
                if not pkl.is_file():
                    self._set_status(f"Demo not found: {name}_t800.pkl")
                    return
                self.source_kind.value = "pkl"
                self.path_box.value = str(pkl)
                self._schedule_auto_process(force=True)

        with s.add_folder("2. Retarget options"):
            self.fps_box = s.add_number("FPS", initial_value=30, min=5, max=120, step=1)
            self.human_height_box = s.add_number(
                "Human height (m, 0=auto)",
                initial_value=0.0,
                min=0.0,
                max=2.5,
                step=0.01,
            )
            self.profile_box = s.add_dropdown(
                "Source profile",
                options=["auto", "lafan1", "human_robot_hit"],
                initial_value="auto",
            )
            self.bvh_format_box = s.add_dropdown(
                "BVH loader format",
                options=["lafan1", "nokov"],
                initial_value="lafan1",
            )
            self.retarget_mode_box = s.add_dropdown(
                "FBX retarget mode",
                options=["position", "orientation"],
                initial_value="position",
            )
            self.root_joint_box = s.add_text("FBX root joint", initial_value="Hips")
            self.auto_ground_box = s.add_checkbox("Auto-ground source (BVH)", initial_value=True)
            self.pelvis_box = s.add_checkbox("Stabilize pelvis (FBX)", initial_value=True)
            self.feet_box = s.add_checkbox("Flat feet on ground", initial_value=True)
            self.frame_start_box = s.add_number("Frame start (BVH)", initial_value=0, min=0, step=1)
            self.frame_end_box = s.add_number(
                "Frame end (BVH, 0=all)",
                initial_value=0,
                min=0,
                step=1,
            )
            self.frame_step_box = s.add_number("Frame step (BVH)", initial_value=1, min=1, step=1)
            self.output_name_box = s.add_text(
                "Output .pkl name (optional)",
                initial_value="",
            )

        with s.add_folder("3. Actions"):
            reload_btn = s.add_button("Re-convert / reload source")

            @reload_btn.on_click
            def _(_) -> None:
                self._schedule_auto_process(force=True)

            save_btn = s.add_button("Save current motion .pkl")

            @save_btn.on_click
            def _(_) -> None:
                threading.Thread(target=self._do_save_current, daemon=True).start()

            refix_btn = s.add_button("Re-apply flat-feet fix")

            @refix_btn.on_click
            def _(_) -> None:
                threading.Thread(target=self._do_refix_feet, daemon=True).start()

        with s.add_folder("4. Appearance"):
            self.skin_box = s.add_dropdown(
                "Robot skin",
                options=["white", "full", "transparent"],
                initial_value="white",
            )
            self.texture_progress = s.add_progress_bar(0.0, visible=False, animated=True)

            @self.skin_box.on_update
            def _(_) -> None:
                skin = self.skin_box.value
                assert skin in ("white", "full", "transparent")
                if skin in ("full", "transparent"):
                    self.texture_progress.visible = True
                    self.texture_progress.value = 0.0
                    threading.Thread(
                        target=self._apply_skin,
                        args=(skin,),  # type: ignore[arg-type]
                        daemon=True,
                    ).start()
                else:
                    self.texture_progress.visible = False
                    self.robot.set_skin(skin)  # type: ignore[arg-type]

            self.grid_box = s.add_checkbox("Show ground grid", initial_value=True)

            @self.grid_box.on_update
            def _(_) -> None:
                self._grid.visible = bool(self.grid_box.value)

        with s.add_folder("5. Playback"):
            self.play_box = s.add_checkbox("Play", initial_value=False)

            @self.play_box.on_update
            def _(_) -> None:
                self.playing = bool(self.play_box.value)

            self.loop_box = s.add_checkbox("Loop", initial_value=True)

            @self.loop_box.on_update
            def _(_) -> None:
                self.loop_playback = bool(self.loop_box.value)

            self.frame_slider = s.add_slider("Frame", min=0, max=1, step=1, initial_value=0)

            @self.frame_slider.on_update
            def _(_) -> None:
                if not self.playing and self.qpos_frames:
                    self.frame_idx = int(self.frame_slider.value)
                    self._show_frame()

            self.speed_box = s.add_slider("Speed", min=0.1, max=2.0, step=0.1, initial_value=1.0)

        with s.add_folder("6. Output"):
            self.pkl_box = s.add_text("Current .pkl", initial_value="", disabled=True)
            s.add_markdown(
                f"Converted files: `{OUT_DIR}`  \n"
                f"Uploads: `{UPLOAD_DIR}`"
            )

    def _set_status(self, msg: str) -> None:
        print(f"[web-studio] {msg}")
        try:
            self.status.value = msg
        except Exception:
            pass

    def _resolve_path(self, raw: str) -> Optional[str]:
        p = raw.strip()
        if not p:
            return None
        candidates = [
            p,
            str(REPO_ROOT / p),
            str(WEB_ROOT / p),
            str(OUT_DIR / p),
            str(OUT_DIR / os.path.basename(p)),
            str(UPLOAD_DIR / os.path.basename(p)),
            str(DEMO_DIR / os.path.basename(p)),
        ]
        for c in candidates:
            if os.path.isfile(c):
                return c
        return None

    def _apply_skin(self, skin: SkinMode) -> None:
        def on_progress(pct: float, message: str) -> None:
            self.texture_progress.value = float(pct)
            self._set_status(message)

        try:
            self.robot.set_skin(skin, on_progress=on_progress)
            self._show_frame()
            self._set_status(f"Skin: {skin}")
        except Exception as exc:  # noqa: BLE001
            self._set_status(f"Skin load failed: {exc}")
        finally:
            self.texture_progress.visible = False

    def _standing_qpos(self) -> np.ndarray:
        return tvr.t800_standing_qpos(self._foot_model)

    def _postprocess_frames(self, frames: List[np.ndarray]) -> List[np.ndarray]:
        return foot_pp.postprocess_robot_qpos_list(self._foot_model, frames, flatten=True)

    def _schedule_auto_process(self, *, force: bool = False) -> None:
        threading.Thread(
            target=self._auto_process_source,
            kwargs={"force": force},
            daemon=True,
        ).start()

    def _auto_process_source(self, *, force: bool = False) -> None:
        if not self._process_lock.acquire(blocking=False):
            self._set_status("Already processing…")
            return
        try:
            path = self._resolve_path(self.path_box.value or "")
            kind = str(self.source_kind.value)
            if not path:
                return
            if not force and path == self._last_processed_path:
                return

            if kind == "pkl":
                if not path.lower().endswith(".pkl"):
                    self._set_status("Source type is PKL — pick a .pkl file.")
                    return
                self._set_status(f"Loading {os.path.basename(path)}…")
                self._load_pkl(path, postprocess_feet=bool(self.feet_on_load_box.value))
            elif kind == "fbx":
                if not path.lower().endswith(".fbx"):
                    self._set_status("Source type is FBX — pick an .fbx file.")
                    return
                if not self._do_convert(path=path):
                    return
            elif kind == "bvh":
                if not path.lower().endswith(".bvh"):
                    self._set_status("Source type is BVH — pick a .bvh file.")
                    return
                if not self._do_convert(path=path):
                    return
            else:
                self._set_status(f"Unknown input type: {kind}")
                return

            self._last_processed_path = path
        except Exception as exc:  # noqa: BLE001
            self._set_status(f"Processing failed: {exc}")
        finally:
            self._process_lock.release()

    def _apply_loaded_frames(self, frames: List[np.ndarray], fps: int, pkl_path: str) -> None:
        with self._lock:
            self.qpos_frames = frames
            self.fps = fps
            self.frame_idx = 0
            self.last_pkl = pkl_path
        self.frame_slider.max = max(1, len(frames) - 1)
        self.frame_slider.value = 0
        self.pkl_box.value = pkl_path
        self._show_frame()

    def _do_convert(self, path: Optional[str] = None) -> bool:
        path = path or self._resolve_path(self.path_box.value or "")
        kind = self.source_kind.value
        if not path:
            self._set_status("Set a valid file path or upload a file first.")
            return False
        try:
            if kind == "fbx":
                if not path.lower().endswith(".fbx"):
                    self._set_status("Source type is FBX — pick an .fbx file.")
                    return False
                frames, pkl = convert_fbx(
                    path,
                    fps=int(self.fps_box.value),
                    src_human=str(self.profile_box.value),
                    human_height=float(self.human_height_box.value),
                    root_joint=str(self.root_joint_box.value or "Hips"),
                    retarget_mode=str(self.retarget_mode_box.value),
                    stabilize_pelvis=bool(self.pelvis_box.value),
                    flatten_feet=bool(self.feet_box.value),
                    output_name=str(self.output_name_box.value or ""),
                    status=self._set_status,
                )
            elif kind == "bvh":
                if not path.lower().endswith(".bvh"):
                    self._set_status("Source type is BVH — pick a .bvh file.")
                    return False
                frames, pkl = convert_bvh(
                    path,
                    fps=int(self.fps_box.value),
                    bvh_format=str(self.bvh_format_box.value),
                    source_profile=str(self.profile_box.value),
                    human_height=float(self.human_height_box.value),
                    auto_ground=bool(self.auto_ground_box.value),
                    flatten_feet=bool(self.feet_box.value),
                    frame_start=int(self.frame_start_box.value),
                    frame_end=int(self.frame_end_box.value),
                    frame_step=int(self.frame_step_box.value),
                    output_name=str(self.output_name_box.value or ""),
                    status=self._set_status,
                )
            else:
                self._set_status("For PKL, set Input type to pkl.")
                return False
        except Exception as exc:  # noqa: BLE001
            self._set_status(f"Convert failed: {exc}")
            return False
        self._apply_loaded_frames(frames, int(self.fps_box.value), pkl)
        self._set_status(f"Ready — {len(frames)} frames from {os.path.basename(pkl)}.")
        return True

    def _load_pkl(self, pkl_path: str, *, postprocess_feet: bool = False) -> None:
        motion = mio.load_motion_pkl(pkl_path)
        frames, fps = mio.motion_to_qpos_list(motion)
        if postprocess_feet:
            frames = self._postprocess_frames(frames)
        self._apply_loaded_frames(frames, fps, pkl_path)
        self._set_status(f"Loaded {os.path.basename(pkl_path)} ({len(frames)} frames).")

    def _do_refix_feet(self) -> None:
        if not self.qpos_frames:
            self._set_status("Load or convert a motion first.")
            return
        try:
            frames = self._postprocess_frames(list(self.qpos_frames))
            pkl = self.last_pkl or str(OUT_DIR / "current_t800.pkl")
            motion = mio.qpos_list_to_motion(frames, self.fps)
            mio.save_motion_pkl(pkl, motion)
            self._apply_loaded_frames(frames, self.fps, pkl)
            self._set_status(f"Re-applied flat-feet fix ({len(frames)} frames).")
        except Exception as exc:  # noqa: BLE001
            self._set_status(f"Foot fix failed: {exc}")

    def _do_save_current(self) -> None:
        if not self.qpos_frames:
            self._set_status("Nothing to save — load or convert first.")
            return
        out_name = str(self.output_name_box.value or "").strip() or "current_t800.pkl"
        out_path = _output_path("current", out_name)
        try:
            motion = mio.qpos_list_to_motion(self.qpos_frames, self.fps)
            mio.save_motion_pkl(str(out_path), motion)
            self.last_pkl = str(out_path)
            self.pkl_box.value = str(out_path)
            self._set_status(f"Saved {out_path.name}")
        except Exception as exc:  # noqa: BLE001
            self._set_status(f"Save failed: {exc}")

    def _show_frame(self) -> None:
        if self.qpos_frames:
            idx = self.frame_idx % len(self.qpos_frames)
            self.robot.update_from_qpos(self.qpos_frames[idx])
        else:
            self.robot.update_from_qpos(self._standing_qpos())

    def run(self) -> None:
        port = os.environ.get("T800_WEB_PORT", os.environ.get("FBX_STUDIO_PORT", "8080"))
        self._set_status(
            f"Open http://localhost:{port} — upload FBX/BVH/PKL in block 1 (auto convert)"
        )
        while True:
            if self.playing and self.qpos_frames:
                n = len(self.qpos_frames)
                next_idx = self.frame_idx + 1
                if next_idx >= n:
                    if not self.loop_playback:
                        self.play_box.value = False
                        self.playing = False
                        continue
                    next_idx = 0
                self.frame_idx = next_idx
                self._show_frame()
                try:
                    self.frame_slider.value = self.frame_idx
                except Exception:
                    pass
                dt = 1.0 / max(1e-3, self.fps * float(self.speed_box.value))
                time.sleep(dt)
            else:
                time.sleep(0.05)


def main() -> None:
    if not (GMR_ROOT / "general_motion_retargeting").is_dir():
        print(f"ERROR: bundled backend missing at {GMR_ROOT}", file=sys.stderr)
        print("Run ./install.sh or ./scripts/bundle_gmr.sh", file=sys.stderr)
        sys.exit(1)
    WebStudio().run()


if __name__ == "__main__":
    main()
