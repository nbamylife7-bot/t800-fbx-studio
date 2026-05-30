#!/usr/bin/env python3
"""Verify T800 FBX Studio dependencies and optional FBX→PKL smoke test."""

from __future__ import annotations

import os
import sys
import tempfile
from pathlib import Path

WEB_ROOT = Path(__file__).resolve().parents[1]
GMR_ROOT = Path(os.environ.get("GMR_ROOT", WEB_ROOT / "gmr")).resolve()

for entry in (WEB_ROOT, GMR_ROOT, GMR_ROOT / "third_party"):
    s = str(entry)
    if s not in sys.path:
        sys.path.insert(0, s)


def check(name: str, fn) -> None:
    try:
        fn()
        print(f"[OK] {name}")
    except Exception as exc:  # noqa: BLE001
        print(f"[FAIL] {name}: {exc}")
        raise


def main() -> int:
    print(f"GMR_ROOT={GMR_ROOT}")
    if not (GMR_ROOT / "assets" / "t800").is_dir():
        print("Missing bundled backend. Run ./scripts/bundle_gmr.sh from maintainer checkout.")
        return 1

    check("mujoco", lambda: __import__("mujoco"))
    check("viser", lambda: __import__("viser"))
    check("torch", lambda: __import__("torch"))
    check("trimesh", lambda: __import__("trimesh"))
    check("mink", lambda: __import__("mink"))

    try:
        __import__("fbx")
        print("[OK] fbx (Autodesk FBX SDK)")
        fbx_ok = True
    except ImportError as exc:
        print(f"[WARN] fbx: {exc}")
        print("       FBX upload disabled until FBX SDK is installed (see install.sh).")
        fbx_ok = False

    try:
        __import__("smplx")
        print("[OK] smplx")
        smplx_pkg_ok = True
    except ImportError as exc:
        print(f"[WARN] smplx: {exc}")
        smplx_pkg_ok = False

    body_models = Path(
        os.environ.get("SMPLX_BODY_MODELS", GMR_ROOT / "assets" / "body_models")
    ).resolve()
    smplx_dir = body_models / "smplx"
    model_globs = list(smplx_dir.glob("SMPLX_*.npz")) + list(smplx_dir.glob("SMPLX_*.pkl"))
    if model_globs:
        print(f"[OK] SMPL-X body models ({len(model_globs)} file(s) in {smplx_dir})")
        smplx_models_ok = True
    else:
        print(f"[WARN] SMPL-X body models: none in {smplx_dir}")
        print("       NPZ upload disabled until you run ./scripts/install_smplx_models.sh")
        smplx_models_ok = False

    if smplx_pkg_ok and smplx_models_ok:
        from general_motion_retargeting.utils.smpl import resolve_smplx_model_file  # noqa: E402

        model_file = resolve_smplx_model_file(str(body_models), "neutral")
        print(f"[OK] SMPL-X loader -> {Path(model_file).name}")

    test_npz = os.environ.get("T800_TEST_NPZ", "").strip()
    if test_npz and smplx_pkg_ok and smplx_models_ok:
        test_path = Path(test_npz)
        if not test_path.is_file():
            print(f"[FAIL] T800_TEST_NPZ not found: {test_path}")
            return 1
        from app.studio import convert_smplx_npz  # noqa: E402

        frames, pkl, fps = convert_smplx_npz(
            str(test_path),
            fps=30,
            human_height=0.0,
            auto_ground=True,
            flatten_feet=False,
            output_name="verify_npz_test.pkl",
            status=lambda msg: print(f"  {msg}"),
        )
        assert len(frames) > 0, "no frames retargeted"
        assert Path(pkl).is_file(), "pkl not written"
        print(f"[OK] NPZ→PKL smoke test: {len(frames)} frames @ {fps} fps → {pkl}")
    elif smplx_pkg_ok and smplx_models_ok:
        print("[INFO] Set T800_TEST_NPZ=/path/to/motion.npz to run NPZ→PKL smoke test.")

    from general_motion_retargeting.params import ROBOT_XML_DICT  # noqa: E402

    check("T800 xml", lambda: Path(ROBOT_XML_DICT["t800"]).read_text()[:10])

    test_fbx = os.environ.get("T800_TEST_FBX", "").strip()
    if test_fbx and fbx_ok:
        test_path = Path(test_fbx)
        if not test_path.is_file():
            print(f"[FAIL] T800_TEST_FBX not found: {test_path}")
            return 1
        from app.studio import convert_fbx  # noqa: E402

        with tempfile.TemporaryDirectory() as tmp:
            out_name = "verify_test.pkl"
            os.environ["T800_VERIFY_TMP"] = tmp
            frames, pkl = convert_fbx(
                str(test_path),
                fps=30,
                src_human="auto",
                human_height=1.75,
                root_joint="Hips",
                retarget_mode="position",
                stabilize_pelvis=True,
                flatten_feet=True,
                output_name=out_name,
                status=lambda msg: print(f"  {msg}"),
            )
            assert len(frames) > 0, "no frames retargeted"
            assert Path(pkl).is_file(), "pkl not written"
            print(f"[OK] FBX→PKL smoke test: {len(frames)} frames → {pkl}")
    elif fbx_ok:
        print("[INFO] Set T800_TEST_FBX=/path/to/motion.fbx to run full FBX→PKL smoke test.")

    print("\nSetup looks good.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
