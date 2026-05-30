"""Resolve web-version paths and add GMR to PYTHONPATH."""

from __future__ import annotations

import os
import sys
from pathlib import Path

WEB_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(os.environ.get("CYANPUPPETS_ROOT", WEB_ROOT.parent)).resolve()
GMR_ROOT = Path(os.environ.get("GMR_ROOT", WEB_ROOT / "gmr")).resolve()

OUT_DIR = WEB_ROOT / "data" / "out"
UPLOAD_DIR = WEB_ROOT / "data" / "uploads"
DEMO_DIR = Path(os.environ.get("T800_DEMO_DIR", REPO_ROOT / "out")).resolve()


def bootstrap() -> tuple[Path, Path, Path]:
    """Insert GMR (+ FBX third_party) on sys.path. Returns (web_root, repo_root, gmr_root)."""
    for path in (GMR_ROOT, GMR_ROOT / "third_party"):
        entry = str(path)
        if entry not in sys.path:
            sys.path.insert(0, entry)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    UPLOAD_DIR.mkdir(parents=True, exist_ok=True)
    return WEB_ROOT, REPO_ROOT, GMR_ROOT
