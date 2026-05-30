from __future__ import annotations

from collections.abc import Iterator
from typing import Any


def iter_retarget_config_entries(table: dict[str, list[Any]]) -> Iterator[tuple[str, str, float, float, list[float], list[float], bool]]:
    """Yield IK config entries while preserving zero-weight offset carriers."""

    for frame_name, entry in table.items():
        body_name, pos_weight, rot_weight, pos_offset, rot_offset = entry
        creates_task = bool(pos_weight != 0 or rot_weight != 0)
        yield frame_name, body_name, pos_weight, rot_weight, pos_offset, rot_offset, creates_task
