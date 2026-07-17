"""Deterministic gates for constrained target re-identification."""

from __future__ import annotations

import math
from typing import Sequence

Box = tuple[float, float, float, float]


def select_reacquire_candidate(
    previous_box: Box | None,
    candidate_boxes: Sequence[Box],
    *,
    elapsed_s: float,
    timeout_s: float,
    image_width: int,
    image_height: int,
    center_ratio: float,
    area_ratio_min: float,
    area_ratio_max: float,
) -> int | None:
    """Return the closest acceptable candidate index, otherwise ``None``."""
    if previous_box is None or elapsed_s < 0.0 or elapsed_s > timeout_s:
        return None
    if image_width <= 0 or image_height <= 0:
        raise ValueError('image dimensions must be positive')
    if not 0.0 <= center_ratio <= 1.0:
        raise ValueError('center_ratio must be in [0, 1]')
    if area_ratio_min <= 0.0 or area_ratio_max < area_ratio_min:
        raise ValueError('invalid area ratio bounds')

    old_x, old_y, old_width, old_height = previous_box
    old_center = (old_x + old_width / 2.0, old_y + old_height / 2.0)
    old_area = max(1.0, old_width * old_height)
    maximum_distance = math.hypot(image_width, image_height) * center_ratio
    accepted: list[tuple[float, int]] = []
    for index, (x, y, width, height) in enumerate(candidate_boxes):
        area_ratio = (width * height) / old_area
        distance = math.hypot(
            x + width / 2.0 - old_center[0],
            y + height / 2.0 - old_center[1],
        )
        if distance <= maximum_distance and area_ratio_min <= area_ratio <= area_ratio_max:
            accepted.append((distance, index))
    return min(accepted)[1] if accepted else None
