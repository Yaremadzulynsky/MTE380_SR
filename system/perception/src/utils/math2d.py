"""Small 2D math helpers used across the perception node."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np


def clamp01(value: float) -> float:
    """Clamp a value into [0, 1]."""
    return max(0.0, min(1.0, float(value)))


def unit(vec: Iterable[float]) -> np.ndarray:
    """Return vec normalized to unit length, or zero vector if tiny."""
    arr = np.asarray(list(vec), dtype=np.float32)
    norm = float(np.linalg.norm(arr))
    if norm < 1e-9:
        return np.zeros_like(arr, dtype=np.float32)
    return arr / norm


def to_robot_frame_clamped(px_image: float, py_image: float) -> tuple[float, float]:
    """
    Convert image-frame heading to robot frame and clamp so norm ≤ 1 (max speed).
    Robot frame: X+ = right, Y+ = forward. Image frame: X+ = right, Y+ = down.
    So robot (px, py) = (px_image, -py_image). Result is clamped to unit circle
    so sqrt(px^2 + py^2) ≤ 1 (unit vector = max possible speed).
    """
    px = float(px_image)
    py = float(-py_image)
    norm = math.hypot(px, py)
    if norm < 1e-9:
        return 0.0, 0.0
    if norm > 1.0:
        px, py = px / norm, py / norm
    return px, py


def circularity(area: float, perimeter: float) -> float:
    """Compute shape circularity = 4*pi*A / P^2."""
    if perimeter <= 1e-9:
        return 0.0
    return float((4.0 * math.pi * area) / (perimeter * perimeter))
