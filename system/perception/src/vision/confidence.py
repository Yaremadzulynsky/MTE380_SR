"""Confidence score computation."""

from __future__ import annotations

from src.utils.math2d import clamp01


def compute_gamma(area_detected: float, expected_area: float) -> float:
    """gamma = clamp(area_detected / expected_area, 0..1)."""
    if expected_area <= 1e-9:
        return 0.0
    return clamp01(float(area_detected) / float(expected_area))
