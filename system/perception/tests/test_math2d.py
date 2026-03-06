import math

import numpy as np

from src.utils.math2d import circularity, clamp01, to_robot_frame_clamped, unit


def test_clamp01() -> None:
    assert clamp01(-1.2) == 0.0
    assert clamp01(0.3) == 0.3
    assert clamp01(2.5) == 1.0


def test_unit() -> None:
    v = unit([3.0, 4.0])
    assert np.allclose(v, np.array([0.6, 0.8], dtype=np.float32), atol=1e-6)
    z = unit([0.0, 0.0])
    assert np.allclose(z, np.array([0.0, 0.0], dtype=np.float32))


def test_circularity_of_circle_like_shape() -> None:
    value = circularity(area=math.pi * 4.0, perimeter=2.0 * math.pi * 2.0)
    assert 0.95 <= value <= 1.05


def test_to_robot_frame_clamped() -> None:
    # Image forward (up) = (0, -1) -> robot (0, 1)
    px, py = to_robot_frame_clamped(0.0, -1.0)
    assert abs(px) < 1e-6 and abs(py - 1.0) < 1e-6
    # Image right = (1, 0) -> robot (1, 0)
    px, py = to_robot_frame_clamped(1.0, 0.0)
    assert abs(px - 1.0) < 1e-6 and abs(py) < 1e-6
    # Clamp: norm > 1 -> unit vector
    px, py = to_robot_frame_clamped(2.0, 0.0)
    assert abs(px - 1.0) < 1e-6 and abs(py) < 1e-6
    assert abs(math.hypot(px, py) - 1.0) < 1e-6
