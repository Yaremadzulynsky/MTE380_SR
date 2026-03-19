import cv2
import numpy as np

from src.vision.heading import extract_path_observation


def _make_mask(start: tuple[int, int], end: tuple[int, int], *, w: int = 220, h: int = 220) -> np.ndarray:
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.line(mask, start, end, 255, 12)
    return mask


def _make_curve(points: list[tuple[int, int]], *, w: int = 220, h: int = 220) -> np.ndarray:
    mask = np.zeros((h, w), dtype=np.uint8)
    curve = np.asarray(points, dtype=np.int32)
    cv2.polylines(mask, [curve], False, 255, 12)
    return mask


def test_centered_vertical_line_has_near_zero_lateral_error() -> None:
    mask = _make_mask((110, 219), (110, 0))
    correction, tangent, _, _, debug = extract_path_observation(mask, min_area=20.0)
    assert debug["fit_ok"] is True
    assert abs(float(correction[0])) < 0.05
    assert abs(float(correction[1]) + 1.0) < 1e-6
    assert float(tangent[1]) < 0.0


def test_left_offset_line_points_left_with_forward_bias_intact() -> None:
    mask = _make_mask((70, 219), (70, 0))
    correction, _, _, _, debug = extract_path_observation(mask, min_area=20.0)
    assert debug["fit_ok"] is True
    assert float(correction[0]) < -0.08
    assert abs(float(correction[1]) + 1.0) < 1e-6


def test_right_offset_line_points_right_with_forward_bias_intact() -> None:
    mask = _make_mask((150, 219), (150, 0))
    correction, _, _, _, debug = extract_path_observation(mask, min_area=20.0)
    assert debug["fit_ok"] is True
    assert float(correction[0]) > 0.08
    assert abs(float(correction[1]) + 1.0) < 1e-6


def test_curve_right_produces_modest_rightward_control_not_tangent_following() -> None:
    mask = _make_curve([(85, 219), (90, 190), (112, 160), (145, 132), (180, 110)])
    correction, tangent, _, _, debug = extract_path_observation(
        mask,
        min_area=20.0,
        lookahead_y_frac=0.58,
        forward_bias=1.0,
        lateral_gain=0.5,
        max_lateral_abs=0.6,
    )
    assert debug["fit_ok"] is True
    assert float(debug["target_point"][1]) == float(debug["lookahead_y"])
    assert 0.05 < float(correction[0]) < 0.45
    assert abs(float(correction[1]) + 1.0) < 1e-6
    assert abs(float(correction[0])) < abs(float(debug["raw_lateral_error"]))
    assert not np.allclose(correction, tangent, atol=0.15)


def test_curve_left_produces_modest_leftward_control_not_tangent_following() -> None:
    mask = _make_curve([(135, 219), (130, 190), (108, 160), (75, 132), (40, 110)])
    correction, tangent, _, _, debug = extract_path_observation(
        mask,
        min_area=20.0,
        lookahead_y_frac=0.58,
        forward_bias=1.0,
        lateral_gain=0.5,
        max_lateral_abs=0.6,
    )
    assert debug["fit_ok"] is True
    assert -0.45 < float(correction[0]) < -0.05
    assert abs(float(correction[1]) + 1.0) < 1e-6
    assert abs(float(correction[0])) < abs(float(debug["raw_lateral_error"]))
    assert not np.allclose(correction, tangent, atol=0.15)


def test_tight_bend_lateral_term_is_clamped_while_forward_bias_dominates() -> None:
    mask = _make_curve([(82, 219), (84, 198), (92, 182), (125, 160), (172, 138), (205, 125)])
    correction, _, _, _, debug = extract_path_observation(
        mask,
        min_area=20.0,
        lookahead_y_frac=0.5,
        forward_bias=1.0,
        lateral_gain=1.5,
        max_lateral_abs=0.35,
    )
    assert debug["fit_ok"] is True
    assert abs(float(correction[0]) - 0.35) < 1e-6
    assert abs(float(correction[1]) + 1.0) < 1e-6
    assert abs(float(correction[0])) < abs(float(correction[1]))


def test_regression_curve_does_not_point_directly_to_lookahead_target() -> None:
    mask = _make_curve([(85, 219), (92, 192), (118, 165), (154, 140), (188, 120)])
    correction, _, _, _, debug = extract_path_observation(
        mask,
        min_area=20.0,
        probe_x_frac=0.5,
        probe_y_frac=0.9,
        lookahead_y_frac=0.58,
        forward_bias=1.0,
        lateral_gain=0.5,
        max_lateral_abs=0.6,
    )
    geometry_vector = np.asarray(debug["geometry_vector_px"], dtype=np.float32)
    geometry_norm = geometry_vector / max(np.linalg.norm(geometry_vector), 1e-6)
    assert debug["fit_ok"] is True
    assert float(correction[1]) == -1.0
    assert abs(float(correction[0])) < abs(float(geometry_norm[0]))
    assert abs(float(correction[0])) < abs(float(correction[1]))


def test_empty_mask_reports_no_detection() -> None:
    mask = np.zeros((160, 160), dtype=np.uint8)
    correction, tangent, area, contours, debug = extract_path_observation(mask, min_area=20.0)
    assert debug["fit_ok"] is False
    assert area == 0.0
    assert contours == []
    assert np.allclose(correction, np.zeros(2))
    assert np.allclose(tangent, np.zeros(2))
