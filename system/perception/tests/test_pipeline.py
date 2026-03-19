import cv2
import numpy as np

from src.config import AppConfig
from src.pipeline import PipelineState, run_pipeline


def _base_cfg() -> AppConfig:
    return AppConfig.from_dict(
        {
            "path_mask_key": "red",
            "roi_y_start_ratio": 0.0,
            "alpha": 0.0,
            "heading": {
                "min_area": 20.0,
                "probe_x_frac": 0.5,
                "probe_y_frac": 0.9,
                "lookahead_y_frac": 0.58,
                "forward_bias": 1.0,
                "lateral_gain": 0.5,
                "max_lateral_abs": 0.6,
            },
        }
    )


def test_centered_line_stays_mostly_forward() -> None:
    cfg = _base_cfg()
    frame = np.zeros((220, 220, 3), dtype=np.uint8)
    cv2.line(frame, (110, 219), (110, 0), (0, 0, 255), 12)
    out = run_pipeline(frame, PipelineState(), cfg)
    assert out.path_detected is True
    assert abs(float(out.image_vector[0])) < 0.05
    assert abs(float(out.image_vector[1]) + 1.0) < 1e-6
    assert float(out.robot_vector[1]) > 0.99


def test_curve_right_gives_modest_positive_lateral_term() -> None:
    cfg = _base_cfg()
    frame = np.zeros((220, 220, 3), dtype=np.uint8)
    curve = np.asarray([(85, 219), (90, 190), (112, 160), (145, 132), (180, 110)], dtype=np.int32)
    cv2.polylines(frame, [curve], False, (0, 0, 255), 12)
    out = run_pipeline(frame, PipelineState(), cfg)
    assert out.path_detected is True
    assert 0.05 < float(out.image_vector[0]) < 0.45
    assert abs(float(out.image_vector[0])) < abs(float(out.image_vector[1]))
    assert float(out.robot_vector[1]) > 0.9
