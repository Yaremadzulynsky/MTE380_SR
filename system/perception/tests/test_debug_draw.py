import numpy as np

from src.vision.debug_draw import draw_overlay


def test_draw_overlay_marks_probe_target_and_control_vectors() -> None:
    roi = np.zeros((120, 120, 3), dtype=np.uint8)
    out = draw_overlay(
        roi,
        np.array([0.2, -1.0], dtype=np.float32),
        zone="PATH",
        gamma=1.0,
        path_detected=True,
        tangent_vector=np.array([0.8, -0.4], dtype=np.float32),
        probe_point=(40.0, 100.0),
        target_point=(52.0, 74.0),
    )
    probe_px = out[100, 40]
    target_px = out[74, 52]

    assert probe_px.tolist() == [255, 255, 255]
    assert int(target_px[1]) > 200
    assert np.count_nonzero(out[:, :, 0]) > 0
    assert np.count_nonzero(out[:, :, 1]) > 0
