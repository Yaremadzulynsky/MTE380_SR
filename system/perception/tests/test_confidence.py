from src.vision.confidence import compute_gamma


def test_compute_gamma() -> None:
    assert compute_gamma(0.0, 100.0) == 0.0
    assert compute_gamma(50.0, 100.0) == 0.5
    assert compute_gamma(150.0, 100.0) == 1.0
    assert compute_gamma(50.0, 0.0) == 0.0
