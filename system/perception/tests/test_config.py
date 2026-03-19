from src.config import AppConfig


def test_config_parses_debug_stream() -> None:
    cfg = AppConfig.from_dict(
        {
            "camera": {"source": "udp://0.0.0.0:5000"},
            "path_mask_key": "blue",
            "roi_y_start_ratio": 0.5,
            "heading": {
                "probe_x_frac": 0.45,
                "probe_y_frac": 0.92,
                "lookahead_y_frac": 0.58,
                "forward_bias": 1.0,
                "lateral_gain": 0.4,
                "max_lateral_abs": 0.55,
            },
            "debug_stream": {"enabled": True, "host": "0.0.0.0", "port": 8081, "jpeg_quality": 70},
        }
    )
    assert cfg.camera.source == "udp://0.0.0.0:5000"
    assert cfg.path_mask_key == "blue"
    assert cfg.roi_y_start_ratio == 0.5
    assert cfg.heading.probe_x_frac == 0.45
    assert cfg.heading.probe_y_frac == 0.92
    assert cfg.heading.lookahead_y_frac == 0.58
    assert cfg.heading.forward_bias == 1.0
    assert cfg.heading.lateral_gain == 0.4
    assert cfg.heading.max_lateral_abs == 0.55
    assert cfg.debug_stream.enabled is True
    assert cfg.debug_stream.port == 8081
    assert cfg.debug_stream.jpeg_quality == 70


def test_config_accepts_legacy_forward_y_alias() -> None:
    cfg = AppConfig.from_dict({"heading": {"forward_y": 0.9}})
    assert cfg.heading.forward_bias == 0.9
