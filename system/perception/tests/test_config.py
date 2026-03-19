from src.config import AppConfig


def test_config_supports_forward_bias_and_legacy_alias() -> None:
    cfg = AppConfig.from_dict({"heading": {"forward_y": 0.8, "lateral_gain": 0.4}})
    assert cfg.heading.forward_bias == 0.8
    assert cfg.heading.lateral_gain == 0.4
