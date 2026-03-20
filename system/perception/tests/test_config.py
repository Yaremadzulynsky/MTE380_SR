from src.config import AppConfig


def test_config_supports_forward_bias_and_legacy_alias() -> None:
    cfg = AppConfig.from_dict({"heading": {"forward_y": 0.8, "lateral_gain": 0.4}})
    assert cfg.heading.forward_bias == 0.8
    assert cfg.heading.lateral_gain == 0.4


def test_output_url_defaults_to_control_comm_vector_endpoint(monkeypatch) -> None:
    monkeypatch.delenv("CONTROL_COMM_BASE_URL", raising=False)
    monkeypatch.delenv("CONTROL_COMM_VECTOR_PATH", raising=False)

    cfg = AppConfig()

    assert cfg.output_url == "http://100.72.60.28:5001/vector"
