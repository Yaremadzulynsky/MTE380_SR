from pathlib import Path

import yaml

from src.config import AppConfig
from src.main import RuntimePerceptionConfig
from src.pipeline import PipelineState


def test_runtime_perception_config_updates_and_persists(tmp_path: Path) -> None:
    config_path = tmp_path / "perception.yaml"
    config_path.write_text(
        yaml.safe_dump(
            {
                "path_mask_key": "red",
                "heading": {
                    "pid_kp": 0.5,
                    "pid_ki": 0.0,
                    "pid_kd": 0.1,
                    "pid_i_max": 0.6,
                    "pid_deadband": 0.01,
                    "lookahead_y_frac": 0.72,
                    "lateral_gain": 0.5,
                    "forward_bias": 1.0,
                    "max_lateral_abs": 0.6,
                    "base_speed": 0.35,
                    "min_speed": 0.1,
                    "max_speed": 0.45,
                    "error_slowdown": 0.25,
                },
                "debug_stream": {"enabled": True},
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    cfg = AppConfig.from_dict(yaml.safe_load(config_path.read_text(encoding="utf-8")))
    state = PipelineState(pid_integral=0.4, pid_prev_error=0.2)
    runtime = RuntimePerceptionConfig(cfg, config_path, state)

    payload, status = runtime.update({"pid_kp": 1.25, "lookahead_y_frac": 0.66})

    assert status == 200
    assert payload["saved"] is True
    assert payload["updated"] == {"pid_kp": 1.25, "lookahead_y_frac": 0.66}
    assert payload["values"]["pid_kp"] == 1.25
    assert payload["values"]["lookahead_y_frac"] == 0.66
    assert state.pid_integral == 0.0
    assert state.pid_prev_error == 0.0

    saved = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    assert saved["heading"]["pid_kp"] == 1.25
    assert saved["heading"]["lookahead_y_frac"] == 0.66
    assert saved["debug_stream"]["enabled"] is True


def test_runtime_perception_config_rejects_invalid_speed_bounds(tmp_path: Path) -> None:
    config_path = tmp_path / "perception.yaml"
    config_path.write_text("heading: {}\n", encoding="utf-8")
    runtime = RuntimePerceptionConfig(AppConfig(), config_path, PipelineState())

    payload, status = runtime.update({"min_speed": 0.8, "max_speed": 0.3})

    assert status == 400
    assert payload["message"] == "Invalid speed bounds."
    assert "min_speed" in payload["errors"]
