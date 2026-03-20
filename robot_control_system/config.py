"""Load config.yaml from the robot_control_system root."""

from __future__ import annotations
from pathlib import Path
import yaml

_PATH = Path(__file__).parent / 'config.yaml'

def load(path: str | Path = _PATH) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)

# Singleton — loaded once on first import
_cfg: dict | None = None

def get() -> dict:
    global _cfg
    if _cfg is None:
        _cfg = load()
    return _cfg

def reload() -> dict:
    global _cfg
    _cfg = load()
    return _cfg

def save(cfg: dict, path: str | Path = _PATH) -> None:
    """Write cfg back to config.yaml, preserving key order."""
    with open(path, 'w') as f:
        yaml.dump(cfg, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
