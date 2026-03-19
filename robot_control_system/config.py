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
