from typing import Optional


def parse_timestamp_ms(value) -> Optional[int]:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    if parsed != parsed or parsed in (float("inf"), float("-inf")):
        return None
    return int(parsed)


def parse_finite_float(value, field_name: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{field_name} must be a finite number.") from None
    if parsed != parsed or parsed in (float("inf"), float("-inf")):
        raise ValueError(f"{field_name} must be a finite number.")
    return parsed
