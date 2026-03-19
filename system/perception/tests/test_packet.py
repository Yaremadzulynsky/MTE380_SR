import json

from src.comms.packet import PerceptionPacket, path_mask_to_line_key


def test_packet_string_zone() -> None:
    p = PerceptionPacket(px=0.1, py=-0.9, zone="PATH", gamma=0.7, t=123.4)
    d = p.to_dict(zone_encoding="string")
    assert d["zone"] == "PATH"


def test_packet_int_zone() -> None:
    p = PerceptionPacket(px=0.1, py=-0.9, zone="DANGER", gamma=0.7, t=123.4)
    d = p.to_dict(zone_encoding="int")
    assert d["zone"] == 2


def test_packet_json_parses() -> None:
    p = PerceptionPacket(px=0.1, py=-0.9, zone="TARGET", gamma=1.0, t=123.4)
    s = p.to_json(zone_encoding="string")
    decoded = json.loads(s)
    assert decoded["zone"] == "TARGET"
    assert decoded["px"] == 0.1


def test_packet_path_fields() -> None:
    p = PerceptionPacket(
        px=0.12, py=0.85, zone="PATH", gamma=0.8, t=1.0,
        path_detected=True, path_mask_key="red",
    )
    d = p.to_dict()
    assert d["path_detected"] is True
    assert d["path_mask_key"] == "red"


def test_path_mask_to_line_key_supports_blue() -> None:
    assert path_mask_to_line_key("red") == "red_line"
    assert path_mask_to_line_key("blue") == "blue_line"
    assert path_mask_to_line_key("black") == "black_line"
