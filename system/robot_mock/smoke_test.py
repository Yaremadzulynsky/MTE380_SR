import os

os.environ["DISABLE_EXTERNAL_REQUESTS"] = "1"

from app import app  # noqa: E402


def main() -> None:
    client = app.test_client()

    resp = client.get("/health")
    assert resp.status_code == 200
    assert resp.get_json()["status"] == "ok"

    resp = client.get("/api/config")
    assert resp.status_code == 200
    payload = resp.get_json()
    assert payload["external_requests_disabled"] is True

    resp = client.get("/api/system")
    assert resp.status_code == 200
    snapshot = resp.get_json()
    assert "control" in snapshot and "state" in snapshot

    resp = client.post("/api/state-machine/inputs", json={"target": {"detected": False, "vector": {"x": 0, "y": 0}}})
    assert resp.status_code == 200

    print("robot_mock smoke test: ok")


if __name__ == "__main__":
    main()
