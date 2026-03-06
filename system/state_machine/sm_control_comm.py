from typing import Any
from urllib.parse import urljoin

import requests

from sm_models import State


class ControlCommClient:
    def __init__(
        self,
        base_url: str,
        state_path: str = "/state",
        control_path: str = "/control",
        line_follow_pid_path: str = "/line-follow-pid",
        timeout: float = 0.5,
    ) -> None:
        self._state_url = urljoin(base_url.rstrip("/") + "/", state_path.lstrip("/"))
        self._control_url = urljoin(
            base_url.rstrip("/") + "/", control_path.lstrip("/")
        )
        self._line_follow_pid_url = urljoin(
            base_url.rstrip("/") + "/", line_follow_pid_path.lstrip("/")
        )
        self._timeout = timeout

    def _get(self, url: str) -> dict[str, Any]:
        try:
            response = requests.get(url, timeout=self._timeout)
        except requests.RequestException as exc:
            return {
                "ok": False,
                "status_code": None,
                "error": str(exc),
                "url": url,
            }

        data = None
        if response.text:
            try:
                data = response.json()
            except ValueError:
                data = {"text": response.text[:500]}

        if response.ok:
            return {
                "ok": True,
                "status_code": response.status_code,
                "url": url,
                "data": data,
            }

        detail = None
        if response.text:
            detail = response.text[:500]
        result = {
            "ok": False,
            "status_code": response.status_code,
            "error": f"HTTP {response.status_code}",
            "url": url,
        }
        if detail:
            result["detail"] = detail
        return result

    def _post(self, url: str, payload: dict[str, Any]) -> dict[str, Any]:
        try:
            response = requests.post(url, json=payload, timeout=self._timeout)
        except requests.RequestException as exc:
            return {
                "ok": False,
                "status_code": None,
                "error": str(exc),
                "url": url,
            }

        if response.ok:
            return {
                "ok": True,
                "status_code": response.status_code,
                "url": url,
            }

        detail = None
        if response.text:
            detail = response.text[:500]
        result = {
            "ok": False,
            "status_code": response.status_code,
            "error": f"HTTP {response.status_code}",
            "url": url,
        }
        if detail:
            result["detail"] = detail
        return result

    def send_state(self, state: State) -> dict[str, Any]:
        _ = state
        return {
            "ok": True,
            "status_code": None,
            "url": self._state_url,
            "state_transport": "disabled",
        }

    def send_control(
        self,
        x: float,
        y: float,
        speed: float,
        servo: int | None = None,
    ) -> dict[str, Any]:
        payload = {"x": x, "y": y, "speed": speed}
        if servo is not None:
            payload["servo"] = int(servo)

        payload_real = {"x": x, "y": y}

        # print(f"Posting to control communication: {payload_real}")
        # post to the actual thing just as a test
        self._post("http://192.168.0.101:5001/vector", payload_real)
        return self._post(self._control_url, payload)

    def get_control(self) -> dict[str, Any]:
        return self._get(self._control_url)

    def get_line_follow_pid(self) -> dict[str, Any]:
        return self._get(self._line_follow_pid_url)
