"""UDP packet sender."""

from __future__ import annotations

import socket


class UDPSender:
    def __init__(self, ip: str, port: int) -> None:
        self.addr = (ip, int(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_line(self, text: str) -> None:
        self.sock.sendto(text.encode("utf-8"), self.addr)

    def close(self) -> None:
        self.sock.close()
