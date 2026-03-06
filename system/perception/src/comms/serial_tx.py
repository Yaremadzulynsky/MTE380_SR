"""Serial packet sender (JSON lines)."""

from __future__ import annotations

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None


class SerialSender:
    def __init__(self, port: str, baud: int) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed; cannot use serial comms.")
        self.ser = serial.Serial(port=port, baudrate=int(baud), timeout=0)

    def send_line(self, text: str) -> None:
        self.ser.write((text + "\n").encode("utf-8"))

    def close(self) -> None:
        self.ser.close()
