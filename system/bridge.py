"""
Serial bridge — custom packet protocol between host and Arduino.

Packet format:
  [0xAA][TYPE][LEN][PAYLOAD...][CRC]
  CRC = (0xFF - (TYPE + LEN + sum(PAYLOAD)) % 256) & 0xFF

Message types:
  0x01  HEARTBEAT   Arduino → Host   no payload
  0x02  DRIVE       Host → Arduino   float32 left, float32 right  [-1, 1]  (8 bytes)
  0x03  CLAW        Host → Arduino   float32 angle                         (4 bytes)
  0x04  ENCODERS    Arduino → Host   int32 left_ticks, int32 right_ticks   (8 bytes)
"""

import struct
import threading
import time
import logging

import serial

log = logging.getLogger(__name__)

SYNC          = 0xAA
MSG_HEARTBEAT = 0x01
MSG_DRIVE     = 0x02
MSG_CLAW      = 0x03
MSG_ENCODERS  = 0x04


class SerialBridge:

    def __init__(self, port: str, baud: int = 115200):
        self.port = port
        self.baud = baud
        self._ser:    serial.Serial | None = None
        self._thread: threading.Thread | None = None
        self._running = False

        self.on_heartbeat = None   # callback()
        self.on_encoders  = None   # callback(left_ticks: int, right_ticks: int)
        self.on_raw       = None   # callback(data: bytes) — called with every raw chunk

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start(self):
        self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
        time.sleep(2)   # wait for Arduino reset after DTR toggle
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        log.info('Connected to %s @ %d baud', self.port, self.baud)

    def stop(self):
        self._running = False
        if self._ser and self._ser.is_open:
            self._ser.close()

    # ── Outgoing ──────────────────────────────────────────────────────────────

    def send_drive(self, left: float, right: float):
        self._send(MSG_DRIVE, struct.pack('<ff', left, right))

    def send_claw(self, angle: float):
        self._send(MSG_CLAW, struct.pack('<f', angle))

    # ── Internal ──────────────────────────────────────────────────────────────

    def _send(self, msg_type: int, payload: bytes):
        if self._ser is None or not self._ser.is_open:
            return
        crc = (0xFF - ((msg_type + len(payload) + sum(payload)) % 256)) & 0xFF
        packet = bytes([SYNC, msg_type, len(payload)]) + payload + bytes([crc])
        try:
            self._ser.write(packet)
        except serial.SerialException as e:
            log.error('Write error: %s', e)

    def _read_loop(self):
        buf = bytearray()
        while self._running:
            try:
                chunk = self._ser.read(256)
                if chunk:
                    if self.on_raw:
                        self.on_raw(chunk)
                    buf.extend(chunk)
                    buf = self._process(buf)
            except serial.SerialException as e:
                log.error('Read error: %s', e)
                time.sleep(1)

    def _process(self, buf: bytearray) -> bytearray:
        while len(buf) >= 4:
            if buf[0] != SYNC:
                buf = buf[1:]
                continue

            msg_type = buf[1]
            length   = buf[2]

            if len(buf) < 4 + length:
                break   # wait for more bytes

            payload  = bytes(buf[3:3 + length])
            crc_recv = buf[3 + length]
            crc_calc = (0xFF - ((msg_type + length + sum(payload)) % 256)) & 0xFF
            buf      = buf[4 + length:]

            if crc_recv != crc_calc:
                log.warning('CRC mismatch (type=0x%02x)', msg_type)
                continue

            self._handle(msg_type, payload)

        return buf

    def _handle(self, msg_type: int, payload: bytes):
        if msg_type == MSG_HEARTBEAT:
            log.debug('Heartbeat')
            if self.on_heartbeat:
                self.on_heartbeat()
        elif msg_type == MSG_ENCODERS and len(payload) == 8:
            left, right = struct.unpack('<ii', payload)
            log.debug('Encoders left=%d right=%d', left, right)
            if self.on_encoders:
                self.on_encoders(left, right)
        else:
            log.debug('Unknown packet type 0x%02x', msg_type)
