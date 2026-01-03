# -*- coding: utf-8 -*-
"""Serial transport layer.

Responsibilities
---------------
- Open/close serial port.
- Read lines from serial.
- Parse each line using :func:`host_gui.core.protocol.parse_line`.
- Emit strongly-typed events to the UI layer.

Non-responsibilities
--------------------
- No UI logic.
- No plotting/recording logic.
"""

from __future__ import annotations

import time
from typing import Optional

from PyQt5 import QtCore
from PyQt5.QtCore import pyqtSignal, pyqtSlot

import serial
import serial.serialutil

from host_gui.core.protocol import (
    parse_line,
    TelemetryFrame,
    AckFrame,
    ReliableCmdAck,
    ReliableCmdRetry,
    ReliableCmdFail,
    ReliableCmdBusyWarn,
    RawLine,
)


class SerialWorker(QtCore.QObject):
    telemetry = pyqtSignal(object)   # TelemetryFrame
    ack = pyqtSignal(object)         # AckFrame
    cmd_ack = pyqtSignal(object)     # ReliableCmdAck
    cmd_retry = pyqtSignal(object)   # ReliableCmdRetry
    cmd_fail = pyqtSignal(object)    # ReliableCmdFail
    cmd_busy = pyqtSignal(object)    # ReliableCmdBusyWarn
    log_line = pyqtSignal(str)
    status_msg = pyqtSignal(str)
    connected = pyqtSignal(bool)

    def __init__(self, port: str, baud: int = 115200, parent=None):
        super().__init__(parent)
        self.port_name = port
        self.baud = int(baud)
        self._running = False
        self.ser: Optional[serial.Serial] = None

    @pyqtSlot()
    def run(self):
        try:
            self.ser = serial.Serial(
                self.port_name,
                self.baud,
                timeout=0.2,
                write_timeout=0.2,
            )
            try:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
            except Exception:
                pass
            self.status_msg.emit(f"已连接串口 {self.port_name} @ {self.baud}")
            self.connected.emit(True)
        except serial.serialutil.SerialException as e:
            self.status_msg.emit(f"串口打开失败: {e}")
            self.connected.emit(False)
            return

        self._running = True
        while self._running:
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                parsed = parse_line(line)
                if parsed is None:
                    continue

                if isinstance(parsed, TelemetryFrame):
                    self.telemetry.emit(parsed)
                    continue
                if isinstance(parsed, AckFrame):
                    self.ack.emit(parsed)
                    self.log_line.emit(line)
                    continue
                if isinstance(parsed, ReliableCmdAck):
                    self.cmd_ack.emit(parsed)
                    self.log_line.emit(line)
                    continue
                if isinstance(parsed, ReliableCmdRetry):
                    self.cmd_retry.emit(parsed)
                    self.log_line.emit(line)
                    continue
                if isinstance(parsed, ReliableCmdFail):
                    self.cmd_fail.emit(parsed)
                    self.log_line.emit(line)
                    continue
                if isinstance(parsed, ReliableCmdBusyWarn):
                    self.cmd_busy.emit(parsed)
                    self.log_line.emit(line)
                    continue

                # RawLine (unknown)
                if isinstance(parsed, RawLine):
                    self.log_line.emit(parsed.line)
                    continue

                # Fallback: stringify
                self.log_line.emit(str(parsed))

            except Exception as e:
                self.status_msg.emit(f"串口读取错误: {e}")
                time.sleep(0.3)

        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.connected.emit(False)
        self.status_msg.emit("串口已关闭")

    def stop(self):
        self._running = False

    @pyqtSlot(str)
    def send_line(self, text: str):
        if self.ser is None or not self.ser.is_open:
            self.status_msg.emit("串口未打开，无法发送")
            return
        text = (text or "").strip()
        if not text:
            return
        if not text.endswith("\n"):
            text += "\n"
        try:
            self.ser.write(text.encode("utf-8", errors="ignore"))
            self.ser.flush()
        except Exception as e:
            self.status_msg.emit(f"发送失败: {e}")

    # Convenience command slots
    @pyqtSlot(str)
    def set_mode(self, mode: str):
        self.send_line(f"mode {mode}")

    @pyqtSlot(int)
    def set_heater(self, pct: int):
        pct = max(0, min(100, int(pct)))
        self.send_line(f"set heater {pct}")

    @pyqtSlot(int)
    def set_valve(self, pct: int):
        pct = max(0, min(100, int(pct)))
        self.send_line(f"set valve {pct}")

    @pyqtSlot()
    def lora_stat(self):
        self.send_line("lora stat")

    @pyqtSlot()
    def lora_ping(self):
        self.send_line("lora ping")

    @pyqtSlot(bool)
    def lora_raw(self, on: bool):
        self.send_line(f"lora raw {'on' if on else 'off'}")
