# -*- coding: utf-8 -*-
"""Protocol parsing for GroundGateway serial output.

The GUI parses *text* lines emitted from the GroundGateway serial console.
Those lines encapsulate telemetry and command reliability state.

If the firmware output format changes, only this module should need edits.

Supported line types
--------------------
- Telemetry:
    ``[TELEM] t=1234 T0=20.5 T1=20.6 P(Pa)=101.3 heater=%=0.0 valve=%=0.0``
- Simple ACK:
    ``[ACK] for=0x12 status=0``
- Reliable-downlink status lines:
    - ACK:   ``[CMD] ACK received for msg=0x12 seq=7 status=0``
    - RETRY: ``[CMD] RETRY #2 msg=0x12 seq=7``
    - FAIL:  ``[CMD] FAIL: no ACK for msg=0x12 seq=7``
    - BUSY:  ``[CMD] WARNING: LoRa TX busy > 2.0s ...``

Unknown lines are wrapped as :class:`RawLine`.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Optional, Union


_FLOAT = r"(?i:nan|inf|-inf|[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)"

RE_TELEM = re.compile(
    rf"^\[TELEM\]\s+t=(\d+)\s+T0=({_FLOAT})\s+T1=({_FLOAT})\s+P\(Pa\)=({_FLOAT})\s+heater=%=({_FLOAT})\s+valve=%=({_FLOAT})\s*$"
)
RE_ACK = re.compile(r"^\[ACK\]\s+for=0x([0-9a-fA-F]+)\s+status=([-+]?\d+)\s*$")
RE_CMD_ACK = re.compile(r"^\[CMD\]\s+ACK received for msg=0x([0-9a-fA-F]+)\s+seq=(\d+)\s+status=([-+]?\d+)\s*$")
RE_CMD_RETRY = re.compile(r"^\[CMD\]\s+RETRY #(?P<retry>\d+)\s+msg=0x([0-9a-fA-F]+)\s+seq=(\d+)\s*$")
RE_CMD_FAIL = re.compile(r"^\[CMD\]\s+FAIL:\s+no ACK for msg=0x([0-9a-fA-F]+)\s+seq=(\d+)\s*$")
RE_CMD_WARN_BUSY = re.compile(r"^\[CMD\]\s+WARNING:\s+LoRa TX busy >\s*([0-9\.]+)s.*$")


@dataclass(frozen=True)
class TelemetryFrame:
    t_ms: int
    t0_c: float
    t1_c: float
    p_pa: float
    heater_pct: float
    valve_pct: float

    @property
    def p_kpa(self) -> float:
        return self.p_pa / 1000.0


@dataclass(frozen=True)
class AckFrame:
    msg_type: int
    status: int


@dataclass(frozen=True)
class ReliableCmdAck:
    msg_type: int
    seq: int
    status: int


@dataclass(frozen=True)
class ReliableCmdRetry:
    msg_type: int
    seq: int
    retry_n: int


@dataclass(frozen=True)
class ReliableCmdFail:
    msg_type: int
    seq: int


@dataclass(frozen=True)
class ReliableCmdBusyWarn:
    busy_s: float


@dataclass(frozen=True)
class RawLine:
    line: str


ParsedLine = Union[
    TelemetryFrame,
    AckFrame,
    ReliableCmdAck,
    ReliableCmdRetry,
    ReliableCmdFail,
    ReliableCmdBusyWarn,
    RawLine,
]


def _safe_float(s: str) -> float:
    try:
        return float(s)
    except Exception:
        return float("nan")


def is_finite(x: float) -> bool:
    try:
        return math.isfinite(float(x))
    except Exception:
        return False


def parse_line(line: str) -> Optional[ParsedLine]:
    """Parse a single text line.

    Returns:
        ParsedLine when recognized; RawLine for unknown non-empty lines; None for empty lines.
    """
    if line is None:
        return None
    text = line.strip()
    if not text:
        return None

    m = RE_TELEM.match(text)
    if m:
        return TelemetryFrame(
            t_ms=int(m.group(1)),
            t0_c=_safe_float(m.group(2)),
            t1_c=_safe_float(m.group(3)),
            p_pa=_safe_float(m.group(4)),
            heater_pct=_safe_float(m.group(5)),
            valve_pct=_safe_float(m.group(6)),
        )

    m = RE_ACK.match(text)
    if m:
        return AckFrame(msg_type=int(m.group(1), 16), status=int(m.group(2)))

    m = RE_CMD_ACK.match(text)
    if m:
        return ReliableCmdAck(msg_type=int(m.group(1), 16), seq=int(m.group(2)), status=int(m.group(3)))

    m = RE_CMD_RETRY.match(text)
    if m:
        return ReliableCmdRetry(
            retry_n=int(m.group("retry")),
            msg_type=int(m.group(2), 16),
            seq=int(m.group(3)),
        )

    m = RE_CMD_FAIL.match(text)
    if m:
        return ReliableCmdFail(msg_type=int(m.group(1), 16), seq=int(m.group(2)))

    m = RE_CMD_WARN_BUSY.match(text)
    if m:
        return ReliableCmdBusyWarn(busy_s=_safe_float(m.group(1)))

    return RawLine(line=text)
