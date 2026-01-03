# -*- coding: utf-8 -*-
"""Filtering (causal) for telemetry channels.

Design goals
-----------
- Keep filter state independent per channel.
- Allow re-computation for existing history when filter configuration changes.

Implemented filters
-------------------
- None
- EMA (exponential moving average)
- SMA (simple moving average)
- Median
- Median+EMA (median filter followed by EMA)
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from collections import deque
from typing import Deque, List, Tuple


class FilterMode:
    NONE = "None"
    EMA = "EMA"
    SMA = "SMA"
    MEDIAN = "Median"
    MEDIAN_EMA = "Median+EMA"


class DisplayMode:
    """Plot display mode for raw/filtered curves."""
    RAW = "仅原始"
    FILT = "仅滤波"
    BOTH = "原始+滤波"


@dataclass
class FilterConfig:
    mode: str = FilterMode.EMA
    alpha: float = 0.20
    window_n: int = 9


def _median(values: List[float]) -> float:
    s = sorted(values)
    n = len(s)
    if n == 0:
        return float("nan")
    mid = n // 2
    if n % 2 == 1:
        return float(s[mid])
    return 0.5 * (float(s[mid - 1]) + float(s[mid]))


def _is_bad_number(x: float) -> bool:
    try:
        return not math.isfinite(float(x))
    except Exception:
        return True


class ChannelFilterState:
    """Per-channel causal filter state."""

    def __init__(self):
        self.ema_y = None
        self.win: Deque[float] = deque()
        self.win_sum = 0.0
        self.med_win: Deque[float] = deque()  # for Median+EMA

    def reset(self):
        self.ema_y = None
        self.win.clear()
        self.win_sum = 0.0
        self.med_win.clear()


class FilterEngine:
    """Applies the same filter configuration to each channel independently."""

    def __init__(self, config: FilterConfig | None = None):
        self.config = config or FilterConfig()
        self.t0 = ChannelFilterState()
        self.t1 = ChannelFilterState()
        self.p = ChannelFilterState()

    def set_config(self, config: FilterConfig):
        self.config = config
        self.reset()

    def reset(self):
        self.t0.reset()
        self.t1.reset()
        self.p.reset()

    def _update_ema(self, st: ChannelFilterState, x: float) -> float:
        if _is_bad_number(x):
            return float("nan")
        a = float(self.config.alpha)
        if st.ema_y is None:
            st.ema_y = float(x)
        else:
            st.ema_y = a * float(x) + (1.0 - a) * st.ema_y
        return float(st.ema_y)

    def _update_sma(self, st: ChannelFilterState, x: float) -> float:
        if _is_bad_number(x):
            return float("nan")
        n = max(1, int(self.config.window_n))
        st.win.append(float(x))
        st.win_sum += float(x)
        if len(st.win) > n:
            st.win_sum -= st.win.popleft()
        return st.win_sum / float(len(st.win))

    def _update_median(self, st: ChannelFilterState, x: float) -> float:
        if _is_bad_number(x):
            return float("nan")
        n = max(1, int(self.config.window_n))
        st.win.append(float(x))
        if len(st.win) > n:
            st.win.popleft()
        return _median(list(st.win))

    def _update_median_ema(self, st: ChannelFilterState, x: float) -> float:
        if _is_bad_number(x):
            return float("nan")
        n = max(1, int(self.config.window_n))
        st.med_win.append(float(x))
        if len(st.med_win) > n:
            st.med_win.popleft()
        med = _median(list(st.med_win))
        a = float(self.config.alpha)
        if st.ema_y is None:
            st.ema_y = float(med)
        else:
            st.ema_y = a * float(med) + (1.0 - a) * st.ema_y
        return float(st.ema_y)

    def update(self, x_t0: float, x_t1: float, x_p: float) -> Tuple[float, float, float]:
        """Update filter state with one new sample."""
        m = self.config.mode
        if m == FilterMode.NONE:
            return float(x_t0), float(x_t1), float(x_p)
        if m == FilterMode.EMA:
            return (
                self._update_ema(self.t0, x_t0),
                self._update_ema(self.t1, x_t1),
                self._update_ema(self.p, x_p),
            )
        if m == FilterMode.SMA:
            return (
                self._update_sma(self.t0, x_t0),
                self._update_sma(self.t1, x_t1),
                self._update_sma(self.p, x_p),
            )
        if m == FilterMode.MEDIAN:
            return (
                self._update_median(self.t0, x_t0),
                self._update_median(self.t1, x_t1),
                self._update_median(self.p, x_p),
            )
        if m == FilterMode.MEDIAN_EMA:
            return (
                self._update_median_ema(self.t0, x_t0),
                self._update_median_ema(self.t1, x_t1),
                self._update_median_ema(self.p, x_p),
            )
        # fallback
        return float(x_t0), float(x_t1), float(x_p)

    def recompute_series(
        self,
        t0_raw: List[float],
        t1_raw: List[float],
        p_raw: List[float],
    ) -> Tuple[List[float], List[float], List[float]]:
        """Recompute filtered arrays from scratch (when filter config changes)."""
        self.reset()
        out_t0, out_t1, out_p = [], [], []
        for a, b, c in zip(t0_raw, t1_raw, p_raw):
            y0, y1, yp = self.update(a, b, c)
            out_t0.append(y0)
            out_t1.append(y1)
            out_p.append(yp)
        return out_t0, out_t1, out_p
