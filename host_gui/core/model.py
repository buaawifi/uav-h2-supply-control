# -*- coding: utf-8 -*-
"""In-memory data model for plotting and recording.

This layer contains no Qt widgets, so it can be tested in isolation.
"""

from __future__ import annotations

import os
import csv
import time
from dataclasses import dataclass
from datetime import datetime
from collections import deque
from typing import Deque, List, Optional, Tuple

from host_gui.core.protocol import TelemetryFrame
from host_gui.core.filtering import FilterConfig, FilterEngine


@dataclass
class CurrentValues:
    t0_c: float = float('nan')
    t1_c: float = float('nan')
    p_kpa: float = float('nan')
    heater_pct: float = float('nan')
    valve_pct: float = float('nan')
    telem_ms: Optional[int] = None


class TelemetryStore:
    """Holds both (a) plot window buffers and (b) full recorded history."""

    def __init__(
        self,
        plot_window_mode: str = "points",
        plot_points: int = 5000,
        plot_seconds: int = 1000,
        filter_config: Optional[FilterConfig] = None,
    ):
        self._t0_epoch = time.time()

        # set by UI (persisted via SettingsStore)
        self.save_dir: str = ""

        self.rec_enabled: bool = True
        self.current = CurrentValues()

        # FilterEngine expects a FilterConfig object.
        self.filter_engine = FilterEngine(config=filter_config)

        self.plot_window_mode = plot_window_mode  # "points" | "seconds"
        self.plot_points = int(plot_points)
        self.plot_seconds = int(plot_seconds)

        # plot buffers
        self._init_plot_buffers()

        # full recorded history
        self.rec_time_s: List[float] = []
        self.rec_telem_ms: List[int] = []
        self.rec_temp0_raw: List[float] = []
        self.rec_temp1_raw: List[float] = []
        self.rec_press_kpa_raw: List[float] = []
        self.rec_heater_pct: List[float] = []
        self.rec_valve_pct: List[float] = []
        self.rec_temp0_f: List[float] = []
        self.rec_temp1_f: List[float] = []
        self.rec_press_kpa_f: List[float] = []

    # --- UI-facing aliases (stable names) ---
    @property
    def plot_window_points(self) -> int:
        """Points mode window length (alias for UI compatibility)."""
        return int(self.plot_points)

    @property
    def plot_window_seconds(self) -> int:
        """Seconds mode window length (alias for UI compatibility)."""
        return int(self.plot_seconds)

    # ----- plot window management -----
    def _init_plot_buffers(self):
        if self.plot_window_mode == "points":
            maxlen = max(1, int(self.plot_points))
            self.time_s: Deque[float] = deque(maxlen=maxlen)
            self.temp0_raw: Deque[float] = deque(maxlen=maxlen)
            self.temp1_raw: Deque[float] = deque(maxlen=maxlen)
            self.press_kpa_raw: Deque[float] = deque(maxlen=maxlen)
            self.temp0_f: Deque[float] = deque(maxlen=maxlen)
            self.temp1_f: Deque[float] = deque(maxlen=maxlen)
            self.press_kpa_f: Deque[float] = deque(maxlen=maxlen)
        else:
            # seconds mode: unbounded, trimmed by time
            self.time_s = deque()
            self.temp0_raw = deque()
            self.temp1_raw = deque()
            self.press_kpa_raw = deque()
            self.temp0_f = deque()
            self.temp1_f = deque()
            self.press_kpa_f = deque()

    def set_plot_window_points(self, n_points: int) -> None:
        self.plot_window_mode = "points"
        self.plot_points = max(1, int(n_points))
        # rebuild with last points
        t = list(self.time_s)[-self.plot_points :]
        a0 = list(self.temp0_raw)[-self.plot_points :]
        a1 = list(self.temp1_raw)[-self.plot_points :]
        ap = list(self.press_kpa_raw)[-self.plot_points :]
        f0 = list(self.temp0_f)[-self.plot_points :]
        f1 = list(self.temp1_f)[-self.plot_points :]
        fp = list(self.press_kpa_f)[-self.plot_points :]
        self._init_plot_buffers()
        self.time_s.extend(t)
        self.temp0_raw.extend(a0)
        self.temp1_raw.extend(a1)
        self.press_kpa_raw.extend(ap)
        self.temp0_f.extend(f0)
        self.temp1_f.extend(f1)
        self.press_kpa_f.extend(fp)

    def set_plot_window_seconds(self, window_s: int) -> None:
        self.plot_window_mode = "seconds"
        self.plot_seconds = max(1, int(window_s))
        # keep existing samples, then trim
        self._init_plot_buffers()
        # no special action; trimming occurs on new samples, but we trim now too
        if self.time_s:
            self._trim_plot_buffers_by_time(self.time_s[-1] - self.plot_seconds)

    def _trim_plot_buffers_by_time(self, tmin: float) -> None:
        while self.time_s and self.time_s[0] < tmin:
            self.time_s.popleft()
            self.temp0_raw.popleft()
            self.temp1_raw.popleft()
            self.press_kpa_raw.popleft()
            self.temp0_f.popleft()
            self.temp1_f.popleft()
            self.press_kpa_f.popleft()

    # ----- data ingestion -----
    def add_telem(self, frame: TelemetryFrame) -> None:
        now_s = time.time() - self._t0_epoch
        p_kpa = frame.p_kpa

        # plot buffers
        self.time_s.append(now_s)
        self.temp0_raw.append(frame.t0_c)
        self.temp1_raw.append(frame.t1_c)
        self.press_kpa_raw.append(p_kpa)

        y0, y1, yp = self.filter_engine.update(frame.t0_c, frame.t1_c, p_kpa)
        self.temp0_f.append(y0)
        self.temp1_f.append(y1)
        self.press_kpa_f.append(yp)

        if self.plot_window_mode == "seconds":
            self._trim_plot_buffers_by_time(now_s - float(self.plot_seconds))

        # recorded history
        if self.rec_enabled:
            self.rec_time_s.append(now_s)
            self.rec_telem_ms.append(frame.t_ms)
            self.rec_temp0_raw.append(frame.t0_c)
            self.rec_temp1_raw.append(frame.t1_c)
            self.rec_press_kpa_raw.append(p_kpa)
            self.rec_heater_pct.append(frame.heater_pct)
            self.rec_valve_pct.append(frame.valve_pct)
            self.rec_temp0_f.append(y0)
            self.rec_temp1_f.append(y1)
            self.rec_press_kpa_f.append(yp)

        # current values
        self.current = CurrentValues(
            t0_c=frame.t0_c,
            t1_c=frame.t1_c,
            p_kpa=p_kpa,
            heater_pct=frame.heater_pct,
            valve_pct=frame.valve_pct,
            telem_ms=frame.t_ms,
        )

    # ----- filter management -----
    def set_filter_config(self, cfg: FilterConfig, recompute_plot_series: bool = True) -> None:
        """Update filter settings.

        The UI calls this with keyword argument name 'recompute_plot_series'.
        """
        self.filter_engine.set_config(cfg)
        if recompute_plot_series:
            self.recompute_plot_filtered()

    def recompute_plot_filtered(self) -> None:
        t0_list = list(self.temp0_raw)
        t1_list = list(self.temp1_raw)
        p_list = list(self.press_kpa_raw)

        out_t0, out_t1, out_p = self.filter_engine.recompute_series(t0_list, t1_list, p_list)
        self.temp0_f.clear(); self.temp1_f.clear(); self.press_kpa_f.clear()
        self.temp0_f.extend(out_t0)
        self.temp1_f.extend(out_t1)
        self.press_kpa_f.extend(out_p)

    # ----- recording / saving -----
    def toggle_recording(self) -> bool:
        self.rec_enabled = not bool(self.rec_enabled)
        return self.rec_enabled

    def clear(self) -> None:
        self._t0_epoch = time.time()
        self.filter_engine.reset()

        # clear recorded
        self.rec_time_s.clear()
        self.rec_telem_ms.clear()
        self.rec_temp0_raw.clear()
        self.rec_temp1_raw.clear()
        self.rec_press_kpa_raw.clear()
        self.rec_heater_pct.clear()
        self.rec_valve_pct.clear()
        self.rec_temp0_f.clear()
        self.rec_temp1_f.clear()
        self.rec_press_kpa_f.clear()

        # clear plot
        self.time_s.clear()
        self.temp0_raw.clear()
        self.temp1_raw.clear()
        self.press_kpa_raw.clear()
        self.temp0_f.clear()
        self.temp1_f.clear()
        self.press_kpa_f.clear()

        self.current = CurrentValues()

    # ---- UI compatibility helpers ----
    def get_plot_series(self):
        return self.plot_series()

    def record_len(self) -> int:
        return self.rec_points()

    def recording_info(self):
        return self.rec_points(), self.rec_duration_s()

    def rec_points(self) -> int:
        return len(self.rec_time_s)

    def rec_duration_s(self) -> float:
        n = len(self.rec_time_s)
        if n >= 2:
            return float(self.rec_time_s[-1]) - float(self.rec_time_s[0])
        return 0.0

    def default_csv_name(self) -> str:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        return f"h2_telem_{ts}.csv"

    def write_csv(self, filename: str, filter_cfg: FilterConfig) -> None:
        """Write recorded history to CSV.

        Notes:
            - The recorded filtered columns reflect the filter config *at sampling time*.
            - The provided filter_cfg is stored alongside each row to document the current GUI config.
        """
        os.makedirs(os.path.dirname(os.path.abspath(filename)), exist_ok=True)

        header = [
            "elapsed_s",
            "telem_ms",
            "T0_C_raw",
            "T1_C_raw",
            "P_kPa_raw",
            "heater_pct",
            "valve_pct",
            "T0_C_filt",
            "T1_C_filt",
            "P_kPa_filt",
            "filter_mode",
            "alpha",
            "window_n",
        ]

        with open(filename, "w", encoding="utf-8-sig", newline="") as f:
            w = csv.writer(f)
            w.writerow(header)
            for i in range(len(self.rec_time_s)):
                w.writerow(
                    [
                        self.rec_time_s[i],
                        self.rec_telem_ms[i],
                        self.rec_temp0_raw[i],
                        self.rec_temp1_raw[i],
                        self.rec_press_kpa_raw[i],
                        self.rec_heater_pct[i],
                        self.rec_valve_pct[i],
                        self.rec_temp0_f[i],
                        self.rec_temp1_f[i],
                        self.rec_press_kpa_f[i],
                        filter_cfg.mode,
                        float(filter_cfg.alpha),
                        int(filter_cfg.window_n),
                    ]
                )

    # ----- plotting access -----
    def plot_series(self) -> Tuple[List[float], List[float], List[float], List[float], List[float], List[float], List[float]]:
        """Return current plot buffers as lists."""
        t = list(self.time_s)
        return (
            t,
            list(self.temp0_raw),
            list(self.temp1_raw),
            list(self.press_kpa_raw),
            list(self.temp0_f),
            list(self.temp1_f),
            list(self.press_kpa_f),
        )
