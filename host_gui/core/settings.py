# -*- coding: utf-8 -*-
"""Persistent user settings.

We use Qt's QSettings (registry on Windows, ini on Linux/macOS) so the GUI
remembers last-used options between runs.

This module is intentionally independent from the GUI widgets.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from PyQt5 import QtCore

from host_gui.core.filtering import FilterConfig, FilterMode, DisplayMode
from host_gui import config


@dataclass
class AppSettings:
    last_port: str = config.DEFAULT_PORT
    baud: int = config.DEFAULT_BAUD
    save_dir: str = ""

    # plot window
    plot_window_mode: str = config.DEFAULT_PLOT_WINDOW_MODE
    plot_window_points: int = config.DEFAULT_PLOT_WINDOW_POINTS
    plot_window_seconds: int = config.DEFAULT_PLOT_WINDOW_SECONDS

    # filter
    # dataclasses disallow mutable defaults. Use default_factory so each
    # AppSettings instance gets its own FilterConfig object.
    filter_config: FilterConfig = field(default_factory=FilterConfig)
    display_mode: str = DisplayMode.BOTH

    # behavior
    wait_ack_gate: bool = True
    show_log: bool = False


class SettingsStore:
    """Thin wrapper around QSettings with explicit schema."""

    ORG = "LH2-UAV"
    APP = "GroundStationGUI"

    def __init__(self):
        self._q = QtCore.QSettings(self.ORG, self.APP)

    def load(self) -> AppSettings:
        s = AppSettings()

        s.last_port = self._q.value("serial/last_port", s.last_port, type=str)
        s.baud = self._q.value("serial/baud", s.baud, type=int)
        s.save_dir = self._q.value("io/save_dir", s.save_dir, type=str)

        s.plot_window_mode = self._q.value("plot/window_mode", s.plot_window_mode, type=str)
        s.plot_window_points = self._q.value("plot/window_points", s.plot_window_points, type=int)
        s.plot_window_seconds = self._q.value("plot/window_seconds", s.plot_window_seconds, type=int)

        mode = self._q.value("filter/mode", s.filter_config.mode, type=str)
        alpha = self._q.value("filter/alpha", s.filter_config.alpha, type=float)
        window_n = self._q.value("filter/window_n", s.filter_config.window_n, type=int)
        # Validate minimally
        if mode not in (FilterMode.NONE, FilterMode.EMA, FilterMode.SMA, FilterMode.MEDIAN, FilterMode.MEDIAN_EMA):
            mode = FilterMode.EMA
        s.filter_config = FilterConfig(mode=mode, alpha=float(alpha), window_n=int(window_n))

        s.display_mode = self._q.value("plot/display_mode", s.display_mode, type=str)
        s.wait_ack_gate = self._q.value("cmd/wait_ack_gate", s.wait_ack_gate, type=bool)
        s.show_log = self._q.value("ui/show_log", s.show_log, type=bool)

        return s

    def save(self, s: AppSettings) -> None:
        self._q.setValue("serial/last_port", s.last_port)
        self._q.setValue("serial/baud", int(s.baud))
        self._q.setValue("io/save_dir", s.save_dir)

        self._q.setValue("plot/window_mode", s.plot_window_mode)
        self._q.setValue("plot/window_points", int(s.plot_window_points))
        self._q.setValue("plot/window_seconds", int(s.plot_window_seconds))

        self._q.setValue("filter/mode", s.filter_config.mode)
        self._q.setValue("filter/alpha", float(s.filter_config.alpha))
        self._q.setValue("filter/window_n", int(s.filter_config.window_n))

        self._q.setValue("plot/display_mode", s.display_mode)
        self._q.setValue("cmd/wait_ack_gate", bool(s.wait_ack_gate))
        self._q.setValue("ui/show_log", bool(s.show_log))

        self._q.sync()
