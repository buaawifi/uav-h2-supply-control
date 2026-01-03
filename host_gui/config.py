# -*- coding: utf-8 -*-
"""Host GUI configuration defaults.

This module intentionally contains only *stable* defaults.
Runtime settings (last port, save directory, filter config, etc.) are persisted
via QSettings in :mod:`host_gui.core.settings`.
"""

from __future__ import annotations

APP_NAME = "供氢系统实验上位机（GroundGateway）"
APP_VERSION = "v11"

DEFAULT_BAUD = 115200
DEFAULT_PORT = "COM8"

# Right-side control panel font size. (Plot fonts are managed by pyqtgraph.)
UI_SIDE_FONT_PT = 9

# Plot refresh interval (ms)
PLOT_REFRESH_MS = 200

# Plot window defaults
DEFAULT_PLOT_WINDOW_MODE = "points"   # "points" | "seconds"
DEFAULT_PLOT_WINDOW_POINTS = 5000
DEFAULT_PLOT_WINDOW_SECONDS = 1000

# Data/log limits
MAX_LOG_LINES = 4000

