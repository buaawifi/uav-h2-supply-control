# -*- coding: utf-8 -*-
"""Logging setup.

Creates a per-user rotating log file useful for field debugging.

The log location uses :class:`QStandardPaths.AppDataLocation` so it is
consistent across Windows/Linux/macOS.
"""

from __future__ import annotations

import logging
import os
from logging.handlers import RotatingFileHandler

from PyQt5 import QtCore


def setup_logging(level: int = logging.INFO) -> str:
    """Configure root logging to a rotating file and console.

    Returns:
        The log file path.
    """
    base = QtCore.QStandardPaths.writableLocation(QtCore.QStandardPaths.AppDataLocation)
    if not base:
        base = os.path.abspath(os.getcwd())

    log_dir = os.path.join(base, "logs")
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(log_dir, "host_gui.log")

    logger = logging.getLogger()
    logger.setLevel(level)

    # Avoid duplicate handlers if called multiple times.
    for h in logger.handlers:
        if isinstance(h, RotatingFileHandler):
            try:
                if os.path.abspath(getattr(h, "baseFilename", "")) == os.path.abspath(log_path):
                    return log_path
            except Exception:
                return log_path

    fmt = logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s")

    fh = RotatingFileHandler(log_path, maxBytes=2 * 1024 * 1024, backupCount=5, encoding="utf-8")
    fh.setFormatter(fmt)
    fh.setLevel(level)

    ch = logging.StreamHandler()
    ch.setFormatter(fmt)
    ch.setLevel(level)

    logger.addHandler(fh)
    logger.addHandler(ch)

    logging.getLogger(__name__).info("Logging initialized: %s", log_path)
    return log_path
