# -*- coding: utf-8 -*-
"""Application entry point."""

from __future__ import annotations

import sys

from PyQt5 import QtCore, QtWidgets

from host_gui.ui.main_window import MainWindow
from host_gui.utils.logging_setup import setup_logging


def main(argv=None) -> int:
    argv = list(sys.argv if argv is None else argv)

    app = QtWidgets.QApplication(argv)

    # Provide stable identity for QSettings / QStandardPaths.
    QtCore.QCoreApplication.setOrganizationName("LH2-UAV")
    QtCore.QCoreApplication.setApplicationName("GroundStationGUI")

    # Rotating log file for field debugging.
    setup_logging()

    w = MainWindow()
    w.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
