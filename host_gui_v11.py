# -*- coding: utf-8 -*-
"""供氢系统实验上位机（GroundGateway）v11

This script is a thin wrapper that launches the refactored, modular GUI.

Dependencies:
  pip install pyqt5 pyqtgraph pyserial

Run:
  python host_gui_v11.py
"""

from host_gui.app import main


if __name__ == "__main__":
    raise SystemExit(main())
