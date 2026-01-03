# -*- coding: utf-8 -*-
"""Main window implementation.

The UI layer composes widgets and binds them to:
- serial transport (SerialWorker)
- in-memory data model (TelemetryStore)
- persistence (SettingsStore)

This file intentionally avoids mixing protocol parsing and filtering logic.
"""

from __future__ import annotations

import os
import time
from typing import Optional

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import pyqtSlot, QThread

import serial.tools.list_ports
import pyqtgraph as pg

from host_gui import config
from host_gui.core.protocol import TelemetryFrame, AckFrame, ReliableCmdAck, ReliableCmdRetry, ReliableCmdFail, ReliableCmdBusyWarn
from host_gui.core.filtering import FilterMode, DisplayMode, FilterConfig
from host_gui.core.model import TelemetryStore
from host_gui.core.settings import SettingsStore, AppSettings
from host_gui.io.serial_worker import SerialWorker


pg.setConfigOptions(antialias=True)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, settings: Optional[AppSettings] = None, parent=None):
        super().__init__(parent)

        self._settings_store = SettingsStore()
        self._settings = settings or self._settings_store.load()

        self.setWindowTitle(f"{config.APP_NAME} {config.APP_VERSION}")
        self.resize(1550, 840)

        # domain model
        self.store = TelemetryStore(
            plot_window_mode=self._settings.plot_window_mode,
            plot_points=self._settings.plot_window_points,
            plot_seconds=self._settings.plot_window_seconds,
            filter_config=self._settings.filter_config,
        )

        # serial worker/thread
        self.thread: Optional[QThread] = None
        self.worker: Optional[SerialWorker] = None

        # reliable-command status
        self._cmd_pending = False
        self._cmd_desc = ""
        self._cmd_retry_n = 0
        self._cmd_last_seq = None
        self._cmd_last_msg = None

        self._build_ui()
        self._wire_signals()

        # periodic plot refresh
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._update_plots)
        self.timer.start(config.PLOT_REFRESH_MS)

        # apply initial UI state from settings
        self._apply_settings_to_ui(self._settings)

    # ---------------- UI construction ----------------

    def _build_ui(self):
        """Compose a professional, maintainable layout.

        Layout strategy (common in professional DAQ / control stations):
        - main plot area on the left (resizable)
        - right-side control panel with a fixed summary header + tabbed functions
        - user-adjustable splitter between plot and control panel
        """

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # Top toolbar (quick actions). Keep buttons in tabs as well for clarity.
        self._build_toolbar()

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        splitter.setChildrenCollapsible(False)
        root.addWidget(splitter, stretch=1)

        # Left: plots
        plot_wrap = QtWidgets.QWidget()
        plot_lay = QtWidgets.QVBoxLayout(plot_wrap)
        plot_lay.setContentsMargins(0, 0, 0, 0)
        plot_lay.setSpacing(8)
        splitter.addWidget(plot_wrap)

        self._plot_col = plot_lay
        self._build_plots()

        # Right: summary + tabs
        side_wrap = QtWidgets.QWidget()
        side_wrap.setMinimumWidth(460)
        side_wrap.setMaximumWidth(620)
        side_wrap.setStyleSheet(self._side_stylesheet())
        side_lay = QtWidgets.QVBoxLayout(side_wrap)
        side_lay.setContentsMargins(0, 0, 0, 0)
        side_lay.setSpacing(8)
        splitter.addWidget(side_wrap)

        # Summary always visible
        side_lay.addWidget(self._build_current_values_group())

        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setDocumentMode(True)
        self.tabs.setMovable(False)
        self.tabs.setTabPosition(QtWidgets.QTabWidget.North)
        side_lay.addWidget(self.tabs, stretch=1)

        # Tabs
        self._build_tabs()

        # reasonable default ratio
        splitter.setStretchFactor(0, 5)
        splitter.setStretchFactor(1, 2)

        self.statusBar().showMessage("未连接")

    def _side_stylesheet(self) -> str:
        """Lightweight QSS to make the UI look consistent and 'tool-like'."""
        pt = int(config.UI_SIDE_FONT_PT)
        return (
            f"QWidget{{font-size:{pt}pt;}}"
            "QGroupBox{border:1px solid #C9C9C9; border-radius:6px; margin-top:10px;}"
            "QGroupBox::title{subcontrol-origin: margin; left:10px; padding:0 4px; font-weight:600;}"
            "QPushButton{padding:4px 8px;}"
            "QLineEdit{padding:4px 6px;}"
            "QComboBox{padding:3px 6px;}"
            "QSpinBox{padding:2px 4px;}"
            "QDoubleSpinBox{padding:2px 4px;}"
        )

    def _build_toolbar(self):
        tb = QtWidgets.QToolBar("Main")
        tb.setMovable(False)
        tb.setFloatable(False)
        tb.setIconSize(QtCore.QSize(16, 16))
        self.addToolBar(tb)

        act_refresh = QtWidgets.QAction("刷新端口", self)
        act_connect = QtWidgets.QAction("连接", self)
        act_disconnect = QtWidgets.QAction("断开", self)
        act_disconnect.setEnabled(False)
        act_rec = QtWidgets.QAction("开始/停止记录", self)
        act_save = QtWidgets.QAction("保存CSV", self)
        act_clear = QtWidgets.QAction("清空数据", self)

        act_refresh.triggered.connect(self._refresh_ports)
        act_connect.triggered.connect(self._on_connect)
        act_disconnect.triggered.connect(self._on_disconnect)
        act_rec.triggered.connect(self._on_toggle_recording)
        act_save.triggered.connect(self._on_save_data)
        act_clear.triggered.connect(self._on_clear_data)

        tb.addAction(act_refresh)
        tb.addSeparator()
        tb.addAction(act_connect)
        tb.addAction(act_disconnect)
        tb.addSeparator()
        tb.addAction(act_rec)
        tb.addAction(act_save)
        tb.addAction(act_clear)

        # keep refs for enabling/disabling together with the buttons
        self._tb_act_connect = act_connect
        self._tb_act_disconnect = act_disconnect

    def _wrap_scroll(self, widget: QtWidgets.QWidget) -> QtWidgets.QScrollArea:
        sc = QtWidgets.QScrollArea()
        sc.setWidgetResizable(True)
        sc.setFrameShape(QtWidgets.QFrame.NoFrame)
        sc.setWidget(widget)
        return sc

    def _build_tabs(self):
        # Connection tab
        tab_conn = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(tab_conn)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(8)
        lay.addWidget(self._build_serial_group())
        lay.addWidget(self._build_lora_group())
        lay.addWidget(self._build_log_group(), stretch=1)
        lay.addStretch(1)
        self.tabs.addTab(self._wrap_scroll(tab_conn), "连接")

        # Control tab
        tab_ctrl = QtWidgets.QWidget()
        lay2 = QtWidgets.QVBoxLayout(tab_ctrl)
        lay2.setContentsMargins(8, 8, 8, 8)
        lay2.setSpacing(8)
        lay2.addWidget(self._build_control_group())
        lay2.addWidget(self._build_cmd_status_group())
        lay2.addStretch(1)
        self.tabs.addTab(self._wrap_scroll(tab_ctrl), "控制")

        # View tab
        tab_view = QtWidgets.QWidget()
        lay3 = QtWidgets.QVBoxLayout(tab_view)
        lay3.setContentsMargins(8, 8, 8, 8)
        lay3.setSpacing(8)
        lay3.addWidget(self._build_filter_group())
        lay3.addWidget(self._build_plot_window_group())
        lay3.addStretch(1)
        self.tabs.addTab(self._wrap_scroll(tab_view), "显示")

        # Data tab
        tab_data = QtWidgets.QWidget()
        lay4 = QtWidgets.QVBoxLayout(tab_data)
        lay4.setContentsMargins(8, 8, 8, 8)
        lay4.setSpacing(8)
        lay4.addWidget(self._build_record_group())
        lay4.addStretch(1)
        self.tabs.addTab(self._wrap_scroll(tab_data), "数据")

    def _build_plots(self):
        # temperature plot
        self.temp_plot = pg.PlotWidget(title="温度-时间（原始+滤波）")
        self.temp_plot.showGrid(x=True, y=True, alpha=0.3)
        self.temp_plot.addLegend()
        self.temp_plot.setLabel("bottom", "时间 (s)")
        self.temp_plot.setLabel("left", "温度 (°C)")

        pen_t0_raw = pg.mkPen('r', width=2, style=QtCore.Qt.DotLine)
        pen_t1_raw = pg.mkPen('g', width=2, style=QtCore.Qt.DotLine)
        pen_t0_f = pg.mkPen('r', width=3)
        pen_t1_f = pg.mkPen('g', width=3)

        self.curve_t0_raw = self.temp_plot.plot(pen=pen_t0_raw, name="T0 原始")
        self.curve_t1_raw = self.temp_plot.plot(pen=pen_t1_raw, name="T1 原始")
        self.curve_t0_f = self.temp_plot.plot(pen=pen_t0_f, name="T0 滤波")
        self.curve_t1_f = self.temp_plot.plot(pen=pen_t1_f, name="T1 滤波")

        # pressure plot
        self.press_plot = pg.PlotWidget(title="压力-时间（原始+滤波）")
        self.press_plot.showGrid(x=True, y=True, alpha=0.3)
        self.press_plot.addLegend()
        self.press_plot.setLabel("bottom", "时间 (s)")
        self.press_plot.setLabel("left", "压力 (kPa)")

        pen_p_raw = pg.mkPen('b', width=2, style=QtCore.Qt.DotLine)
        pen_p_f = pg.mkPen('b', width=3)

        self.curve_p_raw = self.press_plot.plot(pen=pen_p_raw, name="P 原始")
        self.curve_p_f = self.press_plot.plot(pen=pen_p_f, name="P 滤波")

        for c in (self.curve_t0_raw, self.curve_t1_raw, self.curve_t0_f, self.curve_t1_f, self.curve_p_raw, self.curve_p_f):
            try:
                c.setClipToView(True)
            except Exception:
                pass
            try:
                c.setDownsampling(auto=True, method="subsample")
            except Exception:
                pass

        self._plot_col.addWidget(self.temp_plot, stretch=3)
        self._plot_col.addWidget(self.press_plot, stretch=3)

    def _build_serial_group(self) -> QtWidgets.QGroupBox:
        g = QtWidgets.QGroupBox("串口连接")
        lay = QtWidgets.QGridLayout(g)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setHorizontalSpacing(6)
        lay.setVerticalSpacing(6)

        self.port_combo = QtWidgets.QComboBox()
        self.btn_refresh = QtWidgets.QPushButton("刷新")
        self.btn_connect = QtWidgets.QPushButton("连接")
        self.btn_disconnect = QtWidgets.QPushButton("断开")
        self.btn_disconnect.setEnabled(False)

        lay.addWidget(QtWidgets.QLabel("端口"), 0, 0)
        lay.addWidget(self.port_combo, 0, 1, 1, 3)
        lay.addWidget(self.btn_refresh, 1, 1)
        lay.addWidget(self.btn_connect, 1, 2)
        lay.addWidget(self.btn_disconnect, 1, 3)

        self._refresh_ports()
        return g

    def _build_current_values_group(self) -> QtWidgets.QGroupBox:
        g = QtWidgets.QGroupBox("当前数值")
        lay = QtWidgets.QGridLayout(g)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setHorizontalSpacing(8)
        lay.setVerticalSpacing(4)

        self.lbl_t0 = QtWidgets.QLabel("--")
        self.lbl_t1 = QtWidgets.QLabel("--")
        self.lbl_p_kpa = QtWidgets.QLabel("--")
        self.lbl_heater = QtWidgets.QLabel("--")
        self.lbl_valve = QtWidgets.QLabel("--")
        self.lbl_last = QtWidgets.QLabel("--")

        lay.addWidget(QtWidgets.QLabel("T0 (°C)"), 0, 0)
        lay.addWidget(self.lbl_t0, 0, 1)
        lay.addWidget(QtWidgets.QLabel("T1 (°C)"), 0, 2)
        lay.addWidget(self.lbl_t1, 0, 3)

        lay.addWidget(QtWidgets.QLabel("P (kPa)"), 1, 0)
        lay.addWidget(self.lbl_p_kpa, 1, 1)
        lay.addWidget(QtWidgets.QLabel("TELEM t(ms)"), 1, 2)
        lay.addWidget(self.lbl_last, 1, 3)

        lay.addWidget(QtWidgets.QLabel("加热器 (%)"), 2, 0)
        lay.addWidget(self.lbl_heater, 2, 1)
        lay.addWidget(QtWidgets.QLabel("电磁阀 (%)"), 2, 2)
        lay.addWidget(self.lbl_valve, 2, 3)

        return g

    def _build_filter_group(self) -> QtWidgets.QGroupBox:
        g = QtWidgets.QGroupBox("滤波设置（对 T0/T1/P 同时生效）")
        lay = QtWidgets.QGridLayout(g)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setHorizontalSpacing(6)
        lay.setVerticalSpacing(6)

        self.cb_filter = QtWidgets.QComboBox()
        self.cb_filter.addItems([FilterMode.NONE, FilterMode.EMA, FilterMode.SMA, FilterMode.MEDIAN, FilterMode.MEDIAN_EMA])

        self.sp_alpha = QtWidgets.QDoubleSpinBox()
        self.sp_alpha.setRange(0.01, 0.99)
        self.sp_alpha.setSingleStep(0.01)
        self.sp_alpha.setDecimals(2)

        self.sp_window = QtWidgets.QSpinBox()
        self.sp_window.setRange(1, 200)
        self.sp_window.setSingleStep(2)

        self.cb_view = QtWidgets.QComboBox()
        self.cb_view.addItems([DisplayMode.BOTH, DisplayMode.RAW, DisplayMode.FILT])

        self.btn_apply_filter = QtWidgets.QPushButton("应用")
        self.btn_apply_filter.setToolTip("更改滤波参数后点击应用（会从已有绘图数据重新计算滤波曲线）")

        lay.addWidget(QtWidgets.QLabel("方式"), 0, 0)
        lay.addWidget(self.cb_filter, 0, 1, 1, 3)
        lay.addWidget(QtWidgets.QLabel("alpha"), 1, 0)
        lay.addWidget(self.sp_alpha, 1, 1)
        lay.addWidget(QtWidgets.QLabel("窗口N"), 1, 2)
        lay.addWidget(self.sp_window, 1, 3)
        lay.addWidget(QtWidgets.QLabel("曲线显示"), 2, 0)
        lay.addWidget(self.cb_view, 2, 1, 1, 2)
        lay.addWidget(self.btn_apply_filter, 2, 3)

        return g

    def _build_control_group(self) -> QtWidgets.QGroupBox:
        g = QtWidgets.QGroupBox("控制")
        lay = QtWidgets.QGridLayout(g)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setHorizontalSpacing(6)
        lay.setVerticalSpacing(6)

        self.btn_safe = QtWidgets.QPushButton("SAFE")
        self.btn_manual = QtWidgets.QPushButton("MANUAL")
        self.btn_auto = QtWidgets.QPushButton("AUTO(预留)")

        lay.addWidget(QtWidgets.QLabel("Mode"), 0, 0)
        lay.addWidget(self.btn_safe, 0, 1)
        lay.addWidget(self.btn_manual, 0, 2)
        lay.addWidget(self.btn_auto, 0, 3)

        self.sl_heater = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.sl_heater.setRange(0, 100)
        self.sl_heater.setFixedHeight(18)
        self.lb_heater_set = QtWidgets.QLabel("0")
        self.lb_heater_set.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.btn_heater_apply = QtWidgets.QPushButton("发送")
        self.btn_heater_zero = QtWidgets.QPushButton("置0")
        self.btn_heater_full = QtWidgets.QPushButton("置100")
        self.btn_heater_zero.setFixedWidth(52)
        self.btn_heater_full.setFixedWidth(60)

        lay.addWidget(QtWidgets.QLabel("加热器"), 1, 0)
        lay.addWidget(self.sl_heater, 1, 1, 1, 2)
        lay.addWidget(self.lb_heater_set, 1, 3)
        lay.addWidget(self.btn_heater_apply, 2, 1)
        h = QtWidgets.QHBoxLayout()
        h.setSpacing(6)
        h.addWidget(self.btn_heater_zero)
        h.addWidget(self.btn_heater_full)
        lay.addLayout(h, 2, 2, 1, 2)

        self.sl_valve = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.sl_valve.setRange(0, 100)
        self.sl_valve.setFixedHeight(18)
        self.lb_valve_set = QtWidgets.QLabel("0")
        self.lb_valve_set.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.btn_valve_apply = QtWidgets.QPushButton("发送")
        self.btn_valve_zero = QtWidgets.QPushButton("置0")
        self.btn_valve_full = QtWidgets.QPushButton("置100")
        self.btn_valve_zero.setFixedWidth(52)
        self.btn_valve_full.setFixedWidth(60)

        lay.addWidget(QtWidgets.QLabel("电磁阀"), 3, 0)
        lay.addWidget(self.sl_valve, 3, 1, 1, 2)
        lay.addWidget(self.lb_valve_set, 3, 3)
        lay.addWidget(self.btn_valve_apply, 4, 1)
        h2 = QtWidgets.QHBoxLayout()
        h2.setSpacing(6)
        h2.addWidget(self.btn_valve_zero)
        h2.addWidget(self.btn_valve_full)
        lay.addLayout(h2, 4, 2, 1, 2)

        return g

    def _build_cmd_status_group(self) -> QtWidgets.QGroupBox:
        g = QtWidgets.QGroupBox("命令状态（可靠下行）")
        lay = QtWidgets.QGridLayout(g)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setHorizontalSpacing(8)
        lay.setVerticalSpacing(4)

        self.lbl_cmd_desc = QtWidgets.QLabel("--")
        self.lbl_cmd_state = QtWidgets.QLabel("--")
        self.lbl_cmd_seq = QtWidgets.QLabel("--")
        self.lbl_cmd_retry = QtWidgets.QLabel("0")
        self.lbl_cmd_result = QtWidgets.QLabel("--")

        self.chk_wait_ack = QtWidgets.QCheckBox("等待 ACK 后再允许发送")
        self.chk_wait_ack.setChecked(True)

        lay.addWidget(QtWidgets.QLabel("最近命令"), 0, 0)
        lay.addWidget(self.lbl_cmd_desc, 0, 1, 1, 3)

        lay.addWidget(QtWidgets.QLabel("状态"), 1, 0)
        lay.addWidget(self.lbl_cmd_state, 1, 1)
        lay.addWidget(QtWidgets.QLabel("seq"), 1, 2)
        lay.addWidget(self.lbl_cmd_seq, 1, 3)

        lay.addWidget(QtWidgets.QLabel("重试"), 2, 0)
        lay.addWidget(self.lbl_cmd_retry, 2, 1)
        lay.addWidget(QtWidgets.QLabel("结果"), 2, 2)
        lay.addWidget(self.lbl_cmd_result, 2, 3)

        lay.addWidget(self.chk_wait_ack, 3, 0, 1, 4)

        return g

    def _build_plot_window_group(self) -> QtWidgets.QGroupBox:
        g = QtWidgets.QGroupBox("绘图窗口")
        lay = QtWidgets.QGridLayout(g)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setHorizontalSpacing(8)
        lay.setVerticalSpacing(6)

        self.cb_plotwin_unit = QtWidgets.QComboBox()
        self.cb_plotwin_unit.addItems(["最近 N 点", "最近 N 秒"])

        self.sp_plotwin_val = QtWidgets.QSpinBox()
        self.btn_plotwin_apply = QtWidgets.QPushButton("应用")
        self.lbl_plotwin_hint = QtWidgets.QLabel("修改后点击“应用”生效")

        lay.addWidget(QtWidgets.QLabel("窗口类型"), 0, 0)
        lay.addWidget(self.cb_plotwin_unit, 0, 1, 1, 2)
        lay.addWidget(self.btn_plotwin_apply, 0, 3)

        lay.addWidget(QtWidgets.QLabel("窗口大小"), 1, 0)
        lay.addWidget(self.sp_plotwin_val, 1, 1, 1, 3)

        lay.addWidget(self.lbl_plotwin_hint, 2, 0, 1, 4)

        return g

    def _build_record_group(self) -> QtWidgets.QGroupBox:
        g = QtWidgets.QGroupBox("记录 / 保存")
        lay = QtWidgets.QGridLayout(g)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setHorizontalSpacing(8)
        lay.setVerticalSpacing(6)

        self.btn_rec_toggle = QtWidgets.QPushButton("停止记录")
        self.btn_clear_data = QtWidgets.QPushButton("清空数据")

        self.lbl_rec_points = QtWidgets.QLabel("0")
        self.lbl_rec_dur = QtWidgets.QLabel("0.0")

        self.lbl_save_dir = QtWidgets.QLabel(self._settings.save_dir or os.path.abspath(os.getcwd()))
        self.lbl_save_dir.setWordWrap(True)

        self.btn_set_save_dir = QtWidgets.QPushButton("设置保存路径")
        self.btn_save_data = QtWidgets.QPushButton("保存数据(CSV)")

        lay.addWidget(self.btn_rec_toggle, 0, 0, 1, 2)
        lay.addWidget(self.btn_clear_data, 0, 2, 1, 2)

        lay.addWidget(QtWidgets.QLabel("记录点数"), 1, 0)
        lay.addWidget(self.lbl_rec_points, 1, 1)
        lay.addWidget(QtWidgets.QLabel("时长(s)"), 1, 2)
        lay.addWidget(self.lbl_rec_dur, 1, 3)

        lay.addWidget(QtWidgets.QLabel("保存路径"), 2, 0)
        lay.addWidget(self.lbl_save_dir, 2, 1, 1, 3)

        lay.addWidget(self.btn_set_save_dir, 3, 0, 1, 2)
        lay.addWidget(self.btn_save_data, 3, 2, 1, 2)

        return g

    def _build_lora_group(self) -> QtWidgets.QGroupBox:
        g = QtWidgets.QGroupBox("LoRa")
        lay = QtWidgets.QHBoxLayout(g)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(6)

        self.btn_lora_stat = QtWidgets.QPushButton("stat")
        self.btn_lora_ping = QtWidgets.QPushButton("ping")
        self.chk_lora_raw = QtWidgets.QCheckBox("raw")

        lay.addWidget(self.btn_lora_stat)
        lay.addWidget(self.btn_lora_ping)
        lay.addStretch(1)
        lay.addWidget(self.chk_lora_raw)

        return g

    def _build_log_group(self) -> QtWidgets.QGroupBox:
        g = QtWidgets.QGroupBox("串口日志 / 命令")
        lay = QtWidgets.QVBoxLayout(g)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(6)

        self.chk_show_log = QtWidgets.QCheckBox("显示日志")
        self.chk_show_log.setChecked(False)
        row = QtWidgets.QHBoxLayout()
        row.setSpacing(6)
        row.addWidget(self.chk_show_log)
        row.addStretch(1)
        lay.addLayout(row)

        self.txt_log = QtWidgets.QPlainTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setMaximumBlockCount(config.MAX_LOG_LINES)
        self.txt_log.setMinimumHeight(220)
        self.txt_log.setVisible(False)

        cmd_row = QtWidgets.QHBoxLayout()
        cmd_row.setSpacing(6)
        self.edit_cmd = QtWidgets.QLineEdit()
        self.edit_cmd.setPlaceholderText("输入命令：mode manual | set heater 20 | lora stat ...")
        self.btn_send = QtWidgets.QPushButton("发送")
        self.btn_send.setFixedWidth(70)
        cmd_row.addWidget(self.edit_cmd, stretch=1)
        cmd_row.addWidget(self.btn_send)

        lay.addWidget(self.txt_log, stretch=1)
        lay.addLayout(cmd_row)

        return g

    # ---------------- Wiring ----------------

    def _wire_signals(self):
        self.btn_refresh.clicked.connect(self._refresh_ports)
        self.btn_connect.clicked.connect(self._on_connect)
        self.btn_disconnect.clicked.connect(self._on_disconnect)

        self.chk_show_log.toggled.connect(self._on_toggle_log)
        self.btn_send.clicked.connect(self._on_send_cmd)
        self.edit_cmd.returnPressed.connect(self._on_send_cmd)

        self.btn_safe.clicked.connect(lambda: self._send_mode("safe"))
        self.btn_manual.clicked.connect(lambda: self._send_mode("manual"))
        self.btn_auto.clicked.connect(lambda: self._send_mode("auto"))

        self.sl_heater.valueChanged.connect(lambda v: self.lb_heater_set.setText(str(v)))
        self.btn_heater_apply.clicked.connect(self._on_heater_apply)
        self.btn_heater_zero.clicked.connect(lambda: self.sl_heater.setValue(0))
        self.btn_heater_full.clicked.connect(lambda: self.sl_heater.setValue(100))

        self.sl_valve.valueChanged.connect(lambda v: self.lb_valve_set.setText(str(v)))
        self.btn_valve_apply.clicked.connect(self._on_valve_apply)
        self.btn_valve_zero.clicked.connect(lambda: self.sl_valve.setValue(0))
        self.btn_valve_full.clicked.connect(lambda: self.sl_valve.setValue(100))

        self.btn_lora_stat.clicked.connect(self._on_lora_stat)
        self.btn_lora_ping.clicked.connect(self._on_lora_ping)
        self.chk_lora_raw.toggled.connect(self._on_lora_raw)

        self.cb_filter.currentTextChanged.connect(self._on_filter_mode_changed)
        self.cb_view.currentTextChanged.connect(self._on_view_mode_changed)
        self.btn_apply_filter.clicked.connect(self._apply_filter_settings)

        self.cb_plotwin_unit.currentIndexChanged.connect(self._on_plotwin_unit_changed)
        self.btn_plotwin_apply.clicked.connect(self._on_plotwin_apply)

        self.btn_rec_toggle.clicked.connect(self._on_toggle_recording)
        self.btn_clear_data.clicked.connect(self._on_clear_data)
        self.btn_set_save_dir.clicked.connect(self._on_set_save_dir)
        self.btn_save_data.clicked.connect(self._on_save_data)

    # ---------------- Settings -> UI ----------------

    def _apply_settings_to_ui(self, s: AppSettings):
        self._select_port(s.last_port)

        # filter
        self.cb_filter.setCurrentText(s.filter_config.mode)
        self.sp_alpha.setValue(float(s.filter_config.alpha))
        self.sp_window.setValue(int(s.filter_config.window_n))
        self.cb_view.setCurrentText(s.display_mode)
        self.chk_wait_ack.setChecked(bool(s.wait_ack_gate))

        # log
        self.chk_show_log.setChecked(bool(s.show_log))
        self.txt_log.setVisible(bool(s.show_log))

        # plot window
        if s.plot_window_mode == "seconds":
            self.cb_plotwin_unit.setCurrentIndex(1)
            self.sp_plotwin_val.setValue(int(s.plot_window_seconds))
        else:
            self.cb_plotwin_unit.setCurrentIndex(0)
            self.sp_plotwin_val.setValue(int(s.plot_window_points))
        self._on_plotwin_unit_changed(self.cb_plotwin_unit.currentIndex())
        self._on_view_mode_changed(self.cb_view.currentText())
        self._on_filter_mode_changed(self.cb_filter.currentText())

        # save dir
        if s.save_dir:
            self.lbl_save_dir.setText(s.save_dir)
            self.lbl_save_dir.setToolTip(s.save_dir)
            self.store.save_dir = s.save_dir

        # record info
        self._update_rec_info()

    # ---------------- Serial connection ----------------

    def _refresh_ports(self):
        self.port_combo.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if not ports:
            ports = [config.DEFAULT_PORT]
        self.port_combo.addItems(ports)

    def _select_port(self, port_name: str):
        if not port_name:
            return
        for i in range(self.port_combo.count()):
            if self.port_combo.itemText(i).upper() == port_name.upper():
                self.port_combo.setCurrentIndex(i)
                return

    @pyqtSlot()
    def _on_connect(self):
        if self.thread is not None:
            return
        port = self.port_combo.currentText().strip()
        if not port:
            return

        self.thread = QThread(self)
        self.worker = SerialWorker(port=port, baud=self._settings.baud)
        self.worker.moveToThread(self.thread)

        self.thread.started.connect(self.worker.run)
        self.worker.telemetry.connect(self._on_telem)
        self.worker.ack.connect(self._on_ack)
        self.worker.cmd_ack.connect(self._on_cmd_ack)
        self.worker.cmd_retry.connect(self._on_cmd_retry)
        self.worker.cmd_fail.connect(self._on_cmd_fail)
        self.worker.cmd_busy.connect(self._on_cmd_busy)
        self.worker.log_line.connect(self._append_log)
        self.worker.status_msg.connect(self._show_status)
        self.worker.connected.connect(self._on_connected)

        self.thread.start()

    @pyqtSlot()
    def _on_disconnect(self):
        if self.worker:
            self.worker.stop()
        if self.thread:
            self.thread.quit()
            self.thread.wait(1500)
        self.thread = None
        self.worker = None

    @pyqtSlot(bool)
    def _on_connected(self, ok: bool):
        self.btn_connect.setEnabled(not ok)
        self.btn_disconnect.setEnabled(ok)
        # keep toolbar actions in sync
        if hasattr(self, "_tb_act_connect"):
            self._tb_act_connect.setEnabled(not ok)
        if hasattr(self, "_tb_act_disconnect"):
            self._tb_act_disconnect.setEnabled(ok)
        self._cmd_pending = False
        self._cmd_retry_n = 0
        self._cmd_last_msg = None
        self._cmd_last_seq = None

        if ok:
            self.statusBar().showMessage("已连接")
            # remember port
            self._settings.last_port = self.port_combo.currentText().strip()
            if self.worker:
                self.worker.send_line("lora stat")
        else:
            self.statusBar().showMessage("未连接")

    # ---------------- Telemetry / events ----------------

    @pyqtSlot(object)
    def _on_telem(self, f: TelemetryFrame):
        self.store.add_telem(f)
        cv = self.store.current

        self.lbl_t0.setText(f"{cv.t0_c:.2f}")
        self.lbl_t1.setText(f"{cv.t1_c:.2f}")
        self.lbl_p_kpa.setText(f"{cv.p_kpa:.3f}")
        self.lbl_heater.setText(f"{cv.heater_pct:.1f}")
        self.lbl_valve.setText(f"{cv.valve_pct:.1f}")
        self.lbl_last.setText(str(cv.telem_ms or "--"))

        if self.store.rec_enabled:
            self._update_rec_info()

    @pyqtSlot(object)
    def _on_ack(self, ack: AckFrame):
        self.statusBar().showMessage(f"ACK: msg=0x{ack.msg_type:02X} status={ack.status}")

    # reliable-downlink status
    def _set_cmd_pending(self, desc: str):
        self._cmd_pending = True
        self._cmd_desc = desc
        self._cmd_retry_n = 0
        self.lbl_cmd_desc.setText(desc or "--")
        self.lbl_cmd_state.setText("WAIT_ACK")
        self.lbl_cmd_seq.setText("--")
        self.lbl_cmd_retry.setText("0")
        self.lbl_cmd_result.setText("--")

    def _clear_cmd_pending(self, state: str, result: str):
        self._cmd_pending = False
        self.lbl_cmd_state.setText(state)
        self.lbl_cmd_result.setText(result)

    @pyqtSlot(object)
    def _on_cmd_ack(self, e: ReliableCmdAck):
        self._cmd_last_msg = e.msg_type
        self._cmd_last_seq = e.seq
        self.lbl_cmd_seq.setText(str(e.seq))
        self.lbl_cmd_retry.setText(str(self._cmd_retry_n))
        self._clear_cmd_pending("ACK", f"status={e.status}")
        self.statusBar().showMessage(f"[CMD] ACK msg=0x{e.msg_type:02X} seq={e.seq} status={e.status}")

    @pyqtSlot(object)
    def _on_cmd_retry(self, e: ReliableCmdRetry):
        self._cmd_last_msg = e.msg_type
        self._cmd_last_seq = e.seq
        self._cmd_retry_n = e.retry_n
        self.lbl_cmd_state.setText("RETRY")
        self.lbl_cmd_seq.setText(str(e.seq))
        self.lbl_cmd_retry.setText(str(e.retry_n))
        self.statusBar().showMessage(f"[CMD] RETRY #{e.retry_n} msg=0x{e.msg_type:02X} seq={e.seq}")

    @pyqtSlot(object)
    def _on_cmd_fail(self, e: ReliableCmdFail):
        self._cmd_last_msg = e.msg_type
        self._cmd_last_seq = e.seq
        self.lbl_cmd_seq.setText(str(e.seq))
        self.lbl_cmd_retry.setText(str(self._cmd_retry_n))
        self._clear_cmd_pending("FAIL", "no ACK")
        self.statusBar().showMessage(f"[CMD] FAIL msg=0x{e.msg_type:02X} seq={e.seq}")

    @pyqtSlot(object)
    def _on_cmd_busy(self, e: ReliableCmdBusyWarn):
        self.lbl_cmd_state.setText("BUSY")
        self.lbl_cmd_result.setText(f"busy>{e.busy_s:.1f}s")
        self.statusBar().showMessage(f"[CMD] WARNING: LoRa TX busy > {e.busy_s:.1f}s")

    # ---------------- Filter & display ----------------

    @pyqtSlot(str)
    def _on_filter_mode_changed(self, mode: str):
        show_alpha = mode in (FilterMode.EMA, FilterMode.MEDIAN_EMA)
        show_win = mode in (FilterMode.SMA, FilterMode.MEDIAN, FilterMode.MEDIAN_EMA)
        self.sp_alpha.setEnabled(show_alpha)
        self.sp_window.setEnabled(show_win)

    @pyqtSlot(str)
    def _on_view_mode_changed(self, mode: str):
        mode = (mode or DisplayMode.BOTH).strip() or DisplayMode.BOTH
        show_raw = mode in (DisplayMode.RAW, DisplayMode.BOTH)
        show_f = mode in (DisplayMode.FILT, DisplayMode.BOTH)

        self.curve_t0_raw.setVisible(show_raw)
        self.curve_t1_raw.setVisible(show_raw)
        self.curve_p_raw.setVisible(show_raw)

        self.curve_t0_f.setVisible(show_f)
        self.curve_t1_f.setVisible(show_f)
        self.curve_p_f.setVisible(show_f)

        if mode == DisplayMode.RAW:
            self.temp_plot.setTitle("温度-时间（原始）")
            self.press_plot.setTitle("压力-时间（原始）")
        elif mode == DisplayMode.FILT:
            self.temp_plot.setTitle("温度-时间（滤波）")
            self.press_plot.setTitle("压力-时间（滤波）")
        else:
            self.temp_plot.setTitle("温度-时间（原始+滤波）")
            self.press_plot.setTitle("压力-时间（原始+滤波）")

    @pyqtSlot()
    def _apply_filter_settings(self):
        cfg = FilterConfig(
            mode=self.cb_filter.currentText().strip(),
            alpha=float(self.sp_alpha.value()),
            window_n=int(self.sp_window.value()),
        )
        self.store.set_filter_config(cfg, recompute_plot_series=True)
        self._show_status(f"滤波已应用：mode={cfg.mode}, alpha={cfg.alpha:.2f}, N={cfg.window_n}")

    # ---------------- Plot window ----------------

    @pyqtSlot(int)
    def _on_plotwin_unit_changed(self, idx: int):
        if idx == 0:
            self.sp_plotwin_val.blockSignals(True)
            self.sp_plotwin_val.setRange(100, 500000)
            self.sp_plotwin_val.setSingleStep(100)
            self.sp_plotwin_val.setSuffix(" 点")
            self.sp_plotwin_val.setValue(int(self.store.plot_window_points))
            self.sp_plotwin_val.blockSignals(False)
            self.lbl_plotwin_hint.setText(f"当前：最近 {int(self.store.plot_window_points)} 点")
        else:
            self.sp_plotwin_val.blockSignals(True)
            self.sp_plotwin_val.setRange(10, 86400)
            self.sp_plotwin_val.setSingleStep(10)
            self.sp_plotwin_val.setSuffix(" s")
            self.sp_plotwin_val.setValue(int(self.store.plot_window_seconds))
            self.sp_plotwin_val.blockSignals(False)
            self.lbl_plotwin_hint.setText(f"当前：最近 {int(self.store.plot_window_seconds)} s")

    @pyqtSlot()
    def _on_plotwin_apply(self):
        idx = int(self.cb_plotwin_unit.currentIndex())
        val = int(self.sp_plotwin_val.value())
        if idx == 0:
            self.store.set_plot_window_points(val)
            self.lbl_plotwin_hint.setText(f"当前：最近 {val} 点")
        else:
            self.store.set_plot_window_seconds(val)
            self.lbl_plotwin_hint.setText(f"当前：最近 {val} s")
        self._update_plots()

    # ---------------- Recording & saving ----------------

    def _update_rec_info(self):
        n, dur = self.store.recording_info()
        self.lbl_rec_points.setText(str(n))
        self.lbl_rec_dur.setText(f"{dur:.1f}")

    @pyqtSlot()
    def _on_toggle_recording(self):
        self.store.rec_enabled = not self.store.rec_enabled
        self.btn_rec_toggle.setText("停止记录" if self.store.rec_enabled else "开始记录")
        self._show_status("已开始记录" if self.store.rec_enabled else "已停止记录（曲线仍实时刷新，但不会写入历史数据）")

    @pyqtSlot()
    def _on_clear_data(self):
        self.store.clear()
        self._update_rec_info()
        self._show_status("已清空数据并重置时间原点。")

    @pyqtSlot()
    def _on_set_save_dir(self):
        d = QtWidgets.QFileDialog.getExistingDirectory(self, "选择保存目录", self.store.save_dir)
        if d:
            self.store.save_dir = d
            self.lbl_save_dir.setText(d)
            self.lbl_save_dir.setToolTip(d)
            self._show_status(f"保存路径已设置：{d}")

    @pyqtSlot()
    def _on_save_data(self):
        if self.store.record_len() == 0:
            self._show_status("没有可保存的数据：请先开始记录并接收 TELEM。")
            return

        default_path = os.path.join(self.store.save_dir, self.store.default_csv_name())
        fn, _ = QtWidgets.QFileDialog.getSaveFileName(self, "保存数据为 CSV", default_path, "CSV Files (*.csv);;All Files (*)")
        if not fn:
            return
        if not fn.lower().endswith(".csv"):
            fn += ".csv"

        try:
            cfg = FilterConfig(
                mode=self.cb_filter.currentText().strip(),
                alpha=float(self.sp_alpha.value()),
                window_n=int(self.sp_window.value()),
            )
            self.store.write_csv(fn, cfg)
            self._show_status(f"数据已保存：{fn}")
        except Exception as e:
            self._show_status(f"保存失败：{e}")

    # ---------------- Commands ----------------

    def _require_worker(self) -> bool:
        if not self.worker:
            self._show_status("未连接：请先连接串口")
            return False
        return True

    def _try_send_reliable(self, desc: str, send_fn):
        if not self._require_worker():
            return
        if self.chk_wait_ack.isChecked() and self._cmd_pending:
            self._show_status("上一条可靠命令尚未确认（ACK/FAIL），已阻止重复发送。可取消勾选以允许覆盖发送。")
            return
        self._set_cmd_pending(desc)
        try:
            send_fn()
        except Exception as e:
            self._clear_cmd_pending("FAIL", f"host send: {e}")
            raise

    def _send_mode(self, mode: str):
        if not self._require_worker():
            return
        self.worker.set_mode(mode)
        self._show_status(f"发送: mode {mode}")

    @pyqtSlot()
    def _on_heater_apply(self):
        v = int(self.sl_heater.value())
        self._try_send_reliable(f"set heater {v}", lambda: self.worker.set_heater(v))

    @pyqtSlot()
    def _on_valve_apply(self):
        v = int(self.sl_valve.value())
        self._try_send_reliable(f"set valve {v}", lambda: self.worker.set_valve(v))

    @pyqtSlot()
    def _on_lora_stat(self):
        if self._require_worker():
            self.worker.lora_stat()

    @pyqtSlot()
    def _on_lora_ping(self):
        if self._require_worker():
            self.worker.lora_ping()

    @pyqtSlot(bool)
    def _on_lora_raw(self, on: bool):
        if self._require_worker():
            self.worker.lora_raw(on)

    @pyqtSlot()
    def _on_send_cmd(self):
        if not self._require_worker():
            return
        text = (self.edit_cmd.text() or "").strip()
        if not text:
            return
        self.worker.send_line(text)
        self._append_log(f">>> {text}")
        self.edit_cmd.clear()

    # ---------------- Plot refresh ----------------

    @pyqtSlot()
    def _update_plots(self):
        series = self.store.get_plot_series()
        if series is None:
            return
        t, t0_raw, t1_raw, p_raw, t0_f, t1_f, p_f = series

        self.curve_t0_raw.setData(t, t0_raw)
        self.curve_t1_raw.setData(t, t1_raw)
        self.curve_t0_f.setData(t, t0_f)
        self.curve_t1_f.setData(t, t1_f)
        self.curve_p_raw.setData(t, p_raw)
        self.curve_p_f.setData(t, p_f)

    # ---------------- Logging/status ----------------

    @pyqtSlot(bool)
    def _on_toggle_log(self, on: bool):
        self.txt_log.setVisible(bool(on))

    @pyqtSlot(str)
    def _append_log(self, line: str):
        ts = time.strftime("%H:%M:%S")
        self.txt_log.appendPlainText(f"[{ts}] {line}")

    @pyqtSlot(str)
    def _show_status(self, msg: str):
        self.statusBar().showMessage(msg)
        self._append_log(msg)

    # ---------------- Qt lifecycle ----------------

    def closeEvent(self, event):
        # persist settings
        try:
            self._settings.last_port = self.port_combo.currentText().strip()
            self._settings.save_dir = self.store.save_dir
            self._settings.plot_window_mode = self.store.plot_window_mode
            self._settings.plot_window_points = self.store.plot_window_points
            self._settings.plot_window_seconds = self.store.plot_window_seconds

            self._settings.filter_config = FilterConfig(
                mode=self.cb_filter.currentText().strip(),
                alpha=float(self.sp_alpha.value()),
                window_n=int(self.sp_window.value()),
            )
            self._settings.display_mode = self.cb_view.currentText().strip()
            self._settings.wait_ack_gate = self.chk_wait_ack.isChecked()
            self._settings.show_log = self.chk_show_log.isChecked()

            self._settings_store.save(self._settings)
        except Exception:
            pass

        # close serial thread
        try:
            self._on_disconnect()
        except Exception:
            pass

        super().closeEvent(event)

