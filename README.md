# uav-h2-fuel-system-control

本仓库提供一套 **“空中中继 + 地面中继 + PC 上位机”** 的通信与监控链路，用于将 **Nano 33 BLE（控制器）** 的遥测数据回传至 PC，并将 PC 端的控制指令经 **LoRa（RA-01 / SX1278）** 下发至 Nano 33 BLE。

核心特点：

- LoRa 半双工链路下，空中端采用 **TX 排队 + 遥测降采样**，尽量减少“发射时错过下行控制”。
- 地面端对控制类命令实现 **等待 ACK + 超时重发**（并对 LoRa Busy 做特殊处理，避免误计重试）。
- PC 上位机支持实时曲线、可选滤波（EMA/SMA/Median/Median+EMA）、数据记录/清空/CSV 导出、保存路径设置等。

## 1. 目录结构

```
uav-h2-fuel-system-control/
  NanoESP32_AirGateway/
    NanoESP32_AirGateway.ino
    src/
      lora/LoRaLink.{h,cpp}
      proto/FrameCodec.{h,cpp}
      proto/Protocol.h
      util/BoardConfig.h
  NanoESP32_GroundGateway/
    NanoESP32_GroundGateway.ino
    src/
      lora/LoRaLink.{h,cpp}
      proto/FrameCodec.{h,cpp}
      proto/Protocol.h
      util/BoardConfig.h
  NanoESP32_LoRaHealthProbe/
    NanoESP32_LoRaHealthProbe.ino
  host_gui/
    app.py
    config.py
    core/{filtering.py,model.py,protocol.py,settings.py}
    io/serial_worker.py
    ui/main_window.py
    utils/logging_setup.py
  host_gui_v11.py
```

说明：

- 本仓库当前 **不包含** Nano 33 BLE 的控制器固件工程（控制器通过 UART 与空中中继连接）。
- `host_gui/__pycache__` 为运行产生的缓存文件，建议不要提交到版本库（见本文末尾 `.gitignore` 建议）。

## 2. 系统拓扑

```
PC 上位机 (host_gui_v11.py)
        │ USB 串口
        ▼
地面 Nano ESP32（GroundGateway）
        │ LoRa (RA-01 / SX1278)
        ▼
空中 Nano ESP32（AirGateway）
        │ UART (Serial1)
        ▼
Nano 33 BLE（控制器，另行提供固件）
```

## 3. 硬件与接线

### 3.1 LoRa（RA-01 / SX1278）↔ Nano ESP32（空中、地面两端一致）

以下为 `BoardConfig.h` 中的默认约定：

| RA-01 引脚 | Nano ESP32 引脚 |
|---|---|
| SCK | D13 |
| MISO (CIPO) | D12 |
| MOSI (COPI) | D11 |
| NSS / CS | D10 |
| RESET | D6 |
| DIO0 | D2 |
| 3.3V | 3V3 |
| GND | GND |

注意事项：

- **必须 3.3V 供电**，且需要稳定（建议短线、良好接地，必要时增加去耦）。
- 两端 LoRa 参数必须一致（频点 / SyncWord / SF / BW / CR / CRC）。

默认 LoRa 参数（两端一致）：

- 频点：`433000000 Hz`
- SF：7
- BW：125 kHz
- CR：4/5
- CRC：on
- SyncWord：`0x12`
- TX Power：17 dBm

### 3.2 空中 Nano ESP32 ↔ Nano 33 BLE（UART）

空中中继使用 Nano ESP32 的 `D0/D1` 作为 UART（注意该板的 D0/D1 与部分板卡习惯相反，已在 `BoardConfig.h` 注释说明）：

- ESP32 `D1(TX)` → Nano33BLE `D0(RX)`
- ESP32 `D0(RX)` ← Nano33BLE `D1(TX)`
- GND ↔ GND
- 波特率：`115200, 8N1`

## 4. 固件编译与烧录（Arduino IDE）

### 4.1 前置

- Arduino IDE 2.x
- 安装 **Arduino Nano ESP32** 板卡支持包

### 4.2 烧录顺序建议

1) 地面端：打开 `NanoESP32_GroundGateway/NanoESP32_GroundGateway.ino` → 选择端口 → Upload
2) 空中端：打开 `NanoESP32_AirGateway/NanoESP32_AirGateway.ino` → 选择端口 → Upload

烧录后串口监视器建议 `115200`。

## 5. PC 上位机（host_gui v11）

### 5.1 安装依赖

建议 Python 3.9+。

```bash
pip install pyqt5 pyqtgraph pyserial
```

### 5.2 运行

```bash
python host_gui_v11.py
```

默认串口配置见 `host_gui/config.py`：`COM8 @ 115200`（可在 GUI 中刷新并选择）。

### 5.3 功能概览

- 串口：刷新 / 连接 / 断开
- 曲线：温度/压力实时绘图（原始 + 滤波）
- 滤波：None / EMA / SMA / Median / Median+EMA（参数可调）
- 控制：模式切换（safe/manual/auto）、加热器/阀门 0–100% 设定（带 0% / 100% 快捷）
- 记录：开始/停止记录、清空数据、设置保存路径、CSV 导出
- LoRa：`stat` / `ping` / `raw`（raw 用于地面中继原始嗅探）
- 日志：可选显示串口日志与手动输入命令

## 6. 地面中继串口命令（PC → GroundGateway）

地面中继从 USB 串口读取 ASCII 行命令，并封装为二进制帧经 LoRa 下发空中中继。

在地面中继串口输入 `help` 可查看帮助；当前固件实现的主要命令如下：

### 6.1 模式切换

```
mode safe
mode manual
mode auto
```

### 6.2 手动控制

```
set heater <0-100>
set valve  <0-100>
```

### 6.3 自动控制预留（下发 setpoints）

```
set T <degC>
set P <Pa>
set valve_sp <0-100>
```

说明：上述 setpoints 在协议层已实现，但控制器端（Nano 33 BLE）是否使用，取决于你控制器固件的实现。

### 6.4 LoRa 调试

```
lora stat
lora raw on|off
lora tx <text>
lora ping
```

### 6.5 可靠下行（ACK + 重发）行为

对 `mode` / `set heater` / `set valve` / `setpoints` 等控制类消息，地面端会：

- 发送后等待 ACK（默认 `400 ms`）
- 超时则重发（默认最多 `3` 次）
- 若 LoRa 返回 BUSY：**不计入重试次数**，并可输出 busy 警告

相关参数见 `NanoESP32_GroundGateway/src/util/BoardConfig.h`：

- `CMD_ACK_TIMEOUT_MS = 400`
- `CMD_MAX_RETRY = 3`

## 7. 地面串口输出格式（GroundGateway → PC）

上位机解析的是地面中继打印的文本行（解析逻辑位于 `host_gui/core/protocol.py`）。关键行格式：

### 7.1 遥测

```
[TELEM] t=1234 T0=20.5 T1=20.6 P(Pa)=101.3 heater=%=0.0 valve=%=0.0
```

### 7.2 ACK

```
[ACK] for=0x12 status=0
```

### 7.3 可靠下行状态

```
[CMD] ACK received for msg=0x12 seq=7 status=0
[CMD] RETRY #2 msg=0x12 seq=7
[CMD] FAIL: no ACK for msg=0x12 seq=7
[CMD] WARNING: LoRa TX busy > 3s (busy does not count retry)
```

## 8. 空口/串口二进制协议（FrameCodec）

`FrameCodec` 定义了统一的帧格式，用于：

- 地面 ↔ 空中（LoRa 负载内）
- 空中 ↔ 控制器（UART 上）

### 8.1 帧结构

```
SYNC1(0x55) SYNC2(0xAA) LEN MSG_TYPE SEQ PAYLOAD... CRC16(lo,hi)
```

其中 CRC16 为 Modbus CRC16。

### 8.2 已定义消息类型（`proto/Protocol.h`）

- `0x01`：`MSG_TELEM_V1`（遥测）
- `0x10`：`MSG_MODE_SWITCH`
- `0x11`：`MSG_SETPOINTS_V1`
- `0x12`：`MSG_MANUAL_CMD_V1`
- `0x20`：`MSG_ACK`
- `0x23`：`MSG_HEARTBEAT`

## 9. 诊断与排错建议

### 9.1 `LoRa init: FAILED`

优先检查：

- RA-01 是否为 3.3V 供电、供电是否塌陷
- SPI/CS/RST/DIO0 引脚是否接对（尤其 CS=D10、RST=D6、DIO0=D2）
- 两端频点/SyncWord 是否一致

### 9.2 长时间运行后“接收不畅 / 需 reset 才恢复”

地面端固件已加入 RX watchdog：曾经收到包后若 `>5s` 完全无包，会触发 LoRa 重新初始化并打印：

```
[LORA] watchdog: no RX > 5s, reinit radio
```

若仍频繁发生，通常与供电、接线长度、干扰、天线/匹配有关。

### 9.3 `lora raw on` 的使用

- GroundGateway：开启后打印任何 LoRa 包内容（ASCII/HEX），并停止按 FrameCodec 解码打印。
- AirGateway：开启后打印任何 LoRa 包并停止“下行转发到 UART”。

### 9.4 UART 下行拥塞（空中端）

空中端在将 LoRa 下行转发到控制器 UART 时，若 `Serial1.availableForWrite()` 不足，会丢弃并累计计数（避免阻塞导致系统卡死）。调试时可通过 `debug on` 打开日志观察。

## 10. 建议补充文件

为便于环境复现与避免缓存文件入库，建议在仓库根目录增加：

### 10.1 `requirements.txt`

```
pyqt5
pyqtgraph
pyserial
```

### 10.2 `.gitignore`

```
__pycache__/
*.pyc
*.pyo
*.log
*.csv
```

（如需将导出的 CSV 纳入版本管理，可删除 `*.csv` 这一行。）
