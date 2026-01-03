/*
 * NanoESP32_AirGateway.ino
 *
 * 当前阶段：空中中继（Nano ESP32）
 * - 保留 USB 串口调试：PC -> ESP32(USB Serial) -> UART -> Nano33BLE
 * - 加入 LoRa：
 *    1) Nano33BLE 的遥测/ACK 等帧 => 由 ESP32 通过 LoRa 发往地面中继
 *    2) 地面中继发来的命令帧       => 由 ESP32 通过 UART 转发给 Nano33BLE
 */

#include <Arduino.h>

#include "src/util/BoardConfig.h"
#include "src/proto/FrameCodec.h"
#include "src/proto/Protocol.h"

#include "src/lora/LoRaLink.h"

static FrameCodec::Parser g_parser;
static uint8_t g_tx_seq = 0;

static uint32_t g_last_hb_ms = 0;

// LoRa TX 采用“排队发送”以避免在 UART 解析过程中阻塞（LoRa.endPacket() 为阻塞调用）。
// - 高优先级：ACK/关键上行（尽量不丢）
// - 低优先级：遥测（允许覆盖/降采样）
static uint8_t g_tx_hi_buf[256];
static size_t  g_tx_hi_len = 0;
static uint8_t g_tx_telem_buf[256];
static size_t  g_tx_telem_len = 0;
static uint32_t g_last_telem_lora_ms = 0;
static uint32_t g_last_downlink_ms = 0;

static bool g_lora_ok = false;
// 注意：Nano ESP32 的 USB CDC 在未打开串口监视器/上位机未读取时，频繁 Serial.print 可能导致
// 明显延迟甚至卡死（尤其在高频打印/同时读写时）。因此默认关闭周期性日志，仅在需要调试时通过命令打开。
static bool g_verbose = false;          // 是否输出日志
static bool g_verbose_telem = false;    // 是否输出 TELEM 详情
static bool g_verbose_lora_drop = false; // 是否输出 LoRa 丢包原因（默认关闭，避免刷屏）
static bool g_debug_lora_tx = false;     // 是否输出 LoRa 发送内容摘要
static bool g_lora_raw = false;        // LoRa 原始嗅探模式：打印任何包，不做协议转发
static int  g_last_lora_rssi = 0;
static float g_last_lora_snr = 0.0f;
static uint32_t g_last_lora_rx_ms = 0;

// 累积式行缓冲
static char g_line_buf[128];
static size_t g_line_len = 0;

static Proto::PayloadManualCmdV1 g_man = {0, 0.0f, 0.0f, 0.0f};
static Proto::PayloadSetpointsV1 g_sp  = {0.0f, 0.0f, 0.0f, 0.0f, 0};

static uint32_t g_uart_busy_since_ms = 0;
static uint32_t g_uart_last_warn_ms  = 0;
static uint32_t g_uart_drop_downlink = 0;

static bool uart1WriteDropIfBusy(const uint8_t* data, size_t len, const char* tag)
{
    if (!data || len == 0) return true;

    const int avail = Serial1.availableForWrite();
    if (avail < (int)len) {
        ++g_uart_drop_downlink;
        const uint32_t now = millis();

        if (g_uart_busy_since_ms == 0) g_uart_busy_since_ms = now;

        // 可选：busy 超过 3s 且每 1s 打一次 warning，避免刷屏
        // 注意：仅在调试开关开启且 USB 串口已连接时输出，避免 USB CDC 阻塞导致主循环卡死。
        if (g_verbose && Serial && (now - g_uart_busy_since_ms) > 3000 && (now - g_uart_last_warn_ms) > 1000) {
            Serial.print("[UART] TX busy, drop downlink. tag=");
            Serial.print(tag);
            Serial.print(" drop=");
            Serial.print(g_uart_drop_downlink);
            Serial.print(" avail=");
            Serial.print(avail);
            Serial.print(" need=");
            Serial.println((int)len);
            g_uart_last_warn_ms = now;
        }
        return false;
    }

    Serial1.write(data, len);
    g_uart_busy_since_ms = 0;
    return true;
}

static void uartSend(uint8_t msg_type, const void *payload, uint8_t payload_len)
{
    uint8_t buf[256];
    const size_t n = FrameCodec::encode(msg_type, g_tx_seq++,
                                        reinterpret_cast<const uint8_t*>(payload), payload_len,
                                        buf, sizeof(buf));
    if (n) {
        uart1WriteDropIfBusy(buf, n, "UART->BLE");
    }
}

static void sendHeartbeat(uint32_t now_ms)
{
    if (now_ms - g_last_hb_ms >= BoardConfig::HEARTBEAT_PERIOD_MS) {
        g_last_hb_ms = now_ms;
        uartSend(Proto::MSG_HEARTBEAT, nullptr, 0);
    }
}

static void printHelp()
{
    Serial.println("Commands:");
    Serial.println("  help");
    Serial.println("  status");
    Serial.println("  mode safe|manual|auto");
    Serial.println("  set heater <0-100>");
    Serial.println("  set valve <0-100>");
    Serial.println("  set T <degC>            (setpoint, reserved for future auto)");
    Serial.println("  set P <Pa>              (setpoint, reserved for future auto)");
    Serial.println("  set valve_sp <0-100>    (setpoint)");
    Serial.println("  lora stat");
    Serial.println("  lora raw on|off        (print any LoRa packet, disable downlink forwarding)");
    Serial.println("  lora tx <text>         (send raw text over LoRa)");
    Serial.println("  lora ping              (send PING over LoRa)");
    Serial.println("  lora txframe           (send a FrameCodec heartbeat over LoRa)");
    Serial.println("  debug on|off");
    Serial.println("  debug telem on|off");
    Serial.println("  debug lora on|off");
    Serial.println("  debug lora_tx on|off   (print LoRa TX head)");
}

static void printStatus()
{
    Serial.print("UART pins: RX=");
    Serial.print(BoardConfig::UART_RX_PIN);
    Serial.print(" TX=");
    Serial.println(BoardConfig::UART_TX_PIN);

    Serial.print("LoRa init: ");
    Serial.println(g_lora_ok ? "OK" : "FAILED");

    Serial.print("LoRa last_rx: ");
    if (g_last_lora_rx_ms == 0) {
        Serial.println("(none)");
    } else {
        Serial.print(g_last_lora_rx_ms);
        Serial.print(" ms ago, rssi=");
        Serial.print(g_last_lora_rssi);
        Serial.print(" snr=");
        Serial.println(g_last_lora_snr);
    }

    Serial.print("Log: ");
    Serial.print(g_verbose ? "on" : "off");
    Serial.print("  TELEM: ");
    Serial.print(g_verbose_telem ? "on" : "off");
    Serial.print("  LORA_DROP: ");
    Serial.println(g_verbose_lora_drop ? "on" : "off");
}

static bool parseFloat(const char *s, float &out)
{
    if (!s) return false;
    char *endp = nullptr;
    out = strtof(s, &endp);
    return (endp && endp != s);
}

static void handleLine(char *line)
{
    // trim leading spaces
    while (*line == ' ' || *line == '\t' || *line == '\r' || *line == '\n') ++line;
    if (*line == 0) return;

    // tokenize
    char *cmd = strtok(line, " \t\r\n");
    if (!cmd) return;

    if (strcmp(cmd, "help") == 0) {
        if (Serial) {
        printHelp();
    }
        return;
    }

    if (strcmp(cmd, "status") == 0) {
        printStatus();
        return;
    }

    if (strcmp(cmd, "lora") == 0) {
        char *sub = strtok(nullptr, " \t\r\n");
        if (sub && strcmp(sub, "stat") == 0) {
            Serial.print("LoRa freq=");
            Serial.print(BoardConfig::LORA_FREQ_HZ);
            Serial.print(" Hz, SF=");
            Serial.print(BoardConfig::LORA_SPREADING_FACTOR);
            Serial.print(", BW=");
            Serial.print(BoardConfig::LORA_SIGNAL_BW);
            Serial.print(", CR=4/");
            Serial.print(BoardConfig::LORA_CODING_RATE_DENOM);
            Serial.print(", CRC=");
            Serial.print(BoardConfig::LORA_ENABLE_CRC ? "on" : "off");
            Serial.print(", Sync=0x");
            Serial.println(BoardConfig::LORA_SYNC_WORD, HEX);
            if (g_last_lora_rx_ms != 0) {
                Serial.print("Last RX: rssi=");
                Serial.print(g_last_lora_rssi);
                Serial.print(" snr=");
                Serial.println(g_last_lora_snr);
            }
            return;
        }
        if (sub && strcmp(sub, "raw") == 0) {
            char *arg = strtok(nullptr, " \t\r\n");
            if (!arg) {
                Serial.print("LoRa raw is ");
                Serial.println(g_lora_raw ? "on" : "off");
                return;
            }
            if (strcmp(arg, "on") == 0) g_lora_raw = true;
            else if (strcmp(arg, "off") == 0) g_lora_raw = false;
            else {
                Serial.println("Usage: lora raw on|off");
                return;
            }
            Serial.print("OK: lora raw ");
            Serial.println(g_lora_raw ? "on" : "off");
            return;
        }
        if (sub && strcmp(sub, "tx") == 0) {
            char *text = strtok(nullptr, "\r\n");
            if (!text) {
                Serial.println("Usage: lora tx <text>");
                return;
            }
            const bool ok = g_lora_ok ? LoRaLink::send(reinterpret_cast<const uint8_t*>(text), strlen(text)) : false;
            Serial.print(ok ? "OK" : "ERR");
            Serial.println(": lora tx");
            return;
        }
        if (sub && strcmp(sub, "txframe") == 0) {
            uint8_t pkt[64];
            const size_t n = FrameCodec::encode(Proto::MSG_HEARTBEAT, 0x42, nullptr, 0, pkt, sizeof(pkt));
            const bool ok = (g_lora_ok && n) ? LoRaLink::send(pkt, n) : false;
            Serial.print(ok ? "OK" : "ERR");
            Serial.println(": lora txframe");
            return;
        }

        if (sub && strcmp(sub, "ping") == 0) {
            const char *text = "PING";
            const bool ok = g_lora_ok ? LoRaLink::send(reinterpret_cast<const uint8_t*>(text), strlen(text)) : false;
            Serial.print(ok ? "OK" : "ERR");
            Serial.println(": lora ping");
            return;
        }
        Serial.println("Usage: lora stat | lora raw on|off | lora tx <text> | lora ping");
        return;
    }

    if (strcmp(cmd, "debug") == 0) {
        char *what = strtok(nullptr, " \t\r\n");
        char *arg  = strtok(nullptr, " \t\r\n");

        // debug on|off
        if (what && (strcmp(what, "on") == 0 || strcmp(what, "off") == 0)) {
            g_verbose = (strcmp(what, "on") == 0);
            Serial.print("OK: debug ");
            Serial.println(g_verbose ? "on" : "off");
            return;
        }

        // debug telem on|off
        if (what && strcmp(what, "telem") == 0 && arg) {
            if (strcmp(arg, "on") == 0) g_verbose_telem = true;
            else if (strcmp(arg, "off") == 0) g_verbose_telem = false;
            else {
                Serial.println("Usage: debug telem on|off");
                return;
            }
            Serial.print("OK: telem ");
            Serial.println(g_verbose_telem ? "on" : "off");
            return;
        }

        // debug lora on|off  (控制 LoRa 丢包刷屏)
        if (what && strcmp(what, "lora") == 0 && arg) {
            if (strcmp(arg, "on") == 0) g_verbose_lora_drop = true;
            else if (strcmp(arg, "off") == 0) g_verbose_lora_drop = false;
            else {
                Serial.println("Usage: debug lora on|off");
                return;
            }
            Serial.print("OK: lora_drop ");
            Serial.println(g_verbose_lora_drop ? "on" : "off");
            return;
        }

        
        // debug lora_tx on|off (LoRa TX head)
        if (what && strcmp(what, "lora_tx") == 0 && arg) {
            if (strcmp(arg, "on") == 0) g_debug_lora_tx = true;
            else if (strcmp(arg, "off") == 0) g_debug_lora_tx = false;
            else {
                Serial.println("Usage: debug lora_tx on|off");
                return;
            }
            Serial.print("OK: lora_tx ");
            Serial.println(g_debug_lora_tx ? "on" : "off");
            return;
        }
Serial.println("Usage: debug on|off  OR  debug telem on|off  OR  debug lora on|off");
        return;
    }

    if (strcmp(cmd, "mode") == 0) {
        char *arg = strtok(nullptr, " \t\r\n");
        Proto::PayloadModeSwitch p{};
        if (!arg) {
            Serial.println("ERR: mode missing");
            return;
        }
        if (strcmp(arg, "safe") == 0)   p.mode = Proto::MODE_SAFE;
        else if (strcmp(arg, "manual") == 0) p.mode = Proto::MODE_MANUAL;
        else if (strcmp(arg, "auto") == 0)   p.mode = Proto::MODE_AUTO;
        else {
            Serial.println("ERR: unknown mode");
            return;
        }
        uartSend(Proto::MSG_MODE_SWITCH, &p, sizeof(p));
        Serial.println("OK: mode sent");
        return;
    }

    if (strcmp(cmd, "set") == 0) {
        char *what = strtok(nullptr, " \t\r\n");
        char *val  = strtok(nullptr, " \t\r\n");
        if (!what || !val) {
            Serial.println("ERR: set <item> <value>");
            return;
        }

        float f = 0.0f;
        if (!parseFloat(val, f)) {
            Serial.println("ERR: value parse");
            return;
        }

        if (strcmp(what, "heater") == 0) {
            g_man.flags |= Proto::MAN_FLAG_HEATER;
            g_man.heater_power_pct = f;
            uartSend(Proto::MSG_MANUAL_CMD_V1, &g_man, sizeof(g_man));
            Serial.println("OK: heater cmd sent");
            return;
        }
        if (strcmp(what, "valve") == 0) {
            g_man.flags |= Proto::MAN_FLAG_VALVE;
            g_man.valve_opening_pct = f;
            uartSend(Proto::MSG_MANUAL_CMD_V1, &g_man, sizeof(g_man));
            Serial.println("OK: valve cmd sent");
            return;
        }

        // 以下为自动控制预留 setpoints（Nano33BLE 当前不使用）
        if (strcmp(what, "T") == 0) {
            g_sp.target_temp_c = f;
            g_sp.enable_mask |= Proto::SP_ENABLE_TEMP;
            uartSend(Proto::MSG_SETPOINTS_V1, &g_sp, sizeof(g_sp));
            Serial.println("OK: setpoint T sent");
            return;
        }
        if (strcmp(what, "P") == 0) {
            g_sp.target_pressure_pa = f;
            g_sp.enable_mask |= Proto::SP_ENABLE_PRESSURE;
            uartSend(Proto::MSG_SETPOINTS_V1, &g_sp, sizeof(g_sp));
            Serial.println("OK: setpoint P sent");
            return;
        }
        if (strcmp(what, "valve_sp") == 0) {
            g_sp.target_valve_opening_pct = f;
            g_sp.enable_mask |= Proto::SP_ENABLE_VALVE;
            uartSend(Proto::MSG_SETPOINTS_V1, &g_sp, sizeof(g_sp));
            Serial.println("OK: setpoint valve sent");
            return;
        }

        Serial.println("ERR: unknown set item");
        return;
    }

    Serial.println("ERR: unknown command (try: help)");
}

static void handleUartRx()
{
    int budget = 256;  // 每轮最多处理 256 字节，留时间给 LoRa/其他任务
    while (Serial1.available() > 0 && budget-- > 0) {
        uint8_t b = static_cast<uint8_t>(Serial1.read());
        FrameCodec::FrameView f;
        if (g_parser.feed(b, f)) {
            // 1) UART->LoRa：将 Nano33BLE 的帧重新编码后排队，避免在 UART 接收路径上阻塞。
            //    - ACK：高优先级
            //    - TELEM：低优先级（覆盖旧数据，按周期发）
            //    - 其他：高优先级（例如未来的错误/事件上报）
            {
                uint8_t pkt[256];
                const size_t n = FrameCodec::encode(f.msg_type, f.seq,
                                                    f.payload, f.payload_len,
                                                    pkt, sizeof(pkt));
                if (n) {
                    if (f.msg_type == Proto::MSG_TELEM_V1) {
                        memcpy(g_tx_telem_buf, pkt, n);
                        g_tx_telem_len = n;
                    } else {
                        // 若连续产生多个高优先级帧，保留最新一帧即可（ACK 典型为对控制命令的响应）。
                        memcpy(g_tx_hi_buf, pkt, n);
                        g_tx_hi_len = n;
                    }
                }
            }

            // 2) 同时在 USB 串口做可读输出（可通过 debug 开关控制）
            if (!g_verbose) {
                continue;
            }

            if (f.msg_type == Proto::MSG_ACK && f.payload_len == sizeof(Proto::PayloadAck)) {
                Proto::PayloadAck ack;
                memcpy(&ack, f.payload, sizeof(ack));
                Serial.print("[ACK] for=0x");
                Serial.print(ack.acked_msg_type, HEX);
                Serial.print(" status=");
                Serial.println(ack.status);
            } else if (f.msg_type == Proto::MSG_TELEM_V1 && f.payload_len == sizeof(Proto::PayloadTelemetryV1)) {
                Proto::PayloadTelemetryV1 t;
                memcpy(&t, f.payload, sizeof(t));
                if (g_verbose_telem) {
                    Serial.print("[TELEM] t=");
                    Serial.print(t.timestamp_ms);
                    Serial.print(" T0=");
                    Serial.print(t.temp_c[0]);
                    Serial.print(" T1=");
                    Serial.print(t.temp_c[1]);
                    Serial.print(" P(Pa)=");
                    Serial.print(t.pressure_pa);
                    Serial.print(" heater=%=");
                    Serial.print(t.heater_power_pct);
                    Serial.print(" valve=%=");
                    Serial.println(t.valve_opening_pct);
                }
            } else {
                Serial.print("[RX] msg=0x");
                Serial.print(f.msg_type, HEX);
                Serial.print(" len=");
                Serial.println(f.payload_len);
            }
        }
    }
}

static void serviceLoRaTx(uint32_t now_ms)
{
    if (!g_lora_ok) return;

    // 近期刚收到下行控制时，短暂抑制遥测上行，减少“半双工错过命令”的概率。
    const bool suppress_telem = (now_ms - g_last_downlink_ms) < 80;

    // 1) 高优先级先发
    if (g_tx_hi_len > 0) {
        if (g_tx_hi_len < 3 || g_tx_hi_buf[0] != FrameCodec::SYNC1 || g_tx_hi_buf[1] != FrameCodec::SYNC2) {
            if (g_verbose_lora_drop) {
                Serial.print("[LORA][TX] hi buffer invalid, head=");
                dumpHexPrefix(g_tx_hi_buf, (int)g_tx_hi_len, 12);
            }
            g_tx_hi_len = 0;
            return;
        }
        logLoRaTx("HI", g_tx_hi_buf, g_tx_hi_len);
        const LoRaLink::TxResult txr = LoRaLink::sendEx(g_tx_hi_buf, g_tx_hi_len);
        if (txr == LoRaLink::TxResult::OK) {
            g_tx_hi_len = 0;
        } else if (g_debug_lora_tx) {
            Serial.print("[LORA][TX] HI send ");
            Serial.println(txr == LoRaLink::TxResult::BUSY ? "BUSY" : "FAIL");
        }
        return;
    }

    // 2) 低优先级遥测：降采样
    if (!suppress_telem && g_tx_telem_len > 0) {
        if (now_ms - g_last_telem_lora_ms >= BoardConfig::LORA_TELEM_PERIOD_MS) {
            if (g_tx_telem_len >= 3 && g_tx_telem_buf[0] == FrameCodec::SYNC1 && g_tx_telem_buf[1] == FrameCodec::SYNC2) {
                logLoRaTx("TELEM", g_tx_telem_buf, g_tx_telem_len);
            } else {
                if (g_verbose_lora_drop && g_tx_telem_len > 0) {
                    Serial.print("[LORA][TX] telem buffer invalid, head=");
                    dumpHexPrefix(g_tx_telem_buf, (int)g_tx_telem_len, 12);
                }
                g_tx_telem_len = 0;
                return;
            }
            const LoRaLink::TxResult txr = LoRaLink::sendEx(g_tx_telem_buf, g_tx_telem_len);
            if (txr == LoRaLink::TxResult::OK) {
                g_last_telem_lora_ms = now_ms;
                // 遥测允许被覆盖：仅在发送成功后清空；失败时保留，下一轮继续尝试或被新遥测覆盖。
                g_tx_telem_len = 0;
            } else if (g_debug_lora_tx) {
                Serial.print("[LORA][TX] TELEM send ");
                Serial.println(txr == LoRaLink::TxResult::BUSY ? "BUSY" : "FAIL");
            }
        }
    }
}

static bool isAllowedDownlink(uint8_t msg_type, uint8_t payload_len)
{
    // 空中端从 LoRa 下行只接受“控制类”消息，避免误把噪声/遥测误转发到 33BLE
    switch (msg_type) {
    case Proto::MSG_MODE_SWITCH:
        return (payload_len == sizeof(Proto::PayloadModeSwitch));
    case Proto::MSG_MANUAL_CMD_V1:
        return (payload_len == sizeof(Proto::PayloadManualCmdV1));
    case Proto::MSG_SETPOINTS_V1:
        return (payload_len == sizeof(Proto::PayloadSetpointsV1));
    case Proto::MSG_HEARTBEAT:
        return (payload_len == 0);
    default:
        return false;
    }
}

static void dumpHexPrefix(const uint8_t *buf, int len, int maxBytes)
{
    if (!buf || len <= 0 || maxBytes <= 0) return;
    const int n = (len < maxBytes) ? len : maxBytes;
    for (int i = 0; i < n; ++i) {
        if (i) Serial.print(' ');
        if (buf[i] < 0x10) Serial.print('0');
        Serial.print(buf[i], HEX);
    }
    if (len > n) Serial.print(" ...");
    Serial.println();
}

static void logLoRaTx(const char* tag, const uint8_t* buf, size_t len)
{
    if (!g_debug_lora_tx) return;
    Serial.print("[LORA][TX] ");
    Serial.print(tag ? tag : "?");
    Serial.print(" len=");
    Serial.print((int)len);
    if (buf && len >= 5 && buf[0] == FrameCodec::SYNC1 && buf[1] == FrameCodec::SYNC2) {
        Serial.print(" msg=0x");
        Serial.print(buf[3], HEX);
        Serial.print(" seq=");
        Serial.print(buf[4]);
        Serial.print(" head=");
        dumpHexPrefix(buf, (int)len, 12);
    } else {
        Serial.print(" head=");
        dumpHexPrefix(buf, (int)len, 12);
    }
}

static void handleLoRaRx()
{
    uint8_t buf[256];
    LoRaLink::RxPacket rx;
    if (!LoRaLink::pollReceive(buf, sizeof(buf), rx)) return;

    // 无论是否能解析出合法帧，都至少打印一条“收到 LoRa 包”的摘要，避免误以为完全没收到。
    if (g_verbose) {
        Serial.print("[LORA] rx len=");
        Serial.print(rx.len);
        Serial.print(" rssi=");
        Serial.print(rx.rssi);
        Serial.print(" snr=");
        Serial.println(rx.snr);
    }

    // 原始嗅探模式：直接打印 payload（ASCII + hex），不做协议解析/转发。
    if (g_lora_raw) {
        Serial.print("[LORA] payload(ascii): ");
        for (int i = 0; i < rx.len; ++i) {
            const char c = (char)buf[i];
            if (c >= 32 && c <= 126) Serial.write(c);
            else Serial.write('.');
        }
        Serial.println();
        Serial.print("[LORA] payload(hex): ");
        dumpHexPrefix(buf, rx.len, 255);
        g_last_lora_rssi = rx.rssi;
        g_last_lora_snr  = rx.snr;
        g_last_lora_rx_ms = millis();
        return;
    }

    // LoRa 上可能存在其他网络/干扰包；我们只从包内提取“通过 CRC 的合法帧”，并进一步做消息白名单过滤。
    FrameCodec::Parser p;
    FrameCodec::FrameView f;
    int forwarded = 0;

    for (int i = 0; i < rx.len; ++i) {
        if (p.feed(buf[i], f)) {
            if (!isAllowedDownlink(f.msg_type, f.payload_len)) {
                continue;
            }
            // 重新编码为标准帧（避免将 LoRa 包中的前导噪声一并转发）
            uint8_t pkt[256];
            const size_t n = FrameCodec::encode(f.msg_type, f.seq, f.payload, f.payload_len, pkt, sizeof(pkt));
            if (n) {
                uart1WriteDropIfBusy(pkt, n, "LORA->BLE");
                ++forwarded;
            }
        }
    }

    if (forwarded == 0) {
        if (g_verbose_lora_drop) {
            Serial.print("[LORA] no valid downlink frame in packet, head=");
            dumpHexPrefix(buf, rx.len, 12);
        }
        return;
    }

    g_last_downlink_ms = millis();

    g_last_lora_rssi = rx.rssi;
    g_last_lora_snr  = rx.snr;
    g_last_lora_rx_ms = millis();

    if (g_verbose) {
        Serial.print("[LORA] downlink frames forwarded=");
        Serial.print(forwarded);
        Serial.print(" (packet len=");
        Serial.print(rx.len);
        Serial.print(") rssi=");
        Serial.print(rx.rssi);
        Serial.print(" snr=");
        Serial.println(rx.snr);
    }
}

void setup()
{
    Serial.begin(BoardConfig::USB_BAUD);
    // USB CDC 在部分系统上需要稍作等待，避免开机首屏信息丢失
    delay(150);

    // Nano ESP32: 显式指定 UART 引脚，并适当增大缓冲区以提高抗阻塞能力
    Serial1.setRxBufferSize(1024);
    Serial1.setTxBufferSize(1024);
    Serial1.begin(BoardConfig::UART_BAUD, SERIAL_8N1, BoardConfig::UART_RX_PIN, BoardConfig::UART_TX_PIN);

    g_lora_ok = LoRaLink::begin();

    if (Serial) {
        Serial.println("Nano ESP32 AirGateway booted.");
        Serial.print("LoRa init: ");
        Serial.println(g_lora_ok ? "OK" : "FAILED (check wiring / library / freq)");
    }
    // 通过 LoRa 发送一个启动签名，便于地面端确认“发射端固件确实在运行”
    if (g_lora_ok) {
        const char* sig = "AIRGW_BOOT";
        LoRaLink::send(reinterpret_cast<const uint8_t*>(sig), strlen(sig));
        uint8_t pkt[32];
        const size_t n = FrameCodec::encode(Proto::MSG_HEARTBEAT, 0x42, nullptr, 0, pkt, sizeof(pkt));
        if (n) LoRaLink::send(pkt, n);
    }

    // 启动后立即打一帧心跳，让 33BLE 尽快进入 link=OK
    uartSend(Proto::MSG_HEARTBEAT, nullptr, 0);
    g_last_hb_ms = millis();
    if (Serial) {
        printHelp();
    }
}

void loop()
{
    const uint32_t now_ms = millis();

    // 1) 周期心跳
    sendHeartbeat(now_ms);

    // 2) UART 接收遥测/ACK（不在此路径上做 LoRa 发送）
    handleUartRx();

    // 2.5) LoRa 接收来自地面的命令帧 -> UART 转发给 Nano33BLE
    handleLoRaRx();

    // 2.6) LoRa 上行发送服务（可能阻塞，放在 UART/LoRa RX 之后）
    serviceLoRaTx(now_ms);

    // 3) 读取 USB 串口行
    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());
        if (c == '\n') {
            g_line_buf[g_line_len] = 0;
            handleLine(g_line_buf);
            g_line_len = 0;
        } else if (c == '\r') {
            // ignore
        } else {
            if (g_line_len + 1 < sizeof(g_line_buf)) {
                g_line_buf[g_line_len++] = c;
            } else {
                // overflow: reset
                g_line_len = 0;
            }
        }
    }
}
