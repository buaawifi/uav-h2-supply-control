/*
 * NanoESP32_GroundGateway.ino
 *
 * 当前阶段：地面中继（Nano ESP32）
 * - LoRa 收发（与空中中继互通）
 * - USB 串口：
 *    1) 接收 PC ASCII 命令 -> 封装为帧 -> 通过 LoRa 发送给空中中继 -> 再转发至 Nano33BLE
 *    2) 接收来自空中中继的遥测/ACK 帧 -> 在 USB 串口解码打印
 */

#include <Arduino.h>

#include "src/util/BoardConfig.h"
#include "src/lora/LoRaLink.h"
#include "src/proto/FrameCodec.h"
#include "src/proto/Protocol.h"

static uint8_t g_tx_seq = 0;
static FrameCodec::Parser g_rx_parser;

static char g_line_buf[128];
static size_t g_line_len = 0;

static Proto::PayloadManualCmdV1 g_man = {0, 0.0f, 0.0f, 0.0f};
static Proto::PayloadSetpointsV1 g_sp  = {0.0f, 0.0f, 0.0f, 0.0f, 0};

static bool g_lora_raw = false; // 原始嗅探：打印任何包内容（ASCII/HEX），不做协议解码

// LoRa RX 健康监测：避免出现“运行数小时后不再接收，reset 又恢复”的僵死态
static uint32_t g_last_lora_pkt_ms = 0;
static uint32_t g_last_lora_reinit_ms = 0;

static void serviceLoRaWatchdog(uint32_t now_ms)
{
    // 仅在曾经收到过 LoRa 包后启用 watchdog，避免“对端关机”导致无意义频繁重启。
    if (g_last_lora_pkt_ms == 0) return;

    // 正常情况下空中端每 ~500ms 会转发一次遥测（AirGateway::LORA_TELEM_PERIOD_MS）。
    // 留足裕度：>5s 无任何 LoRa 包视为异常，触发自愈。
    if (now_ms - g_last_lora_pkt_ms < 5000) return;

    // 频率限制：避免反复重置导致更不稳定。
    if (now_ms - g_last_lora_reinit_ms < 3000) return;
    g_last_lora_reinit_ms = now_ms;

    Serial.println("[LORA] watchdog: no RX > 5s, reinit radio");
    const bool ok = LoRaLink::begin();
    Serial.print("[LORA] reinit: ");
    Serial.println(ok ? "OK" : "FAIL");

    // 防止下一轮立即再次触发
    g_last_lora_pkt_ms = now_ms;
}

// =======================
// 可靠下行（地面 -> 空中 -> 33BLE）
// =======================
struct PendingCmd {
    bool active = false;
    uint8_t msg_type = 0;
    uint8_t seq = 0;
    uint8_t frame[256] = {0};
    size_t  len = 0;
    uint32_t last_send_ms = 0;
    uint8_t retry = 0;

    // 新增：busy 管理
    bool sent_once = false;
    uint32_t busy_since_ms = 0;
    uint32_t last_busy_warn_ms = 0;
};

static PendingCmd g_pending;

static bool expectsAck(uint8_t msg_type)
{
    return (msg_type == Proto::MSG_MODE_SWITCH) ||
           (msg_type == Proto::MSG_MANUAL_CMD_V1) ||
           (msg_type == Proto::MSG_SETPOINTS_V1);
}

// static bool sendRawFrame(const uint8_t *buf, size_t len)
// {
//     if (!buf || len == 0) return false;
//     return LoRaLink::send(buf, len);
// }

static LoRaLink::TxResult sendRawFrameEx(const uint8_t *buf, size_t len)
{
    if (!buf || len == 0) return LoRaLink::TxResult::FAIL;
    return LoRaLink::sendEx(buf, len);
}

static bool startReliableSend(uint8_t msg_type, const void *payload, uint8_t payload_len)
{
    uint8_t buf[256];
    const uint8_t seq = g_tx_seq++;
    const uint8_t* p = (payload_len > 0) ? static_cast<const uint8_t*>(payload) : nullptr;
    const size_t n = FrameCodec::encode(msg_type, seq, p, payload_len, buf, sizeof(buf));

    if (n == 0) return false;

    g_pending.active = expectsAck(msg_type);
    g_pending.msg_type = msg_type;
    g_pending.seq = seq;
    g_pending.len = n;
    memcpy(g_pending.frame, buf, n);
    g_pending.retry = 0;

    g_pending.sent_once = false;
    g_pending.busy_since_ms = 0;
    g_pending.last_busy_warn_ms = 0;
    g_pending.last_send_ms = 0;

    // 立即尝试一次发送
    const uint32_t now = millis();
    const auto r = sendRawFrameEx(g_pending.frame, g_pending.len);
    if (r == LoRaLink::TxResult::OK || r == LoRaLink::TxResult::FAIL) {
        g_pending.sent_once = true;
        g_pending.last_send_ms = now;
        return (r == LoRaLink::TxResult::OK);
    }
    // BUSY：不算 retry、不启动 ACK 计时，交给 loop 里持续尝试
    g_pending.busy_since_ms = now;
    return false;
}

static void serviceReliableSend(uint32_t now_ms)
{
    if (!g_pending.active) return;

    // 若还没成功发出过（例如一直 BUSY），则持续尝试，但 BUSY 不计 retry
    if (!g_pending.sent_once) {
        const auto r = sendRawFrameEx(g_pending.frame, g_pending.len);
        if (r == LoRaLink::TxResult::OK || r == LoRaLink::TxResult::FAIL) {
            g_pending.sent_once = true;
            g_pending.last_send_ms = now_ms;
        } else { // BUSY
            if (g_pending.busy_since_ms == 0) g_pending.busy_since_ms = now_ms;
            if ((now_ms - g_pending.busy_since_ms) > 3000 && (now_ms - g_pending.last_busy_warn_ms) > 1000) {
                Serial.println("[CMD] WARNING: LoRa TX busy > 3s (busy does not count retry)");
                g_pending.last_busy_warn_ms = now_ms;
            }
        }
        return;
    }

    // 已经发出过：等待 ACK 超时才允许“真正重发”
    if (now_ms - g_pending.last_send_ms < BoardConfig::CMD_ACK_TIMEOUT_MS) return;

    if (g_pending.retry >= BoardConfig::CMD_MAX_RETRY) {
        Serial.print("[CMD] FAIL: no ACK for msg=0x");
        Serial.print(g_pending.msg_type, HEX);
        Serial.print(" seq=");
        Serial.println(g_pending.seq);
        g_pending.active = false;
        return;
    }

    // 先尝试发送；如果 BUSY：不计 retry、不更新时间戳（关键）
    const auto r = sendRawFrameEx(g_pending.frame, g_pending.len);
    if (r == LoRaLink::TxResult::BUSY) {
        if (g_pending.busy_since_ms == 0) g_pending.busy_since_ms = now_ms;
        if ((now_ms - g_pending.busy_since_ms) > 3000 && (now_ms - g_pending.last_busy_warn_ms) > 1000) {
            Serial.println("[CMD] WARNING: LoRa TX busy > 3s (busy does not count retry)");
            g_pending.last_busy_warn_ms = now_ms;
        }
        return;
    }

    // OK/FAIL：这次算一次真正的“重发尝试”
    g_pending.retry++;
    g_pending.last_send_ms = now_ms;
    g_pending.busy_since_ms = 0;

    Serial.print("[CMD] RETRY #");
    Serial.print(g_pending.retry);
    Serial.print(" msg=0x");
    Serial.print(g_pending.msg_type, HEX);
    Serial.print(" seq=");
    Serial.println(g_pending.seq);
}


static void printHelp()
{
    Serial.println("Commands:");
    Serial.println("  help");
    Serial.println("  mode safe|manual|auto");
    Serial.println("  set heater <0-100>");
    Serial.println("  set valve <0-100>");
    Serial.println("  set T <degC>            (setpoint, reserved for future auto)");
    Serial.println("  set P <Pa>              (setpoint, reserved for future auto)");
    Serial.println("  set valve_sp <0-100>    (setpoint)");
    Serial.println("  lora stat");
    Serial.println("  lora raw on|off        (print any LoRa packet, disable frame decode print)");
    Serial.println("  lora tx <text>         (send raw text over LoRa)");
    Serial.println("  lora ping              (send PING over LoRa)");
}

static bool parseFloat(const char *s, float &out)
{
    if (!s) return false;
    char *endp = nullptr;
    out = strtof(s, &endp);
    return (endp && endp != s);
}

// 兼容旧接口：用于无需 ACK 的 raw/测试发送
static bool loraSendFrameUnreliable(uint8_t msg_type, const void *payload, uint8_t payload_len)
{
    uint8_t buf[256];
    const size_t n = FrameCodec::encode(msg_type, g_tx_seq++,
                                        reinterpret_cast<const uint8_t*>(payload), payload_len,
                                        buf, sizeof(buf));
    if (!n) return false;
    return LoRaLink::send(buf, n);
}

static void handleLine(char *line)
{
    while (*line == ' ' || *line == '\t' || *line == '\r' || *line == '\n') ++line;
    if (*line == 0) return;

    char *cmd = strtok(line, " \t\r\n");
    if (!cmd) return;

    if (strcmp(cmd, "help") == 0) {
        printHelp();
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

            const auto& d = LoRaLink::diag();
            Serial.print("SelfHeal reinit_total=");
            Serial.print(d.reinit_total);
            Serial.print(" regver=");
            Serial.print(d.reinit_regver_bad);
            Serial.print(" tx_to=");
            Serial.print(d.reinit_tx_timeout);
            Serial.print(" opmode=");
            Serial.println(d.reinit_opmode_bad);

            Serial.print("SelfHeal last_reason=");
            Serial.print((int)d.last_reason);
            Serial.print(" last_ms=");
            Serial.print(d.last_reinit_ms);
            Serial.print(" samp: ver=0x");
            Serial.print(d.last_regver, HEX);
            Serial.print(" op=0x");
            Serial.print(d.last_opmode, HEX);
            Serial.print(" irq=0x");
            Serial.println(d.last_irqflags, HEX);
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
            const bool ok = LoRaLink::send(reinterpret_cast<const uint8_t*>(text), strlen(text));
            Serial.print(ok ? "OK" : "ERR");
            Serial.println(": lora tx");
            return;
        }
        if (sub && strcmp(sub, "ping") == 0) {
            const char *text = "PING";
            const bool ok = LoRaLink::send(reinterpret_cast<const uint8_t*>(text), strlen(text));
            Serial.print(ok ? "OK" : "ERR");
            Serial.println(": lora ping");
            return;
        }
        Serial.println("Usage: lora stat | lora raw on|off | lora tx <text> | lora ping");
        return;
    }

    if (strcmp(cmd, "mode") == 0) {
        char *arg = strtok(nullptr, " \t\r\n");
        Proto::PayloadModeSwitch p{};
        if (!arg) {
            Serial.println("ERR: mode missing");
            return;
        }
        if (strcmp(arg, "safe") == 0) p.mode = Proto::MODE_SAFE;
        else if (strcmp(arg, "manual") == 0) p.mode = Proto::MODE_MANUAL;
        else if (strcmp(arg, "auto") == 0) p.mode = Proto::MODE_AUTO;
        else {
            Serial.println("ERR: unknown mode");
            return;
        }
        if (startReliableSend(Proto::MSG_MODE_SWITCH, &p, sizeof(p))) {
            Serial.println("OK: mode sent (LoRa, wait ACK)");
        } else {
            Serial.println("ERR: LoRa send failed");
        }
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
            if (startReliableSend(Proto::MSG_MANUAL_CMD_V1, &g_man, sizeof(g_man))) {
                Serial.println("OK: heater cmd sent (LoRa, wait ACK)");
            } else {
                Serial.println("ERR: LoRa send failed");
            }
            return;
        }
        if (strcmp(what, "valve") == 0) {
            g_man.flags |= Proto::MAN_FLAG_VALVE;
            g_man.valve_opening_pct = f;
            if (startReliableSend(Proto::MSG_MANUAL_CMD_V1, &g_man, sizeof(g_man))) {
                Serial.println("OK: valve cmd sent (LoRa, wait ACK)");
            } else {
                Serial.println("ERR: LoRa send failed");
            }
            return;
        }

        // 自动控制预留 setpoints
        if (strcmp(what, "T") == 0) {
            g_sp.target_temp_c = f;
            g_sp.enable_mask |= Proto::SP_ENABLE_TEMP;
            if (startReliableSend(Proto::MSG_SETPOINTS_V1, &g_sp, sizeof(g_sp))) {
                Serial.println("OK: setpoint T sent (LoRa, wait ACK)");
            } else {
                Serial.println("ERR: LoRa send failed");
            }
            return;
        }
        if (strcmp(what, "P") == 0) {
            g_sp.target_pressure_pa = f;
            g_sp.enable_mask |= Proto::SP_ENABLE_PRESSURE;
            if (startReliableSend(Proto::MSG_SETPOINTS_V1, &g_sp, sizeof(g_sp))) {
                Serial.println("OK: setpoint P sent (LoRa, wait ACK)");
            } else {
                Serial.println("ERR: LoRa send failed");
            }
            return;
        }
        if (strcmp(what, "valve_sp") == 0) {
            g_sp.target_valve_opening_pct = f;
            g_sp.enable_mask |= Proto::SP_ENABLE_VALVE;
            if (startReliableSend(Proto::MSG_SETPOINTS_V1, &g_sp, sizeof(g_sp))) {
                Serial.println("OK: setpoint valve sent (LoRa, wait ACK)");
            } else {
                Serial.println("ERR: LoRa send failed");
            }
            return;
        }

        Serial.println("ERR: unknown set item");
        return;
    }

    Serial.println("ERR: unknown command (try: help)");
}

static void handleLoRaRx()
{
    uint8_t buf[256];
    LoRaLink::RxPacket rx;
    if (!LoRaLink::pollReceive(buf, sizeof(buf), rx)) return;

    g_last_lora_pkt_ms = millis();

    Serial.print("[LORA] rx len=");
    Serial.print(rx.len);
    Serial.print(" rssi=");
    Serial.print(rx.rssi);
    Serial.print(" snr=");
    Serial.println(rx.snr);

    if (g_lora_raw) {
        Serial.print("[LORA] payload(ascii): ");
        for (int i = 0; i < rx.len; ++i) {
            const char c = (char)buf[i];
            if (c >= 32 && c <= 126) Serial.write(c);
            else Serial.write('.');
        }
        Serial.println();
        Serial.print("[LORA] payload(hex): ");
        for (int i = 0; i < rx.len; ++i) {
            if (i) Serial.print(' ');
            if (buf[i] < 0x10) Serial.print('0');
            Serial.print(buf[i], HEX);
        }
        Serial.println();
        return;
    }

    // 以流式解析方式解码打印（同一包可能只包含一帧）
    FrameCodec::FrameView f;
    int frames = 0;
    for (int i = 0; i < rx.len; ++i) {
        if (g_rx_parser.feed(buf[i], f)) {
            ++frames;
            if (f.msg_type == Proto::MSG_ACK && f.payload_len == sizeof(Proto::PayloadAck)) {
                Proto::PayloadAck ack;
                memcpy(&ack, f.payload, sizeof(ack));
                Serial.print("[ACK] for=0x");
                Serial.print(ack.acked_msg_type, HEX);
                Serial.print(" status=");
                Serial.println(ack.status);

                // 可靠下行：匹配 pending
                if (g_pending.active && f.seq == g_pending.seq && ack.acked_msg_type == g_pending.msg_type) {
                    Serial.print("[CMD] ACK received for msg=0x");
                    Serial.print(g_pending.msg_type, HEX);
                    Serial.print(" seq=");
                    Serial.print(g_pending.seq);
                    Serial.print(" status=");
                    Serial.println(ack.status);
                    g_pending.active = false;
                }
            } else if (f.msg_type == Proto::MSG_TELEM_V1 && f.payload_len == sizeof(Proto::PayloadTelemetryV1)) {
                Proto::PayloadTelemetryV1 t;
                memcpy(&t, f.payload, sizeof(t));
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
            } else {
                Serial.print("[RX] msg=0x");
                Serial.print(f.msg_type, HEX);
                Serial.print(" len=");
                Serial.println(f.payload_len);
            }
        }
    }

    if (frames == 0) {
        Serial.println("[LORA] packet did not contain a valid frame (ignored)");
    }
}

void setup()
{
    Serial.begin(BoardConfig::USB_BAUD);
    delay(50);

    const bool lora_ok = LoRaLink::begin();

    Serial.println("Nano ESP32 GroundGateway booted.");
    Serial.print("LoRa init: ");
    Serial.println(lora_ok ? "OK" : "FAILED (check wiring / library / freq)");
    printHelp();
}

void loop()
{
    const uint32_t now_ms = millis();

    // 1) LoRa 接收来自空中中继的遥测/ACK
    handleLoRaRx();

    // 1.5) 可靠下行：若超时未收到 ACK，则自动重发
    serviceReliableSend(now_ms);

    // 1.6) LoRa 健康监测：必要时自动重置射频
    serviceLoRaWatchdog(now_ms);

    // 2) 读取 USB 串口行 -> LoRa 发送命令
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
                g_line_len = 0;
            }
        }
    }
}
