#include <Arduino.h>
#include "src/proto/FrameCodec.h"

using namespace Proto;

// === 硬件串口引脚（Nano ESP32 板载硬件 UART1 是 RX=17, TX=16） ===
// 如果你改成其它 GPIO，确保与 Nano33BLE 的 TX1/RX1 交叉连接
static const int UART_RX_PIN = 17;   // ESP32 接收脚 (接 Nano33 BLE 的 TX1)
static const int UART_TX_PIN = 16;   // ESP32 发送脚 (接 Nano33 BLE 的 RX1)

// === 解码后的遥测结构，仅用于打印 ===
struct TelemetryShort {
    uint32_t timestamp_ms;
    uint8_t  mode_wire;   // 0=SAFE,1=MANUAL,2=AUTO
    uint8_t  temp_count;
    float    t1_c;
    float    t2_c;
    float    pressure_pa;
    float    heater_pct;
    float    valve_pct;
};

static float readFloatLE(const uint8_t *buf, uint8_t offset)
{
    float v;
    uint32_t raw = (uint32_t)buf[offset] |
                   ((uint32_t)buf[offset + 1] << 8) |
                   ((uint32_t)buf[offset + 2] << 16) |
                   ((uint32_t)buf[offset + 3] << 24);
    memcpy(&v, &raw, sizeof(float));
    return v;
}

bool decodeTelemetryPayload(const uint8_t *payload, uint8_t length, TelemetryShort &out)
{
    if (length < 26) return false;

    uint8_t idx = 0;

    out.timestamp_ms = (uint32_t)payload[idx] |
                       ((uint32_t)payload[idx+1] << 8) |
                       ((uint32_t)payload[idx+2] << 16) |
                       ((uint32_t)payload[idx+3] << 24);
    idx += 4;

    out.mode_wire  = payload[idx++];
    out.temp_count = payload[idx++];

    out.t1_c       = readFloatLE(payload, idx); idx += 4;
    out.t2_c       = readFloatLE(payload, idx); idx += 4;
    out.pressure_pa = readFloatLE(payload, idx); idx += 4;
    out.heater_pct = readFloatLE(payload, idx); idx += 4;
    out.valve_pct  = readFloatLE(payload, idx); idx += 4;

    return true;
}

// === FrameCodec 接收状态机 ===
FrameCodec g_codec;

// Frame 回调：仅处理遥测 (0x01)
void onFrameFromNano33(const FrameCodec::Frame &frame, void * /*user*/)
{
    if (frame.msg_type != 0x01) {
        // 其他类型暂时忽略
        return;
    }

    TelemetryShort t;
    if (!decodeTelemetryPayload(frame.payload, frame.payload_len, t)) {
        return;
    }

    Serial.print(F("[TELEM] t="));
    Serial.print(t.timestamp_ms);
    Serial.print(F(" ms, mode="));
    Serial.print((int)t.mode_wire);
    Serial.print(F(", T1="));
    Serial.print(t.t1_c, 2);
    Serial.print(F(" C, T2="));
    Serial.print(t.t2_c, 2);
    Serial.print(F(" C, P="));
    Serial.print(t.pressure_pa / 1000.0f, 2);
    Serial.print(F(" kPa, heater="));
    Serial.print(t.heater_pct, 1);
    Serial.print(F("%, valve="));
    Serial.print(t.valve_pct, 1);
    Serial.print(F("%, temp_count="));
    Serial.println((int)t.temp_count);
}

// === 命令发送（ESP32 -> Nano33） ===
uint8_t g_seq_counter = 0;

uint8_t nextSeq()
{
    ++g_seq_counter;
    if (g_seq_counter == 0) g_seq_counter = 1;
    return g_seq_counter;
}

// 小端写入工具
static void writeU32LE(uint8_t *buf, uint8_t &idx, uint32_t v)
{
    buf[idx++] = (uint8_t)(v & 0xFF);
    buf[idx++] = (uint8_t)((v >> 8) & 0xFF);
    buf[idx++] = (uint8_t)((v >> 16) & 0xFF);
    buf[idx++] = (uint8_t)((v >> 24) & 0xFF);
}

static void writeFloatLE(uint8_t *buf, uint8_t &idx, float v)
{
    uint32_t raw;
    memcpy(&raw, &v, sizeof(float));
    writeU32LE(buf, idx, raw);
}

// 模式切换命令
void sendModeCommand(uint8_t wire_mode)
{
    uint8_t payload[1];
    payload[0] = wire_mode;  // 0=SAFE,1=MANUAL,2=AUTO
    uint8_t seq = nextSeq();
    FrameCodec::sendFrame(Serial1, 0x10, seq, payload, 1);

    Serial.print(F("[CMD] send mode="));
    Serial.print((int)wire_mode);
    Serial.print(F(", seq="));
    Serial.println((int)seq);
}

// 自动模式温度设定值命令
void sendSetpointCommand(float target_temp_c)
{
    uint8_t payload[4];
    uint8_t idx = 0;
    writeFloatLE(payload, idx, target_temp_c);
    uint8_t seq = nextSeq();
    FrameCodec::sendFrame(Serial1, 0x11, seq, payload, idx);

    Serial.print(F("[CMD] send set T="));
    Serial.print(target_temp_c, 2);
    Serial.print(F(" C, seq="));
    Serial.println((int)seq);
}

// 手动模式执行器命令 (heater_pct, valve_pct)
float g_last_heater_pct = 0.0f;
float g_last_valve_pct  = 0.0f;

void sendManualOutputs(float heater_pct, float valve_pct)
{
    uint8_t payload[8];
    uint8_t idx = 0;
    writeFloatLE(payload, idx, heater_pct);
    writeFloatLE(payload, idx, valve_pct);
    uint8_t seq = nextSeq();
    FrameCodec::sendFrame(Serial1, 0x12, seq, payload, idx);

    Serial.print(F("[CMD] send manual heater="));
    Serial.print(heater_pct, 1);
    Serial.print(F("%, valve="));
    Serial.print(valve_pct, 1);
    Serial.print(F("%, seq="));
    Serial.println((int)seq);
}

// === PC 串口命令解析 ===
// 支持命令：
//   mode safe
//   mode manual
//   mode auto
//   set T <value>
//   set heater <value>
//   set valve <value>

String g_line;

void handlePcCommand(const String &line)
{
    String s = line;
    s.trim();
    if (s.length() == 0) return;

    if (s.startsWith("mode")) {
        if (s.indexOf("safe") > 0) {
            sendModeCommand(0);
        } else if (s.indexOf("manual") > 0) {
            sendModeCommand(1);
        } else if (s.indexOf("auto") > 0) {
            sendModeCommand(2);
        } else {
            Serial.println(F("[ERR] mode command usage: mode safe|manual|auto"));
        }
    } else if (s.startsWith("set T")) {
        int pos = s.indexOf('T');
        String valStr = s.substring(pos + 1);
        valStr.trim();
        float T = valStr.toFloat();
        sendSetpointCommand(T);
    } else if (s.startsWith("set heater")) {
        int pos = s.indexOf("heater");
        String valStr = s.substring(pos + 6);
        valStr.trim();
        float h = valStr.toFloat();
        g_last_heater_pct = h;
        sendManualOutputs(g_last_heater_pct, g_last_valve_pct);
    } else if (s.startsWith("set valve")) {
        int pos = s.indexOf("valve");
        String valStr = s.substring(pos + 5);
        valStr.trim();
        float v = valStr.toFloat();
        g_last_valve_pct = v;
        sendManualOutputs(g_last_heater_pct, g_last_valve_pct);
    } else {
        Serial.println(F("[ERR] unknown command"));
    }
}

void setup()
{
    Serial.begin(115200);
    // 避免在无 USB 连接时卡死，超时 1.5s 自动继续
    uint32_t start_wait = millis();
    while (!Serial && (millis() - start_wait < 1500)) {
        delay(10);
    }

    Serial.println(F("=== NanoESP32_AirGateway: UART link with Nano33BLE ==="));
    Serial.println(F("Commands:"));
    Serial.println(F("  mode safe|manual|auto"));
    Serial.println(F("  set T <value_C>"));
    Serial.println(F("  set heater <pct>"));
    Serial.println(F("  set valve <pct>"));

    // Nano33 BLE 侧 UART
    Serial1.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
}

void loop()
{
    // 1) 从 Nano33 侧 UART 读数据 -> FrameCodec 解码
    while (Serial1.available() > 0) {
        uint8_t b = (uint8_t)Serial1.read();
        g_codec.processByte(b, &onFrameFromNano33, nullptr);
    }

    // 2) 从 PC 串口读取命令
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (g_line.length() > 0) {
                handlePcCommand(g_line);
                g_line = "";
            }
        } else {
            g_line += c;
        }
    }
}
