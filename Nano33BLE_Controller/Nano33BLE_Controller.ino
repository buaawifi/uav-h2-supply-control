#include <Arduino.h>

#include "src/proto/Messages.h"
#include "src/ctrl/ControlModes.h"
#include "src/ctrl/ControlState.h"
#include "src/ctrl/ModeManager.h"
#include "src/hw/Sensors.h"
#include "src/hw/Actuators.h"
#include "src/util/SafetyManager.h"

// ================= 全局对象 =================
ControlState      g_ctrlState;
Proto::Telemetry  g_telem;
Proto::Outputs    g_outputs;

Sensors           g_sensors;
Actuators         g_actuators;
ModeManager       g_modeManager;
SafetyManager     g_safety;

// ================= 辅助函数 =================

const __FlashStringHelper *modeToString(ControlMode mode)
{
    switch (mode) {
    case ControlMode::MANUAL: return F("MANUAL");
    case ControlMode::AUTO:   return F("AUTO");
    case ControlMode::SAFE:
    default:                  return F("SAFE");
    }
}

void processCommandLine(const String &line, uint32_t now_ms, bool from_link)
{
    if (line.length() == 0) return;

    // 标记链路存活
    if (from_link) {
        g_ctrlState.link_alive = true;
        g_ctrlState.last_link_heartbeat_ms = now_ms;
    }

    if (line.startsWith("mode")) {
        if (line.indexOf("auto") >= 0) {
            g_ctrlState.mode = ControlMode::AUTO;
            Serial.println(F("[CMD] Set mode = AUTO"));
        } else if (line.indexOf("manual") >= 0) {
            g_ctrlState.mode = ControlMode::MANUAL;
            Serial.println(F("[CMD] Set mode = MANUAL"));
        } else if (line.indexOf("safe") >= 0) {
            g_ctrlState.mode = ControlMode::SAFE;
            Serial.println(F("[CMD] Set mode = SAFE"));
        } else {
            Serial.println(F("[CMD] Unknown mode"));
        }
        g_ctrlState.last_cmd_ms = now_ms;
    } else if (line.startsWith("set T")) {
        int idx = line.indexOf('T');
        if (idx >= 0) {
            float val = line.substring(idx + 1).toFloat();
            g_ctrlState.setpoints.target_temp_c = val;
            g_ctrlState.setpoints.enable_temp_ctrl = true;
            g_ctrlState.last_setpoint_ms = now_ms;
            Serial.print(F("[CMD] Set target_temp_c = "));
            Serial.println(val, 2);
        }
    } else if (line.startsWith("set heater")) {
        // 手动模式加热功率: set heater 50
        float val = line.substring(String("set heater").length()).toFloat();
        g_ctrlState.manual_cmd.has_heater_cmd = true;
        g_ctrlState.manual_cmd.heater_power_pct = val;
        g_ctrlState.last_manual_ms = now_ms;
        Serial.print(F("[CMD] Manual heater_power_pct = "));
        Serial.println(val, 2);
    } else if (line.startsWith("set valve")) {
        // 手动模式阀门开度: set valve 30
        float val = line.substring(String("set valve").length()).toFloat();
        g_ctrlState.manual_cmd.has_valve_cmd = true;
        g_ctrlState.manual_cmd.valve_opening_pct = val;
        g_ctrlState.last_manual_ms = now_ms;
        Serial.print(F("[CMD] Manual valve_opening_pct = "));
        Serial.println(val, 2);
    } else {
        Serial.print(F("[CMD] Unknown: "));
        Serial.println(line);
    }
}

// 轮询串口/串口1，读取一行命令
void pollCommandStream(Stream &serial, bool from_link)
{
    while (serial.available()) {
        String line = serial.readStringUntil('\n');
        line.trim();
        processCommandLine(line, millis(), from_link);
    }
}

// 向链接口发送紧凑遥测（供 Nano ESP32 接收）
void sendLinkTelemetry(Stream &serial, uint32_t now_ms)
{
    serial.print(F("TELEM,"));
    serial.print(now_ms);
    serial.print(F(","));
    serial.print(modeToString(g_ctrlState.mode));
    serial.print(F(","));
    serial.print(g_telem.temp_count);
    serial.print(F(","));
    serial.print(g_telem.temp_c[0], 2);
    serial.print(F(","));
    serial.print(g_telem.temp_c[1], 2);
    serial.print(F(","));
    serial.print(g_telem.pressure_pa / 1000.0f, 2); // kPa for可读性
    serial.print(F(","));
    serial.print(g_outputs.heater_power_pct, 1);
    serial.print(F(","));
    serial.print(g_outputs.valve_opening_pct, 1);
    serial.println();
}

void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(10);
    while (!Serial) { ; } // 方便 PC 串口接入

    // 与 Nano ESP32 通过硬件串口交互（RX0/TX1）
    Serial1.begin(115200);
    Serial1.setTimeout(10);

    Serial.println(F("=== Nano33BLE Controller: minimal skeleton ==="));
    Serial1.println(F("[LINK] Nano33BLE ready"));

    g_ctrlState.reset();
    g_ctrlState.mode = ControlMode::SAFE;

    g_sensors.begin();
    g_actuators.begin();
    g_modeManager.begin();
    g_safety.begin();
}

void loop()
{
    static uint32_t lastPrintMs = 0;
    static uint32_t lastLinkTelemMs = 0;
    const uint32_t now = millis();

    // 1. 处理串口命令
    pollCommandStream(Serial, false);
    pollCommandStream(Serial1, true);

    // 2. 更新遥测
    g_sensors.readAll(g_telem);

    // 3. 控制模式决定输出
    g_modeManager.compute(g_ctrlState, g_telem, g_outputs);

    // 4. 安全检查 / 限制输出
    g_safety.checkAndClamp(g_ctrlState, g_telem, g_outputs, now);

    // 4.5 将最终输出抄送到遥测，便于链路传输
    g_telem.heater_power_pct  = g_outputs.heater_power_pct;
    g_telem.valve_opening_pct = g_outputs.valve_opening_pct;

    // 5. 应用输出到执行器
    g_actuators.apply(g_outputs);

    // 5.1 通过链接口周期发送遥测（给 Nano ESP32）
    if (now - lastLinkTelemMs >= 200) { // 5 Hz
        lastLinkTelemMs = now;
        sendLinkTelemetry(Serial1, now);
    }

    // 6. 周期打印状态
    if (now - lastPrintMs >= 1000) {
        lastPrintMs = now;

        Serial.print(F("[STAT] t="));
        Serial.print(now);
        Serial.print(F(" ms, mode="));
        Serial.print(modeToString(g_ctrlState.mode));

        Serial.print(F(", T0="));
        Serial.print(g_telem.temp_c[0], 2);
        Serial.print(F(" C"));
        Serial.print(F(", T1="));
        Serial.print(g_telem.temp_c[1], 2);
        Serial.print(F(" C"));
        Serial.print(F(", P="));
        Serial.print(g_telem.pressure_pa / 1000.0f, 2);  // kPa for readability
        Serial.print(F(" kPa"));
        Serial.print(F(", target="));
        Serial.print(g_ctrlState.setpoints.target_temp_c, 2);

        Serial.print(F(", heater="));
        Serial.print(g_outputs.heater_power_pct, 1);
        Serial.print(F("%, valve="));
        Serial.print(g_outputs.valve_opening_pct, 1);
        Serial.println(F("%"));
    }
}
