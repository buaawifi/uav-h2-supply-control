#include <Arduino.h>

#include "src/proto/Messages.h"
#include "src/ctrl/ControlModes.h"
#include "src/ctrl/ControlState.h"
#include "src/ctrl/ModeManager.h"
#include "src/hw/Sensors.h"
#include "src/hw/Actuators.h"
#include "src/util/SafetyManager.h"
#include "src/drivers/UartLink.h"

// 全局对象
ControlState      g_ctrlState;
Proto::Telemetry  g_telem;
Proto::Outputs    g_outputs;

Sensors           g_sensors;
Actuators         g_actuators;
ModeManager       g_modeManager;
SafetyManager     g_safety;
UartLink          g_uartLink;

// 串口命令处理（非常简化）：
// 支持:
//   mode auto
//   mode manual
//   mode safe
//   set T 20.0
void handleSerialCommand()
{
    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

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
        g_ctrlState.last_cmd_ms = millis();
    } else if (line.startsWith("set T")) {
        int idx = line.indexOf('T');
        if (idx >= 0) {
            float val = line.substring(idx + 1).toFloat();
            g_ctrlState.setpoints.target_temp_c = val;
            g_ctrlState.setpoints.enable_temp_ctrl = true;
            g_ctrlState.last_setpoint_ms = millis();
            Serial.print(F("[CMD] Set target_temp_c = "));
            Serial.println(val, 2);
        }
    } else if (line.startsWith("set heater")) {
        // 手动模式加热功率: set heater 50
        float val = line.substring(String("set heater").length()).toFloat();
        g_ctrlState.manual_cmd.has_heater_cmd = true;
        g_ctrlState.manual_cmd.heater_power_pct = val;
        g_ctrlState.last_manual_ms = millis();
        Serial.print(F("[CMD] Manual heater_power_pct = "));
        Serial.println(val, 2);
    } else if (line.startsWith("set valve")) {
        // 手动模式阀门开度: set valve 30
        float val = line.substring(String("set valve").length()).toFloat();
        g_ctrlState.manual_cmd.has_valve_cmd = true;
        g_ctrlState.manual_cmd.valve_opening_pct = val;
        g_ctrlState.last_manual_ms = millis();
        Serial.print(F("[CMD] Manual valve_opening_pct = "));
        Serial.println(val, 2);
    } else {
        Serial.print(F("[CMD] Unknown: "));
        Serial.println(line);
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) { ; } // 方便 PC 串口接入

    Serial.println(F("=== Nano33BLE Controller: minimal skeleton ==="));

    // 与 ESP32 的硬件串口（根据你的实际 wiring，是 Serial1 或其它）
    Serial1.begin(115200);       // Nano33 D1/TX1 <-> ESP32 RX, D0/RX1 <-> ESP32 TX
    g_uartLink.begin(Serial1);

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
    const uint32_t now = millis();

    // 1) 处理地面经 ESP32 下发来的命令
    g_uartLink.poll(g_ctrlState);

    // 1. 处理串口命令
    handleSerialCommand();

    // 2. 更新遥测
    g_sensors.readAll(g_telem);

    // 3. 控制模式决定输出
    g_modeManager.compute(g_ctrlState, g_telem, g_outputs);

    // 4. 安全检查 / 限制输出
    g_safety.checkAndClamp(g_ctrlState, g_telem, g_outputs, now);

    // 5. 应用输出到执行器
    g_actuators.apply(g_outputs);

    // 4) 周期性通过 UART 向 ESP32 发送遥测
    static uint32_t last_telem_ms = 0;
    if (now - last_telem_ms >= 100) {  // 10 Hz
        g_uartLink.sendTelemetry(g_telem, g_outputs, g_ctrlState);
        last_telem_ms = now;
    }

    // 6. 周期打印状态
    if (now - lastPrintMs >= 1000) {
        lastPrintMs = now;

        Serial.print(F("[STAT] t="));
        Serial.print(now);
        Serial.print(F(" ms, mode="));
        switch (g_ctrlState.mode) {
        case ControlMode::MANUAL: Serial.print(F("MANUAL")); break;
        case ControlMode::AUTO:   Serial.print(F("AUTO"));   break;
        case ControlMode::SAFE:   Serial.print(F("SAFE"));   break;
        }

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