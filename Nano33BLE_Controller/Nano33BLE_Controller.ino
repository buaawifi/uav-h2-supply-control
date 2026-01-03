/*
 * Nano33BLE_Controller.ino
 *
 * 按《程序总体架构.pdf》进行组织：
 * - Drivers: MAX31865 / ADS1115 / PWM(Heater) / TPC(Valve) / UartLink
 * - HW Abstraction: Sensors / Actuators
 * - Ctrl: ModeManager(Manual/Auto占位)/ControlState
 * - Util: SafetyManager
 *
 * 当前阶段约束：
 * 1) 不实现自动控制算法（保留 AutoController 文件/接口）。
 * 2) 温度传感器当前为 2 路 PT100，后续可扩展至 4 路（修改 BoardConfig.h）。
 * 3) 与机载 ESP32 通过 Serial1 (D0/D1) 通讯；USB Serial 仅用于调试输出。
 */

#include <Arduino.h>

#include "src/util/BoardConfig.h"

#include "src/hw/Sensors.h"
#include "src/hw/Actuators.h"

#include "src/ctrl/ControlState.h"
#include "src/ctrl/ModeManager.h"

#include "src/util/SafetyManager.h"
#include "src/drivers/UartLink.h"

static ControlState g_state;
static Sensors g_sensors;
static Actuators g_actuators;
static ModeManager g_mode_mgr;
static SafetyManager g_safety;
static UartLink g_link(Serial1);

static Proto::Telemetry g_telem;
static Proto::Outputs g_out;

static uint32_t g_last_telem_tx_ms = 0;
static uint32_t g_last_debug_ms = 0;

void setup()
{
    Serial.begin(115200);
    delay(50);

    g_sensors.begin();
    g_actuators.begin();
    g_mode_mgr.begin();
    g_safety.begin();

    g_link.begin(BoardConfig::UART_BAUD);

    g_state.reset();
    g_state.mode = ControlMode::SAFE;

    Serial.println("Nano33BLE Controller booted.");
    Serial.println("Link: Serial1 @115200, frame protocol enabled.");
}

void loop()
{
    const uint32_t now_ms = millis();

    // 1) 通讯轮询：接收来自机载 ESP32 的命令 / 心跳
    g_link.poll(g_state, now_ms);

    // 2) 采集
    g_sensors.readAll(g_telem);

    // 3) 控制计算
    g_mode_mgr.compute(g_state, g_telem, g_out);

    // 4) 安全检查 + 输出钳制
    g_safety.checkAndClamp(g_state, g_telem, g_out, now_ms);

    // 5) 执行输出
    g_actuators.apply(g_out, now_ms);

    // 6) 上行遥测
    if (now_ms - g_last_telem_tx_ms >= BoardConfig::TELEMETRY_PERIOD_MS) {
        g_last_telem_tx_ms = now_ms;
        g_link.sendTelemetry(g_telem, g_out, now_ms);
    }

    // 7) USB 调试输出（低频）
    if (now_ms - g_last_debug_ms >= 1000) {
        g_last_debug_ms = now_ms;

        Serial.print("mode=");
        Serial.print(static_cast<int>(g_state.mode));
        Serial.print(" link=");
        Serial.print(g_state.link_alive ? "OK" : "LOST");
        Serial.print(" hb_age(ms)=");
        Serial.print(g_state.link_alive ? (now_ms - g_state.last_link_heartbeat_ms) : 0);
        Serial.print(" cmd_age(ms)=");
        Serial.print(now_ms - g_state.last_cmd_ms);
        Serial.print(" T0=");
        Serial.print(g_telem.temp_c[0]);
        Serial.print(" T1=");
        Serial.print(g_telem.temp_c[1]);
        Serial.print(" P(Pa)=");
        Serial.print(g_telem.pressure_pa);
        Serial.print(" heater=%=");
        Serial.print(g_out.heater_power_pct);
        Serial.print(" valve=%=");
        Serial.println(g_out.valve_opening_pct);
    }
}
