// src/hw/Actuators.cpp
#include "Actuators.h"

using namespace Proto;

namespace {

// 与原 arduino.cpp 保持一致
constexpr uint8_t HEATER_PIN = 2;  // XY-GMOS 加热片 PWM
constexpr uint8_t VALVE_PIN  = 3;  // 电磁阀 时间比例控制

// 电磁阀时间比例参数
constexpr unsigned long VALVE_CYCLE_MS = 500;

unsigned long s_valveCycleStartMs = 0;

// 加热片 PWM：0–100 % -> analogWrite 0–255
void updateHeaterPWM_Internal(int percent) {
    if (percent < 0)   percent = 0;
    if (percent > 100) percent = 100;

    int duty = map(percent, 0, 100, 0, 255);
    analogWrite(HEATER_PIN, duty);
}

// 电磁阀时间比例控制：0–100 %，周期 VALVE_CYCLE_MS
void updateValvePWM_Internal(int percent, unsigned long now) {
    if (percent < 0)   percent = 0;
    if (percent > 100) percent = 100;

    // 极端：0% / 100% 直接关/开
    if (percent == 0) {
        digitalWrite(VALVE_PIN, LOW);
        return;
    }
    if (percent == 100) {
        digitalWrite(VALVE_PIN, HIGH);
        return;
    }

    // 根据 now 对齐周期起点，避免 millis 溢出误差累积
    if ((unsigned long)(now - s_valveCycleStartMs) >= VALVE_CYCLE_MS) {
        unsigned long cycles = (now - s_valveCycleStartMs) / VALVE_CYCLE_MS;
        s_valveCycleStartMs += cycles * VALVE_CYCLE_MS;
    }

    unsigned long phase  = now - s_valveCycleStartMs;             // 当前周期内位置
    unsigned long onTime = (VALVE_CYCLE_MS * (unsigned long)percent) / 100UL;

    if (phase < onTime) {
        digitalWrite(VALVE_PIN, HIGH);
    } else {
        digitalWrite(VALVE_PIN, LOW);
    }
}

} // namespace

// ========== 对外接口 ==========

void Actuators::begin() {
    pinMode(HEATER_PIN, OUTPUT);
    pinMode(VALVE_PIN,  OUTPUT);

    digitalWrite(HEATER_PIN, LOW);
    digitalWrite(VALVE_PIN,  LOW);

    // Nano 33 BLE 默认是 8 bit 分辨率，但显式设一下更安全
    analogWriteResolution(8);

    s_valveCycleStartMs = millis();
}

void Actuators::apply(const Outputs &out) {
    // 这里假定 Outputs 中是 0–100 的百分比，如有不同请按需缩放
    int heaterPercent = static_cast<int>(out.heater_power_pct);
    int valvePercent  = static_cast<int>(out.valve_opening_pct);

    updateHeaterPWM_Internal(heaterPercent);
    updateValvePWM_Internal(valvePercent, millis());
}
