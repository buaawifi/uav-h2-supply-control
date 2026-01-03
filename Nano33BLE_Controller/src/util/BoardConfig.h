// util/BoardConfig.h
#pragma once

#include <Arduino.h>

// 该文件集中管理板级硬件引脚与“当前已接入的通道数”。
// 目标：后续扩展（例如 2 路 PT100 -> 4 路 PT100）仅需改这里。

namespace BoardConfig {

// ===== PT100 / MAX31865 (SPI) =====
static constexpr uint8_t TEMP_SENSOR_MAX_COUNT = 4;
static constexpr uint8_t TEMP_SENSOR_COUNT     = 2;  // 当前已接 2 路

// MAX31865 的 CS 引脚；目前只使用前 TEMP_SENSOR_COUNT 项。
// 备注：PT100 #1 = D10, PT100 #2 = D9 为当前硬件连接。
// D8/D7 仅作为未来扩展占位。
static constexpr uint8_t PT100_CS_PINS[TEMP_SENSOR_MAX_COUNT] = {10, 9, 8, 7};

// 模块参考电阻 402 Ω（丝印“4020”）
static constexpr float PT100_R0   = 100.0f;
static constexpr float PT100_RREF = 402.0f;
static constexpr float PT100_A    = 3.9083e-3f;
static constexpr float PT100_B    = -5.775e-7f;

// ===== ADS1115 (I2C) =====
static constexpr uint8_t  ADS1115_ADDR             = 0x48;
static constexpr uint16_t ADS1115_CONFIG_DIFF_0_1  = 0x8B83; // 单次、差分AIN0-AIN1、±0.256V、128SPS
static constexpr float    ADS1115_LSB_V            = 0.256f / 32768.0f;

// 压力传感器标定：0 kPa 时 2.73 mV，灵敏度 0.117 mV/kPa
static constexpr float PRESS_V0_mV            = 2.73f;
static constexpr float PRESS_SENS_mV_PER_kPa  = 0.117f;

// ===== 执行器 =====
static constexpr uint8_t  HEATER_PIN       = 2;  // XY-GMOS 加热片 PWM
static constexpr uint8_t  VALVE_PIN        = 3;  // 电磁阀 时间比例控制
static constexpr uint32_t VALVE_CYCLE_MS   = 500;

// ===== 串口 / 链路 =====
static constexpr uint32_t UART_BAUD             = 115200;
static constexpr uint32_t TELEMETRY_PERIOD_MS   = 200;
static constexpr uint32_t LINK_TIMEOUT_MS       = 1500;

} // namespace BoardConfig
