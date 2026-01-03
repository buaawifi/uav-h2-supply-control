// proto/Protocol.h
#pragma once

#include <Arduino.h>

namespace Proto {

// 与《程序总体架构.pdf》保持一致的 MsgType 基本分配：
// - 0x01: Telemetry
// - 0x10: ModeSwitch
// - 0x11: Setpoints
// - 0x12: ManualCmd
// - 0x20: ACK
// - 0x23: Heartbeat

static constexpr uint8_t MSG_TELEM_V1      = 0x01;
static constexpr uint8_t MSG_MODE_SWITCH   = 0x10;
static constexpr uint8_t MSG_SETPOINTS_V1  = 0x11;
static constexpr uint8_t MSG_MANUAL_CMD_V1 = 0x12;
static constexpr uint8_t MSG_ACK           = 0x20;
static constexpr uint8_t MSG_HEARTBEAT     = 0x23;

// 控制模式值（payload 中使用）
static constexpr uint8_t MODE_SAFE   = 0;
static constexpr uint8_t MODE_MANUAL = 1;
static constexpr uint8_t MODE_AUTO   = 2;

// ACK status
static constexpr uint8_t ACK_OK  = 0;
static constexpr uint8_t ACK_ERR = 1;

#pragma pack(push, 1)

struct PayloadModeSwitch {
    uint8_t mode; // MODE_* 
};

struct PayloadAck {
    uint8_t acked_msg_type;
    uint8_t status;
};

// ManualCmd: 允许只更新部分通道，由 flags 指定
struct PayloadManualCmdV1 {
    uint8_t flags; // bit0=heater, bit1=valve, bit2=pump
    float heater_power_pct;
    float valve_opening_pct;
    float pump_target_temp_c;
};

// Setpoints: 当前阶段仅保留字段（自动控制未实现，但保留协议/文件）
struct PayloadSetpointsV1 {
    float target_temp_c;
    float target_pressure_pa;
    float target_valve_opening_pct;
    float target_pump_temp_c;
    uint8_t enable_mask; // bit0=temp, bit1=pressure, bit2=valve, bit3=pump
};

// Telemetry: 为跨平台稳定传输，使用 wire-format 的紧凑结构，避免直接发送 C++ struct
struct PayloadTelemetryV1 {
    uint32_t timestamp_ms;
    uint8_t  temp_count;   // <=4
    float    temp_c[4];
    float    pressure_pa;
    float    heater_power_pct;
    float    valve_opening_pct;
};

#pragma pack(pop)

static constexpr uint8_t MAN_FLAG_HEATER = 1u << 0;
static constexpr uint8_t MAN_FLAG_VALVE  = 1u << 1;
static constexpr uint8_t MAN_FLAG_PUMP   = 1u << 2;

static constexpr uint8_t SP_ENABLE_TEMP     = 1u << 0;
static constexpr uint8_t SP_ENABLE_PRESSURE = 1u << 1;
static constexpr uint8_t SP_ENABLE_VALVE    = 1u << 2;
static constexpr uint8_t SP_ENABLE_PUMP     = 1u << 3;

} // namespace Proto
