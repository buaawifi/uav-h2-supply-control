// proto/Protocol.h
#pragma once

#include <Arduino.h>

namespace Proto {

static constexpr uint8_t MSG_TELEM_V1      = 0x01;
static constexpr uint8_t MSG_MODE_SWITCH   = 0x10;
static constexpr uint8_t MSG_SETPOINTS_V1  = 0x11;
static constexpr uint8_t MSG_MANUAL_CMD_V1 = 0x12;
static constexpr uint8_t MSG_ACK           = 0x20;
static constexpr uint8_t MSG_HEARTBEAT     = 0x23;

static constexpr uint8_t MODE_SAFE   = 0;
static constexpr uint8_t MODE_MANUAL = 1;
static constexpr uint8_t MODE_AUTO   = 2;

static constexpr uint8_t ACK_OK  = 0;
static constexpr uint8_t ACK_ERR = 1;

#pragma pack(push, 1)

struct PayloadModeSwitch {
    uint8_t mode;
};

struct PayloadAck {
    uint8_t acked_msg_type;
    uint8_t status;
};

struct PayloadManualCmdV1 {
    uint8_t flags;
    float heater_power_pct;
    float valve_opening_pct;
    float pump_target_temp_c;
};

struct PayloadSetpointsV1 {
    float target_temp_c;
    float target_pressure_pa;
    float target_valve_opening_pct;
    float target_pump_temp_c;
    uint8_t enable_mask;
};

struct PayloadTelemetryV1 {
    uint32_t timestamp_ms;
    uint8_t  temp_count;
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
