// ctrl/ControlState.h
#pragma once

#include <Arduino.h>
#include "ControlModes.h"
#include "../proto/Messages.h"

struct ControlState {
    // 当前控制模式
    ControlMode mode = ControlMode::SAFE;

    // 最近一次生效的自动设定值
    Proto::Setpoints setpoints;

    // 最近一次生效的手动指令
    Proto::ManualCmd manual_cmd;

    // 链路与命令时间戳（毫秒）
    uint32_t last_cmd_ms       = 0;  // 最近一次收到任何命令的时间
    uint32_t last_setpoint_ms  = 0;  // 最近一次更新 setpoints 的时间
    uint32_t last_manual_ms    = 0;  // 最近一次更新 manual_cmd 的时间

    // 通讯状态
    bool link_alive            = false;  // 机载 ESP32 / 地面链路是否正常
    uint32_t last_link_heartbeat_ms = 0; // 最近一次心跳时间

    // 预留字段，便于后续扩展
    uint8_t reserved_u8[4]     = {0};
    float   reserved_f32[4]    = {0.0f};

    // 一些简单帮助方法，可选
    void reset() {
        mode = ControlMode::SAFE;
        setpoints = Proto::Setpoints{};
        manual_cmd = Proto::ManualCmd{};
        last_cmd_ms = last_setpoint_ms = last_manual_ms = 0;
        link_alive = false;
        last_link_heartbeat_ms = 0;
    }
};
