// proto/Messages.h
#pragma once

#include <Arduino.h>   // 提供 uint8_t, uint16_t, uint32_t, String, etc.

namespace Proto {

// 可根据实际传感器数量调整
static const uint8_t kMaxTempSensors = 8;

// 遥测数据：从 Nano 33 BLE 上报给上位系统（机载/地面/PC）
struct Telemetry {
    uint32_t timestamp_ms = 0;               // 本地时间戳

    // 温度传感器
    float    temp_c[kMaxTempSensors] = {0};  // 摄氏度
    uint8_t  temp_count = 0;                 // 实际有效通道数

    // 压力
    float    pressure_pa = 0.0f;             // Pa

    // 阀门与加热状态（当前状态，非指令）
    float    valve_opening_pct = 0.0f;       // 0~100%
    float    heater_power_pct  = 0.0f;       // 0~100%

    // 环境参数
    float    env_temp_c        = 0.0f;
    float    env_humidity_pct  = 0.0f;

    // 预留字段，方便后续扩展
    uint8_t  reserved_u8[4]    = {0};
    float    reserved_f32[4]   = {0.0f};

    // 遥测序号（用于通信链路统计）
    uint32_t telem_seq         = 0;
};

// 控制输出：控制算法给执行器使用（只在控制板内部/机载 ESP32 显示用）
struct Outputs {
    float heater_power_pct     = 0.0f;       // 加热功率 0~100%
    float valve_opening_pct    = 0.0f;       // 阀门开度 0~100%
    float pump_target_temp_c   = 0.0f;       // 泵设定温度（如果需要）

    // 后续可以增加其他执行器：例如风扇、继电器等
    float reserved_f32[3]      = {0.0f};
};

// 自动模式下的设定值（由上位机/地面下发）
struct Setpoints {
    // 目标温度（比如试样温度或腔体温度）
    float target_temp_c        = 0.0f;

    // 目标压力（如果有需要）
    float target_pressure_pa   = 0.0f;

    // 目标阀门开度（有时自动模式会用开度闭环）
    float target_valve_opening_pct = 0.0f;

    // 目标泵温度（如果有低温泵设定）
    float target_pump_temp_c   = 0.0f;

    // 有效性标记（例如暂时不使用某个设定）
    bool  enable_temp_ctrl     = true;
    bool  enable_pressure_ctrl = false;
    bool  enable_valve_ctrl    = false;
    bool  enable_pump_ctrl     = false;

    uint8_t reserved_u8[4]     = {0};
    float   reserved_f32[4]    = {0.0f};
};

// 手动模式下的指令（由地面操作员直接给出）
struct ManualCmd {
    // 单独对每个执行器给“是否有命令”的标记，避免每次都覆盖全部输出
    bool  has_heater_cmd       = false;
    float heater_power_pct     = 0.0f;       // 0~100%

    bool  has_valve_cmd        = false;
    float valve_opening_pct    = 0.0f;       // 0~100%

    bool  has_pump_temp_cmd    = false;
    float pump_target_temp_c   = 0.0f;

    // 命令序号，用于 ACK/重发
    uint32_t cmd_seq           = 0;

    uint8_t reserved_u8[4]     = {0};
    float   reserved_f32[4]    = {0.0f};
};

} // namespace Proto
