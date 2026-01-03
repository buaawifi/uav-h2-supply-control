// ctrl/ModeManager.cpp
#include "ModeManager.h"

void ModeManager::begin()
{
    auto_ctrl_.begin();
}

void ModeManager::compute(const ControlState &state,
                          const Proto::Telemetry &telem,
                          Proto::Outputs &out)
{
    // 默认 SAFE
    out.heater_power_pct = 0.0f;
    out.valve_opening_pct = 0.0f;
    out.pump_target_temp_c = 0.0f;

    switch (state.mode) {
    case ControlMode::SAFE:
        // already zeros
        break;

    case ControlMode::MANUAL:
        // 仅在 MANUAL 模式中执行手动指令
        if (state.manual_cmd.has_heater_cmd) {
            out.heater_power_pct = state.manual_cmd.heater_power_pct;
        }
        if (state.manual_cmd.has_valve_cmd) {
            out.valve_opening_pct = state.manual_cmd.valve_opening_pct;
        }
        if (state.manual_cmd.has_pump_temp_cmd) {
            out.pump_target_temp_c = state.manual_cmd.pump_target_temp_c;
        }
        break;

    case ControlMode::AUTO:
        // 自动控制占位：保留文件/接口，不实现算法
        auto_ctrl_.compute(state, telem, out);
        break;

    default:
        break;
    }
}
