#include "ModeManager.h"

void ModeManager::begin()
{
    // 如需从配置加载参数,可在此处理
    Kp_heater_ = 5.0f;
}

static float clampPct(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 100.0f) v = 100.0f;
    return v;
}

void ModeManager::compute(const ControlState &state,
                          const Proto::Telemetry &telem,
                          Proto::Outputs &out)
{
    switch (state.mode) {
    case ControlMode::SAFE:
        out.heater_power_pct   = 0.0f;
        out.valve_opening_pct  = 0.0f;
        out.pump_target_temp_c = 0.0f;
        break;

    case ControlMode::MANUAL:
        // 只使用有标记的手动指令
        if (state.manual_cmd.has_heater_cmd) {
            out.heater_power_pct = clampPct(state.manual_cmd.heater_power_pct);
        } else {
            out.heater_power_pct = 0.0f;
        }

        if (state.manual_cmd.has_valve_cmd) {
            out.valve_opening_pct = clampPct(state.manual_cmd.valve_opening_pct);
        } else {
            out.valve_opening_pct = 0.0f;
        }

        if (state.manual_cmd.has_pump_temp_cmd) {
            out.pump_target_temp_c = state.manual_cmd.pump_target_temp_c;
        } else {
            out.pump_target_temp_c = 0.0f;
        }
        break;

    case ControlMode::AUTO:
    default:
        // 简单比例控制, 用 T0 和 target_temp_c
        if (state.setpoints.enable_temp_ctrl && telem.temp_count > 0) {
            float T_meas = telem.temp_c[0];
            float T_set  = state.setpoints.target_temp_c;
            float error  = T_set - T_meas;
            float u      = Kp_heater_ * error;  // 正误差 -> 加热输出
            out.heater_power_pct = clampPct(u);
        } else {
            out.heater_power_pct = 0.0f;
        }

        // 阀门暂时不在自动模式中使用
        out.valve_opening_pct  = 0.0f;
        out.pump_target_temp_c = 0.0f;
        break;
    }
}
