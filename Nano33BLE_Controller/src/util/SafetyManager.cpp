#include "SafetyManager.h"

void SafetyManager::begin()
{
    max_temp_c_ = 80.0f; // 后续可做成参数
}

void SafetyManager::checkAndClamp(ControlState &state,
                                  const Proto::Telemetry &telem,
                                  Proto::Outputs &out,
                                  uint32_t /*now_ms*/)
{
    if (telem.temp_count > 0) {
        float T = telem.temp_c[0];
        if (T > max_temp_c_) {
            // 超温 --> 强制 SAFE
            state.mode = ControlMode::SAFE;
            out.heater_power_pct  = 0.0f;
            out.valve_opening_pct = 0.0f;
        }
    }
}
