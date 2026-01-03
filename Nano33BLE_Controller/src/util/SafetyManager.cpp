// util/SafetyManager.cpp
#include "SafetyManager.h"

#include "BoardConfig.h"

void SafetyManager::begin()
{
    // 可在此加载阈值/配置
}

void SafetyManager::checkAndClamp(ControlState &state,
                                  const Proto::Telemetry &telem,
                                  Proto::Outputs &out,
                                  uint32_t now_ms)
{
    // 1) 通讯超时 -> SAFE
    if (state.link_alive) {
        if (now_ms - state.last_link_heartbeat_ms > BoardConfig::LINK_TIMEOUT_MS) {
            state.link_alive = false;
        }
    }

    if (!state.link_alive) {
        // 若链路断开，强制 SAFE
        state.mode = ControlMode::SAFE;
    }

    // 2) 过温保护 -> SAFE
    for (uint8_t i = 0; i < telem.temp_count; ++i) {
        if (!isnan(telem.temp_c[i]) && telem.temp_c[i] > max_temp_c_) {
            state.mode = ControlMode::SAFE;
            break;
        }
    }

    // 3) SAFE 模式输出强制归零
    if (state.mode == ControlMode::SAFE) {
        out.heater_power_pct = 0.0f;
        out.valve_opening_pct = 0.0f;
        out.pump_target_temp_c = 0.0f;
    }
}
