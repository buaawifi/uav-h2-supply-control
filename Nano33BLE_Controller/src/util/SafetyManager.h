// util/SafetyManager.h
#pragma once

#include <Arduino.h>

#include "../ctrl/ControlState.h"
#include "../proto/Messages.h"

class SafetyManager {
public:
    void begin();

    // 根据安全条件调整模式/输出
    void checkAndClamp(ControlState &state,
                       const Proto::Telemetry &telem,
                       Proto::Outputs &out,
                       uint32_t now_ms);

private:
    float max_temp_c_ = 80.0f;  // 示例上限温度
};
