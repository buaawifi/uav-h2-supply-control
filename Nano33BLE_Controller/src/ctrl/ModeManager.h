#pragma once

#include <Arduino.h>
#include "ControlState.h"
#include "../proto/Messages.h"

class ModeManager {
public:
    void begin();

    // 根据当前控制状态和遥测，计算输出
    void compute(const ControlState &state,
                 const Proto::Telemetry &telem,
                 Proto::Outputs &out);

private:
    float Kp_heater_ = 5.0f;  // 简单比例系数,后面再换 PID
};
