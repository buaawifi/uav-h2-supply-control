// ctrl/ControlModes.h
#pragma once

#include <Arduino.h>

enum class ControlMode : uint8_t {
    MANUAL = 0,  // 手动模式
    AUTO   = 1,  // 自动模式
    SAFE   = 2   // 安全模式 (故障/断链)
};

// 可选：简单的模式优先级判断函数（后续 ModeManager/安全逻辑会用到）
inline ControlMode maxPriorityMode(ControlMode a, ControlMode b) {
    // SAFE 优先级 > MANUAL > AUTO
    const uint8_t score[] = {
        1, // MANUAL
        0, // AUTO
        2  // SAFE
    };
    auto idx = [](ControlMode m) -> uint8_t {
        switch (m) {
            case ControlMode::MANUAL: return 0;
            case ControlMode::AUTO:   return 1;
            case ControlMode::SAFE:   return 2;
        }
        return 2; // 默认 SAFE
    };
    return (score[idx(a)] >= score[idx(b)]) ? a : b;
}
