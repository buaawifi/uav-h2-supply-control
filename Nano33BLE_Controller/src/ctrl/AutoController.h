// ctrl/AutoController.h
#pragma once

#include "ControlState.h"
#include "../proto/Messages.h"

// 自动控制器（占位版本）：
// 按总体架构保留模块，但当前阶段不实现自动控制算法。

class AutoController {
public:
    void begin() {}

    // 输入 telemetry + setpoints，输出 outputs
    // 当前返回 SAFE 等价输出：heater=0, valve=0。
    void compute(const ControlState &state, const Proto::Telemetry &telem, Proto::Outputs &out);
};
