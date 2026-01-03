// drivers/ValveDriver.h
#pragma once

#include <Arduino.h>

// 电磁阀时间比例控制（Time-Proportioning Control, TPC）
// - 周期 cycle_ms
// - pct=0/100 时强制关/开

class ValveDriver {
public:
    ValveDriver(uint8_t pin = 3, uint32_t cycle_ms = 500)
        : pin_(pin), cycle_ms_(cycle_ms) {}

    void begin();

    // 0..100
    void setOpeningPct(float pct, uint32_t now_ms);

    float lastOpeningPct() const { return last_pct_; }

private:
    uint8_t pin_;
    uint32_t cycle_ms_;
    uint32_t cycle_start_ms_{0};
    float last_pct_{0.0f};
};
