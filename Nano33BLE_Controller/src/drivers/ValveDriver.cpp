// drivers/ValveDriver.cpp
#include "ValveDriver.h"
#include <math.h>

static float clampPct(float v)
{
    if (!isfinite(v)) return 0.0f;
    if (v < 0.0f) v = 0.0f;
    if (v > 100.0f) v = 100.0f;
    return v;
}

void ValveDriver::begin()
{
    pinMode(pin_, OUTPUT);
    digitalWrite(pin_, LOW);
    cycle_start_ms_ = millis();
}

void ValveDriver::setOpeningPct(float pct, uint32_t now_ms)
{
    pct = clampPct(pct);
    last_pct_ = pct;

    const int ipct = static_cast<int>(pct + 0.5f);

    if (ipct <= 0) {
        digitalWrite(pin_, LOW);
        return;
    }
    if (ipct >= 100) {
        digitalWrite(pin_, HIGH);
        return;
    }

    // 对齐周期起点，避免 millis 溢出导致误差累积
    const uint32_t elapsed = static_cast<uint32_t>(now_ms - cycle_start_ms_);
    if (elapsed >= cycle_ms_) {
        const uint32_t cycles = elapsed / cycle_ms_;
        cycle_start_ms_ += cycles * cycle_ms_;
    }

    const uint32_t phase  = static_cast<uint32_t>(now_ms - cycle_start_ms_);
    const uint32_t on_ms  = (cycle_ms_ * static_cast<uint32_t>(ipct)) / 100UL;

    digitalWrite(pin_, (phase < on_ms) ? HIGH : LOW);
}
