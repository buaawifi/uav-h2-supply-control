// drivers/HeaterDriver.cpp
#include "HeaterDriver.h"
#include <math.h>

static float clampPct(float v)
{
    if (!isfinite(v)) return 0.0f;  // NaN/Inf -> 安全退化
    if (v < 0.0f) v = 0.0f;
    if (v > 100.0f) v = 100.0f;
    return v;
}

void HeaterDriver::begin()
{
    pinMode(pin_, OUTPUT);
    digitalWrite(pin_, LOW);

    // Nano 33 BLE: analogWrite 分辨率可配置
    analogWriteResolution(8);
}

void HeaterDriver::setPowerPct(float pct)
{
    pct = clampPct(pct);
    last_pct_ = pct;

    const int duty = map(static_cast<int>(pct + 0.5f), 0, 100, 0, 255);
    analogWrite(pin_, duty);
}
