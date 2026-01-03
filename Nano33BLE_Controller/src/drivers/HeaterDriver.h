// drivers/HeaterDriver.h
#pragma once

#include <Arduino.h>

class HeaterDriver {
public:
    explicit HeaterDriver(uint8_t pin = 2) : pin_(pin) {}

    void begin();

    // 0..100
    void setPowerPct(float pct);

    float lastPowerPct() const { return last_pct_; }

private:
    uint8_t pin_;
    float last_pct_{0.0f};
};
