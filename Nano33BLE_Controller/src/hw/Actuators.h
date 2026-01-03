// hw/Actuators.h
#pragma once

#include <Arduino.h>

#include "../proto/Messages.h"
#include "../drivers/HeaterDriver.h"
#include "../drivers/ValveDriver.h"

class Actuators {
public:
    void begin();

    // 根据 outputs 输出到硬件
    void apply(const Proto::Outputs &out, uint32_t now_ms);

private:
    HeaterDriver heater_;
    ValveDriver  valve_;
};
