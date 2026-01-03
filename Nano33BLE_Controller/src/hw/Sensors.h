// hw/Sensors.h
#pragma once

#include <Arduino.h>

#include "../proto/Messages.h"
#include "../drivers/Max31865Driver.h"
#include "../drivers/Ads1115Driver.h"

class Sensors {
public:
    void begin();

    void readAll(Proto::Telemetry &telem);

private:
    Max31865Driver pt100_[4];
    Ads1115Driver ads1115_;

    float readPressurePa();
};
