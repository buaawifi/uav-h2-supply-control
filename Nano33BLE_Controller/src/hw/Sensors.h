// src/hw/Sensors.h
#pragma once

#include <Arduino.h>
#include "../proto/Messages.h"

class Sensors {
public:
    void begin();
    void readAll(Proto::Telemetry &telem);
};
