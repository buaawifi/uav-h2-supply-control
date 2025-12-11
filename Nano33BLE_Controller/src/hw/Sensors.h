#pragma once

#include <Arduino.h>
#include "../proto/Messages.h"

class Sensors {
public:
    void begin();

    // 写入到传入的 Telemetry 结构体
    void readAll(Proto::Telemetry &telem);
};
