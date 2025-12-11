#pragma once

#include <Arduino.h>
#include "../proto/Messages.h"

class Actuators {
public:
    void begin();

    // 根据 Outputs 施加到具体 IO
    void apply(const Proto::Outputs &out);
};
