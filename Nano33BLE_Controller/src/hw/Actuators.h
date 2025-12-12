// src/hw/Actuators.h
#pragma once

#include <Arduino.h>
#include "../proto/Messages.h"

class Actuators {
public:
    static void begin();
    static void apply(const Proto::Outputs &out);
};
