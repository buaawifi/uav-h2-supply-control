// hw/Actuators.cpp
#include "Actuators.h"

#include "../util/BoardConfig.h"

void Actuators::begin()
{
    heater_ = HeaterDriver(BoardConfig::HEATER_PIN);
    valve_  = ValveDriver(BoardConfig::VALVE_PIN, BoardConfig::VALVE_CYCLE_MS);

    heater_.begin();
    valve_.begin();
}

void Actuators::apply(const Proto::Outputs &out, uint32_t now_ms)
{
    heater_.setPowerPct(out.heater_power_pct);
    valve_.setOpeningPct(out.valve_opening_pct, now_ms);
}
