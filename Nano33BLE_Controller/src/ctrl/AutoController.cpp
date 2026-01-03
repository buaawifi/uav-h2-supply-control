// ctrl/AutoController.cpp
#include "AutoController.h"

void AutoController::compute(const ControlState & /*state*/, const Proto::Telemetry & /*telem*/, Proto::Outputs &out)
{
    // 占位：自动控制未实现。
    out.heater_power_pct = 0.0f;
    out.valve_opening_pct = 0.0f;
    out.pump_target_temp_c = 0.0f;
}
