#include "Actuators.h"

// 根据自己硬件分配引脚
// 这里只是示例
static const int PIN_HEATER_PWM = 3;
static const int PIN_VALVE_PWM  = 5;

void Actuators::begin()
{
    pinMode(PIN_HEATER_PWM, OUTPUT);
    pinMode(PIN_VALVE_PWM,  OUTPUT);
    analogWrite(PIN_HEATER_PWM, 0);
    analogWrite(PIN_VALVE_PWM,  0);
}

static uint8_t pctToPwm(float pct)
{
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return (uint8_t)(255.0f * (pct / 100.0f));
}

void Actuators::apply(const Proto::Outputs &out)
{
    uint8_t pwm_heater = pctToPwm(out.heater_power_pct);
    uint8_t pwm_valve  = pctToPwm(out.valve_opening_pct);

    analogWrite(PIN_HEATER_PWM, pwm_heater);
    analogWrite(PIN_VALVE_PWM,  pwm_valve);
}
