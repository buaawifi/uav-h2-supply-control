// hw/Sensors.cpp
#include "Sensors.h"

#include <SPI.h>

#include "../util/BoardConfig.h"

void Sensors::begin()
{
    SPI.begin();

    // 2 路 PT100（可扩展至 4 路）
    for (uint8_t i = 0; i < BoardConfig::TEMP_SENSOR_COUNT; ++i) {
        pt100_[i].configure(BoardConfig::PT100_CS_PINS[i],
                            BoardConfig::PT100_R0,
                            BoardConfig::PT100_RREF,
                            BoardConfig::PT100_A,
                            BoardConfig::PT100_B);
        pt100_[i].begin();
    }

    ads1115_ = Ads1115Driver(BoardConfig::ADS1115_ADDR);
    ads1115_.begin();
}

float Sensors::readPressurePa()
{
    // ADS1115 AIN0-AIN1 差分读数（±0.256V）。
    const int16_t raw = ads1115_.readDiff01(BoardConfig::ADS1115_CONFIG_DIFF_0_1, 10);
    if (!ads1115_.lastOk()) {
        return NAN;
    }
    const float volts = static_cast<float>(raw) * BoardConfig::ADS1115_LSB_V; // V
    float mv = volts * 1000.0f;

    // 与旧版 Sensors.cpp 处理方式保持一致：
    // - 差分输入在接线极性相反时会读出负值，这里统一取绝对值
    // - 标定后若出现负压，钳位到 0
    if (mv < 0.0f) mv = -mv;

    float kPa = (mv - BoardConfig::PRESS_V0_mV) / BoardConfig::PRESS_SENS_mV_PER_kPa;
    if (kPa < 0.0f) kPa = 0.0f;

    return kPa * 1000.0f;
}

void Sensors::readAll(Proto::Telemetry &telem)
{
    telem.timestamp_ms = millis();

    telem.temp_count = BoardConfig::TEMP_SENSOR_COUNT;
    for (uint8_t i = 0; i < telem.temp_count; ++i) {
        float tc = NAN;
        if (!pt100_[i].readTemperatureC(tc)) {
            tc = NAN;
        }
        telem.temp_c[i] = tc;
    }

    telem.pressure_pa = readPressurePa();
}
