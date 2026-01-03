// drivers/Ads1115Driver.cpp
#include "Ads1115Driver.h"

void Ads1115Driver::begin() {
    Wire.begin();
}

void Ads1115Driver::writeReg16(uint8_t reg, uint16_t value)
{
    last_ok_ = true;

    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.write(static_cast<uint8_t>(value >> 8));
    Wire.write(static_cast<uint8_t>(value & 0xFF));
    const uint8_t rc = Wire.endTransmission();
    if (rc != 0) last_ok_ = false;
}

int16_t Ads1115Driver::readReg16(uint8_t reg)
{
    last_ok_ = true;

    Wire.beginTransmission(addr_);
    Wire.write(reg);
    const uint8_t rc = Wire.endTransmission();
    if (rc != 0) {
        last_ok_ = false;
        return 0;
    }

    const int req = Wire.requestFrom(addr_, static_cast<uint8_t>(2));
    if (req < 2) {
        last_ok_ = false;
        while (Wire.available()) (void)Wire.read();
        return 0;
    }

    const uint32_t t0 = millis();
    while (Wire.available() < 2) {
        if (millis() - t0 > 20) {     // 20ms 超时，防止 I2C 异常导致死等
            last_ok_ = false;
            while (Wire.available()) (void)Wire.read();
            return 0;
        }
        delay(1);
    }

    const uint8_t msb = Wire.read();
    const uint8_t lsb = Wire.read();
    const uint16_t u16 = (static_cast<uint16_t>(msb) << 8) | static_cast<uint16_t>(lsb);
    return static_cast<int16_t>(u16);
}

int16_t Ads1115Driver::readDiff01(uint16_t config, uint16_t settle_ms)
{
    writeReg16(0x01, config);
    delay(settle_ms);
    return readReg16(0x00);
}