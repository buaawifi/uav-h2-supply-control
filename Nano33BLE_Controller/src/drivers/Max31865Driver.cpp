// drivers/Max31865Driver.cpp
#include "Max31865Driver.h"

#include <math.h>

void Max31865Driver::configure(uint8_t cs_pin, float rtd_r0, float rref, float a, float b)
{
    cs_pin_ = cs_pin;
    rtd_r0_ = rtd_r0;
    rref_   = rref;
    a_      = a;
    b_      = b;
}

void Max31865Driver::begin()
{
    if (cs_pin_ == 255) return;

    pinMode(cs_pin_, OUTPUT);
    digitalWrite(cs_pin_, HIGH);

    // 配置寄存器 0x00:
    // VBIAS=1, MODE=1(连续), 1SHOT=0, 3-wire=0(2/4线), Fault=00,
    // FaultClear=0, Filter=1(50Hz)
    // 0b1100 0011 = 0xC3
    writeReg8(0x00, 0xC3);

    // 清一次故障
    clearFault();
}

void Max31865Driver::writeReg8(uint8_t addr, uint8_t value)
{
    SPI.beginTransaction(spi_);
    digitalWrite(cs_pin_, LOW);
    SPI.transfer(addr | 0x80); // 写寄存器：最高位=1
    SPI.transfer(value);
    digitalWrite(cs_pin_, HIGH);
    SPI.endTransaction();
}

uint16_t Max31865Driver::readReg16(uint8_t addr)
{
    SPI.beginTransaction(spi_);
    digitalWrite(cs_pin_, LOW);
    SPI.transfer(addr & 0x7F); // 读寄存器：最高位=0
    const uint8_t msb = SPI.transfer(0x00);
    const uint8_t lsb = SPI.transfer(0x00);
    digitalWrite(cs_pin_, HIGH);
    SPI.endTransaction();

    return (static_cast<uint16_t>(msb) << 8) | static_cast<uint16_t>(lsb);
}

uint16_t Max31865Driver::readRawRtd()
{
    // RTD_MSB = 0x01, RTD_LSB = 0x02
    uint16_t raw = readReg16(0x01);
    raw >>= 1; // bit0 为 fault
    return raw;
}

uint8_t Max31865Driver::readFault()
{
    // Fault Status = 0x07
    SPI.beginTransaction(spi_);
    digitalWrite(cs_pin_, LOW);
    SPI.transfer(0x07 & 0x7F);
    uint8_t v = SPI.transfer(0x00);
    digitalWrite(cs_pin_, HIGH);
    SPI.endTransaction();
    return v;
}

void Max31865Driver::clearFault()
{
    // 置位 FaultClear(bit1) 即可清除。
    // 读取当前配置，然后 OR 0x02。
    SPI.beginTransaction(spi_);
    digitalWrite(cs_pin_, LOW);
    SPI.transfer(0x00 & 0x7F);
    uint8_t cfg = SPI.transfer(0x00);
    digitalWrite(cs_pin_, HIGH);
    SPI.endTransaction();

    cfg |= 0x02;
    writeReg8(0x00, cfg);
}

float Max31865Driver::resistanceToTempC(float rt_ohm)
{
    // 正温区解析解 (T >= 0 °C)
    const float Z1 = -a_;
    const float Z2 = a_ * a_ - (4.0f * b_);
    const float Z3 = (4.0f * b_) / rtd_r0_;
    const float Z4 = 2.0f * b_;

    float temp = Z2 + (Z3 * rt_ohm);
    temp = (sqrtf(temp) + Z1) / Z4;

    if (temp >= 0.0f) {
        return temp;
    }

    // 负温区近似多项式（与之前版本保持一致）
    float rpoly = rt_ohm;
    temp  = -242.02f;
    temp += 2.2228f * rpoly;
    rpoly *= rt_ohm;
    temp += 2.5859e-3f * rpoly;
    rpoly *= rt_ohm;
    temp -= 4.8260e-6f * rpoly;
    rpoly *= rt_ohm;
    temp -= 2.8183e-8f * rpoly;
    rpoly *= rt_ohm;
    temp += 1.5243e-10f * rpoly;

    return temp;
}

bool Max31865Driver::readResistanceOhm(float &rt_ohm)
{
    if (cs_pin_ == 255) return false;

    const uint8_t fault = readFault();
    if (fault != 0) {
        clearFault();
        return false;
    }

    const uint16_t raw = readRawRtd();
    const float ratio = static_cast<float>(raw) / 32768.0f;
    rt_ohm = ratio * rref_;

    // 粗略 sanity check
    if (!isfinite(rt_ohm) || rt_ohm < 1.0f || rt_ohm > 2000.0f) {
        return false;
    }

    return true;
}

bool Max31865Driver::readTemperatureC(float &temp_c)
{
    float rt;
    if (!readResistanceOhm(rt)) {
        return false;
    }
    temp_c = resistanceToTempC(rt);
    return isfinite(temp_c);
}
