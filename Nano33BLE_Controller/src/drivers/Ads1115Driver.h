// drivers/Ads1115Driver.h
#pragma once

#include <Arduino.h>
#include <Wire.h>

// 极简 ADS1115 驱动：
// - 不依赖 Adafruit 库
// - 支持写配置寄存器、读转换寄存器

class Ads1115Driver {
public:
    explicit Ads1115Driver(uint8_t i2c_addr = 0x48) : addr_(i2c_addr) {}

    void begin();

    void writeReg16(uint8_t reg, uint16_t value);
    int16_t readReg16(uint8_t reg);

    // 差分 AIN0-AIN1，使用用户传入的 config
    int16_t readDiff01(uint16_t config, uint16_t settle_ms = 10);

    // 新增：上一次 I2C 读写是否成功
    bool lastOk() const { return last_ok_; }

private:
    uint8_t addr_;
    bool last_ok_ = true;   // 新增
};
