// drivers/Max31865Driver.h
#pragma once

#include <Arduino.h>
#include <SPI.h>

// 极简 MAX31865 驱动：不依赖第三方库。
// 配置为：
// - 4 线 PT100
// - 连续转换
// - 50 Hz 工频滤波

class Max31865Driver {
public:
    Max31865Driver() = default;

    void configure(uint8_t cs_pin,
                   float rtd_r0 = 100.0f,
                   float rref   = 402.0f,
                   float a      = 3.9083e-3f,
                   float b      = -5.775e-7f);

    void begin();

    // 读取温度（°C）。返回 false 表示出现故障位或读数明显异常。
    bool readTemperatureC(float &temp_c);

    // 读取电阻（Ω）。返回 false 表示故障。
    bool readResistanceOhm(float &rt_ohm);

    // 原始 15-bit RTD 码（已去掉 fault bit）
    uint16_t readRawRtd();

    // 故障寄存器（0x07）
    uint8_t readFault();

    // 清除故障（W1C）
    void clearFault();

private:
    uint8_t cs_pin_{255};
    float rtd_r0_{100.0f};
    float rref_{402.0f};
    float a_{3.9083e-3f};
    float b_{-5.775e-7f};

    SPISettings spi_{500000, MSBFIRST, SPI_MODE1};

    void writeReg8(uint8_t addr, uint8_t value);
    uint16_t readReg16(uint8_t addr);

    float resistanceToTempC(float rt_ohm);
};
