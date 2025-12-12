// src/hw/Sensors.cpp
#include "Sensors.h"

#include <Wire.h>
#include <SPI.h>
#include <math.h>

using namespace Proto;

namespace {

// ========== 硬件引脚与地址，与 arduino.cpp 保持一致 ==========

// MAX31865 CS 引脚
constexpr uint8_t MAX1_CS_PIN = 10;   // PT100 #1
constexpr uint8_t MAX2_CS_PIN = 9;    // PT100 #2

// ADS1115 地址 (ADDR -> GND)
constexpr uint8_t ADS1115_ADDR = 0x48;

// ========== MAX31865 / PT100 常数 ==========
// 模块参考电阻 402 Ω（丝印“4020”）
constexpr float RTD_R0   = 100.0f;
constexpr float RTD_RREF = 402.0f;
// Callendar–Van Dusen 系数
constexpr float RTD_A    = 3.9083e-3f;
constexpr float RTD_B    = -5.775e-7f;

// ========== ADS1115 配置（压差 AIN0–AIN1） ==========
// 单次转换, 差分 AIN0-AIN1, FSR=±0.256V, 128SPS, 关比较器
constexpr uint16_t ADS1115_CONFIG_DIFF_0_1 = 0x8B83;
// FSR=±0.256V -> 每 bit 电压
constexpr float ADS1115_LSB_V = 0.256f / 32768.0f;

// ========== 压力传感器标定（你之前 arduino.cpp 中的参数） ==========
// 0 kPa 时 2.73 mV，灵敏度 0.117 mV/kPa
constexpr float PRESS_V0_mV   = 2.73f;
constexpr float PRESS_SENS_mV = 0.117f;

// ========== MAX31865 底层 SPI ==========

void max31865WriteReg8(int csPin, uint8_t addr, uint8_t value) {
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE1));
    digitalWrite(csPin, LOW);
    SPI.transfer(addr | 0x80); // 写寄存器：最高位=1
    SPI.transfer(value);
    digitalWrite(csPin, HIGH);
    SPI.endTransaction();
}

uint16_t max31865ReadReg16(int csPin, uint8_t addr) {
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE1));
    digitalWrite(csPin, LOW);
    SPI.transfer(addr & 0x7F); // 读寄存器：最高位=0
    uint8_t msb = SPI.transfer(0x00);
    uint8_t lsb = SPI.transfer(0x00);
    digitalWrite(csPin, HIGH);
    SPI.endTransaction();
    return ((uint16_t)msb << 8) | lsb;
}

// 初始化 MAX31865: 4 线 PT100，连续转换，50 Hz 滤波
void max31865Init(int csPin) {
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);

    // 寄存器 0x00:
    // VBIAS=1, MODE=1(连续), 3-wire=0(2/4线), Filter=1(50Hz)
    max31865WriteReg8(csPin, 0x00, 0xC3);
}

// 读取原始 RTD 15 bit (去掉 fault bit)
uint16_t max31865ReadRTDRaw(int csPin) {
    uint16_t raw = max31865ReadReg16(csPin, 0x01);  // RTD_MSB=0x01
    raw >>= 1;                                      // LSB bit0 是 fault 标志
    return raw;
}

// RTD 电阻 -> 温度 (°C)
float rtdResistanceToTemperature(float Rt) {
    // 正温区解析解 (T >= 0 °C)
    float Z1 = -RTD_A;
    float Z2 = RTD_A * RTD_A - (4.0f * RTD_B);
    float Z3 = (4.0f * RTD_B) / RTD_R0;
    float Z4 = 2.0f * RTD_B;

    float temp = Z2 + (Z3 * Rt);
    temp = (sqrtf(temp) + Z1) / Z4;

    if (temp >= 0.0f) {
        return temp;  // 0 ~ 850 °C
    }

    // 负温区近似多项式（与之前 arduino.cpp 一致）
    float rpoly = Rt;
    temp  = -242.02f;
    temp += 2.2228f * rpoly;
    rpoly *= Rt;
    temp += 2.5859e-3f * rpoly;
    rpoly *= Rt;
    temp -= 4.8260e-6f * rpoly;
    rpoly *= Rt;
    temp -= 2.8183e-8f * rpoly;
    rpoly *= Rt;
    temp += 1.5243e-10f * rpoly;

    return temp;
}

// 读某一路 PT100 温度 (°C)
float readPT100TempC(int csPin) {
    uint16_t rtd   = max31865ReadRTDRaw(csPin);
    float    ratio = (float)rtd / 32768.0f;
    float    Rt    = ratio * RTD_RREF;
    return rtdResistanceToTemperature(Rt);
}

// ========== ADS1115 底层 I2C ==========

void ads1115WriteReg16(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(ADS1115_ADDR);
    Wire.write(reg);
    Wire.write((uint8_t)(value >> 8));   // MSB
    Wire.write((uint8_t)(value & 0xFF)); // LSB
    Wire.endTransmission();
}

int16_t ads1115ReadReg16(uint8_t reg) {
    Wire.beginTransmission(ADS1115_ADDR);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom(ADS1115_ADDR, (uint8_t)2);
    while (Wire.available() < 2) {
        // 等两个字节
    }
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    int16_t value = (int16_t)((msb << 8) | lsb);
    return value;
}

// 差分 AIN0-AIN1 单次转换
int16_t ads1115ReadDiff01() {
    ads1115WriteReg16(0x01, ADS1115_CONFIG_DIFF_0_1);
    delay(10);  // 128 SPS 时转换时间 ~7.8 ms，粗略等待即可
    return ads1115ReadReg16(0x00);
}

// 压力 (kPa)
float readPressureKPa() {
    int16_t raw   = ads1115ReadDiff01();
    float   volts = raw * ADS1115_LSB_V;
    float   mV    = volts * 1000.0f;

    // 接法: 负极->A0, 正极->A1，配置 A0-A1，读数为负，取绝对值
    if (mV < 0.0f) mV = -mV;

    float p = (mV - PRESS_V0_mV) / PRESS_SENS_mV;
    if (p < 0.0f) p = 0.0f;
    return p;
}

} // namespace

// ========== 对外接口 ==========

void Sensors::begin() {
    // I2C / SPI 总线
    Wire.begin();
    SPI.begin();

    // MAX31865 两路初始化
    max31865Init(MAX1_CS_PIN);
    max31865Init(MAX2_CS_PIN);
}

void Sensors::readAll(Telemetry &telem) {
    const float t1 = readPT100TempC(MAX1_CS_PIN);
    const float t2 = readPT100TempC(MAX2_CS_PIN);
    const float p  = readPressureKPa();

    telem.temp_c[0] = t1;
    telem.temp_c[1] = t2;
    telem.temp_count = 2;
    telem.pressure_pa = p * 1000.0f;  // Telemetry 中单位是 Pa
}
