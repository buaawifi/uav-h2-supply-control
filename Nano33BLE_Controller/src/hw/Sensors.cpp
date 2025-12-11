#include "Sensors.h"

// 如果有具体硬件，可以在这里 include 对应驱动头文件

void Sensors::begin()
{
    // 初始化传感器硬件 (I2C/SPI/ADC 等)
    // 目前可以留空，后续接真实传感器
}

void Sensors::readAll(Proto::Telemetry &telem)
{
    telem.timestamp_ms = millis();

    // 示例：假装只有 1 路温度
    telem.temp_count   = 1;

    // 暂时用一个简单的“虚拟温度”，方便调试：
    // 20 +/- 2 C 之间缓慢变化
    float base = 20.0f;
    float delta = 2.0f * sinf( (float)millis() / 5000.0f );
    telem.temp_c[0] = base + delta;

    // 其他传感器先填默认值
    telem.pressure_pa          = 101325.0f;
    telem.valve_opening_pct    = 0.0f;  // 当前阀门开度由执行器反馈决定,先留0
    telem.heater_power_pct     = 0.0f;  // 同上,实际可由 Actuators 回写
    telem.env_temp_c           = 25.0f;
    telem.env_humidity_pct     = 50.0f;

    telem.telem_seq++;
}
